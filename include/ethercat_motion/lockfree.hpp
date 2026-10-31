// SPDX-License-Identifier: MIT
//
// Wait-free primitives used to exchange data with the real-time cyclic task.
// None of them allocate after construction and the real-time side never
// blocks on a lock.
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <type_traits>

namespace ethercat_motion {

inline constexpr std::size_t kCacheLine = 64;

/// Bounded single-producer / single-consumer ring buffer.
template <typename T, std::size_t Capacity>
class SpscRing {
    static_assert(Capacity >= 2 && (Capacity & (Capacity - 1)) == 0, "Capacity must be a power of two");
    static_assert(std::is_trivially_copyable_v<T>, "SpscRing requires trivially copyable elements");

public:
    bool tryPush(const T& item) noexcept {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        const std::size_t tail = tail_.load(std::memory_order_acquire);
        if (head - tail >= Capacity) {
            return false;
        }
        buffer_[head & (Capacity - 1)] = item;
        head_.store(head + 1, std::memory_order_release);
        return true;
    }

    bool tryPop(T& out) noexcept {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        const std::size_t head = head_.load(std::memory_order_acquire);
        if (tail == head) {
            return false;
        }
        out = buffer_[tail & (Capacity - 1)];
        tail_.store(tail + 1, std::memory_order_release);
        return true;
    }

    std::size_t size() const noexcept {
        return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire);
    }
    bool empty() const noexcept { return size() == 0; }
    static constexpr std::size_t capacity() noexcept { return Capacity; }

private:
    alignas(kCacheLine) std::atomic<std::size_t> head_{0};
    alignas(kCacheLine) std::atomic<std::size_t> tail_{0};
    alignas(kCacheLine) std::array<T, Capacity> buffer_{};
};

/// Multi-producer / single-consumer ring. Producers (non-real-time threads)
/// serialize on a mutex; the consumer (the real-time task) is lock-free.
template <typename T, std::size_t Capacity>
class MpscRing {
public:
    bool tryPush(const T& item) {
        std::lock_guard<std::mutex> lock(producer_mutex_);
        return ring_.tryPush(item);
    }
    /// Pushes all items contiguously (no interleaving with other producers)
    /// or none of them.
    bool tryPushAll(const T* items, std::size_t count) {
        std::lock_guard<std::mutex> lock(producer_mutex_);
        if (ring_.size() + count > Capacity) {
            return false;
        }
        for (std::size_t i = 0; i < count; ++i) {
            (void)ring_.tryPush(items[i]);
        }
        return true;
    }
    bool tryPop(T& out) noexcept { return ring_.tryPop(out); }
    std::size_t size() const noexcept { return ring_.size(); }

private:
    std::mutex producer_mutex_;
    SpscRing<T, Capacity> ring_;
};

/// Single-writer / multi-reader snapshot (sequence lock). The writer never
/// waits; readers retry while a write is in progress. Payload words are
/// stored as relaxed atomics so concurrent access is free of data races.
template <typename T>
class SeqLock {
    static_assert(std::is_trivially_copyable_v<T>, "SeqLock requires trivially copyable payload");
    static constexpr std::size_t kWords = (sizeof(T) + sizeof(std::uint64_t) - 1) / sizeof(std::uint64_t);

public:
    SeqLock() { store(T{}); }

    /// Writer side (single thread, real-time safe).
    void store(const T& value) noexcept {
        std::array<std::uint64_t, kWords> raw{};
        std::memcpy(raw.data(), static_cast<const void*>(&value), sizeof(T));
        const std::uint64_t seq = sequence_.load(std::memory_order_relaxed);
        sequence_.store(seq + 1, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
        for (std::size_t i = 0; i < kWords; ++i) {
            words_[i].store(raw[i], std::memory_order_relaxed);
        }
        sequence_.store(seq + 2, std::memory_order_release);
    }

    /// Reader side (any thread).
    T load() const noexcept {
        std::array<std::uint64_t, kWords> raw{};
        for (;;) {
            const std::uint64_t before = sequence_.load(std::memory_order_acquire);
            if (before & 1U) {
                continue;
            }
            for (std::size_t i = 0; i < kWords; ++i) {
                raw[i] = words_[i].load(std::memory_order_relaxed);
            }
            std::atomic_thread_fence(std::memory_order_acquire);
            if (sequence_.load(std::memory_order_relaxed) == before) {
                break;
            }
        }
        T value{};
        std::memcpy(static_cast<void*>(&value), raw.data(), sizeof(T));
        return value;
    }

    std::uint64_t version() const noexcept { return sequence_.load(std::memory_order_acquire) / 2; }

private:
    std::atomic<std::uint64_t> sequence_{0};
    std::array<std::atomic<std::uint64_t>, kWords> words_{};
};

/// Fixed-capacity, NUL-terminated name usable inside trivially copyable structs.
template <std::size_t N>
struct FixedString {
    char data[N] = {};

    void assign(const char* text) noexcept {
        std::size_t i = 0;
        if (text != nullptr) {
            for (; i + 1 < N && text[i] != '\0'; ++i) {
                data[i] = text[i];
            }
        }
        data[i] = '\0';
    }
    const char* c_str() const noexcept { return data; }
};

}  // namespace ethercat_motion
