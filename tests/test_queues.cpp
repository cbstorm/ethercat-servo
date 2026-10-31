// SPDX-License-Identifier: MIT
#include <atomic>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "ethercat_motion/lockfree.hpp"

using namespace ethercat_motion;

TEST(Queues, SpscBoundedFifo) {
    SpscRing<int, 4> ring;
    EXPECT_TRUE(ring.empty());
    for (int i = 0; i < 4; ++i) EXPECT_TRUE(ring.tryPush(i));
    EXPECT_FALSE(ring.tryPush(99));  // bounded: never blocks, never grows
    int v = -1;
    for (int i = 0; i < 4; ++i) {
        ASSERT_TRUE(ring.tryPop(v));
        EXPECT_EQ(v, i);
    }
    EXPECT_FALSE(ring.tryPop(v));
}

TEST(Queues, SpscConcurrentOrder) {
    SpscRing<std::uint64_t, 256> ring;
    constexpr std::uint64_t kCount = 200000;
    std::thread producer([&] {
        for (std::uint64_t i = 0; i < kCount;) {
            if (ring.tryPush(i)) ++i;
        }
    });
    std::uint64_t expected = 0;
    while (expected < kCount) {
        std::uint64_t v = 0;
        if (ring.tryPop(v)) {
            ASSERT_EQ(v, expected);
            ++expected;
        }
    }
    producer.join();
}

TEST(Queues, MpscManyProducers) {
    MpscRing<std::uint64_t, 1024> ring;
    constexpr int kProducers = 4;
    constexpr std::uint64_t kPerProducer = 20000;
    std::vector<std::thread> producers;
    for (int p = 0; p < kProducers; ++p) {
        producers.emplace_back([&, p] {
            for (std::uint64_t i = 0; i < kPerProducer;) {
                if (ring.tryPush((static_cast<std::uint64_t>(p) << 32) | i)) ++i;
            }
        });
    }
    std::vector<std::uint64_t> next(kProducers, 0);
    std::uint64_t received = 0;
    while (received < kProducers * kPerProducer) {
        std::uint64_t v = 0;
        if (ring.tryPop(v)) {
            const auto p = static_cast<std::size_t>(v >> 32);
            ASSERT_EQ(v & 0xFFFFFFFF, next[p]);  // per-producer FIFO order
            ++next[p];
            ++received;
        }
    }
    for (auto& t : producers) t.join();
}

namespace {
struct Pair {
    std::uint64_t a = 0;
    std::uint64_t b = 0;
    double c = 0.0;
};
}  // namespace

TEST(Queues, SeqLockNeverTears) {
    SeqLock<Pair> lock;
    std::atomic<bool> stop{false};
    std::thread writer([&] {
        for (std::uint64_t i = 1; !stop.load(); ++i) {
            lock.store(Pair{i, i * 2, static_cast<double>(i)});
        }
    });
    for (int i = 0; i < 200000; ++i) {
        const Pair p = lock.load();
        ASSERT_EQ(p.b, p.a * 2);
        ASSERT_EQ(p.c, static_cast<double>(p.a));
    }
    stop = true;
    writer.join();
}

TEST(Queues, FixedStringTruncates) {
    FixedString<4> s;
    s.assign("abcdef");
    EXPECT_STREQ(s.c_str(), "abc");
    s.assign(nullptr);
    EXPECT_STREQ(s.c_str(), "");
}
