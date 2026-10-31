// SPDX-License-Identifier: MIT
//
// Linux real-time helpers: SCHED_FIFO, CPU affinity, memory locking and
// absolute-time periodic sleeping on CLOCK_MONOTONIC.
//
// These helpers make the cyclic task PREEMPT_RT compatible. They do not by
// themselves provide hard real-time guarantees: that requires a PREEMPT_RT
// kernel, CPU isolation, IRQ affinity tuning and a dedicated NIC.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace ethercat_motion {

struct RtThreadConfig {
    bool enabled = true;          ///< Request SCHED_FIFO for the cyclic task.
    int priority = 80;            ///< SCHED_FIFO priority (1..99).
    int cpu_affinity = -1;        ///< CPU index to pin the cyclic task to; -1 = no pinning.
    bool lock_memory = true;      ///< mlockall(MCL_CURRENT | MCL_FUTURE).
    std::size_t stack_prefault_bytes = 64 * 1024;
};

/// What could actually be applied. Missing privileges are reported, not fatal:
/// the task still runs, but timing is best-effort.
struct RtSetupReport {
    bool sched_fifo = false;
    bool affinity = false;
    bool memory_locked = false;
    std::string warnings;
};

/// Applies scheduling/affinity to the calling thread and optionally locks
/// process memory. Call from inside the cyclic thread before the loop.
RtSetupReport applyRealtimeToCurrentThread(const RtThreadConfig& config);

/// Current CLOCK_MONOTONIC time in nanoseconds.
std::int64_t monotonicNs() noexcept;

/// Sleeps until an absolute CLOCK_MONOTONIC time (restarts on EINTR).
void sleepUntilNs(std::int64_t deadline_ns) noexcept;

/// True when the running kernel reports PREEMPT_RT (via /sys/kernel/realtime
/// or uname). Informational only.
bool kernelIsPreemptRt();

}  // namespace ethercat_motion
