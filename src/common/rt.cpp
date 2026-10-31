// SPDX-License-Identifier: MIT
#include "ethercat_motion/rt.hpp"

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/utsname.h>
#include <time.h>

#include <cerrno>
#include <cstring>
#include <fstream>
#include <string>

namespace ethercat_motion {

namespace {

constexpr std::int64_t kNsPerSec = 1000000000LL;

void prefaultStack(std::size_t bytes) {
    if (bytes == 0) {
        return;
    }
    // Touch the stack so page faults happen now, not inside the cycle.
    // volatile prevents the compiler from optimizing the buffer away.
    constexpr std::size_t kChunk = 4096;
    volatile unsigned char buffer[kChunk];
    for (std::size_t i = 0; i < kChunk; ++i) {
        buffer[i] = 0;
    }
    if (bytes > kChunk) {
        prefaultStack(bytes - kChunk);
    }
    (void)buffer[0];
}

}  // namespace

RtSetupReport applyRealtimeToCurrentThread(const RtThreadConfig& config) {
    RtSetupReport report;

    if (config.lock_memory) {
        if (mlockall(MCL_CURRENT | MCL_FUTURE) == 0) {
            report.memory_locked = true;
        } else {
            report.warnings += std::string("mlockall failed: ") + std::strerror(errno) +
                               " (grant CAP_IPC_LOCK or raise RLIMIT_MEMLOCK); ";
        }
    }

    if (config.cpu_affinity >= 0) {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET(static_cast<unsigned>(config.cpu_affinity), &set);
        const int rc = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
        if (rc == 0) {
            report.affinity = true;
        } else {
            report.warnings += std::string("CPU affinity failed: ") + std::strerror(rc) + "; ";
        }
    }

    if (config.enabled) {
        sched_param param{};
        param.sched_priority = config.priority;
        const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
        if (rc == 0) {
            report.sched_fifo = true;
        } else {
            report.warnings += std::string("SCHED_FIFO failed: ") + std::strerror(rc) +
                               " (grant CAP_SYS_NICE or configure rtprio in limits.conf); ";
        }
    }

    prefaultStack(config.stack_prefault_bytes);
    return report;
}

std::int64_t monotonicNs() noexcept {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<std::int64_t>(ts.tv_sec) * kNsPerSec + ts.tv_nsec;
}

void sleepUntilNs(std::int64_t deadline_ns) noexcept {
    timespec ts{};
    ts.tv_sec = static_cast<time_t>(deadline_ns / kNsPerSec);
    ts.tv_nsec = static_cast<long>(deadline_ns % kNsPerSec);
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {
    }
}

bool kernelIsPreemptRt() {
    std::ifstream realtime("/sys/kernel/realtime");
    int flag = 0;
    if (realtime >> flag && flag == 1) {
        return true;
    }
    utsname name{};
    if (uname(&name) == 0) {
        const std::string version = name.version;
        return version.find("PREEMPT_RT") != std::string::npos || version.find("PREEMPT RT") != std::string::npos;
    }
    return false;
}

}  // namespace ethercat_motion
