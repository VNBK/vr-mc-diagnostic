/*
 * timer.h — minimal WaitTimer helper used by the SkillHand CAN driver.
 *
 * Ported in-tree so the SkillHand sources can build without pulling the
 * gateway workspace's full timing utilities.
 */

#pragma once

#include <chrono>
#include <cstdint>

class WaitTimer
{
public:
    explicit WaitTimer(uint32_t _timeout_ms)
        : m_deadline(std::chrono::steady_clock::now() +
                     std::chrono::milliseconds(_timeout_ms)) {}

    /** Remaining time in milliseconds; 0 when the deadline has passed. */
    uint32_t getRemainTime() const
    {
        auto now = std::chrono::steady_clock::now();
        if (now >= m_deadline){
            return 0;
        }
        auto remain = std::chrono::duration_cast<std::chrono::milliseconds>(
                          m_deadline - now).count();
        return static_cast<uint32_t>(remain);
    }

    bool expired() const { return getRemainTime() == 0; }

private:
    std::chrono::steady_clock::time_point m_deadline;
};
