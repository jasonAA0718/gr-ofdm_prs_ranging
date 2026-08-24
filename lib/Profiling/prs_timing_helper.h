/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_TIMING_HELPER_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_TIMING_HELPER_H

#include <pmt/pmt.h>
#include <chrono>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace gr {
namespace ofdm_prs_ranging {
namespace profiling {

using timing_clock = std::chrono::steady_clock;
using timing_point = timing_clock::time_point;

inline timing_point now() { return timing_clock::now(); }

inline uint64_t to_ns(timing_point value)
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(value.time_since_epoch())
            .count());
}

inline uint64_t elapsed_ns(timing_point start, timing_point stop)
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());
}

class timing_report
{
public:
    timing_report(bool enabled, const char* block_name)
        : d_enabled(enabled),
          d_block_name(block_name),
          d_handler_start(enabled ? now() : timing_point()),
          d_last(d_handler_start)
    {
    }

    bool enabled() const { return d_enabled; }

    timing_point mark() const { return d_enabled ? now() : timing_point(); }

    uint64_t handler_elapsed_ns() const
    {
        return d_enabled ? elapsed_ns(d_handler_start, now()) : 0;
    }

    void add(const char* stage, timing_point start, timing_point stop)
    {
        if (!d_enabled) {
            return;
        }
        add(stage, elapsed_ns(start, stop));
    }

    void add(const char* stage, uint64_t duration_ns)
    {
        if (!d_enabled) {
            return;
        }
        for (auto& entry : d_stages) {
            if (entry.first == stage) {
                entry.second += duration_ns;
                return;
            }
        }
        d_stages.emplace_back(stage, duration_ns);
    }

    void checkpoint(const char* stage)
    {
        if (!d_enabled) {
            return;
        }
        const auto stop = now();
        add(stage, d_last, stop);
        d_last = stop;
    }

    pmt::pmt_t finish(const pmt::pmt_t& metadata)
    {
        if (!d_enabled) {
            return pmt::PMT_NIL;
        }
        const auto handler_stop = now();
        pmt::pmt_t result = pmt::make_dict();
        result = pmt::dict_add(result, pmt::mp("block"), pmt::mp(d_block_name));
        result = pmt::dict_add(result,
                               pmt::mp("handler_start_ns"),
                               pmt::from_uint64(to_ns(d_handler_start)));
        result = pmt::dict_add(
            result, pmt::mp("handler_end_ns"), pmt::from_uint64(to_ns(handler_stop)));
        result =
            pmt::dict_add(result,
                          pmt::mp("handler_duration_ns"),
                          pmt::from_uint64(elapsed_ns(d_handler_start, handler_stop)));
        pmt::pmt_t stages = pmt::make_dict();
        for (const auto& entry : d_stages) {
            stages = pmt::dict_add(
                stages, pmt::intern(entry.first), pmt::from_uint64(entry.second));
        }
        result = pmt::dict_add(result, pmt::mp("stages"), stages);

        static const char* keys[] = {
            "attempt_id", "poll_frame_id", "response_frame_id", "frame_id", "packet_type"
        };
        for (const auto* key_name : keys) {
            const auto key = pmt::mp(key_name);
            const auto value = pmt::dict_ref(metadata, key, pmt::PMT_NIL);
            if (!pmt::is_null(value)) {
                result = pmt::dict_add(result, key, value);
            }
        }
        const auto valid = pmt::dict_ref(metadata, pmt::mp("valid"), pmt::PMT_NIL);
        if (!pmt::is_null(valid)) {
            result = pmt::dict_add(result, pmt::mp("valid"), valid);
        } else {
            const auto frame_valid =
                pmt::dict_ref(metadata, pmt::mp("frame_id_valid"), pmt::PMT_NIL);
            if (!pmt::is_null(frame_valid)) {
                result = pmt::dict_add(result, pmt::mp("valid"), frame_valid);
            }
        }
        return result;
    }

private:
    bool d_enabled;
    std::string d_block_name;
    timing_point d_handler_start;
    timing_point d_last;
    std::vector<std::pair<std::string, uint64_t>> d_stages;
};

class accumulated_timer
{
public:
    accumulated_timer(bool enabled, uint64_t& destination)
        : d_enabled(enabled),
          d_destination(destination),
          d_start(enabled ? now() : timing_point())
    {
    }

    ~accumulated_timer()
    {
        if (d_enabled) {
            d_destination += elapsed_ns(d_start, now());
        }
    }

private:
    bool d_enabled;
    uint64_t& d_destination;
    timing_point d_start;
};

} // namespace profiling
} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_TIMING_HELPER_H */
