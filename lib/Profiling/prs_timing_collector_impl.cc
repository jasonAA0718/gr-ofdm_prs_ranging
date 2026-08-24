/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "prs_timing_collector_impl.h"
#include <gnuradio/io_signature.h>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <map>
#include <numeric>
#include <utility>

namespace gr {
namespace ofdm_prs_ranging {

namespace {

uint64_t dict_uint64(const pmt::pmt_t& dict, const char* key, uint64_t fallback = 0)
{
    const auto value = pmt::dict_ref(dict, pmt::mp(key), pmt::PMT_NIL);
    if (pmt::is_uint64(value)) {
        return pmt::to_uint64(value);
    }
    if (pmt::is_integer(value)) {
        return static_cast<uint64_t>(pmt::to_long(value));
    }
    return fallback;
}

std::string symbol_string(const pmt::pmt_t& value, const char* fallback)
{
    return pmt::is_symbol(value) ? pmt::symbol_to_string(value) : fallback;
}

double percentile(const std::vector<uint64_t>& sorted, double fraction)
{
    if (sorted.empty()) {
        return 0.0;
    }
    const double index = fraction * static_cast<double>(sorted.size() - 1);
    const auto lower = static_cast<size_t>(index);
    const auto upper = std::min(lower + 1, sorted.size() - 1);
    const double weight = index - static_cast<double>(lower);
    return static_cast<double>(sorted[lower]) * (1.0 - weight) +
           static_cast<double>(sorted[upper]) * weight;
}

} // namespace

prs_timing_collector::sptr prs_timing_collector::make(const std::string& raw_csv_path,
                                                      const std::string& summary_csv_path,
                                                      const std::string& role)
{
    return gnuradio::make_block_sptr<prs_timing_collector_impl>(
        raw_csv_path, summary_csv_path, role);
}

prs_timing_collector_impl::prs_timing_collector_impl(std::string raw_csv_path,
                                                     std::string summary_csv_path,
                                                     std::string role)
    : gr::block("prs_timing_collector",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_raw_csv_path(std::move(raw_csv_path)),
      d_summary_csv_path(std::move(summary_csv_path)),
      d_role(std::move(role))
{
    message_port_register_in(pmt::mp("timing_in"));
    set_msg_handler(pmt::mp("timing_in"),
                    [this](const pmt::pmt_t& message) { handle_timing(message); });
}

prs_timing_collector_impl::~prs_timing_collector_impl() { finalize(); }

bool prs_timing_collector_impl::stop()
{
    finalize();
    return gr::block::stop();
}

void prs_timing_collector_impl::handle_timing(const pmt::pmt_t& message)
{
    if (!pmt::is_dict(message)) {
        return;
    }
    const auto block_value = pmt::dict_ref(message, pmt::mp("block"), pmt::PMT_NIL);
    const std::string block = symbol_string(block_value, "unknown");
    const uint64_t handler_start_ns = dict_uint64(message, "handler_start_ns");
    const uint64_t handler_end_ns = dict_uint64(message, "handler_end_ns");
    const uint64_t handler_duration_ns = dict_uint64(message, "handler_duration_ns");
    const uint64_t attempt_id = dict_uint64(message, "attempt_id");
    const uint64_t poll_frame_id = dict_uint64(message, "poll_frame_id");
    const uint64_t response_frame_id = dict_uint64(message, "response_frame_id");
    const uint64_t frame_id = dict_uint64(message, "frame_id");
    const uint64_t packet_type = dict_uint64(message, "packet_type");
    const bool valid = pmt::to_bool(pmt::dict_ref(message, pmt::mp("valid"), pmt::PMT_F));

    std::vector<timing_row> rows;
    rows.push_back({ block,
                     "handler_total",
                     attempt_id,
                     poll_frame_id,
                     response_frame_id,
                     frame_id,
                     packet_type,
                     valid,
                     handler_start_ns,
                     handler_end_ns,
                     handler_duration_ns });

    const auto stages = pmt::dict_ref(message, pmt::mp("stages"), pmt::PMT_NIL);
    if (pmt::is_dict(stages)) {
        auto keys = pmt::dict_keys(stages);
        while (pmt::is_pair(keys)) {
            const auto key = pmt::car(keys);
            const auto value = pmt::dict_ref(stages, key, pmt::from_uint64(0));
            rows.push_back({ block,
                             symbol_string(key, "unknown"),
                             attempt_id,
                             poll_frame_id,
                             response_frame_id,
                             frame_id,
                             packet_type,
                             valid,
                             handler_start_ns,
                             handler_end_ns,
                             pmt::is_uint64(value) ? pmt::to_uint64(value) : 0 });
            keys = pmt::cdr(keys);
        }
    }

    std::lock_guard<std::mutex> lock(d_mutex);
    d_rows.insert(d_rows.end(), rows.begin(), rows.end());
}

void prs_timing_collector_impl::finalize()
{
    std::lock_guard<std::mutex> lock(d_mutex);

    if (!d_raw_csv_path.empty()) {
        std::ofstream raw(d_raw_csv_path, std::ios::trunc);
        raw << "role,block,stage,attempt_id,poll_frame_id,response_frame_id,frame_id,"
               "packet_type,valid,handler_start_ns,handler_end_ns,duration_ns,duration_"
               "us\n";
        raw << std::setprecision(15);
        for (const auto& row : d_rows) {
            raw << d_role << ',' << row.block << ',' << row.stage << ',' << row.attempt_id
                << ',' << row.poll_frame_id << ',' << row.response_frame_id << ','
                << row.frame_id << ',' << row.packet_type << ',' << (row.valid ? 1 : 0)
                << ',' << row.handler_start_ns << ',' << row.handler_end_ns << ','
                << row.duration_ns << ',' << static_cast<double>(row.duration_ns) / 1000.0
                << '\n';
        }
    }

    if (!d_summary_csv_path.empty()) {
        std::map<std::pair<std::string, std::string>, std::vector<uint64_t>> groups;
        for (const auto& row : d_rows) {
            groups[{ row.block, row.stage }].push_back(row.duration_ns);
        }

        std::ofstream summary(d_summary_csv_path, std::ios::trunc);
        summary << "role,block,stage,count,mean_us,median_us,p95_us,p99_us,max_us\n";
        summary << std::setprecision(15);
        for (auto& entry : groups) {
            auto& values = entry.second;
            std::sort(values.begin(), values.end());
            const long double total = std::accumulate(
                values.begin(), values.end(), static_cast<long double>(0));
            const double mean_ns = static_cast<double>(total / values.size());
            summary << d_role << ',' << entry.first.first << ',' << entry.first.second
                    << ',' << values.size() << ',' << mean_ns / 1000.0 << ','
                    << percentile(values, 0.50) / 1000.0 << ','
                    << percentile(values, 0.95) / 1000.0 << ','
                    << percentile(values, 0.99) / 1000.0 << ','
                    << static_cast<double>(values.back()) / 1000.0 << '\n';
        }
    }
}

} // namespace ofdm_prs_ranging
} // namespace gr
