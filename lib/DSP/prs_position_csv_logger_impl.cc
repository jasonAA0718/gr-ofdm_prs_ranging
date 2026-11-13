// SPDX-License-Identifier: GPL-3.0-or-later
#include "prs_position_csv_logger_impl.h"
#include <gnuradio/io_signature.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace gr {
namespace ofdm_prs_ranging {
namespace {
constexpr const char* csv_header =
    "cycle_sequence,poll_id,reference_timestamp,success,reason,x_m,y_m,z_m,"
    "observed_anchor_count,used_anchor_count,anchor_ids,"
    "residual_rms_m,residual_max_abs_m";

std::string csv_text(const std::string& value)
{
    if (value.find_first_of(",\"\r\n") == std::string::npos)
        return value;
    std::string escaped = "\"";
    for (char ch : value) {
        if (ch == '"')
            escaped += '"';
        escaped += ch;
    }
    return escaped + '"';
}

void write_number(std::ostream& out, pmt::pmt_t value, bool integer)
{
    if (integer) {
        if (pmt::is_uint64(value))
            out << pmt::to_uint64(value);
        else if (pmt::is_integer(value) && pmt::to_long(value) >= 0)
            out << pmt::to_long(value);
    } else if (pmt::is_real(value) && std::isfinite(pmt::to_double(value))) {
        out << pmt::to_double(value);
    }
}
} // namespace

prs_position_csv_logger::sptr prs_position_csv_logger::make(const std::string& path,
                                                            bool append)
{
    return gnuradio::make_block_sptr<prs_position_csv_logger_impl>(path, append);
}

prs_position_csv_logger_impl::prs_position_csv_logger_impl(const std::string& path,
                                                           bool append)
    : gr::block("prs_position_csv_logger",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0))
{
    bool needs_header = true;
    if (append) {
        std::ifstream existing(path);
        if (existing && existing.peek() != std::ifstream::traits_type::eof()) {
            std::string header;
            std::getline(existing, header);
            if (!header.empty() && header.back() == '\r')
                header.pop_back();
            if (header != csv_header) {
                throw std::invalid_argument("Position CSV header mismatch: " + path);
            }
            needs_header = false;
        }
    }
    d_file.open(path, append ? std::ios::app : std::ios::trunc);
    if (!d_file.is_open())
        throw std::runtime_error("Cannot open position CSV: " + path);
    d_file.imbue(std::locale::classic());
    d_file << std::setprecision(17);
    if (needs_header) {
        d_file << csv_header << '\n';
        d_file.flush();
    }
    for (const auto* port : { "position_in", "diagnostic_in" }) {
        message_port_register_in(pmt::mp(port));
    }
    set_msg_handler(pmt::mp("position_in"),
                    [this](pmt::pmt_t msg) { handle_result(msg, true); });
    set_msg_handler(pmt::mp("diagnostic_in"),
                    [this](pmt::pmt_t msg) { handle_result(msg, false); });
}

void prs_position_csv_logger_impl::handle_result(pmt::pmt_t msg, bool success)
{
    if (!pmt::is_pair(msg) || !pmt::is_dict(pmt::car(msg)))
        return;
    const auto meta = pmt::car(msg);
    const auto field = [&](const char* key) {
        return pmt::dict_ref(meta, pmt::mp(key), pmt::PMT_NIL);
    };
    const auto position = field("position_m");
    if (success) {
        if (!pmt::is_f64vector(position) || pmt::length(position) != 3)
            return;
        for (size_t i = 0; i < 3; ++i) {
            if (!std::isfinite(pmt::f64vector_ref(position, i)))
                return;
        }
    }
    // The input port determines success; diagnostics never expose trial coordinates.
    const auto reason = field("reason");
    const std::string reason_text = pmt::is_symbol(reason)
                                        ? pmt::symbol_to_string(reason)
                                        : (success ? "OK" : "UNSPECIFIED_FAILURE");
    std::ostringstream row;
    row.imbue(std::locale::classic());
    row << std::setprecision(17);
    write_number(row, field("cycle_sequence"), true);
    row << ',';
    write_number(row, field("poll_id"), true);
    row << ',';
    write_number(row, field("reference_timestamp"), false);
    row << ',' << (success ? 1 : 0) << ',' << csv_text(reason_text);
    for (size_t i = 0; i < 3; ++i) {
        row << ',';
        if (success)
            row << pmt::f64vector_ref(position, i);
    }
    row << ',';
    write_number(row, field("observed_anchor_count"), true);
    row << ',';
    write_number(row, field("used_anchor_count"), true);
    row << ',';
    const auto ids = field("selected_anchor_ids");
    if (pmt::is_u32vector(ids)) {
        for (size_t i = 0; i < pmt::length(ids); ++i) {
            if (i)
                row << ';';
            row << pmt::u32vector_ref(ids, i);
        }
    }
    double norm = 0.0, maximum = 0.0;
    const auto residual = field("residual");
    bool residual_valid = pmt::is_f64vector(residual) && pmt::length(residual) > 0 &&
                          pmt::is_u32vector(ids) &&
                          pmt::length(residual) == pmt::length(ids);
    if (residual_valid) {
        for (size_t i = 0; i < pmt::length(residual); ++i) {
            const double value = pmt::f64vector_ref(residual, i);
            if (!std::isfinite(value)) {
                residual_valid = false;
                break;
            }
            norm = std::hypot(norm, value / std::sqrt(pmt::length(residual)));
            maximum = std::max(maximum, std::abs(value));
        }
    }
    row << ',';
    if (residual_valid)
        row << norm;
    row << ',';
    if (residual_valid)
        row << maximum;
    row << '\n';
    std::lock_guard<std::mutex> lock(d_mutex);
    d_file << row.str();
    d_file.flush();
    if (!d_file)
        GR_LOG_ERROR(d_logger, "Failed to write position CSV");
}

} // namespace ofdm_prs_ranging
} // namespace gr
