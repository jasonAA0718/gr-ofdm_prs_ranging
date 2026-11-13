// SPDX-License-Identifier: GPL-3.0-or-later
#include "prs_position_solver_impl.h"
#include <gnuradio/io_signature.h>
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace gr {
namespace ofdm_prs_ranging {
namespace {
struct range_observation {
    std::array<double, 3> anchor_pos;
    double range;
};
} // namespace

prs_position_solver::sptr
prs_position_solver::make(const std::vector<int>& anchor_ids,
                          const std::vector<std::vector<double>>& anchor_positions_m,
                          double vehicle_height_m)
{
    return gnuradio::make_block_sptr<prs_position_solver_impl>(
        anchor_ids, anchor_positions_m, vehicle_height_m);
}

prs_position_solver_impl::prs_position_solver_impl(
    const std::vector<int>& anchor_ids,
    const std::vector<std::vector<double>>& anchor_positions_m,
    double vehicle_height_m)
    : gr::block("prs_position_solver",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_anchor_ids(anchor_ids),
      d_anchor_positions_m(anchor_positions_m),
      d_vehicle_height_m(vehicle_height_m),
      d_position{ NAN, NAN, NAN }
{
    if (anchor_ids.empty() || anchor_ids.size() > 8 || !std::isfinite(vehicle_height_m)) {
        throw std::invalid_argument("Invalid anchor count or vehicle height");
    }
    std::set<int> unique_ids;
    for (int id : anchor_ids) {
        if (id < 0 || id > 7 || !unique_ids.insert(id).second ||
            static_cast<size_t>(id) >= anchor_positions_m.size() ||
            anchor_positions_m[id].size() != 3) {
            throw std::invalid_argument("Invalid anchor ID or XYZ coordinate layout");
        }
        for (double coordinate : anchor_positions_m[id]) {
            if (!std::isfinite(coordinate)) {
                throw std::invalid_argument("Invalid XYZ coordinate");
            }
        }
    }
    message_port_register_in(pmt::mp("epoch_in"));
    message_port_register_out(pmt::mp("position_out"));
    message_port_register_out(pmt::mp("diagnostic_out"));
    set_msg_handler(pmt::mp("epoch_in"), [this](pmt::pmt_t msg) { handle_epoch(msg); });
}

void prs_position_solver_impl::handle_epoch(pmt::pmt_t msg)
{
    const auto meta = pmt::is_pair(msg) && pmt::is_dict(pmt::car(msg)) ? pmt::car(msg)
                                                                       : pmt::make_dict();
    auto result = pmt::make_dict();
    for (const char* name : { "poll_id",
                              "cycle_sequence",
                              "reference_timestamp",
                              "cycle_completion_timestamp",
                              "summary_emission_timestamp" }) {
        const auto key = pmt::mp(name);
        if (pmt::dict_has_key(meta, key)) {
            result = pmt::dict_add(result, key, pmt::dict_ref(meta, key, pmt::PMT_NIL));
        }
    }
    const auto ids = pmt::dict_ref(meta, pmt::mp("observed_anchor_ids"), pmt::PMT_NIL);
    const size_t observed_count = pmt::is_u32vector(ids) ? pmt::length(ids) : 0;
    std::vector<range_observation> observations;
    std::vector<uint32_t> selected_ids;
    std::vector<double> selected_ranges, residuals;
    std::vector<uint8_t> fine_flags;

    // Exactly one result for each input, including malformed or aborted cycles.
    const auto publish = [&](bool success, const char* reason) {
        result = pmt::dict_add(result, pmt::mp("residual"), pmt::init_f64vector(residuals.size(), residuals));
        result = pmt::dict_add(result, pmt::mp("observed_anchor_count"), pmt::from_uint64(observed_count));
        result = pmt::dict_add(result, pmt::mp("used_anchor_count"), pmt::from_uint64(selected_ids.size()));
        result = pmt::dict_add(result, pmt::mp("used_fine_correction_flags"), pmt::init_u8vector(fine_flags.size(), fine_flags));
        result = pmt::dict_add(result, pmt::mp("selected_ranges_m"), pmt::init_f64vector(selected_ranges.size(), selected_ranges));
        result = pmt::dict_add(result, pmt::mp("selected_anchor_ids"), pmt::init_u32vector(selected_ids.size(), selected_ids));
        result = pmt::dict_add(result, pmt::mp("reason"), pmt::mp(reason));
        result = pmt::dict_add(result, pmt::mp("success"), pmt::from_bool(success));
        message_port_pub(pmt::mp(success ? "position_out" : "diagnostic_out"),
                         pmt::cons(result, pmt::PMT_NIL));
    };


    if (!pmt::is_pair(msg) || !pmt::is_dict(pmt::car(msg))) {
        publish(false, "INVALID_INPUT");
        return;
    }
    const auto completion =
        pmt::dict_ref(meta, pmt::mp("completion_reason"), pmt::PMT_NIL);
    if (pmt::is_symbol(completion) &&
        pmt::symbol_to_string(completion) == "STOPPED_INCOMPLETE") {
        publish(false, "CYCLE_ABORTED");
        return;
    }
    const auto integers = pmt::dict_ref(meta, pmt::mp("integer_ranges_m"), pmt::PMT_NIL);
    const auto corrected =
        pmt::dict_ref(meta, pmt::mp("corrected_ranges_m"), pmt::PMT_NIL);
    const auto flags =
        pmt::dict_ref(meta, pmt::mp("fine_correction_valid_flags"), pmt::PMT_NIL);
    if (!pmt::is_u32vector(ids) || !pmt::is_f64vector(integers) ||
        !pmt::is_f64vector(corrected) || !pmt::is_u8vector(flags) || observed_count > 8 ||
        pmt::length(integers) != observed_count ||
        pmt::length(corrected) != observed_count ||
        pmt::length(flags) != observed_count) {
        publish(false, "INVALID_OBSERVATION_LAYOUT");
        return;
    }
    std::set<uint32_t> seen;
    for (size_t i = 0; i < observed_count; ++i) {
        const uint32_t id = pmt::u32vector_ref(ids, i);
        if (!seen.insert(id).second ||
            std::find(d_anchor_ids.begin(), d_anchor_ids.end(), id) ==
                d_anchor_ids.end()) {
            publish(false, "INVALID_ANCHOR_ID");
            return;
        }
        const double fine_range = pmt::f64vector_ref(corrected, i);
        const bool use_fine =
            pmt::u8vector_ref(flags, i) == 1 && std::isfinite(fine_range);
        const double range = use_fine ? fine_range : pmt::f64vector_ref(integers, i);
        if (!std::isfinite(range) || range <= 0.0) {
            continue;
        }
        const auto& pos = d_anchor_positions_m[id];
        observations.push_back({ { pos[0], pos[1], pos[2] }, range });
        selected_ids.push_back(id);
        selected_ranges.push_back(range);
        fine_flags.push_back(use_fine ? 1 : 0);
    }
    if (observations.size() < 3) {
        publish(false, "INSUFFICIENT_OBSERVATIONS");
        return;
    }

    // Fixed-height 2D least squares. Preserve the existing uniform weights,
    // 40-iteration limit and 1 cm step convergence threshold.
    double x = 0.0, y = 0.0;
    for (const auto& observation : observations) {
        x += observation.anchor_pos[0] / observations.size();
        y += observation.anchor_pos[1] / observations.size();
    }
    if (std::isfinite(d_position[0]) && std::isfinite(d_position[1])) {
        x = d_position[0];
        y = d_position[1];
    }
    bool converged = false;
    for (int iteration = 0; iteration < 40; ++iteration) {
        double axx = 0.0, axy = 0.0, ayy = 0.0, bx = 0.0, by = 0.0;
        residuals.clear();
        for (const auto& observation : observations) {
            const double dx = x - observation.anchor_pos[0];
            const double dy = y - observation.anchor_pos[1];
            const double dz = d_vehicle_height_m - observation.anchor_pos[2];
            const double predicted = std::hypot(dx, dy, dz);
            if (!std::isfinite(predicted) || predicted <= 1e-12) {
                publish(false, "INVALID_PREDICTED_RANGE");
                return;
            }
            const double residual = observation.range - predicted;
            const double hx = dx / predicted, hy = dy / predicted;
            axx += hx * hx;
            axy += hx * hy;
            ayy += hy * hy;
            bx += hx * residual;
            by += hy * residual;
            residuals.push_back(residual);
        }
        const double det = axx * ayy - axy * axy;
        if (!std::isfinite(det) || det <= 1e-12) {
            publish(false, "SINGULAR_GEOMETRY");
            return;
        }
        const double delta_x = (ayy * bx - axy * by) / det;
        const double delta_y = (axx * by - axy * bx) / det;
        if (!std::isfinite(delta_x) || !std::isfinite(delta_y) ||
            !std::isfinite(x + delta_x) || !std::isfinite(y + delta_y)) {
            publish(false, "NONFINITE_UPDATE");
            return;
        }
        x += delta_x;
        y += delta_y;
        if (std::hypot(delta_x, delta_y) < 1e-2) {
            converged = true;
            break;
        }
    }
    residuals.clear();
    for (const auto& observation : observations) {
        const double predicted =
            std::hypot(x - observation.anchor_pos[0],
                       y - observation.anchor_pos[1],
                       d_vehicle_height_m - observation.anchor_pos[2]);
        residuals.push_back(observation.range - predicted);
    }
    if (!converged) {
        publish(false, "NOT_CONVERGED");
        return;
    }
    // Failed attempts never replace the last successful initial guess.
    d_position = { x, y, d_vehicle_height_m };
    result = pmt::dict_add(
        result, pmt::mp("position_m"), pmt::init_f64vector(3, d_position.data()));
    publish(true, "OK");
}

} // namespace ofdm_prs_ranging
} // namespace gr
