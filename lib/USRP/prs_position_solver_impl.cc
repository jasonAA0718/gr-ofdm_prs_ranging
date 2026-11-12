#include "prs_position_solver_impl.h"
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
namespace gr {
namespace ofdm_prs_ranging {


struct range_observation {
    uint32_t anchor_id;
    std::array<double, 3> anchor_pos;
    double range;
    double weight;
    bool used_fine_correction;
};





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
    d_vehicle_height_m(vehicle_height_m)
{
    if (anchor_ids.empty() || anchor_ids.size() > 8)
        throw std::invalid_argument("Invalid anchor count");

    if (!std::isfinite(vehicle_height_m))
        throw std::invalid_argument("Invalid vehicle height");

    for (auto id : anchor_ids) {

        if (static_cast<size_t>(id) >= anchor_positions_m.size())
            throw std::invalid_argument("Anchor position not found");

        if (anchor_positions_m[id].size() != 3)
            throw std::invalid_argument("XYZ must have 3 elements");

        for (double xyz : anchor_positions_m[id]) {
            if (!std::isfinite(xyz))
                throw std::invalid_argument("Invalid XYZ coordinate");
        }
    }
    d_position = { NAN, NAN, NAN };
    message_port_register_in(pmt::mp("epoch_in"));
    message_port_register_out(pmt::mp("position_out"));
    message_port_register_out(pmt::mp("diagnostic_out"));
    set_msg_handler(pmt::mp("epoch_in"), [this](pmt::pmt_t msg) { handle_epoch(msg); });
}

void prs_position_solver_impl::handle_epoch(pmt::pmt_t msg)
{
    // Existing Cycle Manager publishes (dictionary, NIL).
    if (!pmt::is_pair(msg)) {
        return;
    }

    const auto meta = pmt::car(msg);

    if (!pmt::is_dict(meta)) {
        return;
    }

    const auto pmt_ids =
        pmt::dict_ref(meta, pmt::mp("observed_anchor_ids"), pmt::PMT_NIL);

    const auto pmt_integer_ranges =
        pmt::dict_ref(meta, pmt::mp("integer_ranges_m"), pmt::PMT_NIL);

    const auto pmt_corrected_ranges =
        pmt::dict_ref(meta, pmt::mp("corrected_ranges_m"), pmt::PMT_NIL);

    const auto pmt_fine_flags =
        pmt::dict_ref(meta, pmt::mp("fine_correction_valid_flags"), pmt::PMT_NIL);

    if (!pmt::is_u32vector(pmt_ids) || !pmt::is_f64vector(pmt_integer_ranges) ||
        !pmt::is_f64vector(pmt_corrected_ranges) || !pmt::is_u8vector(pmt_fine_flags)) {
        return;
    }

    // Not necessary but for first positioning version
    const size_t N = pmt::length(pmt_ids);
    // Check the length of measurement is reasonable.
    if (pmt::length(pmt_integer_ranges) != N || pmt::length(pmt_corrected_ranges) != N ||
        pmt::length(pmt_fine_flags) != N) {
        return;
    }

    std::vector<range_observation> obs;
    for (size_t i = 0; i < N; i++) {
        double range;
        // Covert PMT data to C++ data type
        uint32_t anchor_id = pmt::u32vector_ref(pmt_ids, i);
        double integer_range = pmt::f64vector_ref(pmt_integer_ranges, i);
        double corrected_range = pmt::f64vector_ref(pmt_corrected_ranges, i);
        uint8_t fine_flag = pmt::u8vector_ref(pmt_fine_flags, i);
        range_observation valid_obs;
        // Check the valid range
        const bool use_fine = fine_flag == 1 && std::isfinite(corrected_range);
        range = use_fine ? corrected_range : integer_range;
        if (!std::isfinite(range) || range <= 0.0) {
            continue;
        }
        // Call the anchor pos based on anchor id
        if (anchor_id >= d_anchor_positions_m.size()) {
            return;
        }
        const auto& position = d_anchor_positions_m[anchor_id];

        // Store the target range measurement
        valid_obs.anchor_id = anchor_id;
        valid_obs.anchor_pos = { position[0], position[1], position[2] };
        valid_obs.used_fine_correction = use_fine;
        valid_obs.range = range;
        valid_obs.weight = 1.0;
        obs.push_back(valid_obs);
    }

    std::vector<double> selected_ranges;
    std::vector<double> residuals;
    std::vector<uint32_t> selected_anchor_ids;
    std::vector<uint8_t> selected_fine_flags;

    std::array<double, 3> initial_guess = { 0, 0, 0 };
    for (const auto& measurement : obs) {
        // Get the measurement info
        const uint32_t anchor_id = measurement.anchor_id;
        const double range = measurement.range;
        const double weight = measurement.weight;
        // Initial guess
        initial_guess[0] += measurement.anchor_pos[0];
        initial_guess[1] += measurement.anchor_pos[1];
        initial_guess[2] += measurement.anchor_pos[2];
        // Log measurement info
        selected_ranges.push_back(range);
        selected_anchor_ids.push_back(anchor_id);
        selected_fine_flags.push_back(measurement.used_fine_correction ? 1 : 0);
    }

    if (obs.size() < 3) {
        // Insufficient Observations may add fail diagnostic_out but later.
        auto result = pmt::make_dict();
        result = pmt::dict_add(result,pmt::mp("selected_ranges_m"),pmt::init_f64vector(selected_ranges.size(), selected_ranges));
        result = pmt::dict_add(result,pmt::mp("selected_anchor_ids"),pmt::init_u32vector(selected_anchor_ids.size(), selected_anchor_ids));
        result = pmt::dict_add(result,pmt::mp("poll_id"),pmt::dict_ref(meta, pmt::mp("poll_id"), pmt::PMT_NIL));
        result = pmt::dict_add(result, pmt::mp("completion_reason"), pmt::mp("The anchor less than 3"));
        message_port_pub(pmt::mp("diagnostic_out"), pmt::cons(result, pmt::PMT_NIL));
        return;
    }
    for (size_t i = 0; i < 3; i++)
        initial_guess[i] = initial_guess[i] / selected_anchor_ids.size();
    // The 2D position constrain remove in 3D implementation.
    initial_guess[2] = d_vehicle_height_m;

    // WLS
    const bool has_previous_position = std::isfinite(d_position[0]) && std::isfinite(d_position[1]);
    constexpr int max_iterations = 40;
    constexpr double convergence_tolerance = 1e-2;
    bool converged = 0;
    double x = (has_previous_position) ? d_position[0] : initial_guess[0];
    double y = (has_previous_position) ? d_position[1] : initial_guess[1];
    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        // residuals clean
        residuals.clear();
        // A = H' * W * H, B = H'* W * residual
        double Axx = 0.0;
        double Axy = 0.0;
        double Ayy = 0.0;
        double bx = 0.0;
        double by = 0.0;
        for (const auto& measurement : obs) {
            // Calculate the residual
            const double dx = x - measurement.anchor_pos[0];
            const double dy = y - measurement.anchor_pos[1];
            const double dz = d_vehicle_height_m - measurement.anchor_pos[2];
            const double predicted_range = std::sqrt(dx * dx + dy * dy + dz * dz);
            const double residual = measurement.range - predicted_range;
            // Jacobian
            if (!std::isfinite(predicted_range) ||
                predicted_range <= 1e-12) {
                auto result = pmt::make_dict();
                result = pmt::dict_add(result,
                    pmt::mp("selected_ranges_m"),
                    pmt::init_f64vector(selected_ranges.size(), selected_ranges));
                result = pmt::dict_add(result,
                    pmt::mp("poll_id"),
                    pmt::dict_ref(meta, pmt::mp("poll_id"), pmt::PMT_NIL));
                result = pmt::dict_add(result, pmt::mp("completion_reason"), pmt::mp("predicted_range is not isfinite or <=1e-12 "));
                message_port_pub(pmt::mp("diagnostic_out"), pmt::cons(result, pmt::PMT_NIL));
                return;
            }
            double hx = dx / predicted_range;
            double hy = dy / predicted_range;
            double w = measurement.weight;
            Axx += hx * hx * w;
            Axy += hx * hy * w;
            Ayy += hy * hy * w;

            bx += w * hx * residual;
            by += w * hy * residual;
            // Log residual
            residuals.push_back(residual);
        }
        const double det = Axx * Ayy - Axy * Axy;

        // Singular or nearly singular geometry
        if (!std::isfinite(det) || det <= 1e-12) {
                auto result = pmt::make_dict();
                result = pmt::dict_add(result,
                                        pmt::mp("residual"),
                                        pmt::init_f64vector(residuals.size(), residuals));
                result = pmt::dict_add(result,
                                        pmt::mp("used_fine_correction_flags"),
                                        pmt::init_u8vector(selected_fine_flags.size(), selected_fine_flags));
                result = pmt::dict_add(result,
                                        pmt::mp("selected_ranges_m"),
                                        pmt::init_f64vector(selected_ranges.size(), selected_ranges));
                result = pmt::dict_add(result, 
                                        pmt::mp("positioning_anchor_count"), 
                                        pmt::from_uint64(obs.size()));
                result = pmt::dict_add(result,
                                        pmt::mp("selected_anchor_ids"),
                                        pmt::init_u32vector(selected_anchor_ids.size(), selected_anchor_ids));
                result = pmt::dict_add(result,
                                        pmt::mp("poll_id"),
                                        pmt::dict_ref(meta, pmt::mp("poll_id"), pmt::PMT_NIL));
                result = pmt::dict_add(result, pmt::mp("completion_reason"), pmt::mp("singular geometry"));
                message_port_pub(pmt::mp("diagnostic_out"), pmt::cons(result, pmt::PMT_NIL));
            return;
        }
        const double delta_x = (Ayy * bx - Axy * by) / det;
        const double delta_y = (Axx * by - Axy * bx) / det;
        if (!std::isfinite(delta_x) || !std::isfinite(delta_y)) {
            auto result = pmt::make_dict();
            result = pmt::dict_add(result,
                                    pmt::mp("det"),
                                    pmt::from_double(det));
            result = pmt::dict_add(result,
                                    pmt::mp("residual"),
                                    pmt::init_f64vector(residuals.size(), residuals));
            result = pmt::dict_add(result,
                                    pmt::mp("selected_ranges_m"),
                                    pmt::init_f64vector(selected_ranges.size(), selected_ranges));
            result = pmt::dict_add(result,
                                    pmt::mp("selected_anchor_ids"),
                                    pmt::init_u32vector(selected_anchor_ids.size(), selected_anchor_ids));
            result = pmt::dict_add(result,
                                    pmt::mp("poll_id"),
                                    pmt::dict_ref(meta, pmt::mp("poll_id"), pmt::PMT_NIL));
            result = pmt::dict_add(result, pmt::mp("completion_reason"), pmt::mp("delta x delta y is not finite"));
            message_port_pub(pmt::mp("diagnostic_out"), pmt::cons(result, pmt::PMT_NIL));
            return;
        }
        x += delta_x;
        y += delta_y;
        const double step_norm = std::hypot(delta_x, delta_y);

        if (step_norm < convergence_tolerance) {
            converged = 1;
            break;
        }
    }
    residuals.clear();

    for (const auto& measurement : obs) {

        const double dx = x - measurement.anchor_pos[0];
        const double dy = y - measurement.anchor_pos[1];
        const double dz =
            d_vehicle_height_m - measurement.anchor_pos[2];

        const double predicted_range =
            std::sqrt(dx * dx + dy * dy + dz * dz);

        const double residual =
            measurement.range - predicted_range;

        residuals.push_back(residual);
    }

    if (!converged){
            auto result = pmt::make_dict();
                d_position = { x, y, d_vehicle_height_m };
            std::vector<double> position = { d_position[0], d_position[1], d_position[2] };
            result = pmt::dict_add(result,
                            pmt::mp("position_m"),
                            pmt::init_f64vector(position.size(), position));
            result = pmt::dict_add(result,
                                    pmt::mp("residual"),
                                    pmt::init_f64vector(residuals.size(), residuals));
            result = pmt::dict_add(result,
                                    pmt::mp("selected_ranges_m"),
                                    pmt::init_f64vector(selected_ranges.size(), selected_ranges));
            result = pmt::dict_add(result,
                                    pmt::mp("selected_anchor_ids"),
                                    pmt::init_u32vector(selected_anchor_ids.size(), selected_anchor_ids));
            result = pmt::dict_add(result,
                                    pmt::mp("poll_id"),
                                    pmt::dict_ref(meta, pmt::mp("poll_id"), pmt::PMT_NIL));
            result = pmt::dict_add(result, pmt::mp("completion_reason"), pmt::mp("WLS is not convergent"));
            message_port_pub(pmt::mp("diagnostic_out"), pmt::cons(result, pmt::PMT_NIL));
            return;
    }

    d_position = { x, y, d_vehicle_height_m };
    std::vector<double> position = { d_position[0], d_position[1], d_position[2] };
    auto result = pmt::make_dict();
    result = pmt::dict_add(result,
                            pmt::mp("position_m"),
                            pmt::init_f64vector(position.size(), position));
    result = pmt::dict_add(result,
                        pmt::mp("residual"),
                        pmt::init_f64vector(residuals.size(), residuals));
    result = pmt::dict_add(result,
                            pmt::mp("selected_ranges_m"),
                            pmt::init_f64vector(selected_ranges.size(), selected_ranges));
    result = pmt::dict_add(result,
                            pmt::mp("selected_anchor_ids"),
                            pmt::init_u32vector(selected_anchor_ids.size(), selected_anchor_ids));
    result = pmt::dict_add(result,
                            pmt::mp("poll_id"),
                            pmt::dict_ref(meta, pmt::mp("poll_id"), pmt::PMT_NIL));
    result = pmt::dict_add(result, pmt::mp("completion_reason"), pmt::mp("Positioning Estimation"));
    
    message_port_pub(pmt::mp("position_out"), pmt::cons(result, pmt::PMT_NIL));
    return;
}


} // namespace ofdm_prs_ranging
} // namespace gr