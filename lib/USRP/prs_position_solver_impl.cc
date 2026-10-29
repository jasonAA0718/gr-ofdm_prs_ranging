#include "prs_position_solver_impl.h"
#include <gnuradio/io_signature.h>
#include <cmath>
#include <stdexcept>
#include <pmt/pmt.h>
#include <array>
#include <unordered_map>
#include <cstdint>
namespace gr{
namespace ofdm_prs_ranging{

prs_position_solver:: sptr prs_position_solver::make(
    const std::vector<int>& anchor_ids,
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
            : gr::block(
                "prs_position_solver",
                gr::io_signature::make(0,0,0),
                gr::io_signature::make(0,0,0)),
                d_vehicle_height_m(vehicle_height_m),
                d_anchor_positions_m(anchor_positions_m),
                d_anchor_ids(anchor_ids)          
{
    if (anchor_ids.empty() || anchor_ids.size() > 8)
        throw std::invalid_argument("Invalid anchor count");

    if (!std::isfinite(vehicle_height_m))
        throw std::invalid_argument("Invalid vehicle height");

    for (auto id : anchor_ids){

        if (static_cast<size_t>(id) >= anchor_positions_m.size())
            throw std::invalid_argument("Anchor position not found");

        if (anchor_positions_m[id].size() != 3)  
            throw std::invalid_argument("XYZ must have 3 elements");

        for (double xyz : anchor_positions_m[id]) {
            if (!std::isfinite(xyz))
                throw std::invalid_argument("Invalid XYZ coordinate");
        }
    }
    message_port_register_in(pmt::mp("epoch_in"));
    message_port_register_out(pmt::mp("position_out"));
    message_port_register_out(pmt::mp("diagnostic_out"));
    set_msg_handler(
                pmt::mp("epoch_in"),
                [this](pmt::pmt_t msg){
                    handle_epoch(msg); 
                });
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

    const auto ids = pmt::dict_ref(
        meta, pmt::mp("observed_anchor_ids"), pmt::PMT_NIL);

    const auto integer_ranges = pmt::dict_ref(
        meta, pmt::mp("integer_ranges_m"), pmt::PMT_NIL);

    const auto corrected_ranges = pmt::dict_ref(
        meta, pmt::mp("corrected_ranges_m"), pmt::PMT_NIL);

    const auto fine_flags = pmt::dict_ref(
        meta, pmt::mp("fine_correction_valid_flags"), pmt::PMT_NIL);

    if (!pmt::is_u32vector(ids) ||
        !pmt::is_f64vector(integer_ranges) ||
        !pmt::is_f64vector(corrected_ranges) ||
        !pmt::is_u8vector(fine_flags)) {
        return;
    }

    const size_t N = pmt::length(ids);
    // Check the length of measurement is reasonable.
    if (pmt::length(integer_ranges) != N ||
        pmt::length(corrected_ranges) != N ||
        pmt::length(fine_flags) != N) {
        return;
    }

    auto result = pmt::make_dict();

    result = pmt::dict_add(
        result,
        pmt::mp("parsed_anchor_count"),
        pmt::from_uint64(N));

    result = pmt::dict_add(
        result,
        pmt::mp("poll_id"),
        pmt::dict_ref(meta, pmt::mp("poll_id"), pmt::PMT_NIL));

    message_port_pub(
        pmt::mp("diagnostic_out"),
        pmt::cons(result, pmt::PMT_NIL));
}           


} // namespace ofdm_prs_ranging
} // namespace gr