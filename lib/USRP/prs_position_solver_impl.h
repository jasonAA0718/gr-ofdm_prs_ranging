#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_POSITION_SOLVER_IMPL_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_POSITION_SOLVER_IMPL_H
#include <gnuradio/ofdm_prs_ranging/prs_position_solver.h>
#include <pmt/pmt.h>
#include <array>
#include <unordered_map>
#include <cstdint>
namespace gr{
namespace ofdm_prs_ranging{

class prs_position_solver_impl: public  prs_position_solver{
    public:
        prs_position_solver_impl(
            const std::vector<int>& anchor_ids,
            const std::vector<std::vector<double>>& anchor_positions_m,
            double vehicle_height_m);
    private:
        double d_vehicle_height_m;
        const std::vector<std::vector<double>> d_anchor_positions_m;
        const std::vector<int>& d_anchor_ids;
        void handle_epoch(pmt::pmt_t msg);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_POSITION_SOLVER_IMPL_H */