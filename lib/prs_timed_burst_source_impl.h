/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_TIMED_BURST_SOURCE_IMPL_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_TIMED_BURST_SOURCE_IMPL_H

#include "prs_burst_scheduler.h"
#include <gnuradio/ofdm_prs_ranging/prs_timed_burst_source.h>
#include <pmt/pmt.h>
#include <vector>

namespace gr {
  namespace ofdm_prs_ranging {

    class prs_timed_burst_source_impl : public prs_timed_burst_source
    {
     private:
      double d_samp_rate;
      int d_fft_len;
      int d_cp_len;
      int d_active_bins;
      int d_prs_symbols;
      int d_preamble_len;
      int d_preamble_repeats;
      int d_coarse_sync_len;
      int d_zero_guard_len;
      int d_tail_guard_len;
      double d_tx_lead_time;
      double d_burst_period;
      float d_tx_amp;
      uint32_t d_seed;
      int d_pings_per_trigger;
      bool d_attach_tx_time;
      int d_coarse_zc_root;

      std::vector<gr_complex> d_frame;
      std::vector<gr_complex> d_burst_frame;
      int d_prs_start;
      int d_prs_len;
      int d_payload_start;
      int d_payload_len;
      prs_burst_scheduler d_scheduler;
      bool d_in_burst;
      prs_pending_burst d_current_burst;
      size_t d_burst_offset;

      bool d_have_rx_time;
      uint64_t d_rx_time_abs_sample;
      double d_rx_time_value;

      void validate_parameters() const;
      void build_frame();
      void prepare_burst_frame(uint64_t frame_id);
      void handle_trigger(pmt::pmt_t msg);
      double current_time_estimate();
      void add_burst_tags(uint64_t abs_offset, const prs_pending_burst& burst);
      void publish_tx_time(const prs_pending_burst& burst);

     public:
      prs_timed_burst_source_impl(double samp_rate, int fft_len, int cp_len, int active_bins, int prs_symbols, int preamble_len, int preamble_repeats, int coarse_sync_len, int zero_guard_len, int tail_guard_len, double tx_lead_time, double burst_period, float tx_amp, uint32_t seed, int pings_per_trigger, bool attach_tx_time, int coarse_zc_root);
      ~prs_timed_burst_source_impl();

      int frame_len() const override { return static_cast<int>(d_frame.size()); }
      int prs_start() const override { return d_prs_start; }
      int prs_len() const override { return d_prs_len; }
      std::vector<gr_complex> frame_samples() const override { return d_frame; }

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

    };

  } // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_TIMED_BURST_SOURCE_IMPL_H */
