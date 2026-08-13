/* -*- c++ -*- */
#ifndef INCLUDED_OFDM_PRS_RANGING_ZC_RTT_CALCULATOR_IMPL_H
#define INCLUDED_OFDM_PRS_RANGING_ZC_RTT_CALCULATOR_IMPL_H

#include <gnuradio/ofdm_prs_ranging/zc_rtt_calculator.h>
#include <deque>
#include <fstream>

namespace gr {
namespace ofdm_prs_ranging {

class zc_rtt_calculator_impl : public zc_rtt_calculator
{
public:
    zc_rtt_calculator_impl(double samp_rate,
                           int zc_length,
                           double delay_secs,
                           float peak_metric_threshold,
                           float fixed_threshold,
                           double distance_setting_m,
                           const std::string& log_path);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items) override;

private:
    struct peak_metrics {
        double noise_floor = 0.0;
        double threshold = 0.0;
        double peak_value = 0.0;
        double corr_peak_metric_db = -INFINITY;
    };

    struct event {
        double tx_time_secs = 0.0;
        int64_t peak_abs = 0;
        int64_t zc_start_abs = 0;
        int64_t payload_start_abs = 0;
        int64_t capture_start_abs = 0;
        double phase_error = 0.0;
        double rx_peak_secs = 0.0;
        double rx_time_tag_secs = 0.0;
        double time_offset_s = 0.0;
        double peak_frac_s = 0.0;
        peak_metrics metrics;
    };

    struct capture_item {
        std::vector<gr_complex> samples;
        size_t pos = 0;
        int64_t capture_start_abs = 0;
        int64_t peak_abs = 0;
        int64_t payload_start_abs = 0;
        int measurement_index = 0;
        int measurement_number = -1;
        int responder_id = -1;
        double phase_error = 0.0;
    };

    void handle_tx_time(pmt::pmt_t msg);
    void ensure_csv_header();
    void append_csv_row(const event& ev,
                        int responder_id,
                        int t2_minus_t3,
                        int measurement_number,
                        int reply_delay_samples,
                        double reply_delay_secs);
    void append_raw_buffer(const gr_complex* samples, int nitems, int64_t abs_start);
    bool get_raw_window(int64_t abs_start, int length, std::vector<gr_complex>& out) const;
    void trim_raw_buffer();
    std::pair<int, double> find_peak(const gr_complex* corr,
                                     const gr_complex* raw,
                                     int nitems,
                                     peak_metrics& metrics) const;
    std::vector<uint8_t> bpsk_demod(const std::vector<gr_complex>& payload,
                                    double phase_error) const;
    bool try_process_event(const event& ev);
    void process_pending_events();
    void queue_capture_window(const std::vector<gr_complex>& samples,
                              const event& ev,
                              int responder_id,
                              int measurement_number);
    int produce_capture_output(gr_complex* out, int noutput_items, int produced);

    double d_samp_rate;
    int d_zc_length;
    double d_delay_secs;
    float d_peak_metric_threshold;
    float d_fixed_threshold;
    double d_distance_setting_m;
    std::string d_log_path;
    double d_zc_duration;
    double d_latest_tx_time_secs = 0.0;
    bool d_have_rx_time = false;
    pmt::pmt_t d_last_rx_time;
    uint64_t d_last_tag_offset = 0;
    double d_conlibration = -9.59849896077843E-06;
    int d_sps = 60;
    int d_n_bits;
    int d_payload_offset = 100;
    int d_payload_len;
    int d_ts_mask;
    int d_capture_pre_noise_len = 2000;
    int d_capture_tail_len = 500;
    int d_capture_window_len;
    int d_measurement_count = 0;
    std::vector<gr_complex> d_raw_buffer;
    int64_t d_raw_buffer_abs_start = 0;
    int64_t d_raw_buffer_abs_end = 0;
    int d_raw_buffer_max_len;
    std::deque<event> d_pending_events;
    std::deque<capture_item> d_capture_queue;
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif
