/* -*- c++ -*- */
#include "zc_rtt_calculator_impl.h"
#include "zc_packet_utils.h"
#include <gnuradio/io_signature.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>

namespace gr {
namespace ofdm_prs_ranging {

static std::string zc_utc_iso_now()
{
    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);
    std::tm tm {};
#if defined(_WIN32)
    gmtime_s(&tm, &time);
#else
    gmtime_r(&time, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S+00:00");
    return oss.str();
}

zc_rtt_calculator::sptr zc_rtt_calculator::make(double samp_rate,
                                                int zc_length,
                                                double delay_secs,
                                                float peak_metric_threshold,
                                                float fixed_threshold,
                                                double distance_setting_m,
                                                const std::string& log_path)
{
    return gnuradio::make_block_sptr<zc_rtt_calculator_impl>(samp_rate,
                                                             zc_length,
                                                             delay_secs,
                                                             peak_metric_threshold,
                                                             fixed_threshold,
                                                             distance_setting_m,
                                                             log_path);
}

static constexpr const char* csv_header =
    "measurement_index,log_time_utc,latest_tx_time_secs,rx_peak_secs,"
    "distance_setting_m,distance_fix_m,tof_us,tof_without_curvefit_us,"
    "rx_time_tag_secs,time_offset_s,peak_frac_s,peak_value,noise_floor,threshold,"
    "corr_peak_metric_db,phase_error,responder_id,t2_minus_t3,measurement_number,"
    "reply_delay_samples,samp_rate,zc_length,delay_secs,k_peak2noise,fixed_threshold";

zc_rtt_calculator_impl::zc_rtt_calculator_impl(double samp_rate,
                                               int zc_length,
                                               double delay_secs,
                                               float peak_metric_threshold,
                                               float fixed_threshold,
                                               double distance_setting_m,
                                               const std::string& log_path)
    : gr::block("zc_rtt_calculator",
                gr::io_signature::make(2, 2, sizeof(gr_complex)),
                gr::io_signature::make(1, 1, sizeof(gr_complex))),
      d_samp_rate(samp_rate),
      d_zc_length(zc_length),
      d_delay_secs(delay_secs),
      d_peak_metric_threshold(peak_metric_threshold),
      d_fixed_threshold(fixed_threshold),
      d_distance_setting_m(distance_setting_m),
      d_log_path(log_path),
      d_zc_duration(static_cast<double>(zc_length) / samp_rate),
      d_n_bits(zc_response_packet_bits),
      d_payload_len(zc_response_packet_bits * d_sps),
      d_ts_mask((1 << zc_t2_minus_t3_bits) - 1)
{
    d_capture_window_len =
        d_capture_pre_noise_len + d_zc_length + d_payload_offset + d_payload_len +
        d_capture_tail_len;
    d_raw_buffer_max_len = std::max(65536, 4 * d_capture_window_len);
    message_port_register_in(pmt::mp("tx_time_in"));
    set_msg_handler(pmt::mp("tx_time_in"),
                    [this](pmt::pmt_t msg) { this->handle_tx_time(msg); });
    ensure_csv_header();
    std::cout << "[RTT CSV] logging to: " << d_log_path << std::endl;
}

void zc_rtt_calculator_impl::handle_tx_time(pmt::pmt_t msg)
{
    if (!pmt::is_tuple(msg)) {
        return;
    }
    const auto sec = pmt::to_uint64(pmt::tuple_ref(msg, 0));
    const double frac = pmt::to_double(pmt::tuple_ref(msg, 1));
    d_latest_tx_time_secs = static_cast<double>(sec) + frac;
}

void zc_rtt_calculator_impl::ensure_csv_header()
{
    const std::filesystem::path path(d_log_path);
    if (!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path());
    }
    if (!std::filesystem::exists(path) || std::filesystem::file_size(path) == 0) {
        std::ofstream f(d_log_path);
        f << csv_header << "\n";
    }
}

void zc_rtt_calculator_impl::append_raw_buffer(const gr_complex* samples,
                                               int nitems,
                                               int64_t abs_start)
{
    if (nitems <= 0) {
        return;
    }
    const int64_t abs_end = abs_start + nitems;
    if (d_raw_buffer.empty()) {
        d_raw_buffer.assign(samples, samples + nitems);
        d_raw_buffer_abs_start = abs_start;
        d_raw_buffer_abs_end = abs_end;
    } else if (abs_start == d_raw_buffer_abs_end) {
        d_raw_buffer.insert(d_raw_buffer.end(), samples, samples + nitems);
        d_raw_buffer_abs_end = abs_end;
    } else if (abs_start > d_raw_buffer_abs_end) {
        d_raw_buffer.assign(samples, samples + nitems);
        d_raw_buffer_abs_start = abs_start;
        d_raw_buffer_abs_end = abs_end;
    } else {
        const int64_t overlap = d_raw_buffer_abs_end - abs_start;
        if (overlap < nitems) {
            d_raw_buffer.insert(d_raw_buffer.end(), samples + overlap, samples + nitems);
            d_raw_buffer_abs_end = abs_end;
        }
    }
    trim_raw_buffer();
}

void zc_rtt_calculator_impl::trim_raw_buffer()
{
    if (static_cast<int>(d_raw_buffer.size()) <= d_raw_buffer_max_len) {
        return;
    }
    const int drop = static_cast<int>(d_raw_buffer.size()) - d_raw_buffer_max_len;
    d_raw_buffer.erase(d_raw_buffer.begin(), d_raw_buffer.begin() + drop);
    d_raw_buffer_abs_start += drop;
}

bool zc_rtt_calculator_impl::get_raw_window(int64_t abs_start,
                                            int length,
                                            std::vector<gr_complex>& out) const
{
    const int64_t abs_end = abs_start + length;
    if (abs_start < d_raw_buffer_abs_start || abs_end > d_raw_buffer_abs_end) {
        return false;
    }
    const int local = static_cast<int>(abs_start - d_raw_buffer_abs_start);
    out.assign(d_raw_buffer.begin() + local, d_raw_buffer.begin() + local + length);
    return true;
}

std::pair<int, double> zc_rtt_calculator_impl::find_peak(const gr_complex* corr,
                                                         const gr_complex* raw,
                                                         int nitems,
                                                         peak_metrics& metrics) const
{
    int best = -1;
    float best_metric = 0.0f;
    float best_mag = 0.0f;
    for (int i = 0; i < nitems; ++i) {
        const int raw_start = i - d_zc_length + 1;
        double raw_power = 0.0;
        if (raw_start >= 0) {
            for (int k = 0; k < d_zc_length; ++k) {
                raw_power += std::norm(raw[raw_start + k]);
            }
        } else {
            raw_power = static_cast<double>(d_zc_length);
        }
        const double denom = std::sqrt(std::max(1e-18, raw_power) * d_zc_length);
        const float mag = std::abs(corr[i]);
        const float metric = static_cast<float>(mag / denom);
        if (metric >= d_peak_metric_threshold && mag >= d_fixed_threshold &&
            metric > best_metric) {
            best = i;
            best_metric = metric;
            best_mag = mag;
        }
    }

    metrics.threshold = d_peak_metric_threshold;
    metrics.peak_value = best_mag;
    metrics.noise_floor = 0.0;
    metrics.corr_peak_metric_db =
        best >= 0 ? 20.0 * std::log10(std::max(1e-12f, best_metric)) : -INFINITY;

    double delta = 0.0;
    if (best > 0 && best < nitems - 1) {
        const double y_m1 = std::abs(corr[best - 1]);
        const double y_0 = std::abs(corr[best]);
        const double y_p1 = std::abs(corr[best + 1]);
        const double den = y_m1 - 2.0 * y_0 + y_p1;
        if (std::abs(den) > 1e-12) {
            delta = 0.5 * (y_m1 - y_p1) / den / d_samp_rate;
            delta = std::clamp(delta, -0.5 / d_samp_rate, 0.5 / d_samp_rate);
        }
    }
    return { best, delta };
}

std::vector<uint8_t>
zc_rtt_calculator_impl::bpsk_demod(const std::vector<gr_complex>& payload,
                                   double phase_error) const
{
    std::vector<uint8_t> bits;
    bits.reserve(d_n_bits);
    const gr_complex rot = std::exp(gr_complex(0.0f, static_cast<float>(-phase_error)));
    for (int k = 0; k < d_n_bits; ++k) {
        const int start = k * d_sps + d_sps / 4;
        const int stop = (k + 1) * d_sps - d_sps / 4;
        gr_complex acc(0, 0);
        for (int i = start; i < stop; ++i) {
            acc += payload[i] * rot;
        }
        const float mean_real = acc.real() / static_cast<float>(std::max(1, stop - start));
        bits.push_back(mean_real < 0.0f ? 1 : 0);
    }
    return bits;
}

void zc_rtt_calculator_impl::queue_capture_window(const std::vector<gr_complex>& samples,
                                                  const event& ev,
                                                  int responder_id,
                                                  int measurement_number)
{
    if (samples.size() != static_cast<size_t>(d_capture_window_len)) {
        return;
    }
    if (d_capture_queue.size() >= 32) {
        d_capture_queue.pop_front();
    }
    capture_item item;
    item.samples = samples;
    item.capture_start_abs = ev.capture_start_abs;
    item.peak_abs = ev.peak_abs;
    item.payload_start_abs = ev.payload_start_abs;
    item.measurement_index = d_measurement_count + 1;
    item.measurement_number = measurement_number;
    item.responder_id = responder_id;
    item.phase_error = ev.phase_error;
    d_capture_queue.push_back(std::move(item));
}

int zc_rtt_calculator_impl::produce_capture_output(gr_complex* out,
                                                   int noutput_items,
                                                   int produced)
{
    while (!d_capture_queue.empty() && produced < noutput_items) {
        auto& item = d_capture_queue.front();
        const int remaining = static_cast<int>(item.samples.size() - item.pos);
        const int room = noutput_items - produced;
        const int to_copy = std::min(remaining, room);
        std::copy(item.samples.begin() + item.pos,
                  item.samples.begin() + item.pos + to_copy,
                  out + produced);
        const uint64_t abs_out = nitems_written(0) + produced;
        if (item.pos == 0) {
            add_item_tag(0, abs_out, pmt::mp("capture_sob"), pmt::PMT_T, pmt::mp("rtt_capture"));
            add_item_tag(0,
                         abs_out,
                         pmt::mp("capture_len"),
                         pmt::from_uint64(d_capture_window_len),
                         pmt::mp("rtt_capture"));
            add_item_tag(0,
                         abs_out,
                         pmt::mp("capture_start_abs"),
                         pmt::from_uint64(item.capture_start_abs),
                         pmt::mp("rtt_capture"));
            add_item_tag(0,
                         abs_out,
                         pmt::mp("capture_peak_abs"),
                         pmt::from_uint64(item.peak_abs),
                         pmt::mp("rtt_capture"));
            add_item_tag(0,
                         abs_out,
                         pmt::mp("payload_start_abs"),
                         pmt::from_uint64(item.payload_start_abs),
                         pmt::mp("rtt_capture"));
            add_item_tag(0,
                         abs_out,
                         pmt::mp("measurement_index"),
                         pmt::from_uint64(item.measurement_index),
                         pmt::mp("rtt_capture"));
        }
        item.pos += to_copy;
        produced += to_copy;
        if (item.pos >= item.samples.size()) {
            add_item_tag(0,
                         nitems_written(0) + produced - 1,
                         pmt::mp("capture_eob"),
                         pmt::PMT_T,
                         pmt::mp("rtt_capture"));
            d_capture_queue.pop_front();
        }
    }
    return produced;
}

bool zc_rtt_calculator_impl::try_process_event(const event& ev)
{
    std::vector<gr_complex> payload;
    if (!get_raw_window(ev.payload_start_abs, d_payload_len, payload)) {
        return false;
    }
    std::vector<gr_complex> capture;
    const bool have_capture =
        get_raw_window(ev.capture_start_abs, d_capture_window_len, capture);

    int responder_id = -1;
    int t2_minus_t3 = 0;
    int measurement_number = -1;
    int reply_delay_samples = static_cast<int>(std::llround(d_delay_secs * d_samp_rate));
    double reply_delay_secs = d_delay_secs;
    const auto bits = bpsk_demod(payload, ev.phase_error);
    uint8_t decoded_responder = 0;
    int32_t decoded_t2_minus_t3 = 0;
    uint16_t decoded_measurement = 0;
    if (zc_decode_response_packet(
            bits, decoded_responder, decoded_t2_minus_t3, decoded_measurement)) {
        responder_id = decoded_responder;
        t2_minus_t3 = decoded_t2_minus_t3;
        measurement_number = decoded_measurement;
        reply_delay_samples = (-t2_minus_t3) & d_ts_mask;
        reply_delay_secs = static_cast<double>(reply_delay_samples) / d_samp_rate;
    } else {
        std::cout << "[RX] payload decode failed, fallback to configured delay" << std::endl;
    }

    if (have_capture) {
        queue_capture_window(capture, ev, responder_id, measurement_number);
    }
    append_csv_row(ev,
                   responder_id,
                   t2_minus_t3,
                   measurement_number,
                   reply_delay_samples,
                   reply_delay_secs);
    return true;
}

void zc_rtt_calculator_impl::process_pending_events()
{
    std::deque<event> remaining;
    while (!d_pending_events.empty()) {
        const auto ev = d_pending_events.front();
        d_pending_events.pop_front();
        if (!try_process_event(ev)) {
            if (ev.payload_start_abs >= d_raw_buffer_abs_start) {
                remaining.push_back(ev);
            }
        }
    }
    d_pending_events.swap(remaining);
}

void zc_rtt_calculator_impl::append_csv_row(const event& ev,
                                            int responder_id,
                                            int t2_minus_t3,
                                            int measurement_number,
                                            int reply_delay_samples,
                                            double reply_delay_secs)
{
    const double round_trip_flight_time =
        ev.rx_peak_secs - ev.tx_time_secs - reply_delay_secs;
    const double tof = (round_trip_flight_time + ev.peak_frac_s) / 2.0 + d_conlibration;
    const double distance_fix = tof * 299792458.0;
    ++d_measurement_count;

    std::ofstream f(d_log_path, std::ios::app);
    f << std::setprecision(12) << d_measurement_count << "," << zc_utc_iso_now()
      << "," << ev.tx_time_secs << "," << ev.rx_peak_secs << ","
      << d_distance_setting_m << "," << distance_fix << "," << tof * 1e6 << ","
      << (round_trip_flight_time / 2.0) * 1e6 << "," << ev.rx_time_tag_secs << ","
      << ev.time_offset_s << "," << ev.peak_frac_s << "," << ev.metrics.peak_value
      << "," << ev.metrics.noise_floor << "," << ev.metrics.threshold << ","
      << ev.metrics.corr_peak_metric_db << "," << ev.phase_error << ","
      << responder_id << "," << t2_minus_t3 << "," << measurement_number << ","
      << reply_delay_samples << "," << d_samp_rate << "," << d_zc_length << ","
      << d_delay_secs << "," << d_peak_metric_threshold << "," << d_fixed_threshold
      << "\n";
    std::cout << "[RX] Measurement " << d_measurement_count
              << ": Valid peak and tx_time found" << std::endl;
}

int zc_rtt_calculator_impl::general_work(int noutput_items,
                                         gr_vector_int& ninput_items,
                                         gr_vector_const_void_star& input_items,
                                         gr_vector_void_star& output_items)
{
    const int n = std::min(ninput_items[0], ninput_items[1]);
    const auto corr = static_cast<const gr_complex*>(input_items[0]);
    const auto raw = static_cast<const gr_complex*>(input_items[1]);
    auto out = static_cast<gr_complex*>(output_items[0]);
    int produced = 0;

    if (n > 0) {
        const int64_t corr_abs_start = nitems_read(0);
        const int64_t raw_abs_start = nitems_read(1);
        append_raw_buffer(raw, n, raw_abs_start);

        std::vector<tag_t> tags;
        get_tags_in_window(tags, 1, 0, n);
        for (const auto& tag : tags) {
            if (pmt::symbol_to_string(tag.key) == "rx_time" && pmt::is_tuple(tag.value)) {
                d_last_rx_time = tag.value;
                d_last_tag_offset = tag.offset;
                d_have_rx_time = true;
            }
        }

        process_pending_events();

        peak_metrics metrics;
        const auto [peak_local, k_frac] = find_peak(corr, raw, n, metrics);
        if (peak_local >= 0 && d_have_rx_time && d_latest_tx_time_secs > 0.0) {
            const int64_t current_peak_offset = corr_abs_start + peak_local;
            const int64_t diff_samples =
                current_peak_offset - static_cast<int64_t>(d_last_tag_offset);
            const auto rx_sec =
                static_cast<uint64_t>(pmt::to_uint64(pmt::tuple_ref(d_last_rx_time, 0)));
            const double rx_frac = pmt::to_double(pmt::tuple_ref(d_last_rx_time, 1));
            const double rx_time_tag_secs = static_cast<double>(rx_sec) + rx_frac;
            const double time_offset = static_cast<double>(diff_samples) / d_samp_rate;
            event ev;
            ev.tx_time_secs = d_latest_tx_time_secs;
            ev.peak_abs = current_peak_offset;
            ev.zc_start_abs = current_peak_offset - d_zc_length + 1;
            ev.payload_start_abs = current_peak_offset + d_payload_offset;
            ev.capture_start_abs = ev.zc_start_abs - d_capture_pre_noise_len;
            ev.phase_error = std::arg(corr[peak_local]);
            ev.rx_peak_secs = rx_time_tag_secs + time_offset - d_zc_duration;
            ev.rx_time_tag_secs = rx_time_tag_secs;
            ev.time_offset_s = time_offset;
            ev.peak_frac_s = k_frac;
            ev.metrics = metrics;
            if (!try_process_event(ev)) {
                if (d_pending_events.size() >= 32) {
                    d_pending_events.pop_front();
                }
                d_pending_events.push_back(ev);
            }
            d_latest_tx_time_secs = 0.0;
        }

        consume(0, n);
        consume(1, n);
    }

    produced = produce_capture_output(out, noutput_items, produced);
    produce(0, produced);
    return WORK_CALLED_PRODUCE;
}

} // namespace ofdm_prs_ranging
} // namespace gr
