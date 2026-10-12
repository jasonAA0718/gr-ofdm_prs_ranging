#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: PRS SS-RTT Responder
# Description: OFDM PRS SS-RTT responder
# GNU Radio version: 3.10.11.0

from gnuradio import gr
from gnuradio.filter import firdes
from gnuradio.fft import window
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import ofdm_prs_ranging
from gnuradio import uhd
import time
import threading




class prs_ssrtt_responder(gr.top_block):

    def __init__(self):
        gr.top_block.__init__(self, "PRS SS-RTT Responder", catch_exceptions=True)
        self.flowgraph_started = threading.Event()

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 30000000
        self.zc_length = zc_length = 819
        self.tx_gain = tx_gain = 55
        self.rx_gain = rx_gain = 48
        self.reply_delay_samples = reply_delay_samples = int(0.031*samp_rate)
        self.premble_rep = premble_rep = 17
        self.premble_length = premble_length = 128
        self.center_freq = center_freq = 1060e6
        self.T_burst = T_burst = 0.2

        ##################################################
        # Blocks
        ##################################################

        self.uhd_usrp_source_0_0_0 = uhd.usrp_source(
            ",".join(("serial=34D0564", "recv_buff_size=20000000,num_recv_frames=700")),
            uhd.stream_args(
                cpu_format="fc32",
                args='',
                channels=list(range(0,1)),
            ),
        )
        self.uhd_usrp_source_0_0_0.set_subdev_spec('A:B', 0)
        self.uhd_usrp_source_0_0_0.set_samp_rate(samp_rate)
        self.uhd_usrp_source_0_0_0.set_time_unknown_pps(uhd.time_spec(0))

        self.uhd_usrp_source_0_0_0.set_center_freq(center_freq, 0)
        self.uhd_usrp_source_0_0_0.set_antenna("TX/RX", 0)
        self.uhd_usrp_source_0_0_0.set_bandwidth(samp_rate, 0)
        self.uhd_usrp_source_0_0_0.set_gain(rx_gain, 0)
        self.uhd_usrp_sink_0_0 = uhd.usrp_sink(
            ",".join(("serial=34D0564", '')),
            uhd.stream_args(
                cpu_format="fc32",
                args='',
                channels=list(range(0,1)),
            ),
            "",
        )
        self.uhd_usrp_sink_0_0.set_subdev_spec('A:B', 0)
        self.uhd_usrp_sink_0_0.set_samp_rate(samp_rate)
        self.uhd_usrp_sink_0_0.set_time_unknown_pps(uhd.time_spec(0))

        self.uhd_usrp_sink_0_0.set_center_freq(center_freq, 0)
        self.uhd_usrp_sink_0_0.set_antenna("TX/RX", 0)
        self.uhd_usrp_sink_0_0.set_bandwidth(samp_rate, 0)
        self.uhd_usrp_sink_0_0.set_gain(tx_gain, 0)
        self.uhd_usrp_sink_0_0.set_min_output_buffer(1048576)
        self.prs_text_ui_0 = ofdm_prs_ranging.prs_text_ui("responder", 1.5, 1.0, 2.0, True)
        self.prs_ssrtt_responder_0 = ofdm_prs_ranging.prs_ssrtt_responder(samp_rate, reply_delay_samples, 0)
        self.prs_source = ofdm_prs_ranging.prs_timed_burst_source(
            samp_rate, 512, 64, 512, 127,
            premble_length, premble_rep, zc_length,
            1000, 1000, 0.1,
            0.1, 0.7, 13990001, 1,
            True, 29, 2)
        self.prs_phase_slope_estimator_0 = ofdm_prs_ranging.prs_phase_slope_estimator(samp_rate, 512, 512, 1.0, 0)
        self.prs_fft_receiver_0 = ofdm_prs_ranging.prs_fft_receiver(samp_rate, 512, 64, 512, 127, 0)
        self.prs_csv_logger_0 = ofdm_prs_ranging.prs_csv_logger("CSV/responder_measurements.csv", 1, 0)
        self.prs_channel_estimator_0 = ofdm_prs_ranging.prs_channel_estimator(samp_rate, 512, 512, 127, 13990001, 0, 2)
        self.prs_acquisition_logger_0 = ofdm_prs_ranging.prs_acquisition_logger("CSV/responder_acquisition.csv", "responder", 1)
        self.ofdm_prs_ranging_prs_zc_frame_sync_0 = ofdm_prs_ranging.prs_zc_frame_sync(30e6, 512, 64, 512, 127, 128, 17, 819, 1000, 1000, 0.15, 0.35, 25, 0, 0)
        self.ofdm_prs_ranging_prs_zc_frame_sync_0.set_processor_affinity([2])
        self.ofdm_prs_ranging_prs_window_capture_0 = ofdm_prs_ranging.prs_window_capture(30e6, 512, 64, 512, 127, 128, 17, 819, 1000, 1000, 0.09, 10000, 4096, 4096, 2, True, False)


        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.ofdm_prs_ranging_prs_window_capture_0, 'candidate_out'), (self.ofdm_prs_ranging_prs_zc_frame_sync_0, 'candidate_in'))
        self.msg_connect((self.ofdm_prs_ranging_prs_window_capture_0, 'event_out'), (self.prs_acquisition_logger_0, 'frame_in'))
        self.msg_connect((self.ofdm_prs_ranging_prs_zc_frame_sync_0, 'slot_release'), (self.ofdm_prs_ranging_prs_window_capture_0, 'slot_release'))
        self.msg_connect((self.ofdm_prs_ranging_prs_zc_frame_sync_0, 'synchronized_frame_out'), (self.prs_fft_receiver_0, 'frame_in'))
        self.msg_connect((self.ofdm_prs_ranging_prs_zc_frame_sync_0, 'synchronized_frame_out'), (self.prs_text_ui_0, 'frame_in'))
        self.msg_connect((self.prs_channel_estimator_0, 'event_out'), (self.prs_acquisition_logger_0, 'frame_in'))
        self.msg_connect((self.prs_channel_estimator_0, 'channel_out'), (self.prs_phase_slope_estimator_0, 'channel_in'))
        self.msg_connect((self.prs_fft_receiver_0, 'symbols_out'), (self.prs_channel_estimator_0, 'symbols_in'))
        self.msg_connect((self.prs_phase_slope_estimator_0, 'measurement_out'), (self.prs_csv_logger_0, 'measurement_in'))
        self.msg_connect((self.prs_phase_slope_estimator_0, 'measurement_out'), (self.prs_ssrtt_responder_0, 'measurement_in'))
        self.msg_connect((self.prs_phase_slope_estimator_0, 'measurement_out'), (self.prs_text_ui_0, 'measurement_in'))
        self.msg_connect((self.prs_source, 'tx_time_out'), (self.prs_text_ui_0, 'tx_in'))
        self.msg_connect((self.prs_ssrtt_responder_0, 'trigger_out'), (self.prs_source, 'trigger'))
        self.connect((self.prs_source, 0), (self.uhd_usrp_sink_0_0, 0))
        self.connect((self.uhd_usrp_source_0_0_0, 0), (self.ofdm_prs_ranging_prs_window_capture_0, 0))


    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_reply_delay_samples(int(0.031*self.samp_rate))
        self.uhd_usrp_sink_0_0.set_samp_rate(self.samp_rate)
        self.uhd_usrp_sink_0_0.set_bandwidth(self.samp_rate, 0)
        self.uhd_usrp_source_0_0_0.set_samp_rate(self.samp_rate)
        self.uhd_usrp_source_0_0_0.set_bandwidth(self.samp_rate, 0)

    def get_zc_length(self):
        return self.zc_length

    def set_zc_length(self, zc_length):
        self.zc_length = zc_length

    def get_tx_gain(self):
        return self.tx_gain

    def set_tx_gain(self, tx_gain):
        self.tx_gain = tx_gain
        self.uhd_usrp_sink_0_0.set_gain(self.tx_gain, 0)

    def get_rx_gain(self):
        return self.rx_gain

    def set_rx_gain(self, rx_gain):
        self.rx_gain = rx_gain
        self.uhd_usrp_source_0_0_0.set_gain(self.rx_gain, 0)

    def get_reply_delay_samples(self):
        return self.reply_delay_samples

    def set_reply_delay_samples(self, reply_delay_samples):
        self.reply_delay_samples = reply_delay_samples

    def get_premble_rep(self):
        return self.premble_rep

    def set_premble_rep(self, premble_rep):
        self.premble_rep = premble_rep

    def get_premble_length(self):
        return self.premble_length

    def set_premble_length(self, premble_length):
        self.premble_length = premble_length

    def get_center_freq(self):
        return self.center_freq

    def set_center_freq(self, center_freq):
        self.center_freq = center_freq
        self.uhd_usrp_sink_0_0.set_center_freq(self.center_freq, 0)
        self.uhd_usrp_source_0_0_0.set_center_freq(self.center_freq, 0)

    def get_T_burst(self):
        return self.T_burst

    def set_T_burst(self, T_burst):
        self.T_burst = T_burst




def main(top_block_cls=prs_ssrtt_responder, options=None):
    tb = top_block_cls()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    tb.start()
    tb.flowgraph_started.set()

    try:
        input('Press Enter to quit: ')
    except EOFError:
        pass
    tb.stop()
    tb.wait()


if __name__ == '__main__':
    main()
