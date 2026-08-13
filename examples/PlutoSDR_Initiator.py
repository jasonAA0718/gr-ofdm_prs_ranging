#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: PlutoSDR PRS Initiator
# Author: cnsl
# Description: PlutoSDR untimed PRS initiator with bounded raw capture
# GNU Radio version: 3.10.11.0

from gnuradio import blocks
import pmt
from gnuradio import gr
from gnuradio.filter import firdes
from gnuradio.fft import window
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import iio
from gnuradio import ofdm_prs_ranging
import threading




class PlutoSDR_Initiator(gr.top_block):

    def __init__(self):
        gr.top_block.__init__(self, "PlutoSDR PRS Initiator", catch_exceptions=True)
        self.flowgraph_started = threading.Event()

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = int(10e6)
        self.center_freq = center_freq = int(1060e6)
        self.capture_samples = capture_samples = int(2*samp_rate)

        ##################################################
        # Blocks
        ##################################################

        self.poll_strobe = blocks.message_strobe(pmt.PMT_T, 500)
        self.pluto_tx = ofdm_prs_ranging.pluto_prs_burst_source(
            samp_rate, 1024, 128, 1024, 16,
            128, 16, 839,
            1000, 1000, 0.6, 13990001,
            1, 0, 25)
        self.phase_slope = ofdm_prs_ranging.prs_phase_slope_estimator(samp_rate, 1024, 1024, 1.0)
        self.measurement_log = ofdm_prs_ranging.prs_csv_logger("CSV/pluto_initiator_phase_slope.csv", True)
        self.iio_tx = iio.fmcomms2_sink_fc32('ip:192.168.6.10' if 'ip:192.168.6.10' else iio.get_pluto_uri(), [True, True], (32768*128), False)
        self.iio_tx.set_len_tag_key('')
        self.iio_tx.set_bandwidth(10000000)
        self.iio_tx.set_frequency(center_freq)
        self.iio_tx.set_samplerate(samp_rate)
        self.iio_tx.set_attenuation(0, 10.0)
        self.iio_tx.set_filter_params('Auto', '', 0, 0)
        self.iio_rx = iio.fmcomms2_source_fc32('ip:192.168.6.10' if 'ip:192.168.6.10' else iio.get_pluto_uri(), [True, True], (32768*128))
        self.iio_rx.set_len_tag_key('packet_len')
        self.iio_rx.set_frequency(center_freq)
        self.iio_rx.set_samplerate(samp_rate)
        self.iio_rx.set_gain_mode(0, 'manual')
        self.iio_rx.set_gain(0, 20)
        self.iio_rx.set_quadrature(True)
        self.iio_rx.set_rfdc(True)
        self.iio_rx.set_bbdc(True)
        self.iio_rx.set_filter_params('Auto', '', 0, 0)
        self.fft_receiver = ofdm_prs_ranging.prs_fft_receiver(samp_rate, 1024, 128, 1024, 16)
        self.detector = ofdm_prs_ranging.prs_frame_detector(samp_rate, 1024, 128, 1024, 16, 128, 16, 839, 1000, 1000, 0.35, 10000, 29, 29, False, 0.05, 0.0002, 0.004, 0.35)
        self.channel_estimator = ofdm_prs_ranging.prs_channel_estimator(samp_rate, 1024, 1024, 16, 13990001)
        self.acquisition_log = ofdm_prs_ranging.prs_acquisition_logger("CSV/pluto_initiator_acquisition.csv", "pluto_initiator", True)


        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.channel_estimator, 'channel_out'), (self.phase_slope, 'channel_in'))
        self.msg_connect((self.detector, 'event_out'), (self.acquisition_log, 'frame_in'))
        self.msg_connect((self.detector, 'frame_out'), (self.fft_receiver, 'frame_in'))
        self.msg_connect((self.fft_receiver, 'symbols_out'), (self.channel_estimator, 'symbols_in'))
        self.msg_connect((self.phase_slope, 'measurement_out'), (self.measurement_log, 'measurement_in'))
        self.msg_connect((self.poll_strobe, 'strobe'), (self.pluto_tx, 'trigger'))
        self.connect((self.iio_rx, 0), (self.detector, 0))
        self.connect((self.pluto_tx, 0), (self.iio_tx, 0))


    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_capture_samples(int(2*self.samp_rate))
        self.iio_rx.set_samplerate(self.samp_rate)
        self.iio_tx.set_samplerate(self.samp_rate)

    def get_center_freq(self):
        return self.center_freq

    def set_center_freq(self, center_freq):
        self.center_freq = center_freq
        self.iio_rx.set_frequency(self.center_freq)
        self.iio_tx.set_frequency(self.center_freq)

    def get_capture_samples(self):
        return self.capture_samples

    def set_capture_samples(self, capture_samples):
        self.capture_samples = capture_samples




def main(top_block_cls=PlutoSDR_Initiator, options=None):
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
