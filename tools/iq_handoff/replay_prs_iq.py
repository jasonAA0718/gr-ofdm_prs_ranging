#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: replay prs iq
# GNU Radio version: 3.10.11.0

from gnuradio import blocks
import pmt
from gnuradio import blocks, gr
from gnuradio import gr
from gnuradio.filter import firdes
from gnuradio.fft import window
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import ofdm_prs_ranging
import threading




class replay_prs_iq(gr.top_block):

    def __init__(self):
        gr.top_block.__init__(self, "replay prs iq", catch_exceptions=True)
        self.flowgraph_started = threading.Event()

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 30e6
        self.expect_response = expect_response = False
        self.iq_file = iq_file = "handoff.cf32"

        ##################################################
        # Blocks
        ##################################################

        self.read_iq = blocks.file_source(gr.sizeof_gr_complex*1, iq_file, False, 000000, 0)
        self.read_iq.set_begin_tag(pmt.PMT_NIL)
        self.ofdm_prs_ranging_prs_zc_frame_sync_0 = ofdm_prs_ranging.prs_zc_frame_sync(
            samp_rate,
            0.15,
            0.35,
            expect_response,
            0)
        self.ofdm_prs_ranging_prs_window_capture_0 = ofdm_prs_ranging.prs_window_capture(
            samp_rate,
            0.09,
            10000,
            4096,
            4096,
            3,
            True)
        self.fft = ofdm_prs_ranging.prs_fft_receiver(samp_rate)
        self.decode_metadata = blocks.message_debug(False, gr.log_levels.info)
        self.channel = ofdm_prs_ranging.prs_channel_estimator(samp_rate)


        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.channel, 'channel_out'), (self.decode_metadata, 'print'))
        self.msg_connect((self.fft, 'symbols_out'), (self.channel, 'symbols_in'))
        self.msg_connect((self.ofdm_prs_ranging_prs_window_capture_0, 'candidate_out'), (self.ofdm_prs_ranging_prs_zc_frame_sync_0, 'candidate_in'))
        self.msg_connect((self.ofdm_prs_ranging_prs_zc_frame_sync_0, 'synchronized_frame_out'), (self.fft, 'frame_in'))
        self.msg_connect((self.ofdm_prs_ranging_prs_zc_frame_sync_0, 'slot_release'), (self.ofdm_prs_ranging_prs_window_capture_0, 'slot_release'))
        self.connect((self.read_iq, 0), (self.ofdm_prs_ranging_prs_window_capture_0, 0))


    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate

    def get_iq_file(self):
        return self.iq_file

    def set_iq_file(self, iq_file):
        self.iq_file = iq_file




def main(top_block_cls=replay_prs_iq, options=None):
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
