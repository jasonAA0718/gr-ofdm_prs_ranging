#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: PRS RX Estimator 10M 1399
# Description: One-way OFDM PRS receiver estimator at 10 MHz and 1399 MHz
# GNU Radio version: 3.10.11.0

from PyQt5 import Qt
from gnuradio import qtgui
from PyQt5 import QtCore
from gnuradio import gr
from gnuradio.filter import firdes
from gnuradio.fft import window
import sys
import signal
from PyQt5 import Qt
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import ofdm_prs_ranging
from gnuradio import uhd
import time
import threading



class prs_rx_estimator_10M_1399(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "PRS RX Estimator 10M 1399", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("PRS RX Estimator 10M 1399")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except BaseException as exc:
            print(f"Qt GUI: Could not set Icon: {str(exc)}", file=sys.stderr)
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("gnuradio/flowgraphs", "prs_rx_estimator_10M_1399")

        try:
            geometry = self.settings.value("geometry")
            if geometry:
                self.restoreGeometry(geometry)
        except BaseException as exc:
            print(f"Qt GUI: Could not restore geometry: {str(exc)}", file=sys.stderr)
        self.flowgraph_started = threading.Event()

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 20e6
        self.rx_gain = rx_gain = 0.5
        self.center_freq = center_freq = 1399e6

        ##################################################
        # Blocks
        ##################################################

        self._rx_gain_range = qtgui.Range(0, 1, 0.01, 0.5, 200)
        self._rx_gain_win = qtgui.RangeWidget(self._rx_gain_range, self.set_rx_gain, "RX Gain", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._rx_gain_win)
        self.uhd_usrp_source_0_0 = uhd.usrp_source(
            ",".join(("serial=34D0563", "recv_buff_size=20000000,num_recv_frames=700")),
            uhd.stream_args(
                cpu_format="fc32",
                args='',
                channels=list(range(0,1)),
            ),
        )
        self.uhd_usrp_source_0_0.set_subdev_spec('A:B', 0)
        self.uhd_usrp_source_0_0.set_samp_rate(samp_rate)
        self.uhd_usrp_source_0_0.set_time_unknown_pps(uhd.time_spec(0))

        self.uhd_usrp_source_0_0.set_center_freq(center_freq, 0)
        self.uhd_usrp_source_0_0.set_antenna("RX2", 0)
        self.uhd_usrp_source_0_0.set_bandwidth(samp_rate, 0)
        self.uhd_usrp_source_0_0.set_normalized_gain(rx_gain, 0)
        self.prs_phase_slope_estimator_0 = ofdm_prs_ranging.prs_phase_slope_estimator(samp_rate, 1024, 600, 1.0)
        self.prs_frame_detector_0 = ofdm_prs_ranging.prs_frame_detector(samp_rate, 1024, 128, 600, 16, 128, 16, 839, 1000, 1000, 0.35, 10000)
        self.prs_fft_receiver_0 = ofdm_prs_ranging.prs_fft_receiver(samp_rate, 1024, 128, 600, 16)
        self.prs_csv_logger_0 = ofdm_prs_ranging.prs_csv_logger("prs_measurements.csv", 0)
        self.prs_channel_estimator_0 = ofdm_prs_ranging.prs_channel_estimator(samp_rate, 1024, 600, 16, 13990001)


        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.prs_channel_estimator_0, 'channel_out'), (self.prs_phase_slope_estimator_0, 'channel_in'))
        self.msg_connect((self.prs_fft_receiver_0, 'symbols_out'), (self.prs_channel_estimator_0, 'symbols_in'))
        self.msg_connect((self.prs_frame_detector_0, 'frame_out'), (self.prs_fft_receiver_0, 'frame_in'))
        self.msg_connect((self.prs_phase_slope_estimator_0, 'measurement_out'), (self.prs_csv_logger_0, 'measurement_in'))
        self.connect((self.uhd_usrp_source_0_0, 0), (self.prs_frame_detector_0, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("gnuradio/flowgraphs", "prs_rx_estimator_10M_1399")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.uhd_usrp_source_0_0.set_samp_rate(self.samp_rate)
        self.uhd_usrp_source_0_0.set_bandwidth(self.samp_rate, 0)

    def get_rx_gain(self):
        return self.rx_gain

    def set_rx_gain(self, rx_gain):
        self.rx_gain = rx_gain
        self.uhd_usrp_source_0_0.set_normalized_gain(self.rx_gain, 0)

    def get_center_freq(self):
        return self.center_freq

    def set_center_freq(self, center_freq):
        self.center_freq = center_freq
        self.uhd_usrp_source_0_0.set_center_freq(self.center_freq, 0)




def main(top_block_cls=prs_rx_estimator_10M_1399, options=None):

    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()

    tb.start()
    tb.flowgraph_started.set()

    tb.show()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        Qt.QApplication.quit()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    qapp.exec_()

if __name__ == '__main__':
    main()
