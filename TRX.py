#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: TRX 全波形收发回环 - 单台Pluto (信息波+红方1/2/3级干扰波+第二源信息波)
# Author: RMUC2026-NC
# Description: Full-wave single-NanoSDR loopback. TX: ZMQ(53101) -> 4 parallel GFSK modulators (info 540kHz@1.5628, int1 940kHz@2.8194, int2 860kHz@2.5681, int3 250kHz@0.6517) with web-controlled gates -> 4-input add -> Pluto Sink. RX: Pluto Source -> dual NCO branches: interfere branch (auto-adapting bw/gain via rx_int_bw) -> ZMQ 53104, info branch (540kHz) -> ZMQ 53103. XMLRPC server on 53106.
# GNU Radio version: 3.10.9.2

from PyQt5 import Qt
from gnuradio import qtgui
from gnuradio import analog
import math
from gnuradio import blocks
from gnuradio import digital
from gnuradio import filter
from gnuradio.filter import firdes
from gnuradio import gr
from gnuradio.fft import window
import sys
import signal
from PyQt5 import Qt
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import iio
from gnuradio import zeromq
from xmlrpc.server import SimpleXMLRPCServer
import threading
import sip



class TRX(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "TRX 全波形收发回环 - 单台Pluto (信息波+红方1/2/3级干扰波+第二源信息波)", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("TRX 全波形收发回环 - 单台Pluto (信息波+红方1/2/3级干扰波+第二源信息波)")
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

        self.settings = Qt.QSettings("GNU Radio", "TRX")

        try:
            geometry = self.settings.value("geometry")
            if geometry:
                self.restoreGeometry(geometry)
        except BaseException as exc:
            print(f"Qt GUI: Could not restore geometry: {str(exc)}", file=sys.stderr)

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 1000000
        self.rx_int_bw = rx_int_bw = 940000
        self.rx_info_bw = rx_info_bw = 540000
        self.tx_mod_int3_en = tx_mod_int3_en = 0.0
        self.tx_mod_int2_en = tx_mod_int2_en = 0.0
        self.tx_mod_int1_en = tx_mod_int1_en = 1.0
        self.tx_mod_info_en = tx_mod_info_en = 0.0
        self.tx_center_freq2 = tx_center_freq2 = 433200000
        self.tx_center_freq1 = tx_center_freq1 = 432200000
        self.tx_attenuation2 = tx_attenuation2 = 60
        self.tx_attenuation1 = tx_attenuation1 = 10
        self.sps = sps = 47
        self.sdr_uri1 = sdr_uri1 = 'ip:192.168.2.1'
        self.rx_interfere_channel_freq = rx_interfere_channel_freq = 432200000
        self.rx_int_fir_taps = rx_int_fir_taps = firdes.low_pass(1, samp_rate, rx_int_bw / 2, 20000)
        self.rx_info_fir_taps = rx_info_fir_taps = firdes.low_pass(1, samp_rate, rx_info_bw / 2, 20000)
        self.rx_info_channel_freq = rx_info_channel_freq = 433200000
        self.rx_gain = rx_gain = 40
        self.rx_center_freq = rx_center_freq = 432200000
        self.loop_bw = loop_bw = 0.005
        self.int3_bw = int3_bw = 250000
        self.int2_bw = int2_bw = 860000
        self.int1_bw = int1_bw = 940000
        self.info_bw = info_bw = 540000
        self.bw = bw = 1000000
        self.bt = bt = 0.35
        self.Time_Sink_Number = Time_Sink_Number = 200

        ##################################################
        # Blocks
        ##################################################

        self.zmq_int_bits_out = zeromq.pub_sink(gr.sizeof_char, 1, 'tcp://0.0.0.0:53104', 100, False, (-1), '', True, True)
        self.zmq_info_bits_out = zeromq.pub_sink(gr.sizeof_char, 1, 'tcp://0.0.0.0:53103', 100, False, (-1), '', True, True)
        self.zmq_bits_in_2nd = zeromq.sub_source(gr.sizeof_char, 1, 'tcp://127.0.0.1:53102', 100, False, (-1), '', False)
        self.zmq_bits_in = zeromq.sub_source(gr.sizeof_char, 1, 'tcp://127.0.0.1:53101', 100, False, (-1), '', False)
        self.xmlrpc_server = SimpleXMLRPCServer(('0.0.0.0', 53106), allow_none=True)
        self.xmlrpc_server.register_instance(self)
        self.xmlrpc_server_thread = threading.Thread(target=self.xmlrpc_server.serve_forever)
        self.xmlrpc_server_thread.daemon = True
        self.xmlrpc_server_thread.start()
        self.tx_wave_add = blocks.add_vcc(1)
        self.tx_mod_int3_gate = blocks.multiply_const_cc(tx_mod_int3_en)
        self.tx_mod_int3 = digital.gfsk_mod(
            samples_per_symbol=sps,
            sensitivity=(2*3.141592653589793*((int3_bw/2)-(samp_rate/sps))/samp_rate),
            bt=bt,
            verbose=False,
            log=False,
            do_unpack=True)
        self.tx_mod_int2_gate = blocks.multiply_const_cc(tx_mod_int2_en)
        self.tx_mod_int2 = digital.gfsk_mod(
            samples_per_symbol=sps,
            sensitivity=(2*3.141592653589793*((int2_bw/2)-(samp_rate/sps))/samp_rate),
            bt=bt,
            verbose=False,
            log=False,
            do_unpack=True)
        self.tx_mod_int1_gate = blocks.multiply_const_cc(tx_mod_int1_en)
        self.tx_mod_int1 = digital.gfsk_mod(
            samples_per_symbol=sps,
            sensitivity=(2*3.141592653589793*((int1_bw/2)-(samp_rate/sps))/samp_rate),
            bt=bt,
            verbose=False,
            log=False,
            do_unpack=True)
        self.tx_mod_info_gate = blocks.multiply_const_cc(tx_mod_info_en)
        self.tx_mod_info_2nd = digital.gfsk_mod(
            samples_per_symbol=sps,
            sensitivity=(2*3.141592653589793*((info_bw/2)-(samp_rate/sps))/samp_rate),
            bt=bt,
            verbose=False,
            log=False,
            do_unpack=True)
        self.tx_mod_info = digital.gfsk_mod(
            samples_per_symbol=sps,
            sensitivity=(2*3.141592653589793*((info_bw/2)-(samp_rate/sps))/samp_rate),
            bt=bt,
            verbose=False,
            log=False,
            do_unpack=True)
        self.rx_int_sync = digital.symbol_sync_ff(
            digital.TED_ZERO_CROSSING,
            sps,
            loop_bw,
            1.0,
            1.0,
            1.5,
            1,
            digital.constellation_bpsk().base(),
            digital.IR_MMSE_8TAP,
            128,
            [])
        self.rx_int_slicer = digital.binary_slicer_fb()
        self.rx_int_shift = blocks.multiply_conjugate_cc(1)
        self.rx_int_rail = analog.rail_ff((-1.2), 1.2)
        self.rx_int_quad = analog.quadrature_demod_cf((1/(2*3.141592653589793*((rx_int_bw/2)-(samp_rate/sps))/samp_rate)))
        self.rx_int_nco = analog.sig_source_c(samp_rate, analog.GR_COS_WAVE, (rx_interfere_channel_freq - rx_center_freq), 1, 0, 0)
        self.rx_int_gauss = filter.fir_filter_fff(1, firdes.gaussian(1.0, sps, bt, int(3*sps) + 1))
        self.rx_int_gauss.declare_sample_delay(0)
        self.rx_int_filter = filter.fir_filter_ccc(1, rx_int_fir_taps)
        self.rx_int_filter.declare_sample_delay(0)
        self.rx_int_dc_iir = filter.single_pole_iir_filter_ff((1e-8), 1)
        self.rx_int_dc_cancel = blocks.sub_ff(1)
        self.rx_int_agc = analog.agc_cc((1e-5), 0.5, 1.0, 65536)
        self.rx_info_sync = digital.symbol_sync_ff(
            digital.TED_ZERO_CROSSING,
            sps,
            loop_bw,
            1.0,
            1.0,
            1.5,
            1,
            digital.constellation_bpsk().base(),
            digital.IR_MMSE_8TAP,
            128,
            [])
        self.rx_info_slicer = digital.binary_slicer_fb()
        self.rx_info_shift = blocks.multiply_conjugate_cc(1)
        self.rx_info_rail = analog.rail_ff((-1.2), 1.2)
        self.rx_info_quad = analog.quadrature_demod_cf((1/(2*3.141592653589793*((rx_info_bw/2)-(samp_rate/sps))/samp_rate)))
        self.rx_info_nco = analog.sig_source_c(samp_rate, analog.GR_COS_WAVE, (rx_info_channel_freq - rx_center_freq), 1, 0, 0)
        self.rx_info_gauss = filter.fir_filter_fff(1, firdes.gaussian(1.0, sps, bt, int(3*sps) + 1))
        self.rx_info_gauss.declare_sample_delay(0)
        self.rx_info_filter = filter.fir_filter_ccc(1, rx_info_fir_taps)
        self.rx_info_filter.declare_sample_delay(0)
        self.rx_info_dc_iir = filter.single_pole_iir_filter_ff((1e-8), 1)
        self.rx_info_dc_cancel = blocks.sub_ff(1)
        self.rx_info_agc2 = analog.agc_cc((1e-5), 0.5, 1.0, 65536)
        self.nanosdr_source = iio.fmcomms2_source_fc32(sdr_uri1 if sdr_uri1 else iio.get_pluto_uri(), [True, True], 32768)
        self.nanosdr_source.set_len_tag_key('packet_len')
        self.nanosdr_source.set_frequency(rx_center_freq)
        self.nanosdr_source.set_samplerate(samp_rate)
        self.nanosdr_source.set_gain_mode(0, 'manual')
        self.nanosdr_source.set_gain(0, rx_gain)
        self.nanosdr_source.set_quadrature(True)
        self.nanosdr_source.set_rfdc(True)
        self.nanosdr_source.set_bbdc(True)
        self.nanosdr_source.set_filter_params('Auto', '', 0, 0)
        self.nanosdr_sink = iio.fmcomms2_sink_fc32(sdr_uri1 if sdr_uri1 else iio.get_pluto_uri(), [True, True], 32768, False)
        self.nanosdr_sink.set_len_tag_key('')
        self.nanosdr_sink.set_bandwidth(samp_rate)
        self.nanosdr_sink.set_frequency(tx_center_freq1)
        self.nanosdr_sink.set_samplerate(samp_rate)
        self.nanosdr_sink.set_attenuation(0, tx_attenuation1)
        self.nanosdr_sink.set_filter_params('Auto', '', 0, 0)
        self.gui_tx_freq = qtgui.freq_sink_c(
            1024, #size
            window.WIN_BLACKMAN_hARRIS, #wintype
            0, #fc
            samp_rate, #bw
            'TX spectrum (sum of gated waves)', #name
            1,
            None # parent
        )
        self.gui_tx_freq.set_update_time(0.10)
        self.gui_tx_freq.set_y_axis((-120), 10)
        self.gui_tx_freq.set_y_label('Relative Gain', 'dB')
        self.gui_tx_freq.set_trigger_mode(qtgui.TRIG_MODE_FREE, 0.0, 0, "")
        self.gui_tx_freq.enable_autoscale(False)
        self.gui_tx_freq.enable_grid(True)
        self.gui_tx_freq.set_fft_average(1.0)
        self.gui_tx_freq.enable_axis_labels(True)
        self.gui_tx_freq.enable_control_panel(False)
        self.gui_tx_freq.set_fft_window_normalized(False)



        labels = ['', '', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
            "magenta", "yellow", "dark red", "dark green", "dark blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.gui_tx_freq.set_line_label(i, "Data {0}".format(i))
            else:
                self.gui_tx_freq.set_line_label(i, labels[i])
            self.gui_tx_freq.set_line_width(i, widths[i])
            self.gui_tx_freq.set_line_color(i, colors[i])
            self.gui_tx_freq.set_line_alpha(i, alphas[i])

        self._gui_tx_freq_win = sip.wrapinstance(self.gui_tx_freq.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._gui_tx_freq_win)
        self.gui_int_freq = qtgui.freq_sink_c(
            1024, #size
            window.WIN_BLACKMAN_hARRIS, #wintype
            0, #fc
            samp_rate, #bw
            'RX interfere spectrum', #name
            1,
            None # parent
        )
        self.gui_int_freq.set_update_time(0.10)
        self.gui_int_freq.set_y_axis((-120), 10)
        self.gui_int_freq.set_y_label('Relative Gain', 'dB')
        self.gui_int_freq.set_trigger_mode(qtgui.TRIG_MODE_FREE, 0.0, 0, "")
        self.gui_int_freq.enable_autoscale(False)
        self.gui_int_freq.enable_grid(True)
        self.gui_int_freq.set_fft_average(1.0)
        self.gui_int_freq.enable_axis_labels(True)
        self.gui_int_freq.enable_control_panel(False)
        self.gui_int_freq.set_fft_window_normalized(False)



        labels = ['', '', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
            "magenta", "yellow", "dark red", "dark green", "dark blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.gui_int_freq.set_line_label(i, "Data {0}".format(i))
            else:
                self.gui_int_freq.set_line_label(i, labels[i])
            self.gui_int_freq.set_line_width(i, widths[i])
            self.gui_int_freq.set_line_color(i, colors[i])
            self.gui_int_freq.set_line_alpha(i, alphas[i])

        self._gui_int_freq_win = sip.wrapinstance(self.gui_int_freq.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._gui_int_freq_win)
        self.gui_int_demod = qtgui.time_sink_f(
            (Time_Sink_Number*sps), #size
            samp_rate, #samp_rate
            'Interfere FM discriminator', #name
            1, #number of inputs
            None # parent
        )
        self.gui_int_demod.set_update_time(0.10)
        self.gui_int_demod.set_y_axis(-1.5, 1.5)

        self.gui_int_demod.set_y_label('Amplitude', "")

        self.gui_int_demod.enable_tags(False)
        self.gui_int_demod.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        self.gui_int_demod.enable_autoscale(False)
        self.gui_int_demod.enable_grid(True)
        self.gui_int_demod.enable_axis_labels(True)
        self.gui_int_demod.enable_control_panel(False)
        self.gui_int_demod.enable_stem_plot(False)


        labels = ['Quad demod', 'Gaussian match', 'Energy discriminator', 'Energy -0.5fdev %', 'Energy +0.5fdev %',
            'Energy +fdev %', 'Signal 7', 'Signal 8', 'Signal 9', 'Signal 10']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ['blue', 'red', 'green', 'black', 'cyan',
            'magenta', 'yellow', 'dark red', 'dark green', 'dark blue']
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]
        styles = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        markers = [-1, -1, -1, -1, -1,
            -1, -1, -1, -1, -1]


        for i in range(1):
            if len(labels[i]) == 0:
                self.gui_int_demod.set_line_label(i, "Data {0}".format(i))
            else:
                self.gui_int_demod.set_line_label(i, labels[i])
            self.gui_int_demod.set_line_width(i, widths[i])
            self.gui_int_demod.set_line_color(i, colors[i])
            self.gui_int_demod.set_line_style(i, styles[i])
            self.gui_int_demod.set_line_marker(i, markers[i])
            self.gui_int_demod.set_line_alpha(i, alphas[i])

        self._gui_int_demod_win = sip.wrapinstance(self.gui_int_demod.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._gui_int_demod_win)
        self.gui_int_bits = qtgui.time_sink_f(
            Time_Sink_Number, #size
            samp_rate/sps, #samp_rate
            'Interfere sync output', #name
            1, #number of inputs
            None # parent
        )
        self.gui_int_bits.set_update_time(0.10)
        self.gui_int_bits.set_y_axis(-1.5, 1.5)

        self.gui_int_bits.set_y_label('Amplitude', "")

        self.gui_int_bits.enable_tags(False)
        self.gui_int_bits.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        self.gui_int_bits.enable_autoscale(False)
        self.gui_int_bits.enable_grid(True)
        self.gui_int_bits.enable_axis_labels(True)
        self.gui_int_bits.enable_control_panel(False)
        self.gui_int_bits.enable_stem_plot(False)


        labels = ['Signal 1', 'Signal 2', 'Signal 3', 'Signal 4', 'Signal 5',
            'Signal 6', 'Signal 7', 'Signal 8', 'Signal 9', 'Signal 10']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ['blue', 'red', 'green', 'black', 'cyan',
            'magenta', 'yellow', 'dark red', 'dark green', 'dark blue']
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]
        styles = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        markers = [-1, -1, -1, -1, -1,
            -1, -1, -1, -1, -1]


        for i in range(1):
            if len(labels[i]) == 0:
                self.gui_int_bits.set_line_label(i, "Data {0}".format(i))
            else:
                self.gui_int_bits.set_line_label(i, labels[i])
            self.gui_int_bits.set_line_width(i, widths[i])
            self.gui_int_bits.set_line_color(i, colors[i])
            self.gui_int_bits.set_line_style(i, styles[i])
            self.gui_int_bits.set_line_marker(i, markers[i])
            self.gui_int_bits.set_line_alpha(i, alphas[i])

        self._gui_int_bits_win = sip.wrapinstance(self.gui_int_bits.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._gui_int_bits_win)
        self.gui_info_freq = qtgui.freq_sink_c(
            1024, #size
            window.WIN_BLACKMAN_hARRIS, #wintype
            0, #fc
            samp_rate, #bw
            'RX info spectrum', #name
            1,
            None # parent
        )
        self.gui_info_freq.set_update_time(0.10)
        self.gui_info_freq.set_y_axis((-120), 10)
        self.gui_info_freq.set_y_label('Relative Gain', 'dB')
        self.gui_info_freq.set_trigger_mode(qtgui.TRIG_MODE_FREE, 0.0, 0, "")
        self.gui_info_freq.enable_autoscale(False)
        self.gui_info_freq.enable_grid(True)
        self.gui_info_freq.set_fft_average(1.0)
        self.gui_info_freq.enable_axis_labels(True)
        self.gui_info_freq.enable_control_panel(False)
        self.gui_info_freq.set_fft_window_normalized(False)



        labels = ['', '', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
            "magenta", "yellow", "dark red", "dark green", "dark blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.gui_info_freq.set_line_label(i, "Data {0}".format(i))
            else:
                self.gui_info_freq.set_line_label(i, labels[i])
            self.gui_info_freq.set_line_width(i, widths[i])
            self.gui_info_freq.set_line_color(i, colors[i])
            self.gui_info_freq.set_line_alpha(i, alphas[i])

        self._gui_info_freq_win = sip.wrapinstance(self.gui_info_freq.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._gui_info_freq_win)
        self.gui_info_bits = qtgui.time_sink_f(
            Time_Sink_Number, #size
            samp_rate/sps, #samp_rate
            'Info sync output', #name
            1, #number of inputs
            None # parent
        )
        self.gui_info_bits.set_update_time(0.10)
        self.gui_info_bits.set_y_axis(-1.5, 1.5)

        self.gui_info_bits.set_y_label('Amplitude', "")

        self.gui_info_bits.enable_tags(False)
        self.gui_info_bits.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        self.gui_info_bits.enable_autoscale(False)
        self.gui_info_bits.enable_grid(True)
        self.gui_info_bits.enable_axis_labels(True)
        self.gui_info_bits.enable_control_panel(False)
        self.gui_info_bits.enable_stem_plot(False)


        labels = ['Signal 1', 'Signal 2', 'Signal 3', 'Signal 4', 'Signal 5',
            'Signal 6', 'Signal 7', 'Signal 8', 'Signal 9', 'Signal 10']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ['blue', 'red', 'green', 'black', 'cyan',
            'magenta', 'yellow', 'dark red', 'dark green', 'dark blue']
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]
        styles = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        markers = [-1, -1, -1, -1, -1,
            -1, -1, -1, -1, -1]


        for i in range(1):
            if len(labels[i]) == 0:
                self.gui_info_bits.set_line_label(i, "Data {0}".format(i))
            else:
                self.gui_info_bits.set_line_label(i, labels[i])
            self.gui_info_bits.set_line_width(i, widths[i])
            self.gui_info_bits.set_line_color(i, colors[i])
            self.gui_info_bits.set_line_style(i, styles[i])
            self.gui_info_bits.set_line_marker(i, markers[i])
            self.gui_info_bits.set_line_alpha(i, alphas[i])

        self._gui_info_bits_win = sip.wrapinstance(self.gui_info_bits.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._gui_info_bits_win)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.nanosdr_source, 0), (self.rx_info_shift, 0))
        self.connect((self.nanosdr_source, 0), (self.rx_int_shift, 0))
        self.connect((self.rx_info_agc2, 0), (self.rx_info_quad, 0))
        self.connect((self.rx_info_dc_cancel, 0), (self.rx_info_sync, 0))
        self.connect((self.rx_info_dc_iir, 0), (self.rx_info_dc_cancel, 1))
        self.connect((self.rx_info_filter, 0), (self.rx_info_agc2, 0))
        self.connect((self.rx_info_gauss, 0), (self.rx_info_dc_cancel, 0))
        self.connect((self.rx_info_gauss, 0), (self.rx_info_dc_iir, 0))
        self.connect((self.rx_info_nco, 0), (self.rx_info_shift, 1))
        self.connect((self.rx_info_quad, 0), (self.rx_info_rail, 0))
        self.connect((self.rx_info_rail, 0), (self.rx_info_gauss, 0))
        self.connect((self.rx_info_shift, 0), (self.gui_info_freq, 0))
        self.connect((self.rx_info_shift, 0), (self.rx_info_filter, 0))
        self.connect((self.rx_info_slicer, 0), (self.zmq_info_bits_out, 0))
        self.connect((self.rx_info_sync, 0), (self.gui_info_bits, 0))
        self.connect((self.rx_info_sync, 0), (self.rx_info_slicer, 0))
        self.connect((self.rx_int_agc, 0), (self.rx_int_quad, 0))
        self.connect((self.rx_int_dc_cancel, 0), (self.rx_int_sync, 0))
        self.connect((self.rx_int_dc_iir, 0), (self.rx_int_dc_cancel, 1))
        self.connect((self.rx_int_filter, 0), (self.rx_int_agc, 0))
        self.connect((self.rx_int_gauss, 0), (self.rx_int_dc_cancel, 0))
        self.connect((self.rx_int_gauss, 0), (self.rx_int_dc_iir, 0))
        self.connect((self.rx_int_nco, 0), (self.rx_int_shift, 1))
        self.connect((self.rx_int_quad, 0), (self.rx_int_rail, 0))
        self.connect((self.rx_int_rail, 0), (self.gui_int_demod, 0))
        self.connect((self.rx_int_rail, 0), (self.rx_int_gauss, 0))
        self.connect((self.rx_int_shift, 0), (self.gui_int_freq, 0))
        self.connect((self.rx_int_shift, 0), (self.rx_int_filter, 0))
        self.connect((self.rx_int_slicer, 0), (self.zmq_int_bits_out, 0))
        self.connect((self.rx_int_sync, 0), (self.gui_int_bits, 0))
        self.connect((self.rx_int_sync, 0), (self.rx_int_slicer, 0))
        self.connect((self.tx_mod_info, 0), (self.tx_mod_info_gate, 0))
        self.connect((self.tx_mod_info_2nd, 0), (self.tx_wave_add, 4))
        self.connect((self.tx_mod_info_gate, 0), (self.tx_wave_add, 0))
        self.connect((self.tx_mod_int1, 0), (self.tx_mod_int1_gate, 0))
        self.connect((self.tx_mod_int1_gate, 0), (self.tx_wave_add, 1))
        self.connect((self.tx_mod_int2, 0), (self.tx_mod_int2_gate, 0))
        self.connect((self.tx_mod_int2_gate, 0), (self.tx_wave_add, 2))
        self.connect((self.tx_mod_int3, 0), (self.tx_mod_int3_gate, 0))
        self.connect((self.tx_mod_int3_gate, 0), (self.tx_wave_add, 3))
        self.connect((self.tx_wave_add, 0), (self.gui_tx_freq, 0))
        self.connect((self.tx_wave_add, 0), (self.nanosdr_sink, 0))
        self.connect((self.zmq_bits_in, 0), (self.tx_mod_info, 0))
        self.connect((self.zmq_bits_in, 0), (self.tx_mod_int1, 0))
        self.connect((self.zmq_bits_in, 0), (self.tx_mod_int2, 0))
        self.connect((self.zmq_bits_in, 0), (self.tx_mod_int3, 0))
        self.connect((self.zmq_bits_in_2nd, 0), (self.tx_mod_info_2nd, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "TRX")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_rx_int_fir_taps(firdes.low_pass(1, self.samp_rate, self.rx_int_bw / 2, 20000))
        self.set_rx_info_fir_taps(firdes.low_pass(1, self.samp_rate, self.rx_info_bw / 2, 20000))
        self.nanosdr_sink.set_bandwidth(self.samp_rate)
        self.nanosdr_sink.set_samplerate(self.samp_rate)
        self.nanosdr_source.set_samplerate(self.samp_rate)
        self.rx_int_nco.set_sampling_freq(self.samp_rate)
        self.rx_int_quad.set_gain((1/(2*3.141592653589793*((self.rx_int_bw/2)-(self.samp_rate/self.sps))/self.samp_rate)))
        self.rx_info_nco.set_sampling_freq(self.samp_rate)
        self.rx_info_quad.set_gain((1/(2*3.141592653589793*((self.rx_info_bw/2)-(self.samp_rate/self.sps))/self.samp_rate)))
        self.gui_tx_freq.set_frequency_range(0, self.samp_rate)
        self.gui_int_freq.set_frequency_range(0, self.samp_rate)
        self.gui_info_freq.set_frequency_range(0, self.samp_rate)
        self.gui_int_demod.set_samp_rate(self.samp_rate)
        self.gui_int_bits.set_samp_rate(self.samp_rate/self.sps)
        self.gui_info_bits.set_samp_rate(self.samp_rate/self.sps)

    def get_rx_int_bw(self):
        return self.rx_int_bw

    def set_rx_int_bw(self, rx_int_bw):
        self.rx_int_bw = rx_int_bw
        self.set_rx_int_fir_taps(firdes.low_pass(1, self.samp_rate, self.rx_int_bw / 2, 20000))
        self.rx_int_quad.set_gain((1/(2*3.141592653589793*((self.rx_int_bw/2)-(self.samp_rate/self.sps))/self.samp_rate)))

    def get_rx_info_bw(self):
        return self.rx_info_bw

    def set_rx_info_bw(self, rx_info_bw):
        self.rx_info_bw = rx_info_bw
        self.set_rx_info_fir_taps(firdes.low_pass(1, self.samp_rate, self.rx_info_bw / 2, 20000))
        self.rx_info_quad.set_gain((1/(2*3.141592653589793*((self.rx_info_bw/2)-(self.samp_rate/self.sps))/self.samp_rate)))

    def get_tx_mod_int3_en(self):
        return self.tx_mod_int3_en

    def set_tx_mod_int3_en(self, tx_mod_int3_en):
        self.tx_mod_int3_en = tx_mod_int3_en
        self.tx_mod_int3_gate.set_k(self.tx_mod_int3_en)

    def get_tx_mod_int2_en(self):
        return self.tx_mod_int2_en

    def set_tx_mod_int2_en(self, tx_mod_int2_en):
        self.tx_mod_int2_en = tx_mod_int2_en
        self.tx_mod_int2_gate.set_k(self.tx_mod_int2_en)

    def get_tx_mod_int1_en(self):
        return self.tx_mod_int1_en

    def set_tx_mod_int1_en(self, tx_mod_int1_en):
        self.tx_mod_int1_en = tx_mod_int1_en
        self.tx_mod_int1_gate.set_k(self.tx_mod_int1_en)

    def get_tx_mod_info_en(self):
        return self.tx_mod_info_en

    def set_tx_mod_info_en(self, tx_mod_info_en):
        self.tx_mod_info_en = tx_mod_info_en
        self.tx_mod_info_gate.set_k(self.tx_mod_info_en)

    def get_tx_center_freq2(self):
        return self.tx_center_freq2

    def set_tx_center_freq2(self, tx_center_freq2):
        self.tx_center_freq2 = tx_center_freq2

    def get_tx_center_freq1(self):
        return self.tx_center_freq1

    def set_tx_center_freq1(self, tx_center_freq1):
        self.tx_center_freq1 = tx_center_freq1
        self.nanosdr_sink.set_frequency(self.tx_center_freq1)

    def get_tx_attenuation2(self):
        return self.tx_attenuation2

    def set_tx_attenuation2(self, tx_attenuation2):
        self.tx_attenuation2 = tx_attenuation2

    def get_tx_attenuation1(self):
        return self.tx_attenuation1

    def set_tx_attenuation1(self, tx_attenuation1):
        self.tx_attenuation1 = tx_attenuation1
        self.nanosdr_sink.set_attenuation(0,self.tx_attenuation1)

    def get_sps(self):
        return self.sps

    def set_sps(self, sps):
        self.sps = sps
        self.rx_int_quad.set_gain((1/(2*3.141592653589793*((self.rx_int_bw/2)-(self.samp_rate/self.sps))/self.samp_rate)))
        self.rx_int_gauss.set_taps(firdes.gaussian(1.0, self.sps, self.bt, int(3*self.sps) + 1))
        self.rx_int_sync.set_sps(self.sps)
        self.rx_info_quad.set_gain((1/(2*3.141592653589793*((self.rx_info_bw/2)-(self.samp_rate/self.sps))/self.samp_rate)))
        self.rx_info_gauss.set_taps(firdes.gaussian(1.0, self.sps, self.bt, int(3*self.sps) + 1))
        self.rx_info_sync.set_sps(self.sps)
        self.gui_int_bits.set_samp_rate(self.samp_rate/self.sps)
        self.gui_info_bits.set_samp_rate(self.samp_rate/self.sps)

    def get_sdr_uri1(self):
        return self.sdr_uri1

    def set_sdr_uri1(self, sdr_uri1):
        self.sdr_uri1 = sdr_uri1

    def get_rx_interfere_channel_freq(self):
        return self.rx_interfere_channel_freq

    def set_rx_interfere_channel_freq(self, rx_interfere_channel_freq):
        self.rx_interfere_channel_freq = rx_interfere_channel_freq
        self.rx_int_nco.set_frequency((self.rx_interfere_channel_freq - self.rx_center_freq))

    def get_rx_int_fir_taps(self):
        return self.rx_int_fir_taps

    def set_rx_int_fir_taps(self, rx_int_fir_taps):
        self.rx_int_fir_taps = rx_int_fir_taps
        self.rx_int_filter.set_taps(self.rx_int_fir_taps)

    def get_rx_info_fir_taps(self):
        return self.rx_info_fir_taps

    def set_rx_info_fir_taps(self, rx_info_fir_taps):
        self.rx_info_fir_taps = rx_info_fir_taps
        self.rx_info_filter.set_taps(self.rx_info_fir_taps)

    def get_rx_info_channel_freq(self):
        return self.rx_info_channel_freq

    def set_rx_info_channel_freq(self, rx_info_channel_freq):
        self.rx_info_channel_freq = rx_info_channel_freq
        self.rx_info_nco.set_frequency((self.rx_info_channel_freq - self.rx_center_freq))

    def get_rx_gain(self):
        return self.rx_gain

    def set_rx_gain(self, rx_gain):
        self.rx_gain = rx_gain
        self.nanosdr_source.set_gain(0, self.rx_gain)

    def get_rx_center_freq(self):
        return self.rx_center_freq

    def set_rx_center_freq(self, rx_center_freq):
        self.rx_center_freq = rx_center_freq
        self.nanosdr_source.set_frequency(self.rx_center_freq)
        self.rx_int_nco.set_frequency((self.rx_interfere_channel_freq - self.rx_center_freq))
        self.rx_info_nco.set_frequency((self.rx_info_channel_freq - self.rx_center_freq))

    def get_loop_bw(self):
        return self.loop_bw

    def set_loop_bw(self, loop_bw):
        self.loop_bw = loop_bw
        self.rx_int_sync.set_loop_bandwidth(self.loop_bw)
        self.rx_info_sync.set_loop_bandwidth(self.loop_bw)

    def get_int3_bw(self):
        return self.int3_bw

    def set_int3_bw(self, int3_bw):
        self.int3_bw = int3_bw

    def get_int2_bw(self):
        return self.int2_bw

    def set_int2_bw(self, int2_bw):
        self.int2_bw = int2_bw

    def get_int1_bw(self):
        return self.int1_bw

    def set_int1_bw(self, int1_bw):
        self.int1_bw = int1_bw

    def get_info_bw(self):
        return self.info_bw

    def set_info_bw(self, info_bw):
        self.info_bw = info_bw

    def get_bw(self):
        return self.bw

    def set_bw(self, bw):
        self.bw = bw

    def get_bt(self):
        return self.bt

    def set_bt(self, bt):
        self.bt = bt
        self.rx_int_gauss.set_taps(firdes.gaussian(1.0, self.sps, self.bt, int(3*self.sps) + 1))
        self.rx_info_gauss.set_taps(firdes.gaussian(1.0, self.sps, self.bt, int(3*self.sps) + 1))

    def get_Time_Sink_Number(self):
        return self.Time_Sink_Number

    def set_Time_Sink_Number(self, Time_Sink_Number):
        self.Time_Sink_Number = Time_Sink_Number




def main(top_block_cls=TRX, options=None):

    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()

    tb.start()

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
