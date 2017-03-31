#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Usrp Energy Detector
# Generated: Thu Mar 23 19:47:22 2017
##################################################

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print "Warning: failed to XInitThreads()"

from PyQt4 import Qt
from gnuradio import blocks
from gnuradio import digital
from gnuradio import eng_notation
from gnuradio import filter
from gnuradio import gr
from gnuradio import qtgui
from gnuradio import uhd
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
import lsa
import pmt
import sip
import sys
import time
from gnuradio import qtgui


class usrp_energy_detector(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Usrp Energy Detector")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Usrp Energy Detector")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except:
            pass
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

        self.settings = Qt.QSettings("GNU Radio", "usrp_energy_detector")
        self.restoreGeometry(self.settings.value("geometry").toByteArray())

        ##################################################
        # Variables
        ##################################################

        self.mod = mod = digital.constellation_calcdist(([-1-1j, -1+1j, 1+1j, 1-1j]), ([0, 1, 3, 2]), 4, 1).base()

        self.bps = bps = mod.bits_per_symbol()
        self.accesscode = accesscode = digital.packet_utils.default_access_code
        self.acc_bits = acc_bits = [int(accesscode[i:i+bps],bps) for i in range(0,len(accesscode),bps)]
        self.sps = sps = 4
        self.pre_code = pre_code = [mod.pre_diff_code()[acc_bits[i]] for i in range(0,len(acc_bits))]
        self.nfilts = nfilts = 32

        self.tx_rrc_taps = tx_rrc_taps = firdes.root_raised_cosine(sps, sps, 1.0, 0.35, 11*sps)

        self.tx_gain = tx_gain = 6
        self.tx_addr = tx_addr = "addr=192.168.10.2"
        self.samp_rate = samp_rate = 2e6

        self.rx_rrc_taps = rx_rrc_taps = firdes.root_raised_cosine(nfilts, nfilts*sps, 1.0, 0.35, 11*sps*nfilts)

        self.rx_gain = rx_gain = 6
        self.rx_addr = rx_addr = "addr=192.168.10.3"
        self.pre_s = pre_s = [mod.points()[pre_code[i]] for i in range(0,len(pre_code)) ]
        self.freq = freq = 2.4e9

        ##################################################
        # Blocks
        ##################################################
        self.rx_tab = Qt.QTabWidget()
        self.rx_tab_widget_0 = Qt.QWidget()
        self.rx_tab_layout_0 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.rx_tab_widget_0)
        self.rx_tab_grid_layout_0 = Qt.QGridLayout()
        self.rx_tab_layout_0.addLayout(self.rx_tab_grid_layout_0)
        self.rx_tab.addTab(self.rx_tab_widget_0, 'Spectrum')
        self.rx_tab_widget_1 = Qt.QWidget()
        self.rx_tab_layout_1 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.rx_tab_widget_1)
        self.rx_tab_grid_layout_1 = Qt.QGridLayout()
        self.rx_tab_layout_1.addLayout(self.rx_tab_grid_layout_1)
        self.rx_tab.addTab(self.rx_tab_widget_1, 'Symbol')
        self.rx_tab_widget_2 = Qt.QWidget()
        self.rx_tab_layout_2 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.rx_tab_widget_2)
        self.rx_tab_grid_layout_2 = Qt.QGridLayout()
        self.rx_tab_layout_2.addLayout(self.rx_tab_grid_layout_2)
        self.rx_tab.addTab(self.rx_tab_widget_2, 'Sync')
        self.top_layout.addWidget(self.rx_tab)
        self.uhd_usrp_source_0 = uhd.usrp_source(
        	",".join((rx_addr, "")),
        	uhd.stream_args(
        		cpu_format="fc32",
        		channels=range(1),
        	),
        )
        self.uhd_usrp_source_0.set_clock_source('internal', 0)
        self.uhd_usrp_source_0.set_subdev_spec('A:0', 0)
        self.uhd_usrp_source_0.set_samp_rate(samp_rate)
        self.uhd_usrp_source_0.set_center_freq(freq, 0)
        self.uhd_usrp_source_0.set_gain(rx_gain, 0)
        self.uhd_usrp_source_0.set_antenna('RX2', 0)
        self.uhd_usrp_source_0.set_bandwidth(samp_rate, 0)
        self.uhd_usrp_sink_0 = uhd.usrp_sink(
        	",".join((tx_addr, "")),
        	uhd.stream_args(
        		cpu_format="fc32",
        		channels=range(1),
        	),
        )
        self.uhd_usrp_sink_0.set_clock_source('internal', 0)
        self.uhd_usrp_sink_0.set_subdev_spec('A:0', 0)
        self.uhd_usrp_sink_0.set_samp_rate(samp_rate)
        self.uhd_usrp_sink_0.set_center_freq(freq, 0)
        self.uhd_usrp_sink_0.set_gain(tx_gain, 0)
        self.uhd_usrp_sink_0.set_antenna('TX/RX', 0)
        self.uhd_usrp_sink_0.set_bandwidth(samp_rate, 0)
        self.qtgui_time_sink_x_1 = qtgui.time_sink_f(
        	10000, #size
        	samp_rate, #samp_rate
        	"", #name
        	1 #number of inputs
        )
        self.qtgui_time_sink_x_1.set_update_time(0.10)
        self.qtgui_time_sink_x_1.set_y_axis(-1, 1)

        self.qtgui_time_sink_x_1.set_y_label('Amplitude', "")

        self.qtgui_time_sink_x_1.enable_tags(-1, True)
        self.qtgui_time_sink_x_1.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        self.qtgui_time_sink_x_1.enable_autoscale(False)
        self.qtgui_time_sink_x_1.enable_grid(False)
        self.qtgui_time_sink_x_1.enable_axis_labels(True)
        self.qtgui_time_sink_x_1.enable_control_panel(False)

        if not True:
          self.qtgui_time_sink_x_1.disable_legend()

        labels = ['', '', '', '', '',
                  '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
                  "magenta", "yellow", "dark red", "dark green", "blue"]
        styles = [1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1]
        markers = [-1, -1, -1, -1, -1,
                   -1, -1, -1, -1, -1]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
                  1.0, 1.0, 1.0, 1.0, 1.0]

        for i in xrange(1):
            if len(labels[i]) == 0:
                self.qtgui_time_sink_x_1.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_time_sink_x_1.set_line_label(i, labels[i])
            self.qtgui_time_sink_x_1.set_line_width(i, widths[i])
            self.qtgui_time_sink_x_1.set_line_color(i, colors[i])
            self.qtgui_time_sink_x_1.set_line_style(i, styles[i])
            self.qtgui_time_sink_x_1.set_line_marker(i, markers[i])
            self.qtgui_time_sink_x_1.set_line_alpha(i, alphas[i])

        self._qtgui_time_sink_x_1_win = sip.wrapinstance(self.qtgui_time_sink_x_1.pyqwidget(), Qt.QWidget)
        self.rx_tab_layout_2.addWidget(self._qtgui_time_sink_x_1_win)
        self.qtgui_time_sink_x_0 = qtgui.time_sink_f(
        	10000, #size
        	samp_rate, #samp_rate
        	"", #name
        	1 #number of inputs
        )
        self.qtgui_time_sink_x_0.set_update_time(0.10)
        self.qtgui_time_sink_x_0.set_y_axis(-100, 0)

        self.qtgui_time_sink_x_0.set_y_label('Amplitude', "")

        self.qtgui_time_sink_x_0.enable_tags(-1, True)
        self.qtgui_time_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        self.qtgui_time_sink_x_0.enable_autoscale(False)
        self.qtgui_time_sink_x_0.enable_grid(False)
        self.qtgui_time_sink_x_0.enable_axis_labels(True)
        self.qtgui_time_sink_x_0.enable_control_panel(False)

        if not True:
          self.qtgui_time_sink_x_0.disable_legend()

        labels = ['', '', '', '', '',
                  '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
                  "magenta", "yellow", "dark red", "dark green", "blue"]
        styles = [1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1]
        markers = [-1, -1, -1, -1, -1,
                   -1, -1, -1, -1, -1]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
                  1.0, 1.0, 1.0, 1.0, 1.0]

        for i in xrange(1):
            if len(labels[i]) == 0:
                self.qtgui_time_sink_x_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_time_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_time_sink_x_0.set_line_width(i, widths[i])
            self.qtgui_time_sink_x_0.set_line_color(i, colors[i])
            self.qtgui_time_sink_x_0.set_line_style(i, styles[i])
            self.qtgui_time_sink_x_0.set_line_marker(i, markers[i])
            self.qtgui_time_sink_x_0.set_line_alpha(i, alphas[i])

        self._qtgui_time_sink_x_0_win = sip.wrapinstance(self.qtgui_time_sink_x_0.pyqwidget(), Qt.QWidget)
        self.rx_tab_layout_2.addWidget(self._qtgui_time_sink_x_0_win)
        self.qtgui_freq_sink_x_0 = qtgui.freq_sink_c(
        	2048, #size
        	firdes.WIN_BLACKMAN_hARRIS, #wintype
        	0, #fc
        	samp_rate, #bw
        	"", #name
        	1 #number of inputs
        )
        self.qtgui_freq_sink_x_0.set_update_time(0.10)
        self.qtgui_freq_sink_x_0.set_y_axis(-140, 10)
        self.qtgui_freq_sink_x_0.set_y_label('Relative Gain', 'dB')
        self.qtgui_freq_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, 0.0, 0, "")
        self.qtgui_freq_sink_x_0.enable_autoscale(False)
        self.qtgui_freq_sink_x_0.enable_grid(False)
        self.qtgui_freq_sink_x_0.set_fft_average(1.0)
        self.qtgui_freq_sink_x_0.enable_axis_labels(True)
        self.qtgui_freq_sink_x_0.enable_control_panel(False)

        if not True:
          self.qtgui_freq_sink_x_0.disable_legend()

        if "complex" == "float" or "complex" == "msg_float":
          self.qtgui_freq_sink_x_0.set_plot_pos_half(not True)

        labels = ['', '', '', '', '',
                  '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
                  "magenta", "yellow", "dark red", "dark green", "dark blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
                  1.0, 1.0, 1.0, 1.0, 1.0]
        for i in xrange(1):
            if len(labels[i]) == 0:
                self.qtgui_freq_sink_x_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_freq_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_freq_sink_x_0.set_line_width(i, widths[i])
            self.qtgui_freq_sink_x_0.set_line_color(i, colors[i])
            self.qtgui_freq_sink_x_0.set_line_alpha(i, alphas[i])

        self._qtgui_freq_sink_x_0_win = sip.wrapinstance(self.qtgui_freq_sink_x_0.pyqwidget(), Qt.QWidget)
        self.rx_tab_layout_0.addWidget(self._qtgui_freq_sink_x_0_win)
        self.qtgui_const_sink_x_0 = qtgui.const_sink_c(
        	1024, #size
        	"", #name
        	1 #number of inputs
        )
        self.qtgui_const_sink_x_0.set_update_time(0.10)
        self.qtgui_const_sink_x_0.set_y_axis(-2, 2)
        self.qtgui_const_sink_x_0.set_x_axis(-2, 2)
        self.qtgui_const_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, "")
        self.qtgui_const_sink_x_0.enable_autoscale(False)
        self.qtgui_const_sink_x_0.enable_grid(False)
        self.qtgui_const_sink_x_0.enable_axis_labels(True)

        if not True:
          self.qtgui_const_sink_x_0.disable_legend()

        labels = ['', '', '', '', '',
                  '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1]
        colors = ["blue", "red", "red", "red", "red",
                  "red", "red", "red", "red", "red"]
        styles = [0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0]
        markers = [0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
                  1.0, 1.0, 1.0, 1.0, 1.0]
        for i in xrange(1):
            if len(labels[i]) == 0:
                self.qtgui_const_sink_x_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_const_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_const_sink_x_0.set_line_width(i, widths[i])
            self.qtgui_const_sink_x_0.set_line_color(i, colors[i])
            self.qtgui_const_sink_x_0.set_line_style(i, styles[i])
            self.qtgui_const_sink_x_0.set_line_marker(i, markers[i])
            self.qtgui_const_sink_x_0.set_line_alpha(i, alphas[i])

        self._qtgui_const_sink_x_0_win = sip.wrapinstance(self.qtgui_const_sink_x_0.pyqwidget(), Qt.QWidget)
        self.rx_tab_layout_1.addWidget(self._qtgui_const_sink_x_0_win)
        self.lsa_su_transmitter_bc_0 = lsa.su_transmitter_bc("packet_len", "sensing", accesscode,
            mod, mod, 1024,2,False)
        self.lsa_protocol_parser_b_0 = lsa.protocol_parser_b(accesscode, False)
        self.lsa_eng_det_cc_0 = lsa.eng_det_cc(-40,20)
        self.lsa_correlate_sync_cc_0 = lsa.correlate_sync_cc((pre_s),0.9)
        self.lsa_burst_tagger_cc_0 = lsa.burst_tagger_cc("packet_len")
        self.interp_fir_filter_xxx_0 = filter.interp_fir_filter_ccf(sps, (tx_rrc_taps))
        self.interp_fir_filter_xxx_0.declare_sample_delay(0)
        self.digital_pfb_clock_sync_xxx_0 = digital.pfb_clock_sync_ccf(sps, 6.28/100, (rx_rrc_taps), nfilts, nfilts/2, 1.5, 1)
        self.digital_map_bb_0 = digital.map_bb((mod.pre_diff_code()))
        self.digital_costas_loop_cc_0 = digital.costas_loop_cc(6.28/100, mod.arity(), False)
        self.digital_constellation_decoder_cb_0 = digital.constellation_decoder_cb(mod)
        self.digital_burst_shaper_xx_0 = digital.burst_shaper_cc((firdes.window(firdes.WIN_HANN, 100, 0)), 0, 0, True, "packet_len")
        self.blocks_tagged_stream_multiply_length_0 = blocks.tagged_stream_multiply_length(gr.sizeof_gr_complex*1, "packet_len", sps)
        self.blocks_repack_bits_bb_0 = blocks.repack_bits_bb(bps, 1, "", False, gr.GR_MSB_FIRST)
        self.blocks_random_pdu_0 = blocks.random_pdu(500, 1000, chr(0xFF), 2)
        self.blocks_pdu_to_tagged_stream_0 = blocks.pdu_to_tagged_stream(blocks.byte_t, 'packet_len')
        self.blocks_nlog10_ff_0 = blocks.nlog10_ff(10, 1, 0)
        self.blocks_message_strobe_random_0 = blocks.message_strobe_random(pmt.intern("TEST"), blocks.STROBE_UNIFORM, 500, 10)
        self.blocks_message_debug_0 = blocks.message_debug()

        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.blocks_message_strobe_random_0, 'strobe'), (self.blocks_random_pdu_0, 'generate'))
        self.msg_connect((self.blocks_random_pdu_0, 'pdus'), (self.blocks_pdu_to_tagged_stream_0, 'pdus'))
        self.msg_connect((self.lsa_protocol_parser_b_0, 'header'), (self.blocks_message_debug_0, 'print'))
        self.connect((self.blocks_nlog10_ff_0, 0), (self.qtgui_time_sink_x_0, 0))
        self.connect((self.blocks_pdu_to_tagged_stream_0, 0), (self.lsa_su_transmitter_bc_0, 0))
        self.connect((self.blocks_repack_bits_bb_0, 0), (self.lsa_protocol_parser_b_0, 0))
        self.connect((self.blocks_tagged_stream_multiply_length_0, 0), (self.lsa_burst_tagger_cc_0, 0))
        self.connect((self.digital_burst_shaper_xx_0, 0), (self.interp_fir_filter_xxx_0, 0))
        self.connect((self.digital_constellation_decoder_cb_0, 0), (self.digital_map_bb_0, 0))
        self.connect((self.digital_costas_loop_cc_0, 0), (self.digital_constellation_decoder_cb_0, 0))
        self.connect((self.digital_costas_loop_cc_0, 0), (self.qtgui_const_sink_x_0, 0))
        self.connect((self.digital_map_bb_0, 0), (self.blocks_repack_bits_bb_0, 0))
        self.connect((self.digital_pfb_clock_sync_xxx_0, 0), (self.lsa_correlate_sync_cc_0, 0))
        self.connect((self.interp_fir_filter_xxx_0, 0), (self.blocks_tagged_stream_multiply_length_0, 0))
        self.connect((self.lsa_burst_tagger_cc_0, 0), (self.uhd_usrp_sink_0, 0))
        self.connect((self.lsa_correlate_sync_cc_0, 0), (self.digital_costas_loop_cc_0, 0))
        self.connect((self.lsa_correlate_sync_cc_0, 1), (self.qtgui_time_sink_x_1, 0))
        self.connect((self.lsa_eng_det_cc_0, 1), (self.blocks_nlog10_ff_0, 0))
        self.connect((self.lsa_eng_det_cc_0, 0), (self.digital_pfb_clock_sync_xxx_0, 0))
        self.connect((self.lsa_su_transmitter_bc_0, 0), (self.digital_burst_shaper_xx_0, 0))
        self.connect((self.uhd_usrp_source_0, 0), (self.lsa_eng_det_cc_0, 0))
        self.connect((self.uhd_usrp_source_0, 0), (self.qtgui_freq_sink_x_0, 0))

    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "usrp_energy_detector")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

    def get_mod(self):
        return self.mod

    def set_mod(self, mod):
        self.mod = mod

    def get_bps(self):
        return self.bps

    def set_bps(self, bps):
        self.bps = bps
        self.blocks_repack_bits_bb_0.set_k_and_l(self.bps,1)
        self.set_acc_bits([int(self.accesscode[i:i+self.bps],self.bps) for i in range(0,len(self.accesscode),self.bps)])

    def get_accesscode(self):
        return self.accesscode

    def set_accesscode(self, accesscode):
        self.accesscode = accesscode
        self.set_acc_bits([int(self.accesscode[i:i+self.bps],self.bps) for i in range(0,len(self.accesscode),self.bps)])

    def get_acc_bits(self):
        return self.acc_bits

    def set_acc_bits(self, acc_bits):
        self.acc_bits = acc_bits
        self.set_pre_code([mod.pre_diff_code()[self.acc_bits[i]] for i in range(0,len(self.acc_bits))])

    def get_sps(self):
        return self.sps

    def set_sps(self, sps):
        self.sps = sps
        self.blocks_tagged_stream_multiply_length_0.set_scalar(self.sps)

    def get_pre_code(self):
        return self.pre_code

    def set_pre_code(self, pre_code):
        self.pre_code = pre_code
        self.set_pre_s([mod.points()[self.pre_code[i]] for i in range(0,len(self.pre_code)) ])

    def get_nfilts(self):
        return self.nfilts

    def set_nfilts(self, nfilts):
        self.nfilts = nfilts

    def get_tx_rrc_taps(self):
        return self.tx_rrc_taps

    def set_tx_rrc_taps(self, tx_rrc_taps):
        self.tx_rrc_taps = tx_rrc_taps
        self.interp_fir_filter_xxx_0.set_taps((self.tx_rrc_taps))

    def get_tx_gain(self):
        return self.tx_gain

    def set_tx_gain(self, tx_gain):
        self.tx_gain = tx_gain
        self.uhd_usrp_sink_0.set_gain(self.tx_gain, 0)


    def get_tx_addr(self):
        return self.tx_addr

    def set_tx_addr(self, tx_addr):
        self.tx_addr = tx_addr

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.uhd_usrp_source_0.set_samp_rate(self.samp_rate)
        self.uhd_usrp_source_0.set_bandwidth(self.samp_rate, 0)
        self.uhd_usrp_sink_0.set_samp_rate(self.samp_rate)
        self.uhd_usrp_sink_0.set_bandwidth(self.samp_rate, 0)
        self.qtgui_time_sink_x_1.set_samp_rate(self.samp_rate)
        self.qtgui_time_sink_x_0.set_samp_rate(self.samp_rate)
        self.qtgui_freq_sink_x_0.set_frequency_range(0, self.samp_rate)

    def get_rx_rrc_taps(self):
        return self.rx_rrc_taps

    def set_rx_rrc_taps(self, rx_rrc_taps):
        self.rx_rrc_taps = rx_rrc_taps
        self.digital_pfb_clock_sync_xxx_0.update_taps((self.rx_rrc_taps))

    def get_rx_gain(self):
        return self.rx_gain

    def set_rx_gain(self, rx_gain):
        self.rx_gain = rx_gain
        self.uhd_usrp_source_0.set_gain(self.rx_gain, 0)


    def get_rx_addr(self):
        return self.rx_addr

    def set_rx_addr(self, rx_addr):
        self.rx_addr = rx_addr

    def get_pre_s(self):
        return self.pre_s

    def set_pre_s(self, pre_s):
        self.pre_s = pre_s

    def get_freq(self):
        return self.freq

    def set_freq(self, freq):
        self.freq = freq
        self.uhd_usrp_source_0.set_center_freq(self.freq, 0)
        self.uhd_usrp_sink_0.set_center_freq(self.freq, 0)


def main(top_block_cls=usrp_energy_detector, options=None):

    from distutils.version import StrictVersion
    if StrictVersion(Qt.qVersion()) >= StrictVersion("4.5.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()
    tb.start()
    tb.show()

    def quitting():
        tb.stop()
        tb.wait()
    qapp.connect(qapp, Qt.SIGNAL("aboutToQuit()"), quitting)
    qapp.exec_()


if __name__ == '__main__':
    main()
