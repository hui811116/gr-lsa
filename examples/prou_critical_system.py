#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Prou Critical System
# Generated: Fri Jun  9 10:44:43 2017
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
from gnuradio import gr
from gnuradio import qtgui
from gnuradio import uhd
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from gnuradio.qtgui import Range, RangeWidget
from optparse import OptionParser
import lsa
import sip
import sys
import time


class prou_critical_system(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Prou Critical System")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Prou Critical System")
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

        self.settings = Qt.QSettings("GNU Radio", "prou_critical_system")
        self.restoreGeometry(self.settings.value("geometry").toByteArray())

        ##################################################
        # Variables
        ##################################################
        
        self.mod = mod = digital.constellation_calcdist(([-1-1j, -1+1j, 1-1j, 1+1j]), ([0, 1, 2, 3]), 4, 1).base()
        
        self.cross_ac = cross_ac = digital.packet_utils.default_access_code
        self.bps = bps = mod.bits_per_symbol()
        self.rxmod = rxmod = digital.generic_mod(mod, False, 4, True, 0.35, False, False)
        self.pre_bytes = pre_bytes = [int(cross_ac[i:i+8],2) for i in range(0,len(cross_ac),8)]
        self.ac_bits = ac_bits = [int(cross_ac[i:i+bps],2) for i in range(0,len(cross_ac),bps)]
        self.zero_symbol = zero_symbol = "11011001110000110101001000101110"
        self.window = window = 128
        self.voe_thres = voe_thres = -40
        self.tx_subdev = tx_subdev = "A:A"
        self.sps = sps = 4
        self.samp_rate = samp_rate = 2e6
        self.rx_subdev = rx_subdev = "A:B"
        
        self.rx_rrc_taps = rx_rrc_taps = firdes.root_raised_cosine(32, 32*4, 1.0, 0.35, 11*4*32)
          
        self.rx_gain = rx_gain = 0.5
        self.preamble = preamble = digital.modulate_vector_bc(rxmod .to_basic_block(), (pre_bytes), ([1]))
        self.pre_sym = pre_sym = [int(cross_ac[i:i+bps],2) for i in range(0,len(cross_ac),bps)]
        self.nfilts = nfilts = 32
        self.freq_data = freq_data = 2.4e9
        self.dev_id = dev_id = "name=MyB210"
        self.delay = delay = 128
        self.ac_qpsk = ac_qpsk = [mod.points()[ac_bits[i]] for i in range(0,len(ac_bits))]

        ##################################################
        # Blocks
        ##################################################
        self.tab1 = Qt.QTabWidget()
        self.tab1_widget_0 = Qt.QWidget()
        self.tab1_layout_0 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.tab1_widget_0)
        self.tab1_grid_layout_0 = Qt.QGridLayout()
        self.tab1_layout_0.addLayout(self.tab1_grid_layout_0)
        self.tab1.addTab(self.tab1_widget_0, 'Control')
        self.top_layout.addWidget(self.tab1)
        self._voe_thres_range = Range(-120, -20, 1, -40, 200)
        self._voe_thres_win = RangeWidget(self._voe_thres_range, self.set_voe_thres, 'VOE threshold', "counter_slider", float)
        self.tab1_layout_0.addWidget(self._voe_thres_win)
        self.tab0 = Qt.QTabWidget()
        self.tab0_widget_0 = Qt.QWidget()
        self.tab0_layout_0 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.tab0_widget_0)
        self.tab0_grid_layout_0 = Qt.QGridLayout()
        self.tab0_layout_0.addLayout(self.tab0_grid_layout_0)
        self.tab0.addTab(self.tab0_widget_0, 'Signal')
        self.tab0_widget_1 = Qt.QWidget()
        self.tab0_layout_1 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.tab0_widget_1)
        self.tab0_grid_layout_1 = Qt.QGridLayout()
        self.tab0_layout_1.addLayout(self.tab0_grid_layout_1)
        self.tab0.addTab(self.tab0_widget_1, 'Detector')
        self.tab0_widget_2 = Qt.QWidget()
        self.tab0_layout_2 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.tab0_widget_2)
        self.tab0_grid_layout_2 = Qt.QGridLayout()
        self.tab0_layout_2.addLayout(self.tab0_grid_layout_2)
        self.tab0.addTab(self.tab0_widget_2, 'Interference Cancellation')
        self.tab0_widget_3 = Qt.QWidget()
        self.tab0_layout_3 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.tab0_widget_3)
        self.tab0_grid_layout_3 = Qt.QGridLayout()
        self.tab0_layout_3.addLayout(self.tab0_grid_layout_3)
        self.tab0.addTab(self.tab0_widget_3, 'Autocorrelation')
        self.tab0_widget_4 = Qt.QWidget()
        self.tab0_layout_4 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.tab0_widget_4)
        self.tab0_grid_layout_4 = Qt.QGridLayout()
        self.tab0_layout_4.addLayout(self.tab0_grid_layout_4)
        self.tab0.addTab(self.tab0_widget_4, 'Cross correlation')
        self.top_layout.addWidget(self.tab0)
        self._rx_gain_range = Range(0, 1, 0.01, 0.5, 200)
        self._rx_gain_win = RangeWidget(self._rx_gain_range, self.set_rx_gain, 'RX gain', "counter_slider", float)
        self.tab1_layout_0.addWidget(self._rx_gain_win)
        self.uhd_usrp_source_0 = uhd.usrp_source(
        	",".join((dev_id, "")),
        	uhd.stream_args(
        		cpu_format="fc32",
        		channels=range(1),
        	),
        )
        self.uhd_usrp_source_0.set_subdev_spec(rx_subdev, 0)
        self.uhd_usrp_source_0.set_samp_rate(samp_rate)
        self.uhd_usrp_source_0.set_center_freq(freq_data, 0)
        self.uhd_usrp_source_0.set_normalized_gain(rx_gain, 0)
        self.uhd_usrp_source_0.set_antenna('RX2', 0)
        self.qtgui_time_sink_x_2_1 = qtgui.time_sink_f(
        	5000, #size
        	samp_rate, #samp_rate
        	'Cross correlation', #name
        	1 #number of inputs
        )
        self.qtgui_time_sink_x_2_1.set_update_time(0.10)
        self.qtgui_time_sink_x_2_1.set_y_axis(-1, 1)
        
        self.qtgui_time_sink_x_2_1.set_y_label('Amplitude', "")
        
        self.qtgui_time_sink_x_2_1.enable_tags(-1, True)
        self.qtgui_time_sink_x_2_1.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, '')
        self.qtgui_time_sink_x_2_1.enable_autoscale(False)
        self.qtgui_time_sink_x_2_1.enable_grid(False)
        self.qtgui_time_sink_x_2_1.enable_axis_labels(True)
        self.qtgui_time_sink_x_2_1.enable_control_panel(True)
        
        if not True:
          self.qtgui_time_sink_x_2_1.disable_legend()
        
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
                self.qtgui_time_sink_x_2_1.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_time_sink_x_2_1.set_line_label(i, labels[i])
            self.qtgui_time_sink_x_2_1.set_line_width(i, widths[i])
            self.qtgui_time_sink_x_2_1.set_line_color(i, colors[i])
            self.qtgui_time_sink_x_2_1.set_line_style(i, styles[i])
            self.qtgui_time_sink_x_2_1.set_line_marker(i, markers[i])
            self.qtgui_time_sink_x_2_1.set_line_alpha(i, alphas[i])
        
        self._qtgui_time_sink_x_2_1_win = sip.wrapinstance(self.qtgui_time_sink_x_2_1.pyqwidget(), Qt.QWidget)
        self.tab0_layout_4.addWidget(self._qtgui_time_sink_x_2_1_win)
        self.qtgui_time_sink_x_2_0 = qtgui.time_sink_f(
        	20000, #size
        	samp_rate, #samp_rate
        	'Autocorrelation', #name
        	1 #number of inputs
        )
        self.qtgui_time_sink_x_2_0.set_update_time(0.10)
        self.qtgui_time_sink_x_2_0.set_y_axis(-1, 1)
        
        self.qtgui_time_sink_x_2_0.set_y_label('Amplitude', "")
        
        self.qtgui_time_sink_x_2_0.enable_tags(-1, True)
        self.qtgui_time_sink_x_2_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, '')
        self.qtgui_time_sink_x_2_0.enable_autoscale(False)
        self.qtgui_time_sink_x_2_0.enable_grid(False)
        self.qtgui_time_sink_x_2_0.enable_axis_labels(True)
        self.qtgui_time_sink_x_2_0.enable_control_panel(True)
        
        if not True:
          self.qtgui_time_sink_x_2_0.disable_legend()
        
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
                self.qtgui_time_sink_x_2_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_time_sink_x_2_0.set_line_label(i, labels[i])
            self.qtgui_time_sink_x_2_0.set_line_width(i, widths[i])
            self.qtgui_time_sink_x_2_0.set_line_color(i, colors[i])
            self.qtgui_time_sink_x_2_0.set_line_style(i, styles[i])
            self.qtgui_time_sink_x_2_0.set_line_marker(i, markers[i])
            self.qtgui_time_sink_x_2_0.set_line_alpha(i, alphas[i])
        
        self._qtgui_time_sink_x_2_0_win = sip.wrapinstance(self.qtgui_time_sink_x_2_0.pyqwidget(), Qt.QWidget)
        self.tab0_layout_3.addWidget(self._qtgui_time_sink_x_2_0_win)
        self.qtgui_time_sink_x_1_0 = qtgui.time_sink_c(
        	100000, #size
        	samp_rate, #samp_rate
        	'Comparison of Signals', #name
        	1 #number of inputs
        )
        self.qtgui_time_sink_x_1_0.set_update_time(0.10)
        self.qtgui_time_sink_x_1_0.set_y_axis(-0.1, 0.1)
        
        self.qtgui_time_sink_x_1_0.set_y_label('Amplitude', "")
        
        self.qtgui_time_sink_x_1_0.enable_tags(-1, False)
        self.qtgui_time_sink_x_1_0.set_trigger_mode(qtgui.TRIG_MODE_TAG, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "ic_out")
        self.qtgui_time_sink_x_1_0.enable_autoscale(False)
        self.qtgui_time_sink_x_1_0.enable_grid(False)
        self.qtgui_time_sink_x_1_0.enable_axis_labels(True)
        self.qtgui_time_sink_x_1_0.enable_control_panel(True)
        
        if not True:
          self.qtgui_time_sink_x_1_0.disable_legend()
        
        labels = ['IC I', 'IC Q', 'Ori I', 'Ori Q', '',
                  '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "magenta", "cyan",
                  "magenta", "yellow", "dark red", "dark green", "blue"]
        styles = [1, 1, 2, 2, 1,
                  1, 1, 1, 1, 1]
        markers = [-1, -1, -1, -1, -1,
                   -1, -1, -1, -1, -1]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
                  1.0, 1.0, 1.0, 1.0, 1.0]
        
        for i in xrange(2*1):
            if len(labels[i]) == 0:
                if(i % 2 == 0):
                    self.qtgui_time_sink_x_1_0.set_line_label(i, "Re{{Data {0}}}".format(i/2))
                else:
                    self.qtgui_time_sink_x_1_0.set_line_label(i, "Im{{Data {0}}}".format(i/2))
            else:
                self.qtgui_time_sink_x_1_0.set_line_label(i, labels[i])
            self.qtgui_time_sink_x_1_0.set_line_width(i, widths[i])
            self.qtgui_time_sink_x_1_0.set_line_color(i, colors[i])
            self.qtgui_time_sink_x_1_0.set_line_style(i, styles[i])
            self.qtgui_time_sink_x_1_0.set_line_marker(i, markers[i])
            self.qtgui_time_sink_x_1_0.set_line_alpha(i, alphas[i])
        
        self._qtgui_time_sink_x_1_0_win = sip.wrapinstance(self.qtgui_time_sink_x_1_0.pyqwidget(), Qt.QWidget)
        self.tab0_layout_2.addWidget(self._qtgui_time_sink_x_1_0_win)
        self.qtgui_time_sink_x_1 = qtgui.time_sink_c(
        	100000, #size
        	samp_rate, #samp_rate
        	'IC Signals', #name
        	1 #number of inputs
        )
        self.qtgui_time_sink_x_1.set_update_time(0.10)
        self.qtgui_time_sink_x_1.set_y_axis(-0.1, 0.1)
        
        self.qtgui_time_sink_x_1.set_y_label('Amplitude', "")
        
        self.qtgui_time_sink_x_1.enable_tags(-1, False)
        self.qtgui_time_sink_x_1.set_trigger_mode(qtgui.TRIG_MODE_TAG, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "ic_out")
        self.qtgui_time_sink_x_1.enable_autoscale(False)
        self.qtgui_time_sink_x_1.enable_grid(False)
        self.qtgui_time_sink_x_1.enable_axis_labels(True)
        self.qtgui_time_sink_x_1.enable_control_panel(True)
        
        if not True:
          self.qtgui_time_sink_x_1.disable_legend()
        
        labels = ['IC I', 'IC Q', 'Ori I', 'Ori Q', '',
                  '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "magenta", "cyan",
                  "magenta", "yellow", "dark red", "dark green", "blue"]
        styles = [1, 1, 2, 2, 1,
                  1, 1, 1, 1, 1]
        markers = [-1, -1, -1, -1, -1,
                   -1, -1, -1, -1, -1]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
                  1.0, 1.0, 1.0, 1.0, 1.0]
        
        for i in xrange(2*1):
            if len(labels[i]) == 0:
                if(i % 2 == 0):
                    self.qtgui_time_sink_x_1.set_line_label(i, "Re{{Data {0}}}".format(i/2))
                else:
                    self.qtgui_time_sink_x_1.set_line_label(i, "Im{{Data {0}}}".format(i/2))
            else:
                self.qtgui_time_sink_x_1.set_line_label(i, labels[i])
            self.qtgui_time_sink_x_1.set_line_width(i, widths[i])
            self.qtgui_time_sink_x_1.set_line_color(i, colors[i])
            self.qtgui_time_sink_x_1.set_line_style(i, styles[i])
            self.qtgui_time_sink_x_1.set_line_marker(i, markers[i])
            self.qtgui_time_sink_x_1.set_line_alpha(i, alphas[i])
        
        self._qtgui_time_sink_x_1_win = sip.wrapinstance(self.qtgui_time_sink_x_1.pyqwidget(), Qt.QWidget)
        self.tab0_layout_2.addWidget(self._qtgui_time_sink_x_1_win)
        self.qtgui_time_sink_x_0 = qtgui.time_sink_f(
        	20000, #size
        	samp_rate, #samp_rate
        	'Variance of energy', #name
        	1 #number of inputs
        )
        self.qtgui_time_sink_x_0.set_update_time(0.10)
        self.qtgui_time_sink_x_0.set_y_axis(-1, 1)
        
        self.qtgui_time_sink_x_0.set_y_label('Amplitude', "")
        
        self.qtgui_time_sink_x_0.enable_tags(-1, True)
        self.qtgui_time_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        self.qtgui_time_sink_x_0.enable_autoscale(False)
        self.qtgui_time_sink_x_0.enable_grid(False)
        self.qtgui_time_sink_x_0.enable_axis_labels(True)
        self.qtgui_time_sink_x_0.enable_control_panel(True)
        
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
        self.tab0_layout_1.addWidget(self._qtgui_time_sink_x_0_win)
        self.lsa_symbol_sync_receiver_cf_0 = lsa.symbol_sync_receiver_cf(mod,10)
        self.lsa_moving_average_ff_1 = lsa.moving_average_ff(window)
        self.lsa_moving_average_ff_0 = lsa.moving_average_ff(128)
        self.lsa_moving_average_cc_0 = lsa.moving_average_cc(window)
        self.lsa_modified_costas_loop_cc_0 = lsa.modified_costas_loop_cc(6.28/200, 4, False)
        self.lsa_interference_tagger_cc_0 = lsa.interference_tagger_cc(voe_thres)
        self.lsa_interference_energy_detector_cc_0 = lsa.interference_energy_detector_cc(128,False)
        self.lsa_ic_critical_cc_0 = lsa.ic_critical_cc(len(ac_qpsk),sps,True)
        self.lsa_expand_symbal_to_sample_ff_0 = lsa.expand_symbal_to_sample_ff(4)
        self.lsa_correlate_sync_cc_0 = lsa.correlate_sync_cc((ac_qpsk),0.75)
        self.lsa_coarse_sync_cc_0 = lsa.coarse_sync_cc(0.75,delay)
        self.lsa_block_tagger_cc_0 = lsa.block_tagger_cc("block_tag",8192,False)
        self.digital_pfb_clock_sync_xxx_0 = digital.pfb_clock_sync_ccf(sps, 6.28/200, (rx_rrc_taps), nfilts, nfilts/2, 1.5, 1)
        self.blocks_nlog10_ff_0 = blocks.nlog10_ff(10, 1, 0)
        self.blocks_multiply_xx_0 = blocks.multiply_vcc(1)
        self.blocks_divide_xx_0 = blocks.divide_ff(1)
        self.blocks_delay_0 = blocks.delay(gr.sizeof_gr_complex*1, delay)
        self.blocks_conjugate_cc_0 = blocks.conjugate_cc()
        self.blocks_complex_to_mag_squared_0 = blocks.complex_to_mag_squared(1)
        self.blocks_complex_to_mag_1 = blocks.complex_to_mag(1)
        self.blocks_complex_to_mag_0 = blocks.complex_to_mag(1)

        ##################################################
        # Connections
        ##################################################
        self.connect((self.blocks_complex_to_mag_0, 0), (self.blocks_divide_xx_0, 0))    
        self.connect((self.blocks_complex_to_mag_1, 0), (self.qtgui_time_sink_x_2_1, 0))    
        self.connect((self.blocks_complex_to_mag_squared_0, 0), (self.lsa_moving_average_ff_1, 0))    
        self.connect((self.blocks_conjugate_cc_0, 0), (self.blocks_multiply_xx_0, 1))    
        self.connect((self.blocks_delay_0, 0), (self.blocks_complex_to_mag_squared_0, 0))    
        self.connect((self.blocks_delay_0, 0), (self.blocks_conjugate_cc_0, 0))    
        self.connect((self.blocks_delay_0, 0), (self.lsa_coarse_sync_cc_0, 0))    
        self.connect((self.blocks_divide_xx_0, 0), (self.lsa_coarse_sync_cc_0, 2))    
        self.connect((self.blocks_divide_xx_0, 0), (self.qtgui_time_sink_x_2_0, 0))    
        self.connect((self.blocks_multiply_xx_0, 0), (self.lsa_moving_average_cc_0, 0))    
        self.connect((self.blocks_nlog10_ff_0, 0), (self.lsa_interference_tagger_cc_0, 1))    
        self.connect((self.blocks_nlog10_ff_0, 0), (self.qtgui_time_sink_x_0, 0))    
        self.connect((self.digital_pfb_clock_sync_xxx_0, 0), (self.lsa_correlate_sync_cc_0, 0))    
        self.connect((self.lsa_block_tagger_cc_0, 0), (self.digital_pfb_clock_sync_xxx_0, 0))    
        self.connect((self.lsa_block_tagger_cc_0, 0), (self.lsa_ic_critical_cc_0, 0))    
        self.connect((self.lsa_coarse_sync_cc_0, 0), (self.lsa_block_tagger_cc_0, 0))    
        self.connect((self.lsa_correlate_sync_cc_0, 1), (self.blocks_complex_to_mag_1, 0))    
        self.connect((self.lsa_correlate_sync_cc_0, 0), (self.lsa_modified_costas_loop_cc_0, 0))    
        self.connect((self.lsa_expand_symbal_to_sample_ff_0, 1), (self.lsa_ic_critical_cc_0, 2))    
        self.connect((self.lsa_expand_symbal_to_sample_ff_0, 0), (self.lsa_ic_critical_cc_0, 1))    
        self.connect((self.lsa_ic_critical_cc_0, 0), (self.qtgui_time_sink_x_1, 0))    
        self.connect((self.lsa_ic_critical_cc_0, 1), (self.qtgui_time_sink_x_1_0, 0))    
        self.connect((self.lsa_interference_energy_detector_cc_0, 0), (self.lsa_interference_tagger_cc_0, 0))    
        self.connect((self.lsa_interference_energy_detector_cc_0, 1), (self.lsa_moving_average_ff_0, 0))    
        self.connect((self.lsa_interference_tagger_cc_0, 0), (self.blocks_delay_0, 0))    
        self.connect((self.lsa_interference_tagger_cc_0, 0), (self.blocks_multiply_xx_0, 0))    
        self.connect((self.lsa_modified_costas_loop_cc_0, 2), (self.lsa_symbol_sync_receiver_cf_0, 2))    
        self.connect((self.lsa_modified_costas_loop_cc_0, 0), (self.lsa_symbol_sync_receiver_cf_0, 0))    
        self.connect((self.lsa_modified_costas_loop_cc_0, 1), (self.lsa_symbol_sync_receiver_cf_0, 1))    
        self.connect((self.lsa_moving_average_cc_0, 0), (self.blocks_complex_to_mag_0, 0))    
        self.connect((self.lsa_moving_average_cc_0, 0), (self.lsa_coarse_sync_cc_0, 1))    
        self.connect((self.lsa_moving_average_ff_0, 0), (self.blocks_nlog10_ff_0, 0))    
        self.connect((self.lsa_moving_average_ff_1, 0), (self.blocks_divide_xx_0, 1))    
        self.connect((self.lsa_symbol_sync_receiver_cf_0, 1), (self.lsa_expand_symbal_to_sample_ff_0, 1))    
        self.connect((self.lsa_symbol_sync_receiver_cf_0, 0), (self.lsa_expand_symbal_to_sample_ff_0, 0))    
        self.connect((self.uhd_usrp_source_0, 0), (self.lsa_interference_energy_detector_cc_0, 0))    

    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "prou_critical_system")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

    def get_mod(self):
        return self.mod

    def set_mod(self, mod):
        self.mod = mod
        self.set_rxmod(digital.generic_mod(self.mod, False, 4, True, 0.35, False, False))

    def get_cross_ac(self):
        return self.cross_ac

    def set_cross_ac(self, cross_ac):
        self.cross_ac = cross_ac
        self.set_pre_sym([int(self.cross_ac[i:i+self.bps],2) for i in range(0,len(self.cross_ac),self.bps)])
        self.set_pre_bytes([int(self.cross_ac[i:i+8],2) for i in range(0,len(self.cross_ac),8)])
        self.set_ac_bits([int(self.cross_ac[i:i+self.bps],2) for i in range(0,len(self.cross_ac),self.bps)])

    def get_bps(self):
        return self.bps

    def set_bps(self, bps):
        self.bps = bps
        self.set_pre_sym([int(self.cross_ac[i:i+self.bps],2) for i in range(0,len(self.cross_ac),self.bps)])
        self.set_ac_bits([int(self.cross_ac[i:i+self.bps],2) for i in range(0,len(self.cross_ac),self.bps)])

    def get_rxmod(self):
        return self.rxmod

    def set_rxmod(self, rxmod):
        self.rxmod = rxmod

    def get_pre_bytes(self):
        return self.pre_bytes

    def set_pre_bytes(self, pre_bytes):
        self.pre_bytes = pre_bytes

    def get_ac_bits(self):
        return self.ac_bits

    def set_ac_bits(self, ac_bits):
        self.ac_bits = ac_bits
        self.set_ac_qpsk([mod.points()[self.ac_bits[i]] for i in range(0,len(self.ac_bits))])

    def get_zero_symbol(self):
        return self.zero_symbol

    def set_zero_symbol(self, zero_symbol):
        self.zero_symbol = zero_symbol

    def get_window(self):
        return self.window

    def set_window(self, window):
        self.window = window

    def get_voe_thres(self):
        return self.voe_thres

    def set_voe_thres(self, voe_thres):
        self.voe_thres = voe_thres
        self.lsa_interference_tagger_cc_0.set_voe_threshold(self.voe_thres)

    def get_tx_subdev(self):
        return self.tx_subdev

    def set_tx_subdev(self, tx_subdev):
        self.tx_subdev = tx_subdev

    def get_sps(self):
        return self.sps

    def set_sps(self, sps):
        self.sps = sps

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.uhd_usrp_source_0.set_samp_rate(self.samp_rate)
        self.qtgui_time_sink_x_2_1.set_samp_rate(self.samp_rate)
        self.qtgui_time_sink_x_2_0.set_samp_rate(self.samp_rate)
        self.qtgui_time_sink_x_1_0.set_samp_rate(self.samp_rate)
        self.qtgui_time_sink_x_1.set_samp_rate(self.samp_rate)
        self.qtgui_time_sink_x_0.set_samp_rate(self.samp_rate)

    def get_rx_subdev(self):
        return self.rx_subdev

    def set_rx_subdev(self, rx_subdev):
        self.rx_subdev = rx_subdev

    def get_rx_rrc_taps(self):
        return self.rx_rrc_taps

    def set_rx_rrc_taps(self, rx_rrc_taps):
        self.rx_rrc_taps = rx_rrc_taps
        self.digital_pfb_clock_sync_xxx_0.update_taps((self.rx_rrc_taps))

    def get_rx_gain(self):
        return self.rx_gain

    def set_rx_gain(self, rx_gain):
        self.rx_gain = rx_gain
        self.uhd_usrp_source_0.set_normalized_gain(self.rx_gain, 0)
        	

    def get_preamble(self):
        return self.preamble

    def set_preamble(self, preamble):
        self.preamble = preamble

    def get_pre_sym(self):
        return self.pre_sym

    def set_pre_sym(self, pre_sym):
        self.pre_sym = pre_sym

    def get_nfilts(self):
        return self.nfilts

    def set_nfilts(self, nfilts):
        self.nfilts = nfilts

    def get_freq_data(self):
        return self.freq_data

    def set_freq_data(self, freq_data):
        self.freq_data = freq_data
        self.uhd_usrp_source_0.set_center_freq(self.freq_data, 0)

    def get_dev_id(self):
        return self.dev_id

    def set_dev_id(self, dev_id):
        self.dev_id = dev_id

    def get_delay(self):
        return self.delay

    def set_delay(self, delay):
        self.delay = delay
        self.blocks_delay_0.set_dly(self.delay)

    def get_ac_qpsk(self):
        return self.ac_qpsk

    def set_ac_qpsk(self, ac_qpsk):
        self.ac_qpsk = ac_qpsk


def main(top_block_cls=prou_critical_system, options=None):

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
