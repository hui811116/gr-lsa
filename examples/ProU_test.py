#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Prou Test
# Generated: Fri Mar 31 21:02:34 2017
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
from gnuradio import channels
from gnuradio import digital
from gnuradio import eng_notation
from gnuradio import filter
from gnuradio import gr
from gnuradio import qtgui
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
import lsa
import numpy
import sip
import sys
from gnuradio import qtgui


class ProU_test(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Prou Test")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Prou Test")
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

        self.settings = Qt.QSettings("GNU Radio", "ProU_test")
        self.restoreGeometry(self.settings.value("geometry").toByteArray())

        ##################################################
        # Variables
        ##################################################

        self.mod = mod = digital.constellation_calcdist(([-1-1j, -1+1j, 1+1j, 1-1j]), ([0, 1, 3, 2]), 4, 1).base()

        self.bps = bps = mod.bits_per_symbol()
        self.accesscode = accesscode = digital.packet_utils.default_access_code
        self.pre_bits = pre_bits = [accesscode[i:i+bps] for i in range(0,len(accesscode),bps)]
        self.pre_symb = pre_symb = [int(pre_bits[i],bps) for i in range(0,len(pre_bits))]
        self.sps = sps = 4
        self.pre_idx = pre_idx = [mod.pre_diff_code().index(pre_symb[i]) for i in range(0,len(pre_symb))]
        self.nfilts = nfilts = 32
        self.eb = eb = 0.35
        self.clean_precode = clean_precode = accesscode+"0000000000000000"

        self.tx_rrc_taps = tx_rrc_taps = firdes.root_raised_cosine(sps, sps, 1.0, eb, 11*sps)

        self.samp_rate = samp_rate = 32000

        self.rx_rrc_taps = rx_rrc_taps = firdes.root_raised_cosine(nfilts, nfilts*sps, 1.0, eb, 11*sps*nfilts)

        self.preamble_s = preamble_s = [mod.points()[pre_idx[i]] for i in range(0,len(pre_idx))]
        self.pre_bytes = pre_bytes = [int(accesscode[i:i+8],2) for i in range(0,len(accesscode),8)]
        self.num = num = 100
        self.clean_hdr = clean_hdr = digital.modulate_vector_bc(digital.generic_mod(mod, False, sps, True, eb, False, False) .to_basic_block(), ([int(clean_precode[i:i+8],2) for i in range(0,len(clean_precode),8)]), ([1]))

        ##################################################
        # Blocks
        ##################################################
        self.qtgui_time_sink_x_1 = qtgui.time_sink_c(
        	10000, #size
        	samp_rate, #samp_rate
        	"", #name
        	1 #number of inputs
        )
        self.qtgui_time_sink_x_1.set_update_time(0.5)
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

        for i in xrange(2):
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
        self.top_layout.addWidget(self._qtgui_time_sink_x_1_win)
        self.qtgui_time_sink_x_0 = qtgui.time_sink_f(
        	8192, #size
        	samp_rate, #samp_rate
        	"", #name
        	1 #number of inputs
        )
        self.qtgui_time_sink_x_0.set_update_time(0.5)
        self.qtgui_time_sink_x_0.set_y_axis(-1, 1)

        self.qtgui_time_sink_x_0.set_y_label('Amplitude', "")

        self.qtgui_time_sink_x_0.enable_tags(-1, True)
        self.qtgui_time_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        self.qtgui_time_sink_x_0.enable_autoscale(True)
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
        self.top_layout.addWidget(self._qtgui_time_sink_x_0_win)
        self.lsa_sync_symbol_to_sample_ff_0 = lsa.sync_symbol_to_sample_ff(sps,nfilts,"LSA_hdr")
        self.lsa_symbol_sync_receiver_cc_0 = lsa.symbol_sync_receiver_cc(accesscode,mod,mod, False)
        self.lsa_su_header_prefix_0 = lsa.su_header_prefix(accesscode, "packet_len", 4, 20)
        self.lsa_prou_sample_queue_cc_0 = lsa.prou_sample_queue_cc("sensing", sps, False)
        self.lsa_modified_polyphase_time_sync_cc_0 = lsa.modified_polyphase_time_sync_cc(sps, 6.28/200, (rx_rrc_taps), nfilts, nfilts/2, 1.5, 1, "sensing")
        self.lsa_modified_costas_loop_cc_0 = lsa.modified_costas_loop_cc(6.28/200, mod.arity(), False,"sensing")
        self.lsa_interference_canceller_cc_0 = lsa.interference_canceller_cc((clean_hdr), "sensing",sps, mod.bits_per_symbol(), len(accesscode) + 64,True)
        self.lsa_correlate_sync_cc_0 = lsa.correlate_sync_cc((preamble_s),0.9)
        self.interp_fir_filter_xxx_0 = filter.interp_fir_filter_ccf(sps, (tx_rrc_taps))
        self.interp_fir_filter_xxx_0.declare_sample_delay(0)
        self.digital_map_bb_0 = digital.map_bb((mod.pre_diff_code()))
        self.digital_chunks_to_symbols_xx_0 = digital.chunks_to_symbols_bc((mod.points()), 1)
        self.channels_channel_model_0 = channels.channel_model(
        	noise_voltage=20e-6,
        	frequency_offset=1e-5,
        	epsilon=1.0,
        	taps=(1, ),
        	noise_seed=0,
        	block_tags=False
        )
        self.blocks_throttle_0 = blocks.throttle(gr.sizeof_gr_complex*1, samp_rate,True)
        self.blocks_stream_to_tagged_stream_0 = blocks.stream_to_tagged_stream(gr.sizeof_char, 1, num, "packet_len")
        self.blocks_repack_bits_bb_0 = blocks.repack_bits_bb(8, mod.bits_per_symbol(), "packet_len", False, gr.GR_MSB_FIRST)
        self.analog_random_source_x_0 = blocks.vector_source_b(map(int, numpy.random.randint(0, 255, 100)), True)

        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.lsa_symbol_sync_receiver_cc_0, 'msg'), (self.lsa_prou_sample_queue_cc_0, 'info'))
        self.connect((self.analog_random_source_x_0, 0), (self.blocks_stream_to_tagged_stream_0, 0))
        self.connect((self.blocks_repack_bits_bb_0, 0), (self.digital_map_bb_0, 0))
        self.connect((self.blocks_stream_to_tagged_stream_0, 0), (self.lsa_su_header_prefix_0, 0))
        self.connect((self.blocks_throttle_0, 0), (self.lsa_prou_sample_queue_cc_0, 0))
        self.connect((self.channels_channel_model_0, 0), (self.blocks_throttle_0, 0))
        self.connect((self.digital_chunks_to_symbols_xx_0, 0), (self.interp_fir_filter_xxx_0, 0))
        self.connect((self.digital_map_bb_0, 0), (self.digital_chunks_to_symbols_xx_0, 0))
        self.connect((self.interp_fir_filter_xxx_0, 0), (self.channels_channel_model_0, 0))
        self.connect((self.lsa_correlate_sync_cc_0, 0), (self.lsa_modified_costas_loop_cc_0, 0))
        self.connect((self.lsa_interference_canceller_cc_0, 1), (self.qtgui_time_sink_x_0, 0))
        self.connect((self.lsa_interference_canceller_cc_0, 0), (self.qtgui_time_sink_x_1, 0))
        self.connect((self.lsa_modified_costas_loop_cc_0, 1), (self.lsa_symbol_sync_receiver_cc_0, 1))
        self.connect((self.lsa_modified_costas_loop_cc_0, 0), (self.lsa_symbol_sync_receiver_cc_0, 0))
        self.connect((self.lsa_modified_costas_loop_cc_0, 2), (self.lsa_symbol_sync_receiver_cc_0, 2))
        self.connect((self.lsa_modified_costas_loop_cc_0, 3), (self.lsa_symbol_sync_receiver_cc_0, 3))
        self.connect((self.lsa_modified_costas_loop_cc_0, 4), (self.lsa_symbol_sync_receiver_cc_0, 4))
        self.connect((self.lsa_modified_polyphase_time_sync_cc_0, 0), (self.lsa_correlate_sync_cc_0, 0))
        self.connect((self.lsa_modified_polyphase_time_sync_cc_0, 1), (self.lsa_modified_costas_loop_cc_0, 1))
        self.connect((self.lsa_modified_polyphase_time_sync_cc_0, 2), (self.lsa_modified_costas_loop_cc_0, 2))
        self.connect((self.lsa_prou_sample_queue_cc_0, 1), (self.lsa_interference_canceller_cc_0, 0))
        self.connect((self.lsa_prou_sample_queue_cc_0, 0), (self.lsa_modified_polyphase_time_sync_cc_0, 0))
        self.connect((self.lsa_su_header_prefix_0, 0), (self.blocks_repack_bits_bb_0, 0))
        self.connect((self.lsa_symbol_sync_receiver_cc_0, 0), (self.lsa_sync_symbol_to_sample_ff_0, 0))
        self.connect((self.lsa_symbol_sync_receiver_cc_0, 1), (self.lsa_sync_symbol_to_sample_ff_0, 1))
        self.connect((self.lsa_symbol_sync_receiver_cc_0, 3), (self.lsa_sync_symbol_to_sample_ff_0, 3))
        self.connect((self.lsa_symbol_sync_receiver_cc_0, 2), (self.lsa_sync_symbol_to_sample_ff_0, 2))
        self.connect((self.lsa_sync_symbol_to_sample_ff_0, 0), (self.lsa_interference_canceller_cc_0, 1))
        self.connect((self.lsa_sync_symbol_to_sample_ff_0, 1), (self.lsa_interference_canceller_cc_0, 2))
        self.connect((self.lsa_sync_symbol_to_sample_ff_0, 3), (self.lsa_interference_canceller_cc_0, 4))
        self.connect((self.lsa_sync_symbol_to_sample_ff_0, 2), (self.lsa_interference_canceller_cc_0, 3))

    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "ProU_test")
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
        self.set_pre_symb([int(self.pre_bits[i],self.bps) for i in range(0,len(self.pre_bits))])
        self.set_pre_bits([self.accesscode[i:i+self.bps] for i in range(0,len(self.accesscode),self.bps)])

    def get_accesscode(self):
        return self.accesscode

    def set_accesscode(self, accesscode):
        self.accesscode = accesscode
        self.set_pre_bytes([int(self.accesscode[i:i+8],2) for i in range(0,len(self.accesscode),8)])
        self.set_pre_bits([self.accesscode[i:i+self.bps] for i in range(0,len(self.accesscode),self.bps)])
        self.set_clean_precode(self.accesscode+"0000000000000000")

    def get_pre_bits(self):
        return self.pre_bits

    def set_pre_bits(self, pre_bits):
        self.pre_bits = pre_bits
        self.set_pre_symb([int(self.pre_bits[i],self.bps) for i in range(0,len(self.pre_bits))])

    def get_pre_symb(self):
        return self.pre_symb

    def set_pre_symb(self, pre_symb):
        self.pre_symb = pre_symb
        self.set_pre_idx([mod.pre_diff_code().index(self.pre_symb[i]) for i in range(0,len(self.pre_symb))])

    def get_sps(self):
        return self.sps

    def set_sps(self, sps):
        self.sps = sps

    def get_pre_idx(self):
        return self.pre_idx

    def set_pre_idx(self, pre_idx):
        self.pre_idx = pre_idx
        self.set_preamble_s([mod.points()[self.pre_idx[i]] for i in range(0,len(self.pre_idx))])

    def get_nfilts(self):
        return self.nfilts

    def set_nfilts(self, nfilts):
        self.nfilts = nfilts

    def get_eb(self):
        return self.eb

    def set_eb(self, eb):
        self.eb = eb

    def get_clean_precode(self):
        return self.clean_precode

    def set_clean_precode(self, clean_precode):
        self.clean_precode = clean_precode

    def get_tx_rrc_taps(self):
        return self.tx_rrc_taps

    def set_tx_rrc_taps(self, tx_rrc_taps):
        self.tx_rrc_taps = tx_rrc_taps
        self.interp_fir_filter_xxx_0.set_taps((self.tx_rrc_taps))

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.qtgui_time_sink_x_1.set_samp_rate(self.samp_rate)
        self.qtgui_time_sink_x_0.set_samp_rate(self.samp_rate)
        self.blocks_throttle_0.set_sample_rate(self.samp_rate)

    def get_rx_rrc_taps(self):
        return self.rx_rrc_taps

    def set_rx_rrc_taps(self, rx_rrc_taps):
        self.rx_rrc_taps = rx_rrc_taps

    def get_preamble_s(self):
        return self.preamble_s

    def set_preamble_s(self, preamble_s):
        self.preamble_s = preamble_s

    def get_pre_bytes(self):
        return self.pre_bytes

    def set_pre_bytes(self, pre_bytes):
        self.pre_bytes = pre_bytes

    def get_num(self):
        return self.num

    def set_num(self, num):
        self.num = num
        self.blocks_stream_to_tagged_stream_0.set_packet_len(self.num)
        self.blocks_stream_to_tagged_stream_0.set_packet_len_pmt(self.num)

    def get_clean_hdr(self):
        return self.clean_hdr

    def set_clean_hdr(self, clean_hdr):
        self.clean_hdr = clean_hdr


def main(top_block_cls=ProU_test, options=None):

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
