#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Sim Interference
# Generated: Wed Mar 22 10:42:30 2017
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
from gnuradio import analog
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


class sim_interference(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Sim Interference")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Sim Interference")
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

        self.settings = Qt.QSettings("GNU Radio", "sim_interference")
        self.restoreGeometry(self.settings.value("geometry").toByteArray())

        ##################################################
        # Variables
        ##################################################
        self.sps = sps = 4

        self.mod = mod = digital.constellation_qpsk().base()

        self.eb1 = eb1 = 0.35
        self.rxmod = rxmod = digital.generic_mod(mod, False, sps, True, eb1, False, False)
        self.nfilts = nfilts = 32
        self.eb2 = eb2 = 0.22
        self.ac_hex = ac_hex = [0xac, 0xdd, 0xa4, 0xe2, 0xf2, 0x8c, 0x20, 0xfc]

        self.tx_rrc_taps = tx_rrc_taps = firdes.root_raised_cosine(sps, sps, 1.0, eb2, 11*sps)

        self.sps2 = sps2 = 2
        self.samp_rate = samp_rate = 32e3

        self.rx_rrc_taps = rx_rrc_taps = firdes.root_raised_cosine(nfilts, sps*nfilts, 1.0, eb1, 11*sps*nfilts)

        self.num = num = 200
        self.modulated_sync_word = modulated_sync_word = digital.modulate_vector_bc(rxmod .to_basic_block(), (ac_hex), ([1]))
        self.ac = ac = map(lambda x: int(x), list(digital.packet_utils.default_access_code))

        ##################################################
        # Blocks
        ##################################################
        self.qtgui_time_sink_x_0 = qtgui.time_sink_c(
        	1024, #size
        	samp_rate, #samp_rate
        	"", #name
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

        for i in xrange(2):
            if len(labels[i]) == 0:
                if(i % 2 == 0):
                    self.qtgui_time_sink_x_0.set_line_label(i, "Re{{Data {0}}}".format(i/2))
                else:
                    self.qtgui_time_sink_x_0.set_line_label(i, "Im{{Data {0}}}".format(i/2))
            else:
                self.qtgui_time_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_time_sink_x_0.set_line_width(i, widths[i])
            self.qtgui_time_sink_x_0.set_line_color(i, colors[i])
            self.qtgui_time_sink_x_0.set_line_style(i, styles[i])
            self.qtgui_time_sink_x_0.set_line_marker(i, markers[i])
            self.qtgui_time_sink_x_0.set_line_alpha(i, alphas[i])

        self._qtgui_time_sink_x_0_win = sip.wrapinstance(self.qtgui_time_sink_x_0.pyqwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_time_sink_x_0_win)
        self.lsa_eng_det_cc_0 = lsa.eng_det_cc(-20,5)
        self.interp_fir_filter_xxx_0 = filter.interp_fir_filter_ccf(sps, (tx_rrc_taps))
        self.interp_fir_filter_xxx_0.declare_sample_delay(0)
        self.digital_qam_mod_0 = digital.qam.qam_mod(
          constellation_points=16,
          mod_code="gray",
          differential=True,
          samples_per_symbol=sps2,
          excess_bw=eb2,
          verbose=False,
          log=False,
          )
        self.channels_channel_model_0 = channels.channel_model(
        	noise_voltage=1e-3,
        	frequency_offset=0.02,
        	epsilon=0.95,
        	taps=(1.0, ),
        	noise_seed=0,
        	block_tags=False
        )
        self.blocks_vector_source_x_0 = blocks.vector_source_c([1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0] + ac, True, 1, [])
        self.blocks_throttle_0 = blocks.throttle(gr.sizeof_gr_complex*1, samp_rate,True)
        self.blocks_stream_to_tagged_stream_0_0 = blocks.stream_to_tagged_stream(gr.sizeof_char, 1, num, "qam_pkt")
        self.blocks_stream_to_tagged_stream_0 = blocks.stream_to_tagged_stream(gr.sizeof_gr_complex, 1, 80, "su_pkt")
        self.blocks_stream_mux_0 = blocks.stream_mux(gr.sizeof_gr_complex*1, (num*8/4*sps2, 10000))
        self.blocks_null_sink_0 = blocks.null_sink(gr.sizeof_gr_complex*1)
        self.blocks_multiply_const_vxx_1 = blocks.multiply_const_vcc((1e-2, ))
        self.blocks_multiply_const_vxx_0 = blocks.multiply_const_vcc((2, ))
        self.blocks_add_xx_0 = blocks.add_vcc(1)
        self.blocks_add_const_vxx_0 = blocks.add_const_vcc((-1, ))
        self.analog_random_source_x_1 = blocks.vector_source_b(map(int, numpy.random.randint(0, 255, num)), True)
        self.analog_const_source_x_0 = analog.sig_source_c(0, analog.GR_CONST_WAVE, 0, 0, 0)

        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_const_source_x_0, 0), (self.blocks_stream_mux_0, 1))
        self.connect((self.analog_random_source_x_1, 0), (self.blocks_stream_to_tagged_stream_0_0, 0))
        self.connect((self.blocks_add_const_vxx_0, 0), (self.blocks_stream_to_tagged_stream_0, 0))
        self.connect((self.blocks_add_xx_0, 0), (self.blocks_throttle_0, 0))
        self.connect((self.blocks_multiply_const_vxx_0, 0), (self.blocks_add_const_vxx_0, 0))
        self.connect((self.blocks_multiply_const_vxx_1, 0), (self.blocks_stream_mux_0, 0))
        self.connect((self.blocks_stream_mux_0, 0), (self.blocks_add_xx_0, 1))
        self.connect((self.blocks_stream_to_tagged_stream_0, 0), (self.interp_fir_filter_xxx_0, 0))
        self.connect((self.blocks_stream_to_tagged_stream_0_0, 0), (self.digital_qam_mod_0, 0))
        self.connect((self.blocks_throttle_0, 0), (self.channels_channel_model_0, 0))
        self.connect((self.blocks_throttle_0, 0), (self.qtgui_time_sink_x_0, 0))
        self.connect((self.blocks_vector_source_x_0, 0), (self.blocks_multiply_const_vxx_0, 0))
        self.connect((self.channels_channel_model_0, 0), (self.lsa_eng_det_cc_0, 0))
        self.connect((self.digital_qam_mod_0, 0), (self.blocks_multiply_const_vxx_1, 0))
        self.connect((self.interp_fir_filter_xxx_0, 0), (self.blocks_add_xx_0, 0))
        self.connect((self.lsa_eng_det_cc_0, 0), (self.blocks_null_sink_0, 0))

    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "sim_interference")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

    def get_sps(self):
        return self.sps

    def set_sps(self, sps):
        self.sps = sps
        self.set_rxmod(digital.generic_mod(self.mod, False, self.sps, True, self.eb1, False, False))

    def get_mod(self):
        return self.mod

    def set_mod(self, mod):
        self.mod = mod
        self.set_rxmod(digital.generic_mod(self.mod, False, self.sps, True, self.eb1, False, False))

    def get_eb1(self):
        return self.eb1

    def set_eb1(self, eb1):
        self.eb1 = eb1
        self.set_rxmod(digital.generic_mod(self.mod, False, self.sps, True, self.eb1, False, False))

    def get_rxmod(self):
        return self.rxmod

    def set_rxmod(self, rxmod):
        self.rxmod = rxmod

    def get_nfilts(self):
        return self.nfilts

    def set_nfilts(self, nfilts):
        self.nfilts = nfilts

    def get_eb2(self):
        return self.eb2

    def set_eb2(self, eb2):
        self.eb2 = eb2

    def get_ac_hex(self):
        return self.ac_hex

    def set_ac_hex(self, ac_hex):
        self.ac_hex = ac_hex

    def get_tx_rrc_taps(self):
        return self.tx_rrc_taps

    def set_tx_rrc_taps(self, tx_rrc_taps):
        self.tx_rrc_taps = tx_rrc_taps
        self.interp_fir_filter_xxx_0.set_taps((self.tx_rrc_taps))

    def get_sps2(self):
        return self.sps2

    def set_sps2(self, sps2):
        self.sps2 = sps2

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.qtgui_time_sink_x_0.set_samp_rate(self.samp_rate)
        self.blocks_throttle_0.set_sample_rate(self.samp_rate)

    def get_rx_rrc_taps(self):
        return self.rx_rrc_taps

    def set_rx_rrc_taps(self, rx_rrc_taps):
        self.rx_rrc_taps = rx_rrc_taps

    def get_num(self):
        return self.num

    def set_num(self, num):
        self.num = num
        self.blocks_stream_to_tagged_stream_0_0.set_packet_len(self.num)
        self.blocks_stream_to_tagged_stream_0_0.set_packet_len_pmt(self.num)

    def get_modulated_sync_word(self):
        return self.modulated_sync_word

    def set_modulated_sync_word(self, modulated_sync_word):
        self.modulated_sync_word = modulated_sync_word

    def get_ac(self):
        return self.ac

    def set_ac(self, ac):
        self.ac = ac
        self.blocks_vector_source_x_0.set_data([1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0] + self.ac, [])


def main(top_block_cls=sim_interference, options=None):

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
