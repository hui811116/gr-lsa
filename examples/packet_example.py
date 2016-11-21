#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Packet Example
# Generated: Mon Nov 21 11:29:20 2016
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
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
import pmt
import sip
import sys


class packet_example(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Packet Example")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Packet Example")
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

        self.settings = Qt.QSettings("GNU Radio", "packet_example")
        self.restoreGeometry(self.settings.value("geometry").toByteArray())

        ##################################################
        # Variables
        ##################################################
        self.sps = sps = 2
        self.nfilts = nfilts = 32
        self.eb = eb = 0.22
        
        self.Const_PLD = Const_PLD = digital.constellation_calcdist((digital.psk_4()[0]), (digital.psk_4()[1]), 4, 1).base()
        
        self.Const_PLD.gen_soft_dec_lut(8)
        
        self.tx_rrc_taps = tx_rrc_taps = firdes.root_raised_cosine(nfilts, nfilts, 1.0, eb, 5*sps*nfilts)
          
        self.samp_rate = samp_rate = 20000
        self.preamble = preamble = [1,-1,1,-1,1,1,-1,-1,1,1,-1,1,1,1,-1,1,1,-1,1,-1,-1,1,-1,-1,1,1,1,-1,-1,-1,1,-1,1,1,1,1,-1,-1,1,-1,1,-1,-1,-1,1,1,-1,-1,-1,-1,1,-1,-1,-1,-1,-1,1,1,1,1,1,1,-1,-1]
        self.hdr_format = hdr_format = digital.header_format_counter(digital.packet_utils.default_access_code, 3, Const_PLD.bits_per_symbol())
        
        self.constel = constel = digital.constellation_calcdist(([1,- 1]), ([0,1]), 2, 1).base()
        
        
        self.Const_HDR = Const_HDR = digital.constellation_calcdist((digital.psk_2()[0]), (digital.psk_2()[1]), 2, 1).base()
        
        self.Const_HDR.gen_soft_dec_lut(8)

        ##################################################
        # Blocks
        ##################################################
        self.tab0 = Qt.QTabWidget()
        self.tab0_widget_0 = Qt.QWidget()
        self.tab0_layout_0 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.tab0_widget_0)
        self.tab0_grid_layout_0 = Qt.QGridLayout()
        self.tab0_layout_0.addLayout(self.tab0_grid_layout_0)
        self.tab0.addTab(self.tab0_widget_0, 'Tab 0')
        self.top_layout.addWidget(self.tab0)
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
        
        for i in xrange(2*1):
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
        self.tab0_layout_0.addWidget(self._qtgui_time_sink_x_0_win)
        self.digital_protocol_formatter_async_0 = digital.protocol_formatter_async(hdr_format)
        self.digital_map_bb_1 = digital.map_bb((Const_PLD.pre_diff_code()))
        self.digital_map_bb_0 = digital.map_bb((Const_HDR.pre_diff_code()))
        self.digital_crc32_async_bb_0 = digital.crc32_async_bb(False)
        self.digital_chunks_to_symbols_xx_1 = digital.chunks_to_symbols_bc((Const_PLD.points()), 2)
        self.digital_chunks_to_symbols_xx_0 = digital.chunks_to_symbols_bc((Const_HDR.points()), 2)
        self.blocks_tagged_stream_mux_0 = blocks.tagged_stream_mux(gr.sizeof_gr_complex*1, 'packet_len', 0)
        self.blocks_repack_bits_bb_1 = blocks.repack_bits_bb(8, Const_HDR.bits_per_symbol(), 'packet_len', False, gr.GR_MSB_FIRST)
        self.blocks_repack_bits_bb_0 = blocks.repack_bits_bb(8, Const_PLD.bits_per_symbol(), 'packet_len', False, gr.GR_MSB_FIRST)
        self.blocks_random_pdu_0 = blocks.random_pdu(50, 200, chr(0xFF), 2)
        self.blocks_pdu_to_tagged_stream_1 = blocks.pdu_to_tagged_stream(blocks.byte_t, 'packet_len')
        self.blocks_pdu_to_tagged_stream_0 = blocks.pdu_to_tagged_stream(blocks.byte_t, 'packet_len')
        self.blocks_message_strobe_0 = blocks.message_strobe(pmt.intern("TEST"), 2000)

        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.blocks_message_strobe_0, 'strobe'), (self.digital_crc32_async_bb_0, 'in'))    
        self.msg_connect((self.blocks_random_pdu_0, 'pdus'), (self.blocks_message_strobe_0, 'set_msg'))    
        self.msg_connect((self.digital_crc32_async_bb_0, 'out'), (self.digital_protocol_formatter_async_0, 'in'))    
        self.msg_connect((self.digital_protocol_formatter_async_0, 'header'), (self.blocks_pdu_to_tagged_stream_0, 'pdus'))    
        self.msg_connect((self.digital_protocol_formatter_async_0, 'payload'), (self.blocks_pdu_to_tagged_stream_1, 'pdus'))    
        self.connect((self.blocks_pdu_to_tagged_stream_0, 0), (self.blocks_repack_bits_bb_1, 0))    
        self.connect((self.blocks_pdu_to_tagged_stream_1, 0), (self.blocks_repack_bits_bb_0, 0))    
        self.connect((self.blocks_repack_bits_bb_0, 0), (self.digital_map_bb_1, 0))    
        self.connect((self.blocks_repack_bits_bb_1, 0), (self.digital_map_bb_0, 0))    
        self.connect((self.blocks_tagged_stream_mux_0, 0), (self.qtgui_time_sink_x_0, 0))    
        self.connect((self.digital_chunks_to_symbols_xx_0, 0), (self.blocks_tagged_stream_mux_0, 0))    
        self.connect((self.digital_chunks_to_symbols_xx_1, 0), (self.blocks_tagged_stream_mux_0, 1))    
        self.connect((self.digital_map_bb_0, 0), (self.digital_chunks_to_symbols_xx_0, 0))    
        self.connect((self.digital_map_bb_1, 0), (self.digital_chunks_to_symbols_xx_1, 0))    

    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "packet_example")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

    def get_sps(self):
        return self.sps

    def set_sps(self, sps):
        self.sps = sps

    def get_nfilts(self):
        return self.nfilts

    def set_nfilts(self, nfilts):
        self.nfilts = nfilts

    def get_eb(self):
        return self.eb

    def set_eb(self, eb):
        self.eb = eb

    def get_Const_PLD(self):
        return self.Const_PLD

    def set_Const_PLD(self, Const_PLD):
        self.Const_PLD = Const_PLD

    def get_tx_rrc_taps(self):
        return self.tx_rrc_taps

    def set_tx_rrc_taps(self, tx_rrc_taps):
        self.tx_rrc_taps = tx_rrc_taps

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.qtgui_time_sink_x_0.set_samp_rate(self.samp_rate)

    def get_preamble(self):
        return self.preamble

    def set_preamble(self, preamble):
        self.preamble = preamble

    def get_hdr_format(self):
        return self.hdr_format

    def set_hdr_format(self, hdr_format):
        self.hdr_format = hdr_format

    def get_constel(self):
        return self.constel

    def set_constel(self, constel):
        self.constel = constel

    def get_Const_HDR(self):
        return self.Const_HDR

    def set_Const_HDR(self, Const_HDR):
        self.Const_HDR = Const_HDR


def main(top_block_cls=packet_example, options=None):

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
