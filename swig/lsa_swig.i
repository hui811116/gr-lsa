/* -*- c++ -*- */

#define LSA_API
#define DIGITAL_API
#define FILTER_API
#define BLOCKS_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "lsa_swig_doc.i"

%{
#include "lsa/eng_det_cc.h"
#include "lsa/interference_energy_detector_cc.h"
#include "lsa/modified_costas_loop_cc.h"
#include "lsa/burst_tagger_cc.h"
#include "lsa/correlate_sync_cc.h"
#include "lsa/preamble_prefixer.h"
#include "lsa/su_ctrl.h"
#include "lsa/chip_mapper.h"
#include "lsa/moving_average_cc.h"
#include "lsa/moving_average_ff.h"
#include "lsa/coarse_sync_cc.h"
#include "lsa/prou_packet_sink_f.h"
#include "lsa/interference_tagger_cc.h"
#include "lsa/block_tagger_cc.h"
#include "lsa/su_sr_transmitter_bb.h"
#include "lsa/su_tx_helper.h"
#include "lsa/su_packet_sink_c.h"
#include "lsa/throughput_report.h"
#include "lsa/stop_n_wait_tx_bb.h"
#include "lsa/stop_n_wait_rx_ctrl_cc.h"
#include "lsa/stop_n_wait_ack.h"
#include "lsa/sns_tx_helper.h"
#include "lsa/burst_gate_cc.h"
#include "lsa/message_file_sink.h"
#include "lsa/sfd_tagger_cc.h"
#include "lsa/simple_tx.h"
#include "lsa/simple_rx.h"
#include "lsa/phy_crc.h"
#include "lsa/byte_to_symbol_bc.h"
#include "lsa/throughput_file_sink.h"
#include "lsa/ic_resync_cc.h"
#include "lsa/su_block_receiver_c.h"
#include "lsa/ic_ncfo_cc.h"
#include "lsa/arq_tx.h"
%}

%include "gnuradio/digital/constellation.h"
%{
#include "gnuradio/filter/fir_filter.h"
%}
%include "gnuradio/filter/fir_filter.h"
%{
#include "gnuradio/blocks/count_bits.h"
%}
%include "gnuradio/blocks/count_bits.h"

%include "lsa/eng_det_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, eng_det_cc);
%include "lsa/interference_energy_detector_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, interference_energy_detector_cc);
%include "lsa/modified_costas_loop_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, modified_costas_loop_cc);
%include "lsa/burst_tagger_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, burst_tagger_cc);
%include "lsa/correlate_sync_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, correlate_sync_cc);
%include "lsa/preamble_prefixer.h"
GR_SWIG_BLOCK_MAGIC2(lsa, preamble_prefixer);
%include "lsa/su_ctrl.h"
GR_SWIG_BLOCK_MAGIC2(lsa, su_ctrl);
%include "lsa/chip_mapper.h"
GR_SWIG_BLOCK_MAGIC2(lsa, chip_mapper);
%include "lsa/moving_average_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, moving_average_cc);
%include "lsa/moving_average_ff.h"
GR_SWIG_BLOCK_MAGIC2(lsa, moving_average_ff);
%include "lsa/coarse_sync_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, coarse_sync_cc);
%include "lsa/prou_packet_sink_f.h"
GR_SWIG_BLOCK_MAGIC2(lsa, prou_packet_sink_f);
%include "lsa/interference_tagger_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, interference_tagger_cc);
%include "lsa/block_tagger_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, block_tagger_cc);
%include "lsa/su_sr_transmitter_bb.h"
GR_SWIG_BLOCK_MAGIC2(lsa, su_sr_transmitter_bb);
%include "lsa/su_tx_helper.h"
GR_SWIG_BLOCK_MAGIC2(lsa, su_tx_helper);
%include "lsa/su_packet_sink_c.h"
GR_SWIG_BLOCK_MAGIC2(lsa, su_packet_sink_c);
%include "lsa/throughput_report.h"
GR_SWIG_BLOCK_MAGIC2(lsa, throughput_report);
%include "lsa/stop_n_wait_tx_bb.h"
GR_SWIG_BLOCK_MAGIC2(lsa, stop_n_wait_tx_bb);
%include "lsa/stop_n_wait_rx_ctrl_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, stop_n_wait_rx_ctrl_cc);
%include "lsa/stop_n_wait_ack.h"
GR_SWIG_BLOCK_MAGIC2(lsa, stop_n_wait_ack);
%include "lsa/sns_tx_helper.h"
GR_SWIG_BLOCK_MAGIC2(lsa, sns_tx_helper);
%include "lsa/burst_gate_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, burst_gate_cc);
%include "lsa/message_file_sink.h"
GR_SWIG_BLOCK_MAGIC2(lsa, message_file_sink);
%include "lsa/sfd_tagger_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, sfd_tagger_cc);
%include "lsa/simple_tx.h"
GR_SWIG_BLOCK_MAGIC2(lsa, simple_tx);
%include "lsa/simple_rx.h"
GR_SWIG_BLOCK_MAGIC2(lsa, simple_rx);
%include "lsa/phy_crc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, phy_crc);
%include "lsa/byte_to_symbol_bc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, byte_to_symbol_bc);
%include "lsa/throughput_file_sink.h"
GR_SWIG_BLOCK_MAGIC2(lsa, throughput_file_sink);
%include "lsa/ic_resync_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, ic_resync_cc);
%include "lsa/su_block_receiver_c.h"
GR_SWIG_BLOCK_MAGIC2(lsa, su_block_receiver_c);
%include "lsa/ic_ncfo_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, ic_ncfo_cc);
%include "lsa/arq_tx.h"
GR_SWIG_BLOCK_MAGIC2(lsa, arq_tx);
