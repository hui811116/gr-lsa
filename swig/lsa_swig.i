/* -*- c++ -*- */

#define LSA_API
#define DIGITAL_API
#define FILTER_API
//#define FFT_API
#define BLOCKS_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "lsa_swig_doc.i"


%include "gnuradio/digital/constellation.h"
%include "gnuradio/filter/fir_filter.h"
#include "gnuradio/blocks/count_bits.h"


//#include "gnuradio/fft/fft.h"


%{
#include "lsa/eng_det_cc.h"
#include "lsa/interference_energy_detector_cc.h"
#include "lsa/modified_polyphase_time_sync_cc.h"
#include "lsa/modified_costas_loop_cc.h"
#include "lsa/prou_sample_queue_cc.h"
#include "lsa/interference_canceller_cc.h"
#include "lsa/burst_tagger_cc.h"
#include "lsa/protocol_parser_b.h"
#include "lsa/correlate_sync_cc.h"
#include "lsa/expand_symbal_to_sample_ff.h"
#include "lsa/symbol_sync_receiver_cf.h"
#include "lsa/preamble_prefixer.h"
#include "lsa/symbol_corrector_cc.h"
#include "lsa/control_source_b.h"
#include "lsa/app_simple.h"
#include "lsa/block_mac.h"
#include "lsa/su_ctrl.h"
#include "lsa/chip_mapper.h"
#include "lsa/packet_sink.h"
#include "lsa/su_transmitter_bb.h"
#include "lsa/chip_mapper_bb.h"
#include "lsa/moving_average_cc.h"
#include "lsa/moving_average_ff.h"
#include "lsa/coarse_sync_cc.h"
#include "lsa/stat_report.h"
#include "lsa/prou_packet_sink_f.h"
#include "lsa/interference_tagger_cc.h"
#include "lsa/su_mac.h"
#include "lsa/prou_ring_queue_cc.h"
#include "lsa/interference_cancellation_core_cc.h"
#include "lsa/block_tagger_cc.h"
#include "lsa/su_sr_transmitter_bb.h"
#include "lsa/su_tx_helper.h"
#include "lsa/ic_critical_cc.h"
#include "lsa/su_packet_sink_c.h"
#include "lsa/throughput_report.h"
#include "lsa/sinr_helper.h"
%}

%include "lsa/eng_det_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, eng_det_cc);
%include "lsa/interference_energy_detector_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, interference_energy_detector_cc);
%include "lsa/modified_polyphase_time_sync_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, modified_polyphase_time_sync_cc);
%include "lsa/modified_costas_loop_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, modified_costas_loop_cc);
%include "lsa/prou_sample_queue_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, prou_sample_queue_cc);
%include "lsa/interference_canceller_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, interference_canceller_cc);
%include "lsa/burst_tagger_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, burst_tagger_cc);
%include "lsa/protocol_parser_b.h"
GR_SWIG_BLOCK_MAGIC2(lsa, protocol_parser_b);
%include "lsa/correlate_sync_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, correlate_sync_cc);
%include "lsa/expand_symbal_to_sample_ff.h"
GR_SWIG_BLOCK_MAGIC2(lsa, expand_symbal_to_sample_ff);
%include "lsa/symbol_sync_receiver_cf.h"
GR_SWIG_BLOCK_MAGIC2(lsa, symbol_sync_receiver_cf);
%include "lsa/preamble_prefixer.h"
GR_SWIG_BLOCK_MAGIC2(lsa, preamble_prefixer);
%include "lsa/symbol_corrector_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, symbol_corrector_cc);
%include "lsa/control_source_b.h"
GR_SWIG_BLOCK_MAGIC2(lsa, control_source_b);
%include "lsa/app_simple.h"
GR_SWIG_BLOCK_MAGIC2(lsa, app_simple);
%include "lsa/block_mac.h"
GR_SWIG_BLOCK_MAGIC2(lsa, block_mac);
%include "lsa/su_ctrl.h"
GR_SWIG_BLOCK_MAGIC2(lsa, su_ctrl);
%include "lsa/chip_mapper.h"
GR_SWIG_BLOCK_MAGIC2(lsa, chip_mapper);
%include "lsa/packet_sink.h"
GR_SWIG_BLOCK_MAGIC2(lsa, packet_sink);
%include "lsa/su_transmitter_bb.h"
GR_SWIG_BLOCK_MAGIC2(lsa, su_transmitter_bb);
%include "lsa/chip_mapper_bb.h"
GR_SWIG_BLOCK_MAGIC2(lsa, chip_mapper_bb);
%include "lsa/moving_average_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, moving_average_cc);
%include "lsa/moving_average_ff.h"
GR_SWIG_BLOCK_MAGIC2(lsa, moving_average_ff);
%include "lsa/coarse_sync_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, coarse_sync_cc);
%include "lsa/stat_report.h"
GR_SWIG_BLOCK_MAGIC2(lsa, stat_report);
%include "lsa/prou_packet_sink_f.h"
GR_SWIG_BLOCK_MAGIC2(lsa, prou_packet_sink_f);
%include "lsa/interference_tagger_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, interference_tagger_cc);
%include "lsa/su_mac.h"
GR_SWIG_BLOCK_MAGIC2(lsa, su_mac);
%include "lsa/prou_ring_queue_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, prou_ring_queue_cc);
%include "lsa/interference_cancellation_core_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, interference_cancellation_core_cc);
%include "lsa/block_tagger_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, block_tagger_cc);
%include "lsa/su_sr_transmitter_bb.h"
GR_SWIG_BLOCK_MAGIC2(lsa, su_sr_transmitter_bb);
%include "lsa/su_tx_helper.h"
GR_SWIG_BLOCK_MAGIC2(lsa, su_tx_helper);
%include "lsa/ic_critical_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, ic_critical_cc);
%include "lsa/su_packet_sink_c.h"
GR_SWIG_BLOCK_MAGIC2(lsa, su_packet_sink_c);
%include "lsa/throughput_report.h"
GR_SWIG_BLOCK_MAGIC2(lsa, throughput_report);
%include "lsa/sinr_helper.h"
GR_SWIG_BLOCK_MAGIC2(lsa, sinr_helper);
