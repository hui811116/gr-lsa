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
#include "lsa/burst_tagger_cc.h"
#include "lsa/correlate_sync_cc.h"
#include "lsa/expand_symbol_to_sample_ff.h"
#include "lsa/symbol_sync_receiver_cf.h"
#include "lsa/preamble_prefixer.h"
#include "lsa/su_ctrl.h"
#include "lsa/chip_mapper.h"
#include "lsa/packet_sink.h"
#include "lsa/chip_mapper_bb.h"
#include "lsa/moving_average_cc.h"
#include "lsa/moving_average_ff.h"
#include "lsa/coarse_sync_cc.h"
#include "lsa/stat_report.h"
#include "lsa/prou_packet_sink_f.h"
#include "lsa/interference_tagger_cc.h"
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


%include "lsa/burst_tagger_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, burst_tagger_cc);

%include "lsa/correlate_sync_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, correlate_sync_cc);
%include "lsa/expand_symbol_to_sample_ff.h"
GR_SWIG_BLOCK_MAGIC2(lsa, expand_symbol_to_sample_ff);
%include "lsa/symbol_sync_receiver_cf.h"
GR_SWIG_BLOCK_MAGIC2(lsa, symbol_sync_receiver_cf);
%include "lsa/preamble_prefixer.h"
GR_SWIG_BLOCK_MAGIC2(lsa, preamble_prefixer);



%include "lsa/su_ctrl.h"
GR_SWIG_BLOCK_MAGIC2(lsa, su_ctrl);
%include "lsa/chip_mapper.h"
GR_SWIG_BLOCK_MAGIC2(lsa, chip_mapper);
%include "lsa/packet_sink.h"
GR_SWIG_BLOCK_MAGIC2(lsa, packet_sink);

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
