/* -*- c++ -*- */

#define LSA_API
#define DIGITAL_API
#define FILTER_API
#define FFT_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "lsa_swig_doc.i"

%include "gnuradio/digital/constellation.h"
%include "gnuradio/filter/fir_filter.h"
#include "gnuradio/fft/fft.h"

%{
#include "lsa/eng_det_cc.h"
#include "lsa/header_payload_parser_cb.h"
#include "lsa/su_header_prefix.h"
#include "lsa/su_transmitter_bc.h"
#include "lsa/interference_energy_detector_cc.h"
#include "lsa/modified_polyphase_time_sync_cc.h"
#include "lsa/modified_costas_loop_cc.h"
#include "lsa/prou_sample_queue_cc.h"
#include "lsa/interference_canceller_cc.h"
#include "lsa/burst_tagger_cc.h"
#include "lsa/protocol_parser_b.h"
#include "lsa/correlate_sync_cc.h"
#include "lsa/symbol_queue_receiver_cc.h"
#include "lsa/symbol_level_ic_cc.h"
#include "lsa/expand_symbal_to_sample_ff.h"
#include "lsa/symbol_sync_receiver_cf.h"
#include "lsa/burst_synchronizer_cc.h"
%}

%include "lsa/eng_det_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, eng_det_cc);


%include "lsa/header_payload_parser_cb.h"
GR_SWIG_BLOCK_MAGIC2(lsa, header_payload_parser_cb);
%include "lsa/su_header_prefix.h"
GR_SWIG_BLOCK_MAGIC2(lsa, su_header_prefix);

%include "lsa/su_transmitter_bc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, su_transmitter_bc);

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
%include "lsa/symbol_queue_receiver_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, symbol_queue_receiver_cc);
%include "lsa/symbol_level_ic_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, symbol_level_ic_cc);

%include "lsa/expand_symbal_to_sample_ff.h"
GR_SWIG_BLOCK_MAGIC2(lsa, expand_symbal_to_sample_ff);
%include "lsa/symbol_sync_receiver_cf.h"
GR_SWIG_BLOCK_MAGIC2(lsa, symbol_sync_receiver_cf);
%include "lsa/burst_synchronizer_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, burst_synchronizer_cc);
