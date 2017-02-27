/* -*- c++ -*- */

#define LSA_API
#define DIGITAL_API
#define FILTER_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "lsa_swig_doc.i"

%include "gnuradio/digital/constellation.h"
%include "gnuradio/filter/fir_filter.h"

%{
#include "lsa/eng_det_cc.h"
#include "lsa/header_payload_parser_cb.h"
#include "lsa/su_header_prefix.h"
#include "lsa/su_sample_receiver_cb.h"
#include "lsa/su_transmitter_bc.h"
#include "lsa/prou_sample_receiver_cb.h"
#include "lsa/interference_energy_detector_cc.h"
#include "lsa/poly_costas_sync_cc.h"
#include "lsa/modified_polyphase_time_sync_cc.h"
#include "lsa/modified_costas_loop_cc.h"
#include "lsa/prou_sample_queue_cc.h"
#include "lsa/symbol_receiver_c.h"
%}

%include "lsa/eng_det_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, eng_det_cc);


%include "lsa/header_payload_parser_cb.h"
GR_SWIG_BLOCK_MAGIC2(lsa, header_payload_parser_cb);
%include "lsa/su_header_prefix.h"
GR_SWIG_BLOCK_MAGIC2(lsa, su_header_prefix);

%include "lsa/su_sample_receiver_cb.h"
GR_SWIG_BLOCK_MAGIC2(lsa, su_sample_receiver_cb);
%include "lsa/su_transmitter_bc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, su_transmitter_bc);
%include "lsa/prou_sample_receiver_cb.h"
GR_SWIG_BLOCK_MAGIC2(lsa, prou_sample_receiver_cb);
%include "lsa/interference_energy_detector_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, interference_energy_detector_cc);
%include "lsa/poly_costas_sync_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, poly_costas_sync_cc);
%include "lsa/modified_polyphase_time_sync_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, modified_polyphase_time_sync_cc);

%include "lsa/modified_costas_loop_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, modified_costas_loop_cc);
%include "lsa/prou_sample_queue_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, prou_sample_queue_cc);
%include "lsa/symbol_receiver_c.h"
GR_SWIG_BLOCK_MAGIC2(lsa, symbol_receiver_c);
