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
#include "lsa/square_cc.h"
#include "lsa/eng_det_cc.h"
#include "lsa/autoCorr_cc.h"
#include "lsa/correlate_extract_cc.h"
#include "lsa/my_access_corr_bb.h"
#include "lsa/header_payload_parser_cb.h"
#include "lsa/su_header_prefix.h"
#include "lsa/su_sample_receiver_cb.h"
#include "lsa/su_transmitter_bc.h"
#include "lsa/prou_sample_receiver_cb.h"
%}


%include "lsa/square_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, square_cc);
%include "lsa/eng_det_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, eng_det_cc);
%include "lsa/autoCorr_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, autoCorr_cc);
%include "lsa/correlate_extract_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, correlate_extract_cc);
%include "lsa/my_access_corr_bb.h"
GR_SWIG_BLOCK_MAGIC2(lsa, my_access_corr_bb);
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
