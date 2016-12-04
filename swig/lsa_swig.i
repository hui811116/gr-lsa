/* -*- c++ -*- */

#define LSA_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "lsa_swig_doc.i"

%{
#include "lsa/square_cc.h"
#include "lsa/eng_det_cc.h"
#include "lsa/lsa_queue_cc.h"
#include "lsa/autoCorr_cc.h"
#include "lsa/correlate_extract_cc.h"
#include "lsa/my_access_corr_bb.h"
#include "lsa/header_payload_parser_cb.h"
%}


%include "lsa/square_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, square_cc);
%include "lsa/eng_det_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, eng_det_cc);
%include "lsa/lsa_queue_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, lsa_queue_cc);
%include "lsa/autoCorr_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, autoCorr_cc);
%include "lsa/correlate_extract_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, correlate_extract_cc);
%include "lsa/my_access_corr_bb.h"
GR_SWIG_BLOCK_MAGIC2(lsa, my_access_corr_bb);
%include "lsa/header_payload_parser_cb.h"
GR_SWIG_BLOCK_MAGIC2(lsa, header_payload_parser_cb);
