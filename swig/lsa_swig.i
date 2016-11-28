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
%}


%include "lsa/square_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, square_cc);
%include "lsa/eng_det_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, eng_det_cc);
%include "lsa/lsa_queue_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, lsa_queue_cc);
%include "lsa/autoCorr_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, autoCorr_cc);
