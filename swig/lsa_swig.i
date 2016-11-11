/* -*- c++ -*- */

#define LSA_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "lsa_swig_doc.i"

%{
#include "lsa/square_cc.h"
%}


%include "lsa/square_cc.h"
GR_SWIG_BLOCK_MAGIC2(lsa, square_cc);
