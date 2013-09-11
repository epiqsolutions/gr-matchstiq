/* -*- c++ -*- */

#define MATCHSTIQ_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "matchstiq_swig_doc.i"

%{
#include "matchstiq/matchstiq_source_s.h"
%}

%include "matchstiq/matchstiq_defs.h"

%include "matchstiq/matchstiq_source_s.h"
GR_SWIG_BLOCK_MAGIC2(matchstiq, matchstiq_source_s);
