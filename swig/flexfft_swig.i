/* -*- c++ -*- */

#define FLEXFFT_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "flexfft_swig_doc.i"

enum graph_t {
  NUM_GRAPH_NONE = 0,
  NUM_GRAPH_HORIZ,
  NUM_GRAPH_VERT,
};

enum{
  INTENSITY_COLOR_MAP_TYPE_MULTI_COLOR = 0,
  INTENSITY_COLOR_MAP_TYPE_WHITE_HOT = 1,
  INTENSITY_COLOR_MAP_TYPE_BLACK_HOT = 2,
  INTENSITY_COLOR_MAP_TYPE_INCANDESCENT = 3,
  INTENSITY_COLOR_MAP_TYPE_USER_DEFINED = 4,
  INTENSITY_COLOR_MAP_TYPE_SUNSET = 5,
  INTENSITY_COLOR_MAP_TYPE_COOL = 6,
};


%include "flexfft/trigger_mode.h"

// So we understand the firdes window types
%import "gnuradio/filter/firdes.h"

%{
#include "flexfft/form_menus.h"
#include "flexfft/DisplayPlot.h"
#include "flexfft/displayform.h"
#include "flexfft/flex_fft_sink.h"
%}


%include "flexfft/flex_fft_sink.h"
GR_SWIG_BLOCK_MAGIC2(flexfft, flex_fft_sink);
