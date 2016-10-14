/* -*- c++ -*- */
/* 
 * Copyright 2016 DRS.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_FLEXFFT_FLEX_FFT_SINK_IMPL_H
#define INCLUDED_FLEXFFT_FLEX_FFT_SINK_IMPL_H

#include <flexfft/flex_fft_sink.h>
#include <flexfft/timedisplayform.h>

#include "flex_fft_parser.h"

#include <Python.h>
#include <QApplication>

/**
 * @def FLEX_RATE_TAG 
 * @brief The string used to identify the flex streams sample 
 *        rate.
 *  
 * FLEX_RATE_TAG defines a string used for identifying the flex 
 * fft stream tag for sample rate. 
 */
#define FLEX_RATE_TAG "flex_rate"

/**
 * @def FLEX_SIZE_TAG 
 * @brief The string used to identify the flex streams fft size.
 *  
 * FLEX_SIZE_TAG defines a string used for identifying the flex 
 * fft stream tag for fft size.
 */
#define FLEX_SIZE_TAG "flex_size"

/**
 * @def FLEX_REF_TAG 
 * @brief The string used to identify the flex streams reference 
 *        level.
 *  
 * FLEX_REF_TAG defines a string used for identifying the flex 
 * fft stream tag for reference level.
 */
#define FLEX_REF_TAG "flex_rlvl"

/**
 * @def FLEX_AVE_TAG 
 * @brief The string used to identify the flex streams 
 *        averaging.
 *  
 * FLEX_AVE_TAG defines a string used for identifying the flex 
 * fft stream tag for averaging. 
 */
#define FLEX_AVE_TAG "flex_nave"

/**
 * @def FLEX_FREQ_TAG 
 * @brief The string used to identify the flex streams 
 *        frequency.
 *  
 * FLEX_FREQ_TAG defines a string used for identifying the flex 
 * fft stream tag for frequency.
 */
#define FLEX_FREQ_TAG "flex_freq"

/**
 * @def DEFAULT_MINIMUM 
 * @brief The minimum Y value used when auto adjust is enabled. 
 *  
 * The lowest Y value used when auto adjust is enabled. 
 */
#define DEFAULT_MINIMUM -140

namespace gr
{
namespace flexfft
{

class flex_fft_sink_impl : public flex_fft_sink
{
private:
    flex_fft_parser d_parser;
    int d_stream_id;
    int d_packet_count;

    void initialize();

    int d_size, d_buffer_size;
    double d_frequency, d_sample_rate, d_axis_max;
    double d_samp_rate;
    std::string d_name;
    int d_nconnections;

    int d_index, d_start, d_end;
    std::vector<float *> d_fbuffers;
    std::vector<double *> d_buffers;
    std::vector<std::vector<gr::tag_t> > d_tags;

    int d_argc;
    char *d_argv;
    QWidget *d_parent;
    TimeDisplayForm *d_main_gui;

    gr::high_res_timer_type d_update_time;
    gr::high_res_timer_type d_last_time;

    // Members used for triggering scope
    trigger_mode d_trigger_mode;
    trigger_slope d_trigger_slope;
    float d_trigger_level;
    int d_trigger_channel;
    int d_trigger_delay;
    int d_input_size;
    pmt::pmt_t d_trigger_tag_key;
    bool d_triggered;
    int d_trigger_count;
    int d_initial_delay; // used for limiting d_trigger_delay
    bool d_auto_adjust;
    double d_y_min, d_y_max;

    void _reset();
    void _npoints_resize();
    void _adjust_tags(int adj);
    void _gui_update_trigger();
    void _test_trigger_tags(int nitems);
    void _test_trigger_norm(int nitems, const float* input);
    bool _test_trigger_slope(const float *in) const;
public:
    flex_fft_sink_impl(int input_size, QWidget *parent);
    ~flex_fft_sink_impl();

    QWidget*  qwidget();
    PyObject* pyqwidget();

    QApplication *d_qApplication;

    bool check_topology(int ninputs, int noutputs);
    void exec_(); 

    void set_y_axis(double min, double max);
    void set_y_label(const std::string &label,
                     const std::string &unit = "");
    void set_update_time(double t);
    void set_title(const std::string &title);
    void set_line_label(int which, const std::string &label);
    void set_line_color(int which, const std::string &color);
    void set_line_width(int which, int width);
    void set_line_style(int which, int style);
    void set_line_marker(int which, int marker);
    void set_nsamps(const int size);
    void set_samp_rate(const double samp_rate);
    void set_line_alpha(int which, double alpha);
    void set_trigger_mode(trigger_mode mode, trigger_slope slope,
                          float level, float delay, int channel,
                          const std::string &tag_key = "");

    void set_stream_id(int stream_id);
    void set_auto_adjust(bool auto_adjust);

    std::string title();
    std::string line_label(int which);
    std::string line_color(int which);
    int line_width(int which);
    int line_style(int which);
    int line_marker(int which);
    double line_alpha(int which);

    void set_size(int width, int height);

    int nsamps() const;

    void enable_menu(bool en);
    void enable_grid(bool en);
    void enable_autoscale(bool en);
    void enable_stem_plot(bool en);
    void enable_semilogx(bool en);
    void enable_semilogy(bool en);
    void enable_control_panel(bool en);
    void enable_tags(int which, bool en);
    void disable_legend();

    void reset();

    int work(int noutput_items,
             gr_vector_const_void_star &input_items,
             gr_vector_void_star &output_items);
};

} // namespace flexfft
} // namespace gr

#endif /* INCLUDED_FLEXFFT_FLEX_FFT_SINK_IMPL_H */

