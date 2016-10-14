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


#ifndef INCLUDED_FLEXFFT_FLEX_FFT_SINK_H
#define INCLUDED_FLEXFFT_FLEX_FFT_SINK_H

#include <flexfft/api.h>
#include <flexfft/trigger_mode.h>
#include <gnuradio/sync_block.h>
#include <QWidget>
#include <Python.h>

class QApplication;
class ShowPngPicture;

namespace gr {
  namespace flexfft {

    /*!
     * \brief <+description of block+>
     * \ingroup flexfft
     *
     */
    class FLEXFFT_API flex_fft_sink : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<flex_fft_sink> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of flexfft::flex_fft_sink.
       *
       * To avoid accidental use of raw pointers, flexfft::flex_fft_sink's
       * constructor is in a private implementation
       * class. flexfft::flex_fft_sink::make is the public interface for
       * creating new instances.
       */
      static sptr make(int input_size, QWidget* parent = NULL);

      virtual void exec_() = 0;
      virtual QWidget* qwidget() = 0;
      virtual PyObject* pyqwidget() = 0;

      virtual void set_y_axis(double min, double max) = 0;
      virtual void set_y_label(const std::string &label,
                               const std::string &unit="") = 0;
      virtual void set_update_time(double t) = 0;
      virtual void set_title(const std::string &title) = 0;
      virtual void set_line_label(int which, const std::string &label) = 0;
      virtual void set_line_color(int which, const std::string &color) = 0;
      virtual void set_line_width(int which, int width) = 0;
      virtual void set_line_style(int which, int style) = 0;
      virtual void set_line_marker(int which, int marker) = 0;
      virtual void set_nsamps(const int newsize) = 0;
      virtual void set_samp_rate(const double samp_rate) = 0;
      virtual void set_line_alpha(int which, double alpha) = 0;

      virtual void set_stream_id(int stream_id) = 0;
      virtual void set_auto_adjust(bool auto_adjust) = 0;

      /*!
       * Set up a trigger for the sink to know when to start
       * plotting. Useful to isolate events and avoid noise.
       *
       * The trigger modes are Free, Auto, Normal, and Tag (see
       * gr::qtgui::trigger_mode). The first three are like a normal
       * oscope trigger function. Free means free running with no
       * trigger, auto will trigger if the trigger event is seen, but
       * will still plot otherwise, and normal will hold until the
       * trigger event is observed. The Tag trigger mode allows us to
       * trigger off a specific stream tag. The tag trigger is based
       * only on the name of the tag, so when a tag of the given name
       * is seen, the trigger is activated.
       *
       * In auto and normal mode, we look for the slope of the of the
       * signal. Given a gr::qtgui::trigger_slope as either Positive
       * or Negative, if the value between two samples moves in the
       * given direction (x[1] > x[0] for Positive or x[1] < x[0] for
       * Negative), then the trigger is activated.
       *
       * With the complex time sink, each input has two lines drawn
       * for the real and imaginary parts of the signal. When
       * selecting the \p channel value, channel 0 is the real signal
       * and channel 1 is the imaginary signal. For more than 1 input
       * stream, channel 2i is the real part of the ith input and
       * channel (2i+1) is the imaginary part of the ith input
       * channel.
       *
       * The \p delay value is specified in time based off the sample
       * rate. If the sample rate of the block is set to 1, the delay
       * is then also the sample number offset. This is the offset
       * from the left-hand y-axis of the plot. It delays the signal
       * to show the trigger event at the given delay along with some
       * portion of the signal before the event. The delay must be
       * within 0 - t_max where t_max is the maximum amount of time
       * displayed on the time plot.
       *
       * \param mode The trigger_mode: free, auto, normal, or tag.
       * \param slope The trigger_slope: positive or negative. Only
       *              used for auto and normal modes.
       * \param level The magnitude of the trigger even for auto or normal modes.
       * \param delay The delay (in units of time) for where the trigger happens.
       * \param channel Which input channel to use for the trigger events.
       * \param tag_key The name (as a string) of the tag to trigger off
       *                 of if using the tag mode.
       */
      virtual void set_trigger_mode(trigger_mode mode, trigger_slope slope,
                                    float level, float delay, int channel,
                                    const std::string &tag_key="") = 0;

      virtual std::string title() = 0;
      virtual std::string line_label(int which) = 0;
      virtual std::string line_color(int which) = 0;
      virtual int line_width(int which) = 0;
      virtual int line_style(int which) = 0;
      virtual int line_marker(int which) = 0;
      virtual double line_alpha(int which) = 0;

      virtual void set_size(int width, int height) = 0;

      virtual void enable_menu(bool en=true) = 0;
      virtual void enable_grid(bool en=true) = 0;
      virtual void enable_autoscale(bool en=true) = 0;
      virtual void enable_stem_plot(bool en=true) = 0;
      virtual void enable_semilogx(bool en=true) = 0;
      virtual void enable_semilogy(bool en=true) = 0;
      virtual void enable_control_panel(bool en=true) = 0;
      virtual void enable_tags(int which, bool en) = 0;
      virtual void disable_legend() = 0;

      virtual int nsamps() const = 0;
      virtual void reset() = 0;
    };

  } // namespace flexfft
} // namespace gr

#endif /* INCLUDED_FLEXFFT_FLEX_FFT_SINK_H */

