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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "flex_fft_sink_impl.h"
#include <gnuradio/block_detail.h>
#include <gnuradio/buffer.h>
#include <gnuradio/prefs.h>
#include <volk/volk.h>

namespace gr
{
namespace flexfft
{

flex_fft_sink::sptr
flex_fft_sink::make(int input_size, QWidget *parent)
{
    return gnuradio::get_initial_sptr
           (new flex_fft_sink_impl(input_size, parent));
}

/*
 * The private constructor
 */
flex_fft_sink_impl::flex_fft_sink_impl(int input_size, QWidget *parent)
    : gr::sync_block("flex_fft_sink",
                     gr::io_signature::make(1, 1, input_size),
                     gr::io_signature::make(0, 0, 0))
{
    // Required now for Qt; argc must be greater than 0 and argv
    // must have at least one valid character. Must be valid through
    // life of the qApplication:
    // http://harmattan-dev.nokia.com/docs/library/html/qt4/qapplication.html
    d_argc = 1;
    d_argv = new char;
    d_argv[0] = '\0';

    d_main_gui = NULL;

    d_nconnections = 1;
    d_size = 1024;
    d_input_size = input_size;
    d_buffer_size = (2 * d_size);
    d_name = "Flex FFT Sink";
    d_parent = parent;
    d_samp_rate = 1e6;
    d_stream_id = -1;
    d_packet_count = -1;

    for (int n = 0; n < d_nconnections; n++) {
        d_buffers.push_back((double *)volk_malloc(d_buffer_size * sizeof(double),
                                                  volk_get_alignment()));
        memset(d_buffers[n], 0, d_buffer_size * sizeof(double));

        d_fbuffers.push_back((float *)volk_malloc(d_buffer_size * sizeof(float),
                                                  volk_get_alignment()));
        memset(d_fbuffers[n], 0, d_buffer_size * sizeof(float));
    }

    // Set alignment properties for VOLK
    const int alignment_multiple =
        volk_get_alignment() / sizeof(float);
    set_alignment(std::max(1, alignment_multiple));

    d_tags = std::vector<std::vector<gr::tag_t> >(d_nconnections);
    initialize();

    d_auto_adjust = false;
    d_y_min = -1, d_y_max = 1;

    d_main_gui->setNPoints(d_size); // setup GUI box with size
    set_trigger_mode(TRIG_MODE_FREE, TRIG_SLOPE_POS, 0, 0, 0);

    set_history(2);          // so we can look ahead for the trigger slope
    declare_sample_delay(1); // delay the tags for a history of 2
}

/*
 * Our virtual destructor.
 */
flex_fft_sink_impl::~flex_fft_sink_impl()
{
    if (!d_main_gui->isClosed()) d_main_gui->close();

    // d_main_gui is a qwidget destroyed with its parent
    for (int n = 0; n < d_nconnections; n++) {
        volk_free(d_buffers[n]);
        volk_free(d_fbuffers[n]);
    }

    delete d_argv;
}

bool
flex_fft_sink_impl::check_topology(int ninputs, int noutputs)
{
    return ninputs == d_nconnections;
}

void
flex_fft_sink_impl::set_stream_id(int stream_id)
{
    d_stream_id = stream_id;
}

void
flex_fft_sink_impl::set_auto_adjust(bool auto_adjust)
{
    d_auto_adjust = auto_adjust;
}


void
flex_fft_sink_impl::initialize()
{
    if (qApp != NULL) {
        d_qApplication = qApp;
    } else {
#if QT_VERSION >= 0x040500
        std::string style = prefs::singleton()->get_string("qtgui", "style", "raster");
        QApplication::setGraphicsSystem(QString(style.c_str()));
#endif
        d_qApplication = new QApplication(d_argc, &d_argv);
    }

    // If a style sheet is set in the prefs file, enable it here.
    std::string qssfile = prefs::singleton()->get_string("qtgui", "qss", "");
    if (qssfile.size() > 0) {
        QString sstext = get_qt_style_sheet(QString(qssfile.c_str()));
        d_qApplication->setStyleSheet(sstext);
    }

    d_main_gui = new TimeDisplayForm(d_nconnections, d_parent);
    d_main_gui->setNPoints(d_size);
    d_main_gui->setSampleRate(d_samp_rate);

    if (d_name.size() > 0) set_title(d_name);

    // initialize update time to 10 times a second
    set_update_time(0.1);
}

void
flex_fft_sink_impl::exec_()
{
    d_qApplication->exec();
}

QWidget*
flex_fft_sink_impl::qwidget()
{
    return d_main_gui;
}

PyObject*
flex_fft_sink_impl::pyqwidget()
{
    PyObject *w = PyLong_FromVoidPtr((void *)d_main_gui);
    PyObject *retarg = Py_BuildValue("N", w);
    return retarg;
}

void
flex_fft_sink_impl::set_y_axis(double min, double max)
{
    d_y_min = min;
    d_y_max = max;
    d_main_gui->setYaxis(min, max);
}

void
flex_fft_sink_impl::set_y_label(const std::string &label,
                                const std::string &unit)
{
    d_main_gui->setYLabel(label, unit);
}

void
flex_fft_sink_impl::set_update_time(double t)
{
    //convert update time to ticks
    gr::high_res_timer_type tps = gr::high_res_timer_tps();
    d_update_time = t * tps;
    d_main_gui->setUpdateTime(t);
    d_last_time = 0;
}

void
flex_fft_sink_impl::set_title(const std::string &title)
{
    if (!title.empty()) {
        d_main_gui->setTitle(title.c_str());
    }
}

void
flex_fft_sink_impl::set_line_label(int which, const std::string &label)
{
    d_main_gui->setLineLabel(which, label.c_str());
}

void
flex_fft_sink_impl::set_line_color(int which, const std::string &color)
{
    d_main_gui->setLineColor(which, color.c_str());
}

void
flex_fft_sink_impl::set_line_width(int which, int width)
{
    d_main_gui->setLineWidth(which, width);
}

void
flex_fft_sink_impl::set_line_style(int which, int style)
{
    d_main_gui->setLineStyle(which, (Qt::PenStyle)style);
}

void
flex_fft_sink_impl::set_line_marker(int which, int marker)
{
    d_main_gui->setLineMarker(which, (QwtSymbol::Style)marker);
}

void
flex_fft_sink_impl::set_line_alpha(int which, double alpha)
{
    d_main_gui->setMarkerAlpha(which, (int)(255.0 * alpha));
}

void
flex_fft_sink_impl::set_trigger_mode(trigger_mode mode,
                                     trigger_slope slope,
                                     float level,
                                     float delay, int channel,
                                     const std::string &tag_key)
{
    gr::thread::scoped_lock lock(d_setlock);

    d_trigger_mode = mode;
    d_trigger_slope = slope;
    d_trigger_level = level;
    d_trigger_delay = 0;
    d_trigger_channel = channel;
    d_trigger_tag_key = pmt::intern(tag_key);
    d_triggered = false;
    d_trigger_count = 0;

    if ((d_trigger_delay < 0) || (d_trigger_delay >= d_size)) {
        GR_LOG_WARN(d_logger, boost::format("Trigger delay (%1%) outside of display range (0:%2%).")\
                        % (d_trigger_delay / d_samp_rate) % ((d_size - 1) / d_samp_rate));
        d_trigger_delay = std::max(0, std::min(d_size - 1, d_trigger_delay));
        delay = d_trigger_delay / d_samp_rate;
    }

    d_main_gui->setTriggerMode(d_trigger_mode);
    d_main_gui->setTriggerSlope(d_trigger_slope);
    d_main_gui->setTriggerLevel(d_trigger_level);
    d_main_gui->setTriggerDelay(delay);
    d_main_gui->setTriggerChannel(d_trigger_channel);
    d_main_gui->setTriggerTagKey(tag_key);

    _reset();
}

void
flex_fft_sink_impl::set_size(int width, int height)
{
    d_main_gui->resize(QSize(width, height));
}

std::string
flex_fft_sink_impl::title()
{
    return d_main_gui->title().toStdString();
}

std::string
flex_fft_sink_impl::line_label(int which)
{
    return d_main_gui->lineLabel(which).toStdString();
}

std::string
flex_fft_sink_impl::line_color(int which)
{
    return d_main_gui->lineColor(which).toStdString();
}

int
flex_fft_sink_impl::line_width(int which)
{
    return d_main_gui->lineWidth(which);
}

int
flex_fft_sink_impl::line_style(int which)
{
    return d_main_gui->lineStyle(which);
}

int
flex_fft_sink_impl::line_marker(int which)
{
    return d_main_gui->lineMarker(which);
}

double
flex_fft_sink_impl::line_alpha(int which)
{
    return (double)(d_main_gui->markerAlpha(which)) / 255.0;
}

void
flex_fft_sink_impl::set_nsamps(const int newsize)
{
    if (newsize != d_size) {
        gr::thread::scoped_lock lock(d_setlock);

        // Set new size and reset buffer index
        // (throws away any currently held data, but who cares?)
        d_size = newsize;
        d_buffer_size = 2 * d_size;

        // Resize buffers and replace data
        for (int n = 0; n < d_nconnections; n++) {
            volk_free(d_buffers[n]);
            d_buffers[n] = (double *)volk_malloc(d_buffer_size * sizeof(double),
                                                 volk_get_alignment());
            memset(d_buffers[n], 0, d_buffer_size * sizeof(double));

            volk_free(d_fbuffers[n]);
            d_fbuffers[n] = (float *)volk_malloc(d_buffer_size * sizeof(float),
                                                 volk_get_alignment());
            memset(d_fbuffers[n], 0, d_buffer_size * sizeof(float));
        }

        // If delay was set beyond the new boundary, pull it back.
        if (d_trigger_delay >= d_size) {
            GR_LOG_WARN(d_logger, boost::format("Trigger delay (%1%) outside of display range (0:%2%). Moving to 50%% point.")\
                            % (d_trigger_delay / d_samp_rate) % ((d_size - 1) / d_samp_rate));
            d_trigger_delay = d_size - 1;
            d_main_gui->setTriggerDelay(d_trigger_delay / d_samp_rate);
        }

        d_main_gui->setNPoints(d_size);
        _reset();
    }
}

void
flex_fft_sink_impl::set_samp_rate(const double samp_rate)
{
    gr::thread::scoped_lock lock(d_setlock);
    d_samp_rate = samp_rate;
    d_main_gui->setSampleRate(d_samp_rate);
}

int
flex_fft_sink_impl::nsamps() const
{
    return d_size;
}

void
flex_fft_sink_impl::enable_stem_plot(bool en)
{
    d_main_gui->setStem(en);
}

void
flex_fft_sink_impl::enable_menu(bool en)
{
    d_main_gui->enableMenu(en);
}

void
flex_fft_sink_impl::enable_grid(bool en)
{
    d_main_gui->setGrid(en);
}

void
flex_fft_sink_impl::enable_autoscale(bool en)
{
    d_main_gui->autoScale(en);
}

void
flex_fft_sink_impl::enable_semilogx(bool en)
{
    d_main_gui->setSemilogx(en);
}

void
flex_fft_sink_impl::enable_semilogy(bool en)
{
    d_main_gui->setSemilogy(en);
}

void
flex_fft_sink_impl::enable_control_panel(bool en)
{
    if (en) d_main_gui->setupControlPanel();
    else d_main_gui->teardownControlPanel();
}

void
flex_fft_sink_impl::enable_tags(int which, bool en)
{
    if (which == -1) {
        for (int n = 0; n < d_nconnections; n++) {
            d_main_gui->setTagMenu(n, en);
        }
    } else d_main_gui->setTagMenu(which, en);
}

void
flex_fft_sink_impl::disable_legend()
{
    d_main_gui->disableLegend();
}

void
flex_fft_sink_impl::reset()
{
    gr::thread::scoped_lock lock(d_setlock);
    _reset();
}

void
flex_fft_sink_impl::_reset()
{
    int n;
    if (d_trigger_delay) {
        for (n = 0; n < d_nconnections; n++) {
            // Move the tail of the buffers to the front. This section
            // represents data that might have to be plotted again if a
            // trigger occurs and we have a trigger delay set.  The tail
            // section is between (d_end-d_trigger_delay) and d_end.
            memmove(d_fbuffers[n], &d_fbuffers[n][d_end - d_trigger_delay],
                    d_trigger_delay * sizeof(float));

            // Also move the offsets of any tags that occur in the tail
            // section so they would be plotted again, too.
            std::vector<gr::tag_t> tmp_tags;
            for (size_t t = 0; t < d_tags[n].size(); t++) {
                if (d_tags[n][t].offset > (uint64_t)(d_size - d_trigger_delay)) {
                    d_tags[n][t].offset = d_tags[n][t].offset - (d_size - d_trigger_delay);
                    tmp_tags.push_back(d_tags[n][t]);
                }
            }
            d_tags[n] = tmp_tags;
        }
    }
    // Otherwise, just clear the local list of tags.
    else {
        for (n = 0; n < d_nconnections; n++) {
            d_tags[n].clear();
        }
    }

    // Reset the start and end indices.
    d_start = 0;
    d_end = d_size;

    // Reset the trigger. If in free running mode, ignore the
    // trigger delay and always set trigger to true.
    if (d_trigger_mode == TRIG_MODE_FREE) {
        d_index = 0;
        d_triggered = true;
    } else {
        d_index = d_trigger_delay;
        d_triggered = false;
    }
}

void
flex_fft_sink_impl::_npoints_resize()
{
    int newsize = d_main_gui->getNPoints();
    set_nsamps(newsize);
}

void
flex_fft_sink_impl::_adjust_tags(int adj)
{
    for (size_t n = 0; n < d_tags.size(); n++) {
        for (size_t t = 0; t < d_tags[n].size(); t++) {
            d_tags[n][t].offset += adj;
        }
    }
}

void
flex_fft_sink_impl::_gui_update_trigger()
{
    d_trigger_mode = d_main_gui->getTriggerMode();
    d_trigger_slope = d_main_gui->getTriggerSlope();
    d_trigger_level = d_main_gui->getTriggerLevel();
    d_trigger_channel = d_main_gui->getTriggerChannel();
    d_trigger_count = 0;

    float delayf = d_main_gui->getTriggerDelay();
    int delay = static_cast<int>(delayf * d_samp_rate);

    if (delay != d_trigger_delay) {
        // We restrict the delay to be within the window of time being
        // plotted.
        if ((delay < 0) || (delay >= d_size)) {
            GR_LOG_WARN(d_logger, boost::format("Trigger delay (%1%) outside of display range (0:%2%).")\
                            % (delay / d_samp_rate) % ((d_size - 1) / d_samp_rate));
            delay = std::max(0, std::min(d_size - 1, delay));
            delayf = delay / d_samp_rate;
        }

        d_trigger_delay = delay;
        d_main_gui->setTriggerDelay(delayf);
        _reset();
    }

    std::string tagkey = d_main_gui->getTriggerTagKey();
    d_trigger_tag_key = pmt::intern(tagkey);
}

void
flex_fft_sink_impl::_test_trigger_tags(int nitems)
{
    int trigger_index;

    uint64_t nr = nitems_read(d_trigger_channel);
    std::vector<gr::tag_t> tags;
    get_tags_in_range(tags, d_trigger_channel,
                      nr, nr + nitems + 1,
                      d_trigger_tag_key);
    if (tags.size() > 0) {
        d_triggered = true;
        trigger_index = tags[0].offset - nr;
        d_start = d_index + trigger_index - d_trigger_delay - 1;
        d_end = d_start + d_size;
        d_trigger_count = 0;
        _adjust_tags(-d_start);
    }
}

void
flex_fft_sink_impl::_test_trigger_norm(int nitems, const float* input)
{
    int trigger_index;
    for (trigger_index = 0; trigger_index < nitems; trigger_index++) {
        d_trigger_count++;

        // Test if trigger has occurred based on the input stream,
        // channel number, and slope direction
        if (_test_trigger_slope(&input[trigger_index])) {
            d_triggered = true;
            d_trigger_count = 0;
            break;
        }
    }

    // If using auto trigger mode, trigger periodically even
    // without a trigger event.
    if ((d_trigger_mode == TRIG_MODE_AUTO) && (d_trigger_count > d_size)) {
        d_triggered = true;
        d_trigger_count = 0;
    }
}

bool
flex_fft_sink_impl::_test_trigger_slope(const float *in) const
{
    float x0, x1;
    x0 = in[0];
    x1 = in[1];

    return ((x0 <= d_trigger_level) && (x1 > d_trigger_level)) ||
        ((x0 >= d_trigger_level) && (x1 < d_trigger_level));
}

int
flex_fft_sink_impl::work(int noutput_items,
                         gr_vector_const_void_star &input_items,
                         gr_vector_void_star &output_items)
{
    int n = 0, idx = 0;

    _npoints_resize();
    _gui_update_trigger();

    if (d_input_size == 1) {
        if (!d_parser.parse_flex_packet(reinterpret_cast<const unsigned char *>(input_items[0]), noutput_items)){
            std::cout << "Parser rejected data." << std::endl;
            return 0;
        }
        flex_packet::sptr temp = d_parser.get_next_packet();
        if (temp.get() != NULL) {
            if (d_packet_count < 0 && d_stream_id < 0) {
                // We haven't selected our stream id yet!  Let's go with
                // this guy then...
                d_stream_id = temp->get_stream_id();
                d_packet_count = temp->get_data_packet_counter();
                std::cout << "Locked onto stream " << d_stream_id << std::endl;
            }else{
                // We have already locked onto a stream, check if this
                // packet is for us.
                while (temp->get_stream_id() != d_stream_id) {
                    temp = d_parser.get_next_packet();
                    if (temp.get() == NULL) {
                        break;
                    }
                }
                if (temp.get() == NULL) {
                    return noutput_items;
                }
                // This packet is for us.  Check if we have any packet
                // loss.
                d_packet_count += 1;
                if (d_packet_count > 15) {
                    d_packet_count = 0;
                }
                if (d_packet_count != temp->get_data_packet_counter()) {
                    std::cout << "D";
                    d_packet_count = temp->get_data_packet_counter();
                }
            }
            set_nsamps(temp->get_fft_size());
            gr::thread::scoped_lock lock(d_setlock);
            int nfill = d_end - d_index;                 // how much room left in buffers
            int nitems = std::min(temp->get_num_samples(), nfill); // num items we can put in buffers
            // If auto, normal, or tag trigger, look for the trigger
            if ((d_trigger_mode != TRIG_MODE_FREE) && !d_triggered) {
                // trigger off a tag key (first one found)
                if (d_trigger_mode == TRIG_MODE_TAG) {
                    _test_trigger_tags(nitems);
                }
                // Normal or Auto trigger
                else {
                    _test_trigger_norm(temp->get_fft_size(), temp->get_fft_data());
                }
            }

            // Setup the tag information.
            for (n = 0; n < d_nconnections; n++) {
                uint64_t nr = nitems_read(idx);
                std::vector<gr::tag_t> tags;
                get_tags_in_range(tags, idx, nr, nr + nitems + 1);
                for (size_t t = 0; t < tags.size(); t++) {
                    tags[t].offset = d_size / 2;
                }
                d_tags[idx].insert(d_tags[idx].end(), tags.begin(), tags.end());
                idx++;
            }
            d_index += nitems;
            // If we've have a trigger and a full d_size of items in the buffers, plot.
            if (d_triggered) {
                // Copy data to be plotted to start of buffers.
                for (n = 0; n < d_nconnections; n++) {
                    volk_32f_convert_64f(d_buffers[n], temp->get_fft_data(), d_size);
                }

                // Plot if we are able to update
                if (gr::high_res_timer_now() - d_last_time > d_update_time) {
                    d_last_time = gr::high_res_timer_now();
                    double y_min = DEFAULT_MINIMUM;
                    double y_max = temp->get_reference_level();
                    if (!d_auto_adjust) {
                        y_min = d_y_min;
                        y_max = d_y_max;
                    }
                    d_qApplication->postEvent(d_main_gui,
                                              new TimeUpdateEvent(
                                                  d_buffers, 
                                                  d_size, 
                                                  d_tags, 
                                                  temp->get_frequency(), 
                                                  temp->get_sample_rate(),
                                                  y_min,
                                                  y_max));
                }

                // We've plotting, so reset the state
                _reset();
            }

            // If we've filled up the buffers but haven't triggered, reset.
            if (d_index == d_end) {
                _reset();
            }
        }
    }else{
        const float *in = reinterpret_cast<const float*>(input_items[0]);
        
        // We need to run through the tags, and find any that
        // we care about.
        // flex_rate, flex_size, flex_rlvl, flex_nave, flex_freq
        // flex_nave - number of averages, can be dropped.
        // flex_freq - current reference frequency can be updated
        // flex_rate - current sample rate can be updated
        // flex_rlvl - reference level 0dBFS, can be dropped
        // flex_size - fft size can be updated
        pmt::pmt_t flex_rate_key = pmt::string_to_symbol(FLEX_RATE_TAG);
        pmt::pmt_t flex_freq_key = pmt::string_to_symbol(FLEX_FREQ_TAG);
        pmt::pmt_t flex_rlvl_key = pmt::string_to_symbol(FLEX_REF_TAG);
        pmt::pmt_t flex_size_key = pmt::string_to_symbol(FLEX_SIZE_TAG);
        pmt::pmt_t flex_nave_key = pmt::string_to_symbol(FLEX_AVE_TAG);

        // So we need to prepare a packet's worth of data at a
        // time to be updated.
        /*
        Start i @ 0 
            check for tags @ i
            if tags
                update_vars
            send away draw call
            increment i by size
        */
        int current_index = 0;
        boost::uint64_t items_read = nitems_read(0);
        while (current_index < noutput_items) {
            std::vector<gr::tag_t> tags;
            get_tags_in_range(tags, 0,
                              items_read + current_index, 
                              items_read + current_index + static_cast<boost::uint64_t>(d_size));
            for (int i = 0; i < tags.size(); i++) {
                if (tags[i].key == flex_rate_key) {
                    d_sample_rate = pmt::to_double(tags[i].value);
                }else if(tags[i].key == flex_freq_key){
                    d_frequency = pmt::to_double(tags[i].value);
                }else if(tags[i].key == flex_rlvl_key){
                    // Do nothing
                    d_axis_max = pmt::to_double(tags[i].value);
                }else if(tags[i].key == flex_size_key){
                    set_nsamps(pmt::to_double(tags[i].value));
                }
            }
            gr::thread::scoped_lock lock(d_setlock);
            volk_32f_convert_64f(d_buffers[0], &in[current_index],
                                 d_size);
            // If auto, normal, or tag trigger, look for the trigger
            if ((d_trigger_mode != TRIG_MODE_FREE) && !d_triggered) {
                // trigger off a tag key (first one found)
                if (d_trigger_mode == TRIG_MODE_TAG) {
                    _test_trigger_tags(noutput_items);
                }
                // Normal or Auto trigger
                else {
                    _test_trigger_norm(d_size, &in[current_index]);
                }
            }
            int start_index = 0;
            for (int i = start_index; i < tags.size(); i++) {
                tags[i].offset = d_size / 2;
            }
            if (gr::high_res_timer_now() - d_last_time > d_update_time && d_triggered) {
                d_last_time = gr::high_res_timer_now();
                d_tags[0].insert(d_tags[0].begin(), tags.begin() + start_index, tags.end());
                double y_min = DEFAULT_MINIMUM;
                double y_max = d_axis_max;
                if (!d_auto_adjust) {
                    y_min = d_y_min;
                    y_max = d_y_max;
                }
                d_qApplication->postEvent(d_main_gui,
                                          new TimeUpdateEvent(
                                              d_buffers, 
                                              d_size, 
                                              d_tags, 
                                              d_frequency, 
                                              d_sample_rate,
                                              y_min,
                                              y_max));
                // We've plotting, so reset the state
                _reset();
            }

            current_index += d_size;
        }
        if (current_index != noutput_items) {
            current_index -= d_size;
            // copy remainder into waiting buffer, along with
            // tag copies.
        }
    }
    return noutput_items;
}
} /* namespace flexfft */
} /* namespace gr */

