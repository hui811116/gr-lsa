/* -*- c++ -*- */
/* 
 * Copyright 2017 <+YOU OR YOUR COMPANY+>.
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
#include "modified_polyphase_time_sync_cc_impl.h"

#include <cstdio>
#include <cmath>
#include <algorithm>
#include <gnuradio/math.h>

namespace gr {
  namespace lsa {

    modified_polyphase_time_sync_cc::sptr
    modified_polyphase_time_sync_cc::make(double sps, float loop_bw,
           const std::vector<float> &taps,
           unsigned int filter_size,
           float init_phase,
           float max_rate_deviation,
           int osps,
           const std::string& intf_tagname,
           bool burst_mode)
    {
      return gnuradio::get_initial_sptr
        (new modified_polyphase_time_sync_cc_impl(sps, loop_bw, taps,
             filter_size,
             init_phase,
             max_rate_deviation,
             osps,
             intf_tagname,
             burst_mode));
    }

    /*
     * The private constructor
     */
    modified_polyphase_time_sync_cc_impl::modified_polyphase_time_sync_cc_impl(double sps, float loop_bw,
           const std::vector<float> &taps,
           unsigned int filter_size,
           float init_phase,
           float max_rate_deviation,
           int osps,
           const std::string& intf_tagname,
           bool burst_mode)
      : gr::block("modified_polyphase_time_sync_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make2(1, 2, sizeof(gr_complex), sizeof(float))),
  d_updated(false), d_nfilters(filter_size),
  d_max_dev(max_rate_deviation),
  d_osps(osps), d_error(0), d_out_idx(0)
    {
      if(taps.size() == 0)
        throw std::runtime_error("pfb_clock_sync_ccf: please specify a filter.\n");

      d_intf_tagname = pmt::string_to_symbol(intf_tagname);
      d_prev_k = 0;
      d_prev_f = 0;
      d_prev_time_count = 0;
      d_intf_state = false;
      d_burst_mode = burst_mode;
      d_found_burst = false;
      // Let scheduler adjust our relative_rate.
      //enable_update_rate(true);
      set_tag_propagation_policy(TPP_DONT);
      d_nfilters = filter_size;
      d_sps = floor(sps);
      // Set the damping factor for a critically damped system
      d_damping = 2*d_nfilters;
      // Set the bandwidth, which will then call update_gains()
      d_loop_bw=loop_bw;
      // Store the last filter between calls to work
      // The accumulator keeps track of overflow to increment the stride correctly.
      // set it here to the fractional difference based on the initial phaes
      d_k = init_phase;
      d_rate = (sps-floor(sps))*(double)d_nfilters;
      d_rate_i = (int)floor(d_rate);
      d_rate_f = d_rate - (float)d_rate_i;
      d_filtnum = (int)floor(d_k);

      d_filters = std::vector<kernel::fir_filter_ccf*>(d_nfilters);
      d_diff_filters = std::vector<kernel::fir_filter_ccf*>(d_nfilters);

      // Create an FIR filter for each channel and zero out the taps
      std::vector<float> vtaps(1,0);
      for(int i = 0; i < d_nfilters; i++) {
  d_filters[i] = new kernel::fir_filter_ccf(1, vtaps);
  d_diff_filters[i] = new kernel::fir_filter_ccf(1, vtaps);
      }

      // Now, actually set the filters' taps
      std::vector<float> dtaps;
      create_diff_taps(taps, dtaps);
      set_taps(taps, d_taps, d_filters);
      set_taps(dtaps, d_dtaps, d_diff_filters);

      d_old_in = 0;
      d_new_in = 0;
      d_last_out = 0;

      set_relative_rate((float)d_osps/(float)d_sps);

      float denom = (1.0 + 2.0*d_damping*d_loop_bw + d_loop_bw*d_loop_bw);
      d_alpha = (4*d_damping*d_loop_bw) / denom;
      d_beta = (4*d_loop_bw*d_loop_bw) / denom;
    }

    /*
     * Our virtual destructor.
     */
    modified_polyphase_time_sync_cc_impl::~modified_polyphase_time_sync_cc_impl()
    {
      for(int i = 0; i < d_nfilters; i++) {
  delete d_filters[i];
  delete d_diff_filters[i];
      }
    }

    void
    modified_polyphase_time_sync_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      // samples to be consumed
      ninput_items_required[0] = (noutput_items + history()) * (d_sps/d_osps);
      // for samples
      
    }

void
    modified_polyphase_time_sync_cc_impl::set_taps(const std::vector<float> &newtaps,
              std::vector< std::vector<float> > &ourtaps,
              std::vector<kernel::fir_filter_ccf*> &ourfilter)
    {
      int i,j;

      unsigned int ntaps = newtaps.size();
      d_taps_per_filter = (unsigned int)ceil((double)ntaps/(double)d_nfilters);

      // Create d_numchan vectors to store each channel's taps
      ourtaps.resize(d_nfilters);

      // Make a vector of the taps plus fill it out with 0's to fill
      // each polyphase filter with exactly d_taps_per_filter
      std::vector<float> tmp_taps;
      tmp_taps = newtaps;
      while((float)(tmp_taps.size()) < d_nfilters*d_taps_per_filter) {
  tmp_taps.push_back(0.0);
      }

      // Partition the filter
      for(i = 0; i < d_nfilters; i++) {
  // Each channel usesd_intf_tagname all d_taps_per_filter with 0's if not enough taps to fill out
  ourtaps[i] = std::vector<float>(d_taps_per_filter, 0);
  for(j = 0; j < d_taps_per_filter; j++) {
    ourtaps[i][j] = tmp_taps[i + j*d_nfilters];
  }

  // Build a filter for each channel and add it's taps to it
  ourfilter[i]->set_taps(ourtaps[i]);
      }

      // Set the history to ensure enough input items for each filter
      set_history(d_taps_per_filter + d_sps + d_sps);

      // Make sure there is enough output space for d_osps outputs/input.
      set_output_multiple(d_osps);
    }

    void
    modified_polyphase_time_sync_cc_impl::create_diff_taps(const std::vector<float> &newtaps,
                std::vector<float> &difftaps)
    {
      std::vector<float> diff_filter(3);
      diff_filter[0] = -1;
      diff_filter[1] = 0;
      diff_filter[2] = 1;

      float pwr = 0;
      difftaps.clear();
      difftaps.push_back(0);
      for(unsigned int i = 0; i < newtaps.size()-2; i++) {
  float tap = 0;
  for(unsigned int j = 0; j < diff_filter.size(); j++) {
    tap += diff_filter[j]*newtaps[i+j];
  }
  difftaps.push_back(tap);
        pwr += fabsf(tap);
      }
      difftaps.push_back(0);

      // Normalize the taps
      for(unsigned int i = 0; i < difftaps.size(); i++) {
        difftaps[i] *= d_nfilters/pwr;
        if(difftaps[i] != difftaps[i]) {
          throw std::runtime_error("pfb_clock_sync_ccf::create_diff_taps produced NaN.");
        }
      }
    }

    int
    modified_polyphase_time_sync_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      // for samples
      gr_complex *out = (gr_complex *) output_items[0];

      if(d_updated) {
        std::vector<float> dtaps;
        create_diff_taps(d_updated_taps, dtaps);
        set_taps(d_updated_taps, d_taps, d_filters);
        set_taps(dtaps, d_dtaps, d_diff_filters);
  d_updated = false;
  return 0;        // history requirements may have changed.
      }

      float *err = NULL, *outk = NULL;
      if(output_items.size() == 2) {
        outk = (float*)output_items[1];
      }

      std::vector<tag_t> tags;
      get_tags_in_window(tags, 0, 0,
                        d_sps*noutput_items,
                        pmt::intern("time_est"));
      std::vector<tag_t> intf_tags;
      get_tags_in_window(intf_tags, 0,0,
                        d_sps*noutput_items,
                        d_intf_tagname);
      std::vector<tag_t> ed_tags;
      get_tags_in_window(ed_tags, 0,0,d_sps*noutput_items,pmt::intern("ed_tag"));

      int i = 0, count = 0;
      float error_r, error_i;

      // produce output as long as we can and there are enough input samples
      while(i < noutput_items) {
        if(!tags.empty()) {
          size_t offset = tags[0].offset-nitems_read(0);
          if((offset >= (size_t)count) && (offset < (size_t)(count + d_sps))) {
            float center = (float)pmt::to_double(tags[0].value);
            d_k = d_nfilters*(center + (offset - count));

            tags.erase(tags.begin());
          }
        }
        if(!intf_tags.empty()) {
          size_t offset = intf_tags[0].offset-nitems_read(0);
          if((offset >= (size_t)count) && (offset < (size_t)(count + d_sps))) {
            if(pmt::to_bool(intf_tags[0].value) && !d_intf_state){
              //should add condition for interference detection
              d_prev_k = d_k;
              d_prev_f = d_rate_f;
              d_prev_time_count=0;
              d_intf_state = true;
            }
            else if(!pmt::to_bool(intf_tags[0].value) && d_intf_state){
              //should add condition for interference detection
              //reapply phase and freq before interference
              d_intf_state = false;
              d_rate_f = d_prev_f;
            }
            intf_tags.erase(intf_tags.begin());
          }
        }
        if(!ed_tags.empty()){
          size_t offset = ed_tags[0].offset - nitems_read(0);
          if((offset>= (size_t)count) && (offset<(size_t)(count+d_sps))){
            d_found_burst = pmt::to_bool(ed_tags[0].value);
              // signal found
            ed_tags.erase(ed_tags.begin());
          }
        }

  while(d_out_idx < d_osps) {

    d_filtnum = (int)floor(d_k);

    // Keep the current filter number in [0, d_nfilters]
    // If we've run beyond the last filter, wrap around and go to next sample
    // If we've gone below 0, wrap around and go to previous sample
    while(d_filtnum >= d_nfilters) {
      d_k -= d_nfilters;
      d_filtnum -= d_nfilters;
      count += 1;
    }
    while(d_filtnum < 0) {
      d_k += d_nfilters;
      d_filtnum += d_nfilters;
      count -= 1;
    }

    out[i+d_out_idx] = d_filters[d_filtnum]->filter(&in[count+d_out_idx]);
    d_k = d_k + d_rate_i + d_rate_f; // update phase
    //
    d_prev_time_count++;

          // Manage Tags
          std::vector<tag_t> xtags;
          std::vector<tag_t>::iterator itags;
          d_new_in = nitems_read(0) + count + d_out_idx + d_sps;
          get_tags_in_range(xtags, 0, d_old_in, d_new_in);
          for(itags = xtags.begin(); itags != xtags.end(); itags++) {
            tag_t new_tag = *itags;
            //new_tag.offset = d_last_out + d_taps_per_filter/(2*d_sps) - 2;
            new_tag.offset = d_last_out + d_taps_per_filter/4 - 2;
            add_item_tag(0, new_tag);            
          }
          d_old_in = d_new_in;
          d_last_out = nitems_written(0) + i + d_out_idx;
if(output_items.size() == 2) {
      outk[i+d_out_idx] = d_k;
    }
          d_out_idx++;
    // We've run out of output items we can create; return now.
    if(i+d_out_idx >= noutput_items) {
      consume_each(count);
      return i;
    }
  }
  // reset here; if we didn't complete a full osps samples last time,
  // the early return would take care of it.
  d_out_idx = 0;
  // Update the phase and rate estimates for this symbol
  gr_complex diff = d_diff_filters[d_filtnum]->filter(&in[count]);
  error_r = out[i].real() * diff.real();
  error_i = out[i].imag() * diff.imag();
  d_error = (error_i + error_r) / 2.0;       // average error from I&Q channel

  // for burst mode, only update when in burst!
  if(d_burst_mode && !d_found_burst){
    d_error = 0; //forced to zero to stop updating
  }

        // Run the control loop to update the current phase (k) and
        // tracking rate estimates based on the error value
        // Interpolating here to update rates for ever sps.
        for(int s = 0; s < d_sps; s++) {
          d_rate_f = d_rate_f + d_beta*d_error;
          d_k = d_k + d_rate_f + d_alpha*d_error;
        }

  // Keep our rate within a good range
  d_rate_f = gr::branchless_clip(d_rate_f, d_max_dev);

  i+=d_osps;
  count += (int)floor(d_sps);
  //
  d_prev_time_count += (int)floor(d_sps);
      }
      //pass samples to next block
      consume_each(count);
      return i;
    }

  } /* namespace lsa */
} /* namespace gr */

