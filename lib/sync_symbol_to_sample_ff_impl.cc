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
#include "sync_symbol_to_sample_ff_impl.h"

namespace gr {
  namespace lsa {

#define TWO_PI (2.0f*M_PI)


    sync_symbol_to_sample_ff::sptr
    sync_symbol_to_sample_ff::make(
      int sps,
      int nfilts,
      const std::string& hdr_tagname
      )
    {
      return gnuradio::get_initial_sptr
        (new sync_symbol_to_sample_ff_impl(
          sps,
          nfilts,
          hdr_tagname));
    }

    /*
     * The private constructor
     */

    sync_symbol_to_sample_ff_impl::sync_symbol_to_sample_ff_impl(
      int sps,
      int nfilts,
      const std::string& hdr_tagname)
      : gr::sync_interpolator("sync_symbol_to_sample_ff",
              gr::io_signature::make2(2, 2, sizeof(float),sizeof(float)),
              gr::io_signature::make2(2, 2, sizeof(float),sizeof(float)), sps)
    {
      if(sps <0){
        throw std::invalid_argument("Parameter samples per symbol cannot be negative");
      }
      d_sps = sps;
      if(nfilts<0){
        throw std::invalid_argument("Parameter number of filters cannot be negative");
      }
      d_nfilts = nfilts;
      d_hdr_tagname = pmt::string_to_symbol(hdr_tagname);
      set_tag_propagation_policy(TPP_DONT);
      set_output_multiple(d_sps);
      set_history(2);
      // additional sample for calculating differences
    }

    /*
     * Our virtual destructor.
     */
    sync_symbol_to_sample_ff_impl::~sync_symbol_to_sample_ff_impl()
    {
    }

    int
    sync_symbol_to_sample_ff_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      const float *in_phase = (const float *) input_items[0];
      const float *in_time_phase = (const float *) input_items[1];
      float *out_phase = (float *) output_items[0];
      float *out_time_phase = (float *) output_items[1];

      std::vector<tag_t> tags;
      get_tags_in_range(tags,0, nitems_read(0),nitems_read(0)+noutput_items/d_sps);
      for(int i=0;i<tags.size();++i){
        tag_t tmp_tag = tags[i];
        int offset = tmp_tag.offset - nitems_read(0);
        add_item_tag(0,nitems_written(0)+offset*d_sps,tmp_tag.key,tmp_tag.value);        
      }

      float smp_freq,smp_time_freq;
      float smp_phase, smp_time_phase;
      for(int i=0;i<noutput_items/d_sps;++i){
        smp_phase = in_phase[i];
        smp_freq = (in_phase[i+1]-in_phase[i])/static_cast<float>(d_sps);
        smp_time_phase = in_time_phase[i];
        smp_time_freq = (in_time_phase[i+1]-in_time_phase[i])/static_cast<float>(d_sps);
        for(int j=0;j<d_sps;++j){
          out_phase[i+j] = smp_phase + smp_freq;
          smp_phase+= smp_freq;
          out_time_phase[i+j] = smp_time_phase + smp_time_freq;
          smp_time_phase += smp_time_freq;
          while(smp_phase>TWO_PI)
            smp_phase-=TWO_PI;
          while(smp_phase<-TWO_PI)
            smp_phase+=TWO_PI;
          while(smp_time_phase>d_nfilts)
            smp_time_phase-=d_nfilts;
          while(smp_time_phase<0)
            smp_time_phase+=d_nfilts;
        }
      }
      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace lsa */
} /* namespace gr */

