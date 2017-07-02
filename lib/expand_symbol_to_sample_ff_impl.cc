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
#include "expand_symbol_to_sample_ff_impl.h"
#include <gnuradio/math.h>
#include <gnuradio/expj.h>
#include <algorithm>

namespace gr {
  namespace lsa {

#define TWO_PI (M_PI*2.0f)

inline void phase_wrap(float& phase)
{
  while(phase>=TWO_PI)
    phase-=TWO_PI;
  while(phase<=-TWO_PI)
    phase+=TWO_PI;
}

    expand_symbol_to_sample_ff::sptr
    expand_symbol_to_sample_ff::make(int sps)
    {
      return gnuradio::get_initial_sptr
        (new expand_symbol_to_sample_ff_impl(sps));
    }

    /*
     * The private constructor
     */
    expand_symbol_to_sample_ff_impl::expand_symbol_to_sample_ff_impl(
      int sps)
      : gr::block("expand_symbol_to_sample_ff",
              gr::io_signature::make2(2, 2, sizeof(float),sizeof(float)),
              gr::io_signature::make2(2, 2, sizeof(float),sizeof(float))),
              d_src_id(pmt::intern(alias()))
    {
      if(sps<0){
        throw std::invalid_argument("sample per symbol cannot be negative");
      }
      d_sps = sps;
      //set_relative_rate(d_sps);
      set_min_noutput_items(d_sps);
      set_tag_propagation_policy(TPP_DONT);
      set_history(2);
    }

    /*
     * Our virtual destructor.
     */
    expand_symbol_to_sample_ff_impl::~expand_symbol_to_sample_ff_impl()
    {
    }

    void
    expand_symbol_to_sample_ff_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      for(int i=0;i<ninput_items_required.size();++i){
        ninput_items_required[i] = noutput_items/d_sps+history();
      }
    }

    int
    expand_symbol_to_sample_ff_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const float *phase = (const float *) input_items[0];
      const float *freq =(const float*) input_items[1];
      float *out_phase = (float *) output_items[0];
      float * out_freq = (float*) output_items[1];
      int nin = std::min(ninput_items[0],ninput_items[1]);
      int nout_fix = noutput_items/d_sps * d_sps;
      int count =0;
      int nout=0;
      float p_base, p_frac;
      float f_frac;
      gr_complex p_diff;
      std::vector<tag_t> tags;
      while(nout<nout_fix && (count< (nin-1)) ){
        p_diff = gr_expj(phase[count+1])/gr_expj(phase[count]);
        f_frac = std::arg(p_diff)/(float)d_sps;
        p_base = phase[count];
        for(int i=0;i<d_sps;++i){
          out_phase[nout+i] = p_base;
          out_freq[nout+i] = f_frac;
          p_base+= f_frac;
          phase_wrap(p_base);
        }
        nout+=d_sps;
        count++;
      }
      get_tags_in_window(tags,0,0,count);
      for(int i=0;i<tags.size();++i){
        int offset = tags[i].offset-nitems_read(0);
        add_item_tag(0,nitems_written(0)+offset*d_sps,tags[i].key,tags[i].value,d_src_id);
      }
      consume_each(count);
      return nout;
    }

  } /* namespace lsa */
} /* namespace gr */

