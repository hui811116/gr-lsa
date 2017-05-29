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
#include "expand_symbal_to_sample_ff_impl.h"
#include <gnuradio/math.h>
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

    expand_symbal_to_sample_ff::sptr
    expand_symbal_to_sample_ff::make(int sps)
    //expand_symbal_to_sample_ff::make(int sps, int nfilts)
    {
      return gnuradio::get_initial_sptr
        (new expand_symbal_to_sample_ff_impl(sps));
        //(new expand_symbal_to_sample_ff_impl(sps,nfilts));
    }

    /*
     * The private constructor
     */
    expand_symbal_to_sample_ff_impl::expand_symbal_to_sample_ff_impl(
      int sps)
      //int sps,
      //int nfilts)
      : gr::block("expand_symbal_to_sample_ff",
              gr::io_signature::make2(2, 2, sizeof(float),sizeof(float)),
              gr::io_signature::make2(2, 2, sizeof(float),sizeof(float)))
    {
      if(sps<0){
        throw std::invalid_argument("sample per symbol cannot be negative");
      }
      d_sps = sps;
      //if(nfilts<0){
        //throw std::invalid_argument("number of filters cannot be negative");
      //}
      //d_nfilts = nfilts;
      set_relative_rate(d_sps);
      set_min_noutput_items(d_sps);
      set_history(2);
    }

    /*
     * Our virtual destructor.
     */
    expand_symbal_to_sample_ff_impl::~expand_symbal_to_sample_ff_impl()
    {
    }

    void
    expand_symbal_to_sample_ff_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      for(int i=0;i<ninput_items_required.size();++i){
        ninput_items_required[i] = noutput_items/relative_rate()+history();
      }
    }

    int
    expand_symbal_to_sample_ff_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const float *phase = (const float *) input_items[0];
      //const float *time = (const float *) input_items[1];
      const float *freq =(const float*) input_items[1];
      float *out_phase = (float *) output_items[0];
      float * out_freq = (float*) output_items[1];
      //float *out_time = (float *) output_items[1];
      //float p_freq, t_freq;
      //float p_phase, t_phase;
      int nin = std::min(ninput_items[0],ninput_items[1]);
      //int nin = (noutput_items/relative_rate() < ninput_items[0])? noutput_items/relative_rate() : ninput_items[0];
      //nin = (nin< ninput_items[1]) ? nin: ninput_items[1];
      int count =0;
      int nout=0;
      float p_base, p_frac;
      float f_base, f_frac;
      while(nout<noutput_items && count<nin-1){
        f_frac = (freq[count+1]-freq[count])/(float)(d_sps*d_sps) ;
        f_base = freq[count]/(float)d_sps-((d_sps-1)/2.0f)*f_frac;
        p_base= phase[count]-((d_sps)/2.0f)*f_base;
        phase_wrap(p_base);
        for(int i=0;i<d_sps;++i){
          out_phase[nout+i]=p_base;
          out_freq[nout+i] =f_base;
          f_base += f_frac;
          p_base += f_base;
          phase_wrap(p_base);
        }
        nout+=d_sps;
        count++;
      }

      /*
      for(int i=0;i<nout;++i){
        if(i%d_sps == 0){
          p_phase = phase[i/d_sps];
          t_phase = time[i/d_sps];
          p_freq = phase[i/d_sps+1]-phase[i/d_sps];
          if(fabs(p_freq+TWO_PI)<fabs(p_freq)){
            p_freq+=TWO_PI;
          }
          else if(fabs(p_freq-TWO_PI)<fabs(p_freq)){
            p_freq-=TWO_PI;
          }
          p_freq/=(float)d_sps;
          t_freq = time[i/d_sps+1]-time[i/d_sps];
          if(fabs(t_freq+d_nfilts)<fabs(t_freq)){
            t_freq+= d_nfilts;
          }
          else if(fabs(t_freq-d_nfilts)<fabs(t_freq)){
            t_freq -= d_nfilts;
          }
          t_freq/=(float)d_nfilts;
        }
        out_phase[i] = p_phase + p_freq;
        out_time[i] = time[i/d_sps] + t_freq;
        p_phase+=p_freq;
        while(p_phase>TWO_PI)
          p_phase -= TWO_PI;
        while(p_phase<-TWO_PI)
          p_phase += TWO_PI;
        t_phase+=t_freq;
        while(t_phase >=d_nfilts)
          t_phase-= d_nfilts;
        while(t_phase<0)
          t_phase+= d_nfilts;
      }*/
      consume_each(count);
      //consume_each (consume);
      return nout;
    }

  } /* namespace lsa */
} /* namespace gr */

