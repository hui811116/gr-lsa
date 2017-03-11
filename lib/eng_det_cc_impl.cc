/* -*- c++ -*- */
/* 
 * Copyright 2016 <+YOU OR YOUR COMPANY+>.
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
#include "eng_det_cc_impl.h"
#include <pmt/pmt.h>
#include <numeric>
#include <cmath>
#include <volk/volk.h>

namespace gr {
  namespace lsa {

    eng_det_cc::sptr
    eng_det_cc::make(float threshold, int bin)
    {
      return gnuradio::get_initial_sptr
        (new eng_det_cc_impl(threshold, bin));
    }

    /*
     * The private constructor
     */
    eng_det_cc_impl::eng_det_cc_impl(float threshold, int bin)
      : gr::block("eng_det_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make2(1, 2, sizeof(gr_complex),sizeof(float))),
      d_src_id(pmt::intern(alias())),
      d_state_reg(false)
    {
      set_threshold(threshold);
      if(bin<0){
        throw std::invalid_argument("Energy average bin size cannot be negative");
      }
      set_bin_size(bin);
      set_tag_propagation_policy(TPP_DONT);
      d_cap = 24 * 1024;
      set_max_noutput_items(d_cap);
      d_eng = (float*)volk_malloc(sizeof(float)*d_cap, volk_get_alignment());
      set_history(d_bin);
    }

    /*
     * Our virtual destructor.
     */
    eng_det_cc_impl::~eng_det_cc_impl()
    {
      volk_free(d_eng);
    }

    void
    eng_det_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      for(int i=0;i<ninput_items_required.size();++i){
        ninput_items_required[i] = noutput_items + history();
      }
    }

    int
    eng_det_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex* out = (gr_complex*) output_items[0];
      float* out_eng = d_eng;
      bool have_eng=false;
      if(output_items.size()>=2){
        out_eng = (float*)output_items[1];
        have_eng = true;
      }

      int out_count = 0;
      volk_32fc_magnitude_squared_32f(out_eng,in,noutput_items + history());

      // fixed bin, may be a parameter in future implementation
      float float_test;
      for(int i=0;i<noutput_items;++i)
      {
          float_test=std::accumulate(out_eng+i,out_eng+i+d_bin,0.0);
          if(out_eng[i]>=d_threshold)
          {
            if(!d_state_reg){
              add_item_tag(0,nitems_written(0)+out_count, pmt::intern("ed_begin"), pmt::PMT_T, d_src_id);
              if(have_eng){
                add_item_tag(1,nitems_written(1)+i, pmt::intern("ed_begin"), pmt::PMT_T, d_src_id);
                }
              d_state_reg=true;
            }
          }// not yet found 
          else
          {
            if(d_state_reg){
              add_item_tag(0,nitems_written(0)+out_count, pmt::intern("ed_end"), pmt::PMT_F, d_src_id);
              if(have_eng){
              add_item_tag(1,nitems_written(1)+i, pmt::intern("ed_end"), pmt::PMT_F, d_src_id);
              }
              d_state_reg=false;
            } 
            out[out_count++] = in[i];
          }
      }
      
      // Do <+signal processing+>
      produce(0,out_count);
      produce(1,noutput_items);
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (noutput_items);

      // Tell runtime system how many output items we produced.
      return WORK_CALLED_PRODUCE;
    }
//**************************
//    SET functions
//**************************    
    void eng_det_cc_impl::set_threshold(float thres_db)
    {
      d_threshold = pow(thres_db/10,10);
    }
    void eng_det_cc_impl::set_bin_size(int bin)
    {
      d_bin=bin;
    }
//***************************
//   GET functions
//***************************
    float eng_det_cc_impl::threshold() const
    {
      return 10*log10(d_threshold);
    }

    int eng_det_cc_impl::bin_size() const
    {
      return d_bin;
    }

  } /* namespace lsa */
} /* namespace gr */

