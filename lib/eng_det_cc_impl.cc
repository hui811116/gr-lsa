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
#include <cfloat>
// float minimum: 
   //FLT_MIN  <= 1e-37
   //FLT_MIN_10_EXP <= -37
   //FLT_MIN_EXP <= minimum integer of exponent part of a float 

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
      d_threshold_db=threshold;
      d_bin=bin;
      set_tag_propagation_policy(TPP_DONT);
      d_cap = 24 * 1024;
      d_sample_reg = new gr_complex[d_cap];
    }

    /*
     * Our virtual destructor.
     */
    eng_det_cc_impl::~eng_det_cc_impl()
    {
      delete [] d_sample_reg;
    }

    void
    eng_det_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      ninput_items_required[0] =noutput_items + d_bin -1;
    }

    int
    eng_det_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      // NOTE: ninput_items is fixed, but noutput_items is not.
      //int nin = ninput_items[0];
      //int noutput_items_calc=nin-d_bin+1;
      gr_complex * out_wave= (gr_complex*) output_items[0];

      int out_count = 0;

      float ed_val[noutput_items];
      float *out ;
      out = (output_items.size()>1)? ((float *) output_items[1]) : ed_val;
      float temp[ninput_items[0]];
      for(int it=0;it<ninput_items[0];++it){
        temp[it]=norm(in[it]);
      }
      // fixed bin, may be a parameter in future implementation
      float float_test;
      for(int i=0;i<noutput_items;++i)
      {
          float_test=std::accumulate(temp+i,temp+i+d_bin,0.0);
          out[i]=(float_test==0.0)? 10*FLT_MIN_10_EXP : 10*std::log10(float_test);
          if((out[i]>=d_threshold_db))
          {
            if(!d_state_reg){
              add_item_tag(0,nitems_written(0)+out_count, pmt::intern("detected"), pmt::from_float(float_test), d_src_id);
              add_item_tag(0,nitems_written(0)+out_count, pmt::intern("begin"), pmt::PMT_T, d_src_id);
              if(output_items.size()>1){
                add_item_tag(1,nitems_written(1)+i, pmt::intern("detected"), pmt::from_float(float_test), d_src_id);
                add_item_tag(1,nitems_written(1)+i, pmt::intern("begin"), pmt::PMT_T, d_src_id);
                }
              d_state_reg=true;
            }
            out_wave[out_count++]= in[i];
          }// not yet found 
          else if((out[i]<d_threshold_db)&& d_state_reg)
          {
            //add_item_tag(0,nitems_written(0)+i, pmt::intern("detected"), pmt::from_float(float_test), d_src_id);
            //add_item_tag(0,nitems_written(0)+i, pmt::intern("state"), pmt::PMT_F, d_src_id);
            if(output_items.size()>1){
              add_item_tag(1,nitems_written(1)+i, pmt::intern("detected"), pmt::from_float(float_test), d_src_id);
              add_item_tag(1,nitems_written(1)+i, pmt::intern("begin"), pmt::PMT_F, d_src_id);
            }
            d_state_reg=false;
          }
      }
      
      // Do <+signal processing+>
      //memcpy(out_wave,in,sizeof(gr_complex)*noutput_items);
      produce(0,out_count);
      produce(1,noutput_items);
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (noutput_items);
      // output buffer maybe not enough?
      //produce(0,noutput_items);
      //produce(0,ninput_items[0]);
      //produce(1,noutput_items_calc);
      // Tell runtime system how many output items we produced.
      return WORK_CALLED_PRODUCE;
    }
//**************************
//    SET functions
//**************************    
    void eng_det_cc_impl::set_threshold(float thes_db)
    {
      d_threshold_db=thes_db;
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
      return d_threshold_db;
    }

    int eng_det_cc_impl::bin_size() const
    {
      return d_bin;
    }

  } /* namespace lsa */
} /* namespace gr */

