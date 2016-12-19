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
              gr::io_signature::make(1, 1, sizeof(float))),
      d_src_id(pmt::intern(alias())),
      d_state_reg(false)
    {
      d_threshold_db=threshold;
      d_bin=bin;
      set_tag_propagation_policy(TPP_DONT);
    }

    /*
     * Our virtual destructor.
     */
    eng_det_cc_impl::~eng_det_cc_impl()
    {
    }

    void
    eng_det_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      ninput_items_required[0] =noutput_items;
    }

    int
    eng_det_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      // NOTE: ninput_items is fixed, but noutput_items is not.
      int nin = ninput_items[0];  
      float *out = (float *) output_items[0];
      float temp[nin];
      for(int it=0;it<nin;++it){
        temp[it]=norm(in[it]);
      }
      // fixed bin, may be a parameter in future implementation
      int noutput_items_calc=nin-d_bin+1;
      float float_test;
      //bool state_reg=false;
      for(int i=0;i<noutput_items_calc;++i)
      {
          float_test=std::accumulate(temp+i,temp+i+d_bin,0.0);
          out[i]=(float_test==0.0)? FLT_MIN_10_EXP : std::log10(float_test);
          if((out[i]>=d_threshold_db) && (!d_state_reg))
          {
            add_item_tag(0,nitems_written(0)+i, pmt::intern("sensing"), pmt::from_float(float_test), d_src_id);
            add_item_tag(0,nitems_written(0)+i, pmt::intern("state"), pmt::PMT_T, d_src_id);
            d_state_reg=true;
          }// not yet found 
          else if((out[i]<d_threshold_db)&& d_state_reg)
          {
            add_item_tag(0,nitems_written(0)+i, pmt::intern("sensing"), pmt::from_float(float_test), d_src_id);
            add_item_tag(0,nitems_written(0)+i, pmt::intern("state"), pmt::PMT_F, d_src_id);
            d_state_reg=false;
          }
      }
      
      // Do <+signal processing+>
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (nin);
      // Tell runtime system how many output items we produced.
      return noutput_items_calc;
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

