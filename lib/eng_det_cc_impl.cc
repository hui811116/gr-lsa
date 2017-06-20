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

    #define d_debug false
    #define DEBUG d_debug && std::cout

    static int d_ed_valid = 64;
    static const pmt::pmt_t d_pwr_tag =pmt::intern("pwr_tag");

    eng_det_cc::sptr
    eng_det_cc::make(float threshold,bool tag_power)
    {
      return gnuradio::get_initial_sptr
        (new eng_det_cc_impl(threshold, tag_power));
    }

    /*
     * The private constructor
     */
    eng_det_cc_impl::eng_det_cc_impl(float threshold,bool tag_power)
      : gr::block("eng_det_cc",
              gr::io_signature::make2(2, 2, sizeof(gr_complex),sizeof(float)),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
      d_src_id(pmt::intern(alias())),
      d_state_reg(false),
      d_ed_tagname(pmt::intern("ed_tag"))
    {
      set_threshold(threshold);
      set_tag_propagation_policy(TPP_DONT);
      d_state_reg = false;
      d_ed_cnt =0;
      d_tag_power = tag_power;
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
      for(int i=0;i<ninput_items_required.size();++i){
        ninput_items_required[i] = noutput_items;
      }
    }

    int
    eng_det_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      const float* ed = (const float *) input_items[1];
      gr_complex* out = (gr_complex*) output_items[0];
      int nin = std::min(ninput_items[0],ninput_items[1]);
      int nout =0;
      int count =0;
      if(!d_state_reg){
        while(nout<noutput_items && count<nin){
          if(ed[count]>d_threshold){
            d_ed_cnt++;
            if(d_ed_cnt>=d_ed_valid){
              d_state_reg = true;
              d_ed_cnt = 0;
              d_burst_cnt=0;
              d_eng_acc=0;
              add_item_tag(0,nitems_written(0)+nout,d_ed_tagname,pmt::PMT_T,d_src_id);
              DEBUG<<"\033[33;1m"<<"<ED DET>detect a energy trigger, start record burst"<<"\033[0m"<<std::endl;
              break;
            }
          }else{
            d_ed_cnt=0;
          }
          out[nout++] = in[count++];
        }
      }else{
        while(nout<noutput_items && count<nin){
          d_burst_cnt++;
          d_eng_acc+= ed[count];
          if(ed[count]<d_threshold){
            d_ed_cnt++;
            if(d_ed_cnt>= d_ed_valid){
              d_state_reg = false;
              d_ed_cnt=0;
              add_item_tag(0,nitems_written(0)+nout,d_ed_tagname,pmt::PMT_F,d_src_id);
              if(d_tag_power){
                float avg_pwr = d_eng_acc/(float)d_burst_cnt;
                add_item_tag(0,nitems_written(0)+nout,d_pwr_tag,pmt::from_float(avg_pwr),d_src_id);
              }
              DEBUG<<"\033[33;1m"<<"<ED DET>end of burst."<<"\033[0m"<<" ,length="<<d_burst_cnt
              <<" ,avg_pwr="<<d_eng_acc/(double)d_burst_cnt<<std::endl;
              break;
            }
          }else{
            d_ed_cnt=0;
          }
          out[nout++] = in[count++];
        }
      }     
      consume_each(count);
      return nout;
    }
//**************************
//    SET functions
//**************************    
    void 
    eng_det_cc_impl::set_threshold(float thres_db)
    {
      d_threshold = thres_db;
    }
//***************************
//   GET functions
//***************************
    float 
    eng_det_cc_impl::threshold() const
    {
      return d_threshold;
    }
  } /* namespace lsa */
} /* namespace gr */

