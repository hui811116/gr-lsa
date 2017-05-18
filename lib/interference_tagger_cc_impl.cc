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
#include "interference_tagger_cc_impl.h"

namespace gr {
  namespace lsa {

    static const pmt::pmt_t d_voe_tag= pmt::intern("voe_tag");
    static const int MINLEN = 32;
    static const uint32_t MINGAP = 1024;

    interference_tagger_cc::sptr
    interference_tagger_cc::make(const float& voe_thres)
    {
      return gnuradio::get_initial_sptr
        (new interference_tagger_cc_impl(voe_thres));
    }

    /*
     * The private constructor
     */
    interference_tagger_cc_impl::interference_tagger_cc_impl(const float& voe_thres)
      : gr::block("interference_tagger_cc",
              gr::io_signature::make2(2, 2, sizeof(gr_complex),sizeof(float)),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
              d_msg_port(pmt::mp("msg_out"))
    {
      d_intf_state = false;
      d_intf_cnt =0;
      d_duration_cnt =0;
      set_voe_threshold(voe_thres);
      set_tag_propagation_policy(TPP_DONT);
      message_port_register_out(d_msg_port);
    }

    /*
     * Our virtual destructor.
     */
    interference_tagger_cc_impl::~interference_tagger_cc_impl()
    {
    }

    void
    interference_tagger_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      ninput_items_required[0] = noutput_items;
      ninput_items_required[1] = noutput_items;
    }
    void
    interference_tagger_cc_impl::set_voe_threshold(const float& voe_thres)
    {
      d_voe_thres = voe_thres;
    }

    float
    interference_tagger_cc_impl::voe_threshold() const
    {
      return d_voe_thres;
    }

    void
    interference_tagger_cc_impl::report_interference()
    {
      pmt::pmt_t dict = pmt::make_dict();
      dict = pmt::dict_add(dict,pmt::intern("LSA_sensing"),pmt::PMT_T);
      message_port_pub(d_msg_port,dict);
    }

    int
    interference_tagger_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      //const float* in_eng = (const float*) input_items[1];
      const float* in_voe = (const float*) input_items[1];
      gr_complex *out = (gr_complex *) output_items[0];
      int nin = std::min(std::min(noutput_items, ninput_items[0]),ninput_items[1]);

      const uint64_t nread = nitems_read(0);
      const uint64_t nwrite= nitems_written(0);
      memcpy(out,in,sizeof(gr_complex)*nin);

      for(int i=0;i<nin;++i){
        if(d_intf_state == false){
          if(in_voe[i]>=d_voe_thres){
            d_intf_cnt++;
            if(d_intf_cnt>=MINLEN){
              if(d_duration_cnt>=MINGAP){
                d_duration_cnt = 0;
                report_interference();
                add_item_tag(0,nwrite+i,d_voe_tag,pmt::PMT_T);
              }
              d_intf_cnt =0;
              d_intf_state= true;
            }
          }
          else{
            d_intf_cnt =0;
          }
        }
        else{
          if(in_voe[i]<d_voe_thres){
            d_intf_cnt++;
            if(d_intf_cnt>=MINLEN){
              d_intf_cnt=0;
              d_intf_state = false;
              if(d_duration_cnt>=MINGAP){
                d_duration_cnt =0;
                add_item_tag(0,nwrite+i,d_voe_tag,pmt::PMT_F);
              }
            }
          }
          else{
            d_intf_cnt =0;
          }
        }
        d_duration_cnt++;
      }
      
      consume_each (nin);
      return nin;
    }

  } /* namespace lsa */
} /* namespace gr */

