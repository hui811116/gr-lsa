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
#include "signal_power_tagger_cc_impl.h"
#include <volk/volk.h>
#include <gnuradio/math.h>

namespace gr {
  namespace lsa {
    #define d_debug false
    #define DEBUG d_debug && std::cout

    static pmt::pmt_t d_cfo_tag = pmt::intern("cfo_est");
    static pmt::pmt_t d_voe_tag = pmt::intern("voe_tag");
    static pmt::pmt_t d_corr_tag= pmt::intern("corr_val");

    enum TAGTYPE{
      CFO=0,
      CORR=1
    };

    signal_power_tagger_cc::sptr
    signal_power_tagger_cc::make(int calc_len,int tag)
    {
      return gnuradio::get_initial_sptr
        (new signal_power_tagger_cc_impl(calc_len,tag));
    }

    /*
     * The private constructor
     */
    signal_power_tagger_cc_impl::signal_power_tagger_cc_impl(int calc_len,int tag)
      : gr::block("signal_power_tagger_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
              d_pwr_tag(pmt::intern("pwr_tag")),
              d_src_id(pmt::intern(alias())),
              d_pwr_port(pmt::mp("pwr"))
    {
      if(calc_len<=0){
        throw std::invalid_argument("Averaging length cannot be negative");
      }
      d_calc_len = calc_len;
      d_state = false;
      set_tag_propagation_policy(TPP_DONT);
      message_port_register_out(d_pwr_port);
      switch(tag){
        case CFO:
          d_cap_tag = d_cfo_tag;
        break;
        case CORR:
          d_cap_tag = d_corr_tag;
        break;
        default:
          throw std::invalid_argument("Undefined tagtype");
        break;
      }
    }

    /*
     * Our virtual destructor.
     */
    signal_power_tagger_cc_impl::~signal_power_tagger_cc_impl()
    {
    }

    void
    signal_power_tagger_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      if(d_state){
        ninput_items_required[0] = std::max(d_calc_len,noutput_items);
      }else{
        ninput_items_required[0] = noutput_items;
      }
      
    }

    int
    signal_power_tagger_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
      int nin = std::min(ninput_items[0],noutput_items);
      int count =nin;
      std::vector<tag_t> tags;
      get_tags_in_window(tags,0,0,nin,d_cap_tag);
      if(d_state){
        if(noutput_items<d_calc_len){
          consume_each(0);
          return 0;
        }
        volk_32fc_x2_conjugate_dot_prod_32fc(&d_eng_acc,in,in,d_calc_len);
        d_eng_acc/=gr_complex(d_calc_len,0);
        float pwr_db = 10*log10(abs(d_eng_acc));
        add_item_tag(0,nitems_written(0),d_pwr_tag,pmt::from_float(pwr_db));
        message_port_pub(d_pwr_port,pmt::cons(d_pwr_tag,pmt::from_float(pwr_db)) );
        d_state = false;
        DEBUG<<"<PWR TAGGER>Calculated pwr of given length (in dB):"<<pwr_db<<std::endl;
      }else{
        if(!tags.empty()){
          count = tags[0].offset - nitems_read(0);
          d_state = true;
          // reset d_accumulator 
          d_eng_acc=gr_complex(0,0);
        }
      }
      if(count==0){
        consume_each(0);
        return 0;
      }
      int nout = count;
      memcpy(out,in,sizeof(gr_complex)*count);
      std::vector<tag_t> other_tags;
      get_tags_in_window(other_tags,0,0,count);
      for(int i=0;i<other_tags.size();++i){
        int offset = other_tags[i].offset - nitems_read(0);
        add_item_tag(0,nitems_written(0)+offset,other_tags[i].key,other_tags[i].value);
      }
      consume_each (count);
      return nout;
    }

  } /* namespace lsa */
} /* namespace gr */

