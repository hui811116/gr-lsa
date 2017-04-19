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
#include "burst_parser_cc_impl.h"
#include <volk/volk.h>

namespace gr {
  namespace lsa {
    enum burstStatus{
      FIND_BURST,
      OUTPUT_BURST
    };

    burst_parser_cc::sptr
    burst_parser_cc::make(int min_len)
    {
      return gnuradio::get_initial_sptr
        (new burst_parser_cc_impl(min_len));
    }

    /*
     * The private constructor
     */
    burst_parser_cc_impl::burst_parser_cc_impl(int min_len)
      : gr::block("burst_parser_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
              d_cap(8192)
    {
      if(min_len<0)
      throw std::runtime_error("minimum burst length cannot smaller than 0");
      d_min_len = min_len;
      d_sample_buffer = (gr_complex*)volk_malloc(sizeof(gr_complex)*d_cap,volk_get_alignment());
      d_samp_size = 0;
      d_out_counter = 0;
      d_state = false;
      d_burst_status= FIND_BURST;
    }

    /*
     * Our virtual destructor.
     */
    burst_parser_cc_impl::~burst_parser_cc_impl()
    {
      volk_free(d_sample_buffer);
    }

    void
    burst_parser_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      switch(d_burst_status){
        case FIND_BURST:
        break;
        case OUTPUT_BURST:
          noutput_items = 0;
        break;
        default:
          throw std::runtime_error("Wrong state");
        break;
        
      }
      for(int i=0;i<ninput_items_required.size();++i){
        ninput_items_required[i] = noutput_items;
      }
    }

    int
    burst_parser_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];

      int nin = (noutput_items<ninput_items[0]) ? noutput_items: ninput_items[0];
      // copy samples out for this is still working project;
      std::vector<tag_t> tags;

      int tmp_bgn=0;
      int consume=0,nout = 0;
      get_tags_in_window(tags, 0,0, nin);
      //for tagging debug
      pmt::pmt_t dict = pmt::make_dict();

      switch(d_burst_status){
        case FIND_BURST:
        { 
          consume=nin;
          for(int i=0;i<tags.size();++i){
            if((!d_state) && (pmt::eqv(pmt::intern("ed_begin"),tags[i].key))){
              d_state = true;
              tmp_bgn = tags[i].offset - nitems_read(0);
              consume = tmp_bgn;
            }
            if((d_state) && (pmt::eqv(pmt::intern("ed_end"),tags[i].key))){
              d_state = false;
              consume = tags[i].offset - nitems_read(0)+1;
              if( ((d_samp_size + (consume-tmp_bgn))>= d_min_len)
              && ((d_samp_size + (consume-tmp_bgn))<=d_cap) ){
                int con_len = tags[i].offset - nitems_read(0)-tmp_bgn+1;
                memcpy(d_sample_buffer+d_samp_size,in+tmp_bgn,sizeof(gr_complex)*con_len);
                d_samp_size += con_len;
                d_burst_status = OUTPUT_BURST;
                d_out_counter = 0;
                consume_each(consume);
                return 0;
              }
              else{
                d_samp_size = 0;
              }
            }
          }
          if(d_state){
            //there is somthing left in inputs
            consume = nin;
            int buf_len = nin-tmp_bgn;
            if( (buf_len + d_samp_size)>d_cap ){
              d_state = false;
              d_samp_size =0;
              consume_each(consume);
              return 0;
            }
            if(buf_len>0)
              memcpy(d_sample_buffer+d_samp_size,in+tmp_bgn,sizeof(gr_complex)*buf_len);
            d_samp_size+=buf_len;
          }
        break;
        }
        
        case OUTPUT_BURST:
        {
          int samp_left = d_samp_size - d_out_counter;
          nout = (samp_left > noutput_items) ? noutput_items : samp_left;
          if(nout>0){
            if(d_out_counter == 0 ){
              add_item_tag(0,nitems_written(0),pmt::intern("burst_begin"),pmt::PMT_T);
            }
            memcpy(out,d_sample_buffer+d_out_counter,sizeof(gr_complex)*nout);
          }
          d_out_counter += nout;
          if(d_out_counter ==d_samp_size){
              add_item_tag(0,nitems_written(0)+nout-1,pmt::intern("burst_end"),pmt::PMT_T);
            d_burst_status = FIND_BURST;
            d_state = false;
            d_samp_size = 0;
          }
        break;
        }
        default:
        {
          throw std::runtime_error("Entering wrong state");
          break;
        }
      }
      consume_each(consume);
      return nout;
    }

  } /* namespace lsa */
} /* namespace gr */

