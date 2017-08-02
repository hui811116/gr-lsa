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
#include "stop_n_wait_tag_gate_cc_impl.h"

namespace gr {
  namespace lsa {

    stop_n_wait_tag_gate_cc::sptr
    stop_n_wait_tag_gate_cc::make()
    {
      return gnuradio::get_initial_sptr
        (new stop_n_wait_tag_gate_cc_impl());
    }

    /*
     * The private constructor
     */
    stop_n_wait_tag_gate_cc_impl::stop_n_wait_tag_gate_cc_impl()
      : gr::block("stop_n_wait_tag_gate_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
              d_start_tag(pmt::intern("sns_start")),
              d_end_tag(pmt::intern("sns_stop"))
    {
      d_state = false;
      set_tag_propagation_policy(TPP_DONT);
    }

    /*
     * Our virtual destructor.
     */
    stop_n_wait_tag_gate_cc_impl::~stop_n_wait_tag_gate_cc_impl()
    {
    }

    void
    stop_n_wait_tag_gate_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = noutput_items;
    }

    int
    stop_n_wait_tag_gate_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
      int nin = std::min(noutput_items, ninput_items[0]);
      if(nin==0){
        consume_each(0);
        return 0;
      }
      std::vector<tag_t> tags;
      if(d_state){
        get_tags_in_range(tags,0,nitems_read(0),nitems_read(0)+nin,d_end_tag);
        if(!tags.empty()){
          d_state = false;
          int offset = tags[0].offset- nitems_read(0);
          add_item_tag(0,nitems_written(0)+offset,pmt::intern("tx_eob"),pmt::PMT_T);
          memcpy(out,in,sizeof(gr_complex)*offset+1);
          consume_each(offset+1);
          return offset+1;
        }else{
          memcpy(out,in,sizeof(gr_complex)*nin);
          consume_each(nin);
          return nin;
        }
      }else{
        get_tags_in_range(tags,0,nitems_read(0),nitems_read(0)+nin,d_start_tag);
        if(!tags.empty()){
          int offset = tags[0].offset - nitems_read(0);
          if(offset==0){
            add_item_tag(0,nitems_written(0),pmt::intern("tx_sob"),pmt::PMT_T);
            d_state = true;
            consume_each(0);
            return 0;
          }else{
            consume_each(offset);
            return 0;
          }
        }else{
          consume_each(nin);
          return 0;
        }
      }
    }

  } /* namespace lsa */
} /* namespace gr */

