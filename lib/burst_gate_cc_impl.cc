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
#include "burst_gate_cc_impl.h"

namespace gr {
  namespace lsa {

    #define DEBUG false && std::cout

    static const pmt::pmt_t d_sob_tag = pmt::intern("tx_sob");
    static const pmt::pmt_t d_eob_tag = pmt::intern("tx_eob");
    burst_gate_cc::sptr
    burst_gate_cc::make()
    {
      return gnuradio::get_initial_sptr
        (new burst_gate_cc_impl());
    }

    /*
     * The private constructor
     */
    burst_gate_cc_impl::burst_gate_cc_impl()
      : gr::block("burst_gate_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(gr_complex)))
    {
      set_tag_propagation_policy(TPP_DONT);
      d_state = false;
    }

    /*
     * Our virtual destructor.
     */
    burst_gate_cc_impl::~burst_gate_cc_impl()
    {
    }

    void
    burst_gate_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = noutput_items;
    }

    int
    burst_gate_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
      std::vector<tag_t> tags;
      int nin = std::min(ninput_items[0],noutput_items);
      int nout = 0;
      int nout_add = 0;
      if(nin==0){
        consume_each(0);
        return 0;
      }
      if(d_state){
        get_tags_in_window(tags,0,0,nin,d_eob_tag);
        nout = nin;
        if(!tags.empty()){
          int offset = tags[0].offset - nitems_read(0);
          DEBUG<<"<Burst gate> found a eob tag:"<<offset <<std::endl;
          if(noutput_items <offset+1){
            consume_each(0);
            return 0;
          }
          nout = offset;
          nin = offset;
          add_item_tag(0,nitems_written(0)+nout,d_eob_tag,pmt::PMT_T);
          out[offset] = gr_complex(0,0);
          nout_add = 1;
          d_state = false;
        }
        memcpy(out,in,sizeof(gr_complex)*nout);
        nout+=nout_add;
      }else{
        get_tags_in_window(tags,0,0,nin,d_sob_tag);
        if(!tags.empty()){
          int offset = tags[0].offset - nitems_read(0);
          nin = offset;
          add_item_tag(0,nitems_written(0),d_sob_tag,pmt::PMT_T);
          nout = 0;
          d_state = true;
          DEBUG<<"<Burst gate> change state to eob searching"<<std::endl;
        }
      }
      consume_each (nin);
      return nout;
    }

  } /* namespace lsa */
} /* namespace gr */

