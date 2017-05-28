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
#include "block_tagger_cc_impl.h"

namespace gr {
  namespace lsa {

#define DEBUG d_debug && std::cout
    block_tagger_cc::sptr
    block_tagger_cc::make(const std::string& tagname,int block_size,bool debug)
    {
      return gnuradio::get_initial_sptr
        (new block_tagger_cc_impl(tagname,block_size,debug));
    }

    /*
     * The private constructor
     */
    block_tagger_cc_impl::block_tagger_cc_impl(const std::string& tagname,int block_size, bool debug)
      : gr::block("block_tagger_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
              d_block_tag(pmt::intern(tagname)),
              d_src_id(pmt::intern("block_tagger")),
              d_block_size(block_size)
    {
      if(block_size<0){
        throw std::invalid_argument("block size cannot be negative");
      }
      //set_tag_propagation_policy(TPP_DONT);
      d_debug = debug;
      d_block_cnt =0;
    }

    /*
     * Our virtual destructor.
     */
    block_tagger_cc_impl::~block_tagger_cc_impl()
    {
    }

    void
    block_tagger_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = noutput_items;
    }

    int
    block_tagger_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
      int nin = std::min(ninput_items[0],noutput_items);
      int nout=0;
      const uint64_t nread = nitems_read(0);
      const uint64_t nwrite= nitems_written(0);
      for(int i=0;i<nin;++i){
        out[nout++] = in[i];
        if(d_block_cnt%d_block_size==0){
          d_block_cnt = 0;
          add_item_tag(0,nread+i,d_block_tag,pmt::from_uint64(d_block_no++),d_src_id);
        }
        d_block_cnt++;
        
      }

      consume_each (nin);
      return nout;
    }

  } /* namespace lsa */
} /* namespace gr */

