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
#include "control_source_b_impl.h"

namespace gr {
  namespace lsa {

    control_source_b::sptr
    control_source_b::make(int seed)
    {
      return gnuradio::get_initial_sptr
        (new control_source_b_impl(seed));
    }

    /*
     * The private constructor
     */
    control_source_b_impl::control_source_b_impl(int seed)
      : gr::sync_block("control_source_b",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(1,1, sizeof(char))),
              d_cap(8192)
    {
      d_rng = new gr::random(0,255,seed);
      d_data_count = 0;
      d_data_buf = new unsigned char[d_cap];
      d_data_port = pmt::mp("pld");
      message_port_register_in(d_data_port);
      set_msg_handler(d_data_port,boost::bind(&control_source_b_impl::set_data,this,_1));
    }

    /*
     * Our virtual destructor.
     */
    control_source_b_impl::~control_source_b_impl()
    {
      delete d_rng;
      delete [] d_data_buf;
    }

    int
    control_source_b_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      unsigned char *out = (unsigned char *) output_items[0];
      if(d_data_count){
        assert(d_data_count<=noutput_items);
        memcpy(out,d_data_buf,sizeof(char)*d_data_count);
        noutput_items = d_data_count;
        d_data_count = 0;
      }
      else{
        for(int i=0;i<noutput_items;++i){
          out[i] =(unsigned char) d_rng->ran_int();
        }
      }
      return noutput_items;
    }

    void
    control_source_b_impl::set_data(pmt::pmt_t msg)
    {
      pmt::pmt_t k = pmt::car(msg);
      pmt::pmt_t v = pmt::cdr(msg);
      assert(pmt::is_blob(v));
      size_t vlen = pmt::blob_length(v);
      memcpy(d_data_buf,pmt::blob_data(v),vlen);
      d_data_count = vlen;
    }

  } /* namespace lsa */
} /* namespace gr */

