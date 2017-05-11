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
#include "moving_average_cc_impl.h"

namespace gr {
  namespace lsa {

    moving_average_cc::sptr
    moving_average_cc::make(int length)
    {
      return gnuradio::get_initial_sptr
        (new moving_average_cc_impl(length));
    }

    /*
     * The private constructor
     */
    moving_average_cc_impl::moving_average_cc_impl(int length)
      : gr::sync_block("moving_average_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(gr_complex)))
    {
      if(length<0){
        throw std::invalid_argument("avg length cannot be negative");
      }
      d_length = length;
      set_history(length);
    }

    /*
     * Our virtual destructor.
     */
    moving_average_cc_impl::~moving_average_cc_impl()
    {
    }

    int
    moving_average_cc_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];

      gr_complex avg=0;
      for(int i=0;i<d_length-1;++i){
        avg+=in[i];
      }
      for(int i=0;i<noutput_items;++i){
        avg+=in[i+d_length -1];
        out[i] = avg;
        avg-=in[i];
      }
      return noutput_items;
    }

  } /* namespace lsa */
} /* namespace gr */

