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
#include "interference_canceller_cc_impl.h"

namespace gr {
  namespace lsa {

    interference_canceller_cc::sptr
    interference_canceller_cc::make(const std::vector<gr_complex>& clean_preamble,
      const std::string& sensing_tagname)
    {
      return gnuradio::get_initial_sptr
        (new interference_canceller_cc_impl(clean_preamble,
          sensing_tagname));
    }

    /*
     * The private constructor
     */
    interference_canceller_cc_impl::interference_canceller_cc_impl(const std::vector<gr_complex>& clean_preamble,
      const std::string& sensing_tagname)
      : gr::block("interference_canceller_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make2(1, 2, sizeof(gr_complex), sizeof(float)))
    {
      const size_t capacity = 1024*4096;
      d_sample_buffer = new gr_complex[capacity];
      d_sample_size =0;
      d_sample_idx =0;

      d_retx_buffer.clear(); 
    }

    /*
     * Our virtual destructor.
     */
    interference_canceller_cc_impl::~interference_canceller_cc_impl()
    {
      delete [] d_sample_buffer;
      for(int i=0;i<d_retx_buffer.size();++i){
        delete [] d_retx_buffer[i];
      }
      d_retx_buffer.clear();
    }

    void
    interference_canceller_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      for(int i=0;i<ninput_items_required.size();++i){
        ninput_items_required[i] = noutput_items;
      }
    }

    int
    interference_canceller_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
      float* eng =  (float*) output_items[1];
      bool have_eng = output_items.size()>=2;

      // Do <+signal processing+>
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (noutput_items);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace lsa */
} /* namespace gr */

