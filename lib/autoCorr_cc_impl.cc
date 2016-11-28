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
#include "autoCorr_cc_impl.h"

namespace gr {
  namespace lsa {

    autoCorr_cc::sptr
    autoCorr_cc::make(unsigned int delay)
    {
      return gnuradio::get_initial_sptr
        (new autoCorr_cc_impl(delay));
    }

    /*
     * The private constructor
     */
    autoCorr_cc_impl::autoCorr_cc_impl(unsigned int delay)
      : gr::block("autoCorr_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(gr_complex)))
    {
      d_delay=(delay==0)?1:delay;
    }

    /*
     * Our virtual destructor.
     */
    autoCorr_cc_impl::~autoCorr_cc_impl()
    {
    }

    void
    autoCorr_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      for (int i=0;i<ninput_items_required.size();++i){
        ninput_items_required[i]=noutput_items;
      }
    }

    int
    autoCorr_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
      int nin = ninput_items[0];
      gr_complex temp(0,0);
      for(int i=0;i<nin-2*d_delay+1;++i){
        temp=std::complex<float>(0,0);
        for(int j=0;j<d_delay;++j){
          temp+=in[i+j]*conj(in[i+j+d_delay]);
        }
        out[i]=temp;
      }
      // Do <+signal processing+>
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (noutput_items);
      int n_gen=(nin-2*d_delay<0)? 0:nin-2*d_delay+1;
      // Tell runtime system how many output items we produced.
      return n_gen;
    }

  } /* namespace lsa */
} /* namespace gr */

