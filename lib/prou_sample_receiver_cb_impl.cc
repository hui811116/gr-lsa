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
#include "prou_sample_receiver_cb_impl.h"

namespace gr {
  namespace lsa {

    prou_sample_receiver_cb::sptr
    prou_sample_receiver_cb::make(
      int pu_nfilts,
      int su_nfilts)
    {
      return gnuradio::get_initial_sptr
        (new prou_sample_receiver_cb_impl(
          pu_nfilts,
          su_nfilts));
    }

    /*
     * The private constructor
     */
    prou_sample_receiver_cb_impl::prou_sample_receiver_cb_impl(
      int pu_nfilts,
      int su_nfilts)
      : gr::block("prou_sample_receiver_cb",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(char)))
    {
      d_pu_filters = std::vector<gr::filter::kernel::fir_filter_ccf*>(32);
      std::vector<float> vtaps(1.0);
      for(int i=0;i<32;++i){
        d_pu_filters[i] = new gr::filter::kernel::fir_filter_ccf(1, vtaps);
      }
    }

    /*
     * Our virtual destructor.
     */
    prou_sample_receiver_cb_impl::~prou_sample_receiver_cb_impl()
    {
      for(int i=0;i<32;++i)
      {
        delete d_pu_filters[i];
      }
    }

    void
    prou_sample_receiver_cb_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
    }

    int
    prou_sample_receiver_cb_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      unsigned char *out = (unsigned char *) output_items[0];

      // Do <+signal processing+>
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (noutput_items);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace lsa */
} /* namespace gr */

