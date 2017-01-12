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
#include "interference_energy_detector_cc_impl.h"
#include <volk/volk.h>

namespace gr {
  namespace lsa {

    interference_energy_detector_cc::sptr
    interference_energy_detector_cc::make(
      const std::string& ed_tagname,
      const std::string& voe_tagname,
      float ed_threshold,
      float voe_threshold,
      size_t blocklength)
    {
      return gnuradio::get_initial_sptr
        (new interference_energy_detector_cc_impl(
          ed_tagname,
          voe_tagname,
          ed_threshold,
          voe_tagname,
          blocklength));
    }

    /*
     * The private constructor
     */
    interference_energy_detector_cc_impl::interference_energy_detector_cc_impl(
      const std::string& ed_tagname,
      const std::string& voe_tagname,
      float ed_threshold,
      float voe_threshold,
      size_t blocklength)
      : gr::sync_block("interference_energy_detector_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make3(1, 3, sizeof(gr_complex), sizeof(float), sizeof(float))),
      d_ed_tagname(pmt::string_to_symbol(ed_tagname)),
      d_voe_tagname(pmt::string_to_symbol(voe_tagname))
    {
      if(blocklength==0){
        std::invalid_argument("Invalid Blocklength");
        return;
      }
      set_ed_threshold(ed_threshold);
      set_voe_threshold(voe_threshold);
      set_blocklength(blocklength);

      const size_t max_size = 32*1024;
      d_energy_reg = (float*) volk_malloc( sizeof(float) * max_size, volk_get_alignment());
      d_voe_reg    = (float*) volk_malloc( sizeof(float) * max_size, volk_get_alignment());

      set_tag_propagation_policy(TPP_DONT);
    }

    /*
     * Our virtual destructor.
     */
    interference_energy_detector_cc_impl::~interference_energy_detector_cc_impl()
    {
      volk_free(d_energy_reg);
      volk_free(d_voe_reg);
    }

    int
    interference_energy_detector_cc_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
      float* ed_val, voe_val;
      if(output_items.size()==3){
        //require energy value and interference value
        ed_val = (float*) output_items[1];
        voe_val = (float*) output_items[2];
      }

      // Do <+signal processing+>
      volk_32fc_magnitude_squared_32f(d_energy_reg, in, )

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace lsa */
} /* namespace gr */

