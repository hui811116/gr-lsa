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
#include <pmt/pmt.h>
#include <numeric>

namespace gr {
  namespace lsa {

    interference_energy_detector_cc::sptr
    interference_energy_detector_cc::make(
      size_t blocklength,
      bool debug)
    {
      return gnuradio::get_initial_sptr
        (new interference_energy_detector_cc_impl(
          blocklength,
          debug));
    }

    /*
     * The private constructor
     */
    interference_energy_detector_cc_impl::interference_energy_detector_cc_impl(
      size_t blocklength,
      bool debug)
      : gr::block("interference_energy_detector_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make2(2, 2, sizeof(gr_complex), sizeof(float))),
      d_src_id(pmt::intern(alias()))
    {
      set_blocklength(blocklength);
      d_debug = debug;
      const size_t max_size = 32*1024;
      d_energy_reg = (float*) volk_malloc( sizeof(float)*max_size, volk_get_alignment());
      //set_tag_propagation_policy(TPP_DONT);
      v_stddev = (float*) volk_malloc(sizeof(float),volk_get_alignment());
      v_mean = (float*) volk_malloc(sizeof(float),volk_get_alignment());
    }

    /*
     * Our virtual destructor.
     */
    interference_energy_detector_cc_impl::~interference_energy_detector_cc_impl()
    {
      volk_free(d_energy_reg);
      volk_free(v_stddev);
      volk_free(v_mean);
    }

    void
    interference_energy_detector_cc_impl::set_blocklength(size_t blocklength)
    {
      if(blocklength<0){
        throw std::invalid_argument("Block length cannot be negative");
      }
      d_blocklength = blocklength;
      set_history(d_blocklength);
    }

    size_t
    interference_energy_detector_cc_impl::blocklength() const
    {
      return d_blocklength;
    }

    void
    interference_energy_detector_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      for(int i=0;i<ninput_items_required.size();++i)
        ninput_items_required[0] = noutput_items + history();
    }

    int
    interference_energy_detector_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
      //float ed_test;
      float var =0;
      float* voe_out   = (float*)output_items[1];
      int nin = std::min(ninput_items[0]-d_blocklength,noutput_items);
      memcpy(out, in, sizeof(gr_complex) * nin);
      volk_32fc_magnitude_squared_32f(d_energy_reg, in, nin+d_blocklength);
      for(int i=0;i<nin;++i){
        volk_32f_stddev_and_mean_32f_x2(v_stddev, v_mean, d_energy_reg+i,d_blocklength);
        var = pow(*(v_stddev),2);
        //ed_test = std::accumulate(d_energy_reg+i,d_energy_reg+i+d_blocklength-1,0.0)/(float)d_blocklength;
        voe_out[i] = var;
      }
      consume_each (nin);
      return nin;
    }

  } /* namespace lsa */
} /* namespace gr */

