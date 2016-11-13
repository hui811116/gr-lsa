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
#include "eng_det_cc_impl.h"

#include <numeric>
#include <cmath>

namespace gr {
  namespace lsa {

    eng_det_cc::sptr
    eng_det_cc::make()
    {
      return gnuradio::get_initial_sptr
        (new eng_det_cc_impl());
    }

    /*
     * The private constructor
     */
    eng_det_cc_impl::eng_det_cc_impl()
      : gr::block("eng_det_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(float)))
    {
      d_threshold_db=-30.0;
    }

    /*
     * Our virtual destructor.
     */
    eng_det_cc_impl::~eng_det_cc_impl()
    {
    }

    void
    eng_det_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      ninput_items_required[0] =noutput_items;
    }

    int
    eng_det_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      // NOTE: ninput_items is fixed, but noutput_items is not.
      int nin = ninput_items[0];  
      float *out = (float *) output_items[0];
      float temp[nin];
      for(int it=0;it<nin;++it){
        temp[it]=norm(in[it]);
      }
      // fixed bin, may be a parameter in future implementation
      //const int ss=noutput_items;
      int bin=5;
      
      for(int i=0;i<nin;++i)
      {
        if(i<nin-bin+1)
          out[i]=std::accumulate(temp+i,temp+i+bin,0.0);
        else
          out[i]=0.0;
      }
      
      // Do <+signal processing+>
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (noutput_items);
      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

    void eng_det_cc_impl::set_thres(float thes_db)
    {
      d_threshold_db=thes_db;
    }

    float eng_det_cc_impl::get_thres() const
    {
      return d_threshold_db;
    }

    

  } /* namespace lsa */
} /* namespace gr */

