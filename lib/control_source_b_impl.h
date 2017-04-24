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

#ifndef INCLUDED_LSA_CONTROL_SOURCE_B_IMPL_H
#define INCLUDED_LSA_CONTROL_SOURCE_B_IMPL_H

#include <lsa/control_source_b.h>
#include <gnuradio/random.h>

namespace gr {
  namespace lsa {

    class control_source_b_impl : public control_source_b
    {
     private:
      gr::random *d_rng;
      unsigned char* d_data_buf;
      const size_t d_cap;
      pmt::pmt_t d_data_port;
      int d_data_count;

     public:
      control_source_b_impl(int seed);
      ~control_source_b_impl();

      // Where all the action really happens
      int work(int noutput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);

      void set_data(pmt::pmt_t msg);

    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_CONTROL_SOURCE_B_IMPL_H */

