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

#ifndef INCLUDED_LSA_SU_HEADER_PREFIX_IMPL_H
#define INCLUDED_LSA_SU_HEADER_PREFIX_IMPL_H

#include <lsa/su_header_prefix.h>

namespace gr {
  namespace lsa {

    class su_header_prefix_impl : public su_header_prefix
    {
     private:
      // Nothing to declare in this block.
      int d_prefix_bytes;
      unsigned char* d_accessbytes;

      void init_accesscode(const std::string & accesscode);

     protected:
      int calculate_output_stream_length(const gr_vector_int &ninput_items);

     public:
      su_header_prefix_impl(const std::string& accesscode, const std::string& lengthtagname);
      ~su_header_prefix_impl();

      // Where all the action really happens
      int work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_SU_HEADER_PREFIX_IMPL_H */

