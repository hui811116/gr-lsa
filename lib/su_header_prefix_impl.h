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
      bool d_mode;
      uint64_t d_accesscode;
      unsigned long long d_mask;
      size_t d_accesscode_len;

      void gen_header(unsigned char* out, uint16_t payload_size);

     protected:
      int calculate_output_stream_length(const gr_vector_int &ninput_items);

     public:

      bool set_accesscode(const std::string& accesscode);
      unsigned long long accesscode()const;
      size_t header_nbytes() const;

      size_t header_nbits() const;

      su_header_prefix_impl(const std::string& accesscode, const std::string& lengthtagname, bool mode);
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

