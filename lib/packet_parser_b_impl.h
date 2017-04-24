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

#ifndef INCLUDED_LSA_PACKET_PARSER_B_IMPL_H
#define INCLUDED_LSA_PACKET_PARSER_B_IMPL_H

#include <lsa/packet_parser_b.h>

namespace gr {
  namespace lsa {

    class packet_parser_b_impl : public packet_parser_b
    {
     private:
      unsigned char* d_hdr_buf;
      unsigned char* d_pld_buf;
      const size_t d_cap;
      int d_state;

      uint64_t d_accesscode;
      size_t d_accesscode_len;
      unsigned long long d_data_reg;

      size_t d_hdr_count;
      size_t d_byte_count;
      size_t d_pld_count;

      bool parse_hdr();
      void pub_data();

      size_t d_hdr_rx;

      pmt::pmt_t d_phy_port;

     public:
      packet_parser_b_impl();
      ~packet_parser_b_impl();

      // Where all the action really happens
      int work(int noutput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_PACKET_PARSER_B_IMPL_H */

