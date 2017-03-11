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

#ifndef INCLUDED_LSA_PROTOCOL_PARSER_B_IMPL_H
#define INCLUDED_LSA_PROTOCOL_PARSER_B_IMPL_H

#include <lsa/protocol_parser_b.h>
#include <pmt/pmt.h>

namespace gr {
  namespace lsa {

    class protocol_parser_b_impl : public protocol_parser_b
    {
     private:
      // Nothing to declare in this block.
      pmt::pmt_t d_msg_port;
      bool d_debug;
      int d_state;

      std::vector<unsigned char> d_input;
      //uint64_t d_
      unsigned long long d_data_reg;
      unsigned long long d_mask;

      uint64_t d_accesscode;
      size_t d_accesscode_len;
      size_t d_hdr_len;

      uint16_t d_payload_len;
      uint16_t d_counter;
      uint8_t d_qsize;
      uint8_t d_qidx;

      uint32_t _get_bit32(int begin_idx);
      uint16_t _get_bit16(int begin_idx);
      uint8_t _get_bit8(int begin_idx);

     public:
      protocol_parser_b_impl(const std::string& code, bool debug);
      ~protocol_parser_b_impl();

      bool set_accesscode(const std::string& code);
      size_t header_nbits() const;

      void output_msg();
      
      bool parse_header();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_PROTOCOL_PARSER_B_IMPL_H */

