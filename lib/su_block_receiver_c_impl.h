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

#ifndef INCLUDED_LSA_SU_BLOCK_RECEIVER_C_IMPL_H
#define INCLUDED_LSA_SU_BLOCK_RECEIVER_C_IMPL_H

#include <lsa/su_block_receiver_c.h>
#include <gnuradio/digital/constellation.h>

namespace gr {
  namespace lsa {

    class su_block_receiver_c_impl : public su_block_receiver_c
    {
     private:
      const pmt::pmt_t d_out_port;
      int d_state;
      int d_hdr_bps;
      unsigned char d_out_buf[256];
      unsigned char d_mod_buf;
      gr::digital::constellation_sptr d_hdr_const;
      int d_threshold;
      unsigned int d_data_reg;
      unsigned int d_byte_cnt;
      int d_pld_cnt;
      int d_pre_cnt;
      int d_chip_cnt;
      unsigned char d_pkt_byte;
      unsigned char d_pkt_pld;
      unsigned char d_symbol_cnt;

      uint64_t d_block;
      int d_offset;
      uint64_t d_latest_bid;
      int d_latest_offset;
      bool d_voe_state;
      bool d_voe_do_not_pub;

      void enter_search();
      void enter_have_sync();
      void enter_load_payload();
      unsigned char decode_chip(const unsigned int& reg);

     public:
      su_block_receiver_c_impl(const gr::digital::constellation_sptr& hdr_const, int threshold);
      ~su_block_receiver_c_impl();

      // Where all the action really happens
      int work(int noutput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_SU_BLOCK_RECEIVER_C_IMPL_H */

