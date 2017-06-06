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

#ifndef INCLUDED_LSA_SU_PACKET_SINK_C_IMPL_H
#define INCLUDED_LSA_SU_PACKET_SINK_C_IMPL_H

#include <lsa/su_packet_sink_c.h>
#include <gnuradio/digital/constellation.h>

namespace gr {
  namespace lsa {

    class su_packet_sink_c_impl : public su_packet_sink_c
    {
     private:
      int d_state;
      int d_hdr_bps;
      gr::digital::constellation_sptr d_hdr_const;
      const pmt::pmt_t d_msg_port;
      const int d_cap;
      // buffer for constellation
      unsigned char* d_const_buf;
      unsigned char d_buf[256];
      // coded version
      int d_threshold;
      unsigned int d_data_reg;
      unsigned int d_byte_cnt;
      
      uint16_t d_qidx;
      uint16_t d_qsize;
      uint16_t d_base;
      
      int d_pld_cnt;
      int d_pre_cnt;
      int d_chip_cnt;
      unsigned char d_pkt_byte;
      unsigned char d_symbol_cnt;

      unsigned char decode_chip(const unsigned int& reg);
      void enter_search();
      void enter_have_sync();
      void enter_load_payload();

     public:
      su_packet_sink_c_impl(const gr::digital::constellation_sptr& hdr_const,
      int threshold);
      ~su_packet_sink_c_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_SU_PACKET_SINK_C_IMPL_H */

