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

#ifndef INCLUDED_LSA_PROU_PACKET_SINK_F_IMPL_H
#define INCLUDED_LSA_PROU_PACKET_SINK_F_IMPL_H

#include <lsa/prou_packet_sink_f.h>

namespace gr {
  namespace lsa {

    class prou_packet_sink_f_impl : public prou_packet_sink_f
    {
     private:
      int d_threshold;
      int d_state;
      int d_pre_cnt;
      int d_chip_cnt;
      int d_symbol_cnt;
      int d_pld_len;
      unsigned int d_data_reg;
      unsigned char d_buf[1024];
      unsigned char d_byte_reg;

      pmt::pmt_t d_pkt_out;

      void enter_search();
      void enter_sync();
      void enter_payload(const unsigned char& pld_len);
      unsigned char chip_decoder(const unsigned int& c);
      


     public:
      prou_packet_sink_f_impl(int thres);
      ~prou_packet_sink_f_impl();

      void set_threshold(int thres);
      int threshold()const;

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_PROU_PACKET_SINK_F_IMPL_H */

