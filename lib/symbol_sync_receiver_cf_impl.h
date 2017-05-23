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

#ifndef INCLUDED_LSA_SYMBOL_SYNC_RECEIVER_CF_IMPL_H
#define INCLUDED_LSA_SYMBOL_SYNC_RECEIVER_CF_IMPL_H

#include <lsa/symbol_sync_receiver_cf.h>
#include <pmt/pmt.h>
#include <utility>
#include <gnuradio/digital/constellation.h>

namespace gr {
  namespace lsa {

    class symbol_sync_receiver_cf_impl : public symbol_sync_receiver_cf
    {
     private:
      int d_state;
      bool d_debug;
      bool d_buf_verbose;
      int d_hdr_bps;

      gr::digital::constellation_sptr d_hdr_const;
      long int d_current_time;
      unsigned int d_time_offset_count;
      const pmt::pmt_t d_timetag;
      const pmt::pmt_t d_msg_port;

      // buffer for constellation
      unsigned char d_bytes_buf[8192];
      unsigned char d_out_buf[256];
      // coded version
      int d_threshold;
      unsigned int d_data_reg;
      unsigned int d_byte_cnt;
      unsigned char d_qidx;
      unsigned char d_qsize;
      
      int d_pld_cnt;
      int d_pre_cnt;
      int d_chip_cnt;
      unsigned char d_pkt_byte;
      unsigned char d_pkt_pld;
      unsigned char d_symbol_cnt;

      void msg_out();
      unsigned char decode_chip(const unsigned int& reg);
      void enter_search();
      void enter_have_sync();
      void enter_load_payload();

     public:
      symbol_sync_receiver_cf_impl(
        const gr::digital::constellation_sptr& hdr_const,
        int threshold,
        bool buf_verbose,
        bool debug);
      ~symbol_sync_receiver_cf_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_SYMBOL_SYNC_RECEIVER_CF_IMPL_H */

