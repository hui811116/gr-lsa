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

namespace gr {
  namespace lsa {

    class symbol_sync_receiver_cf_impl : public symbol_sync_receiver_cf
    {
     private:
      int d_state;
      bool d_debug;

      uint64_t d_accesscode;
      size_t d_accesscode_len;
      unsigned long long d_data_reg;
      unsigned long long d_mask;

      std::vector<unsigned int> d_input;

      int d_hdr_bps;
      int d_pld_bps;

      unsigned int d_qsize;
      unsigned int d_qidx;

      unsigned int d_counter;
      unsigned int d_pld_len;

      std::vector<int> d_hdr_map;
      std::vector<int> d_pld_map;

      gr::digital::constellation_sptr d_pld_const;
      gr::digital::constellation_sptr d_hdr_const;

      long int d_current_time;
      int d_symbol_count;
      pmt::pmt_t d_timetag;
      pmt::pmt_t d_msg_port;

      uint16_t _get_bit16(int begin_idx);
      uint8_t _get_bit8(int begin_idx);
      bool insert_symbol(const gr_complex& symbol);
      bool parse_header();
      void msg_out(int noutput_items, bool hdr);

     public:
      symbol_sync_receiver_cf_impl(
        const std::string& accesscode,
        const gr::digital::constellation_sptr& hdr_const,
        const gr::digital::constellation_sptr& pld_const,
        bool debug);
      ~symbol_sync_receiver_cf_impl();

      bool set_accesscode(const std::string& accesscode);
      
      size_t header_nbits() const;

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

