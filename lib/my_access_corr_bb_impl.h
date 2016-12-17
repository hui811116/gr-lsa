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

#ifndef INCLUDED_LSA_MY_ACCESS_CORR_BB_IMPL_H
#define INCLUDED_LSA_MY_ACCESS_CORR_BB_IMPL_H

#include <lsa/my_access_corr_bb.h>

namespace gr {
  namespace lsa {

    class my_access_corr_bb_impl : public my_access_corr_bb
    {
     private:
      // Nothing to declare in this block.
      pmt::pmt_t d_out_port;
      pmt::pmt_t d_info;
      //std::vector<unsigned char> d_access_code;
      unsigned int d_threshold;
      // new header accesscode bits
      uint64_t d_accesscode;
      size_t d_accesscode_len;

      unsigned long long d_data_reg;
      unsigned long long d_mask;

      int d_state;
      std::vector<bool> d_input;

      bool parse_bits(int n_nits_in, const unsigned char* in, std::vector<pmt::pmt_t>& info, int& count);

      void insert_bits(unsigned char);

      bool payload_matched(int& payload_len);

      void extract_header(std::vector<pmt::pmt_t>& info);
      //

     public:
      my_access_corr_bb_impl(const std::string& access_code, unsigned int threshold);
      ~my_access_corr_bb_impl();

      bool set_access_code(const std::string& access_code);

      unsigned long long access_code() const;

      void set_threshold(unsigned int threshold);

      unsigned int get_threshold() const;

      size_t header_nbits() const;

      //std::string get_access_code() const;
      
      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_MY_ACCESS_CORR_BB_IMPL_H */

