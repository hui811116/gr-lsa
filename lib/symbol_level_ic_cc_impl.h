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

#ifndef INCLUDED_LSA_SYMBOL_LEVEL_IC_CC_IMPL_H
#define INCLUDED_LSA_SYMBOL_LEVEL_IC_CC_IMPL_H

#include <lsa/symbol_level_ic_cc.h>
#include <pmt/pmt.h>

namespace gr {
  namespace lsa {

    class symbol_level_ic_cc_impl : public symbol_level_ic_cc
    {
     private:
      // Nothing to declare in this block.
      const int d_cap;
      gr_complex* d_buffer;
      float* d_eng;
      int d_buf_size;
      int d_buf_idx;

      std::vector<pmt::pmt_t> d_buf_info;
      std::vector<int> d_info_idx;
      std::vector<int> d_end_idx;

      pmt::pmt_t d_sensing;

      bool d_debug;

      std::vector<gr_complex> d_clean_preamble;
      
      int d_hdr_len;
      uint64_t d_accesscode;
      uint64_t d_accesscode_len;
      unsigned long long d_mask;

      int d_bps;

      gr_complex* d_output_buffer;
      int d_out_size;
      int d_out_idx;
      std::vector<pmt::pmt_t> d_out_info;
      std::vector<int> d_out_info_idx;

      std::vector<gr_complex*> d_retx_buffer;
      std::vector<int> d_retx_idx;
      std::vector<int> d_retx_pld;
      int d_retx_count;

      int d_info_count;


      bool ic_detector();
      void clean_buffer();
      void do_interference_cancellation();
      void clean_retx();

      bool set_accesscode(const std::string& accesscode);

     public:
      symbol_level_ic_cc_impl(const std::string& accesscode,
      const std::vector<gr_complex>& clean_preamble,
      int bps,
      bool debug);
      ~symbol_level_ic_cc_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_SYMBOL_LEVEL_IC_CC_IMPL_H */

