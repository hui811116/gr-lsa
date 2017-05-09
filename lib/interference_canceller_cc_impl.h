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

#ifndef INCLUDED_LSA_INTERFERENCE_CANCELLER_CC_IMPL_H
#define INCLUDED_LSA_INTERFERENCE_CANCELLER_CC_IMPL_H

#include <lsa/interference_canceller_cc.h>
#include <pmt/pmt.h>
#include <map>

namespace gr {
  namespace lsa {

    class interference_canceller_cc_impl : public interference_canceller_cc
    {
     private:
      // Nothing to declare in this block.
      pmt::pmt_t d_sensing_tagname;
      std::vector<gr_complex> d_clean_preamble;

      bool d_debug;

      int d_sps;
      int d_bps;
      int d_hdr_bits;
      int d_hdr_sample_len;
      int d_last_info_idx;
      const size_t d_cap;

      gr_complex* d_sample_buffer;
      gr_complex* d_output_buffer;
      gr_complex* d_sync_buffer;

      //for sync purpose
      float* d_phase_buffer;
      int d_sync_size;

      float* d_eng_buffer;
      int d_output_size;
      int d_output_idx;

      std::vector<pmt::pmt_t> d_out_info;
      std::vector<int> d_out_info_idx;

      int d_sample_size;
      long int d_current_time;

      //for retransmission
      std::vector<gr_complex*> d_retx_buffer;
      std::vector<int> d_retx_pkt_size;
      std::vector<int> d_retx_pkt_index;
      std::vector<pmt::pmt_t> d_retx_info;
      int d_retx_count;

      // for header labeling
      std::vector<pmt::pmt_t> d_buffer_info;
      std::vector<int> d_info_index;

      std::vector<int> d_end_index;
      std::vector<int> d_out_index;

      std::map<long int, int> d_sync_map;
      std::map<long int, int> d_samp_map;

      void tags_handler(std::vector<tag_t>& tags,int offset, int nin, long int current_time);
      void update_system_index(int queue_index);
      void update_system_hdr();

      void retx_check(pmt::pmt_t hdr_info, int qindex,int qsize,int offset);
      void do_interference_cancellation();
      void sync_hdr_index(
        std::vector<int>& coerced_packet_len,
        std::vector<pmt::pmt_t>& buffer_info,
        std::vector<int>& info_index, 
        int end_idx);

      void cfo_correction(int end_idx);

      bool cancellation_detector();

      void output_result(int noutput_items, gr_complex* out, float* eng);
     public:
      interference_canceller_cc_impl(
        const std::vector<gr_complex>& clean_preamble, 
        const std::string& sensing_tagname,
        int sps,
        int bps,
        int hdr_bits,
        bool debug);
      ~interference_canceller_cc_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_INTERFERENCE_CANCELLER_CC_IMPL_H */

