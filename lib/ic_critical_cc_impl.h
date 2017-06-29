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

#ifndef INCLUDED_LSA_IC_CRITICAL_CC_IMPL_H
#define INCLUDED_LSA_IC_CRITICAL_CC_IMPL_H

#include <lsa/ic_critical_cc.h>
#include "utils.h"
#include <tuple>
#include <utility>

namespace gr {
  namespace lsa {

    enum BUFFERTYPE{
      SAMPLE,
      SYNC,
      INTF,
      RETX
    };

    class ic_critical_cc_impl : public ic_critical_cc
    {
     private:
      bool d_debug;
      const int d_cap;
      gr_complex* d_in_mem;
      gr_complex* d_out_mem;
      gr_complex* d_comp_mem;
      float* d_phase_mem;
      float* d_freq_mem;
      int d_in_idx;
      int d_sync_idx;
      int d_out_size;
      int d_out_idx;
      std::vector<tag_t> d_out_tags;          // for debugging
      const pmt::pmt_t d_out_msg_port;

      int d_block_size;
      uint64_t d_current_block;
      uint32_t d_phase_block_idx;             // for counting offset in stream 2
      uint32_t d_in_block_idx;                // for counting offset in stream 1

      std::list<block_t> d_smp_list;
      //std::list<block_t> d_sync_list;
      std::list< std::pair<block_t,int> > d_sync_list;

      std::list<hdr_t> d_tag_list;
      std::list<hdr_t> d_pending_list;        // used to record possible preambles.
 
      std::vector<tag_t> d_voe_tags;
      bool d_voe_state;
      int d_state;
      
      int d_prelen;
      int d_sps;

      std::list<hdr_t> d_retx_candidate;
      std::vector<hdr_t> d_retx_tag;
      std::vector<block_t> d_retx_block;
      int d_retx_cnt;
      gr_complex* d_retx_mem;
      int d_retx_idx;

      gr_complex* d_intf_mem;
      int d_intf_idx;
      std::vector<intf_t> d_intf_stack;
      float* d_intf_freq;
      intf_t d_current_intf_tag;
      
      void update_voe_state(int idx);                 // variance of energy tag capturing
      bool detect_ic_chance(const hdr_t& new_tag);    // update retransmission
      void reset_retx();                              // clear retransmission registers
      bool check_and_copy_retx(hdr_t& tag);           // copy and compensate retransmission signals
      void init_intf();                               // initialize a interfereing object
      bool new_intf();                                // captureing front header from ring memory
      bool update_intf();                // captureing end header from ring memory
      void do_ic();                                   // main core to do interference cancellation
      bool matching_header(hdr_t& header);            // labeling bit level header information to preamble candidate
      void check_before_reset();                      // find ic avalability before resetting retx
      int  search_id(uint64_t id);
      bool buffer_index_check(int idxToCheck, int duration,BUFFERTYPE type);
      bool preprocess_hdr(hdr_t& raw_hdr);
      std::pair<uint64_t,int> sync_block_offset_converter(uint64_t bid, int offset, int revdis);

     public:
      ic_critical_cc_impl(int prelen,int sps,int block_size,bool d_debug);
      ~ic_critical_cc_impl();
      
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_IC_CRITICAL_CC_IMPL_H */

