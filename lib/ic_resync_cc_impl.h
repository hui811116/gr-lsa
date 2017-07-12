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

#ifndef INCLUDED_LSA_IC_RESYNC_CC_IMPL_H
#define INCLUDED_LSA_IC_RESYNC_CC_IMPL_H

#include <lsa/ic_resync_cc.h>
#include "utils.h"

namespace gr {
  namespace lsa {

    class ic_resync_cc_impl : public ic_resync_cc
    {
     private:
      const size_t d_cap;
      const pmt::pmt_t d_in_port;
      bool d_intf_protect;
      int d_protect_cnt;
      gr_complex* d_in_mem;
      gr_complex* d_out_mem;
      gr_complex* d_demo_mem;
      gr_complex* d_intf_mem;
      gr_complex* d_fir_buffer;
      std::vector<float> d_taps;

      int d_in_idx;
      int d_out_idx;
      int d_intf_idx;
      int d_out_size;

      int d_offset;
      uint64_t d_block;
      uint64_t d_nex_block;
      int d_nex_block_idx;
      std::list< std::pair<uint64_t,int> > d_block_list;
      std::vector<tag_t> d_voe_tags;
      std::vector<tag_t> d_sfd_tags;
      std::vector<tag_t> d_block_tags;
      int d_latest_voe_end;
      int d_latest_voe_begin;
      int d_state;

      
      gr::thread::mutex d_mutex;
      std::list<hdr_t> d_pkt_history;
      std::list< std::pair<int, hdr_t> > d_sfd_list;

      intf_t d_cur_intf;
      std::list<intf_t> d_intf_list;
      std::list<std::pair<intf_t,std::vector<int> > > d_ic_list;

      int d_retx_cnt;
      std::vector< std::tuple<int,pmt::pmt_t,uint16_t> > d_retx_stack;

      bool voe_update(int idx);
      void system_update(int idx);
      void msg_in(pmt::pmt_t msg);
      bool pkt_validate(hdr_t& hdr,uint64_t bid,int offset,int pktlen, uint16_t qidx,uint16_t qsize, uint16_t base);
      bool matching_pkt(hdr_t& hdr);
      bool create_intf();
      void tags_update(int idx);
      void retx_detector(uint16_t qidx,uint16_t qsize,uint16_t base,pmt::pmt_t blob, int pktlen);
      void intf_detector();
      void do_ic(std::pair<intf_t,std::vector<int> > obj);

     public:
      ic_resync_cc_impl(const std::vector<float>& taps);
      ~ic_resync_cc_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_IC_RESYNC_CC_IMPL_H */

