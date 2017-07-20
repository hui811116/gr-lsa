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

#ifndef INCLUDED_LSA_IC_NCFO_CC_IMPL_H
#define INCLUDED_LSA_IC_NCFO_CC_IMPL_H

#include <lsa/ic_ncfo_cc.h>
#include "utils.h"

namespace gr {
  namespace lsa {

    enum VOESTATE{
      VOE_CLEAR,
      VOE_TRIGGERED,
      VOE_PROTECT
    };
    enum ICSTATE{
      CLEAR,
      COLLECT,
      RESET
    };
    class ic_ncfo_cc_impl : public ic_ncfo_cc
    {
     private:
      const int d_cap;
      const int d_buff_lim;
      const pmt::pmt_t d_in_port;

      std::vector<tag_t> d_voe_tags;
      std::vector<tag_t> d_block_tags;
      std::vector<tag_t> d_cross_tags;
      gr_complex* d_in_mem;
      gr_complex* d_out_mem;
      gr_complex* d_demo_mem;
      gr_complex* d_intf_mem;
      gr_complex* d_fir_buffer;
      gr_complex* d_ic_mem;
      int d_in_idx;
      int d_out_size;
      int d_out_idx;
      int d_intf_idx;
      std::vector<gr_complex> d_su_rebuild;
      std::vector<gr_complex> d_taps;
      std::vector<gr_complex> d_tap_buffer;
      uint64_t d_block;
      gr::thread::mutex d_mutex;
      intf_t d_cur_intf;
      int d_voe_state;
      int d_protect_cnt;
      std::list<std::pair<uint64_t,int> > d_block_list;
      std::list<std::pair<int,hdr_t> > d_sfd_list;
      std::list<hdr_t> d_pkt_history;

      int d_retx_cnt;
      std::vector< std::tuple<int,pmt::pmt_t,uint16_t> > d_retx_stack;
      std::list<std::pair<intf_t,std::vector<int> > > d_ic_list;
      std::list<intf_t> d_intf_list;

      std::list<tag_t> d_out_tags;

      gr_complex d_corr_test[1024];
      gr_complex d_chunk_buf[1024];
      float d_su_gain;
      float d_su_phase;
      int d_cancel_idx;

      bool voe_update(int idx);
      void system_update(int idx);
      void tags_update(int idx);
      void msg_in(pmt::pmt_t msg);
      
      // pkt matching functions
      bool pkt_validate(hdr_t& hdr,uint64_t bid,int offset,int pktlen, uint16_t qidx,uint16_t qsize, uint16_t base);
      bool matching_pkt(hdr_t& hdr);
      void retx_detector(uint16_t qidx,uint16_t qsize,uint16_t base,pmt::pmt_t blob, int pktlen);
      // TODO
      bool create_intf();
      void intf_detector();
      void do_ic(std::pair<intf_t,std::vector<int> > obj);
      void rebuild_su(bool retx,const std::vector<int>& retx_idx,std::vector<int>& pkt_len);
     public:
      ic_ncfo_cc_impl(const std::vector<float>& taps);
      ~ic_ncfo_cc_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

    // su header and physical layer info
    typedef std::complex<float> CPX;
    static const CPX d_map[][16] = {
{CPX(1,1),CPX(-1,1),CPX(1,-1),CPX(-1,1),CPX(1,1),CPX(-1,-1),CPX(-1,-1),CPX(1,1),CPX(-1,1),CPX(-1,1),CPX(-1,-1),CPX(1,-1),CPX(-1,-1),CPX(1,-1),CPX(1,1),CPX(1,-1)},
{CPX(1,1),CPX(1,-1),CPX(1,1),CPX(-1,1),CPX(1,-1),CPX(-1,1),CPX(1,1),CPX(-1,-1),CPX(-1,-1),CPX(1,1),CPX(-1,1),CPX(-1,1),CPX(-1,-1),CPX(1,-1),CPX(-1,-1),CPX(1,-1)},
{CPX(-1,-1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(-1,1),CPX(1,-1),CPX(-1,1),CPX(1,1),CPX(-1,-1),CPX(-1,-1),CPX(1,1),CPX(-1,1),CPX(-1,1),CPX(-1,-1),CPX(1,-1)},
{CPX(-1,-1),CPX(1,-1),CPX(-1,-1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(-1,1),CPX(1,-1),CPX(-1,1),CPX(1,1),CPX(-1,-1),CPX(-1,-1),CPX(1,1),CPX(-1,1),CPX(-1,1)},
{CPX(-1,1),CPX(-1,1),CPX(-1,-1),CPX(1,-1),CPX(-1,-1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(-1,1),CPX(1,-1),CPX(-1,1),CPX(1,1),CPX(-1,-1),CPX(-1,-1),CPX(1,1)},
{CPX(-1,-1),CPX(1,1),CPX(-1,1),CPX(-1,1),CPX(-1,-1),CPX(1,-1),CPX(-1,-1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(-1,1),CPX(1,-1),CPX(-1,1),CPX(1,1),CPX(-1,-1)},
{CPX(1,1),CPX(-1,-1),CPX(-1,-1),CPX(1,1),CPX(-1,1),CPX(-1,1),CPX(-1,-1),CPX(1,-1),CPX(-1,-1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(-1,1),CPX(1,-1),CPX(-1,1)},
{CPX(1,-1),CPX(-1,1),CPX(1,1),CPX(-1,-1),CPX(-1,-1),CPX(1,1),CPX(-1,1),CPX(-1,1),CPX(-1,-1),CPX(1,-1),CPX(-1,-1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(-1,1)},
{CPX(1,-1),CPX(-1,-1),CPX(1,1),CPX(-1,-1),CPX(1,-1),CPX(-1,1),CPX(-1,1),CPX(1,-1),CPX(-1,-1),CPX(-1,-1),CPX(-1,1),CPX(1,1),CPX(-1,1),CPX(1,1),CPX(1,-1),CPX(1,1)},
{CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(-1,-1),CPX(1,1),CPX(-1,-1),CPX(1,-1),CPX(-1,1),CPX(-1,1),CPX(1,-1),CPX(-1,-1),CPX(-1,-1),CPX(-1,1),CPX(1,1),CPX(-1,1),CPX(1,1)},
{CPX(-1,1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(-1,-1),CPX(1,1),CPX(-1,-1),CPX(1,-1),CPX(-1,1),CPX(-1,1),CPX(1,-1),CPX(-1,-1),CPX(-1,-1),CPX(-1,1),CPX(1,1)},
{CPX(-1,1),CPX(1,1),CPX(-1,1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(-1,-1),CPX(1,1),CPX(-1,-1),CPX(1,-1),CPX(-1,1),CPX(-1,1),CPX(1,-1),CPX(-1,-1),CPX(-1,-1)},
{CPX(-1,-1),CPX(-1,-1),CPX(-1,1),CPX(1,1),CPX(-1,1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(-1,-1),CPX(1,1),CPX(-1,-1),CPX(1,-1),CPX(-1,1),CPX(-1,1),CPX(1,-1)},
{CPX(-1,1),CPX(1,-1),CPX(-1,-1),CPX(-1,-1),CPX(-1,1),CPX(1,1),CPX(-1,1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(-1,-1),CPX(1,1),CPX(-1,-1),CPX(1,-1),CPX(-1,1)},
{CPX(1,-1),CPX(-1,1),CPX(-1,1),CPX(1,-1),CPX(-1,-1),CPX(-1,-1),CPX(-1,1),CPX(1,1),CPX(-1,1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(-1,-1),CPX(1,1),CPX(-1,-1)},
{CPX(1,1),CPX(-1,-1),CPX(1,-1),CPX(-1,1),CPX(-1,1),CPX(1,-1),CPX(-1,-1),CPX(-1,-1),CPX(-1,1),CPX(1,1),CPX(-1,1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(-1,-1)} 
    };
    static const int d_lsaphy_idx[] = {0,0,0,0,0,0,0,0,14,6}; //0x00 0x00 0x00 0x00 0xe6

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_IC_NCFO_CC_IMPL_H */

