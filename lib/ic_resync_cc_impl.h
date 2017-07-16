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
#include <gnuradio/filter/mmse_fir_interpolator_ff.h>
#include "utils.h"

namespace gr {
  namespace lsa {
    // pu decoder state
    enum PUDECSTATE{
      SEARCH,
      SYNC,
      PAYLOAD
    };

    class ic_resync_cc_impl : public ic_resync_cc
    {
     private:
      const size_t d_cap;
      const pmt::pmt_t d_in_port;
      const pmt::pmt_t d_out_port;
      bool d_intf_protect;
      int d_protect_cnt;
      gr_complex* d_in_mem;
      gr_complex* d_out_mem;
      gr_complex* d_demo_mem;
      gr_complex* d_intf_mem;
      gr_complex* d_fir_buffer;
      gr_complex* d_ic_mem;
      std::vector<gr_complex> d_taps;
      std::vector<gr_complex> d_tap_buffer;
      std::vector<gr_complex> d_su_rebuild;
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
      // synchronizers
      int d_last_su_sync_idx;
      bool d_found_first_pu;
      float d_pu_gain_gain;
      float d_pu_cfo_gain;
      // MM_ff
      filter::mmse_fir_interpolator_ff * d_interp;
      float d_mu;
      float d_omega;
      float d_last_sample;
      float d_omega_mid;
      float d_omega_lim;
      float* d_mm_mem;
      int d_mm_size;
      int d_mm_cnt;
      int d_mm_consume;
      // single pole
      float d_pole_alpha;
      float d_pole_one_alpha;
      float d_pole_prevo;
      // quadrature demod
      float* d_qmod_mem;
      int d_qmod_cnt;
      std::vector<gr_complex> d_qmod_tmp;
      // registers and counters
      int d_chunk_size;
      int d_cancel_idx;
      float d_su_phase;
      float d_su_cfo;
      float d_pu_cfo;
      float d_pu_phase;
      float d_su_gain;
      float d_pu_gain;
      float d_tracking_gain;
      float d_gain_gain;
      gr_complex d_corr_test[1024];
      gr_complex d_chunk_buf[1024];
      // prou regen
      float d_offset_d1;
      float d_offset_d2;
      std::vector<gr_complex> d_pu_rebuild;
      std::vector<gr_complex> d_pu_tmp;
      std::vector<gr_complex> d_pu_cancel_buf;
      std::vector<float> d_kay_taps;
      std::vector<gr_complex> d_kay_tmp;
      // debug and demo purpose
      std::list<tag_t> d_out_tags;
      // prou decoder
      int d_dec_threshold;
      PUDECSTATE d_dec_state;
      int d_dec_pre_cnt;
      int d_dec_chip_cnt;
      int d_dec_symbol_cnt;
      int d_dec_pld_len;
      unsigned int d_dec_data_reg;
      unsigned char d_dec_buf[1024];
      unsigned char d_dec_byte_reg;
      void enter_search();
      void enter_sync();
      void enter_payload(const unsigned char& pld_len);
      unsigned char chip_decoder(const unsigned int& c, int& quality);

      // stream functions
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

      // functions to reconstruct both su and pu signal
      void rebuild_su(bool retx,const std::vector<int>& retx_idx,std::vector<int>& pkt_len);
      void reset_sync(); // reset clock, registers
      void rebuild_pu(int chip_id);
      void cancel_pu_and_resync(int cur_sync_idx,int ic_mem_idx,int su_mem_idx,int prev_mm_size,int cur_mm_idx);

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
    static const unsigned int d_mask = 0x7ffffffe;
    static const int MAXPLD = 128-1;
    static const unsigned int CHIPSET[16] = {
      3765939820,
      3456596710,
      1826650030,
      1724778362,
      778887287,
      2061946375,
      4155403488,
      2272978638,
      2676511123,
      2985854233,
      320833617,
      422705285,
      1368596360,
      85537272,
      2287047455,
      4169472305
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_IC_RESYNC_CC_IMPL_H */

