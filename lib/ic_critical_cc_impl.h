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

namespace gr {
  namespace lsa {

    class block_t{
      public:
       friend class ic_critical_cc_impl;
       friend std::ostream& operator<<(std::ostream& out, const block_t& block){
         out<<"block_id:"<<block.d_id<<" ,index:"<<block.d_idx;
       }
       block_t(){d_id=0;d_idx=0;}
       block_t(const block_t& block){d_id = block.d_id; d_idx = block.d_idx;}
       block_t(uint64_t id,uint32_t idx){d_id = id; d_idx = idx;}
       ~block_t(){}
       const block_t& operator=(const block_t& block){d_id = block.d_id;d_idx=block.d_idx; return *this;}
       const block_t& operator*(){return *this;}
       uint64_t id()const{return d_id;}
       uint32_t index()const{return d_idx;}
       void set_id(uint64_t id){d_id= id;}
       void set_index(uint32_t idx){d_idx = idx;}
      private:
       uint64_t d_id;
       uint32_t d_idx;
    };

    class hdr_t{
      public:
       friend class ic_critical_cc_impl;
       friend std::ostream& operator<<(std::ostream& out, const hdr_t& hdr){
         out<<"index:"<<hdr.d_idx<<" ,msg:"<<hdr.d_msg;
         return out;
       }
       hdr_t(){d_idx = 0; d_msg = pmt::PMT_NIL;}
       hdr_t(const hdr_t& hdr){d_msg = hdr.d_msg; d_idx = hdr.d_idx;}
       hdr_t(unsigned int idx, const pmt::pmt_t& msg){d_idx = idx; d_msg = msg;}
       ~hdr_t(){}
       const hdr_t& operator=(const hdr_t& hdr){d_msg = hdr.d_msg; d_idx = hdr.d_idx;return *this;}
       const hdr_t& operator*(){return *this;}
       pmt::pmt_t msg()const{return d_msg;}
       unsigned int index()const{return d_idx;}
       void add_msg(pmt::pmt_t k, pmt::pmt_t v){d_msg = pmt::dict_add(d_msg,k,v);}
       void set_index(unsigned int idx){d_idx = idx;}
       void init(){d_msg = pmt::make_dict();}
       void reset(){d_msg = pmt::PMT_NIL; d_idx = 0;}
       bool empty()const{return pmt::is_null(d_msg);}
      private:
       pmt::pmt_t d_msg;
       unsigned int d_idx;
    };

    class ic_critical_cc_impl : public ic_critical_cc
    {
     private:
      gr_complex* d_in_mem;
      gr_complex* d_out_mem;
      int d_in_idx;
      int d_out_size;
      int d_out_idx;
      float* d_phase_mem;
      float* d_freq_mem;
      int d_sync_idx;
      const int d_cap;
      uint64_t d_current_block;
      uint32_t d_block_idx;
      std::list<block_t> d_smp_list;
      std::list<block_t> d_sync_list;
      std::list<hdr_t> d_tag_list;
      bool d_debug;
      bool d_voe_state;
      int d_voe_cnt;
      int d_state;

      float d_threshold;

      std::list<hdr_t> d_retx_candidate;
      std::vector<hdr_t> d_retx_tag;
      std::vector<block_t> d_retx_block;
      int d_retx_cnt;
      gr_complex* d_retx_mem;
      int d_retx_idx;

      bool detect_ic_chance(const hdr_t& new_tag);
      void reset_retx();

     public:
      ic_critical_cc_impl(float thres,bool d_debug);
      ~ic_critical_cc_impl();

      void set_threshold(float thres);
      float threshold()const;
      
      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_IC_CRITICAL_CC_IMPL_H */

