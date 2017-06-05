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
       void delete_msg(pmt::pmt_t k){d_msg = pmt::dict_delete(d_msg,k);}
       void add_msg(pmt::pmt_t k, pmt::pmt_t v){d_msg = pmt::dict_add(d_msg,k,v);}
       void set_index(unsigned int idx){d_idx = idx;}
       void init(){d_msg = pmt::make_dict();}
       void reset(){d_msg = pmt::PMT_NIL; d_idx = 0;}
       bool empty()const{return pmt::is_null(d_msg);}
      private:
       pmt::pmt_t d_msg;
       unsigned int d_idx;
    };

    class intf_t{
      public:
       friend class ic_critical_cc_impl;
       friend std::ostream& operator<<(std::ostream& out,const intf_t& intf){
         out<<"total size:"<<intf.d_end_idx-intf.d_begin_idx+1
         <<" ,begin_idx:"<<intf.d_begin_idx<<" ,end_idx:"<<intf.d_end_idx<<std::endl
         <<" ,front tag:"<<intf.d_front<<std::endl
         <<" ,back tag:"<<intf.d_back;
         return out;
       }
       intf_t(){d_begin_idx=0;d_end_idx=0;d_front = hdr_t();d_front=hdr_t();}
       intf_t(const intf_t& intf){
         d_begin_idx = intf.d_begin_idx;
         d_end_idx=intf.d_end_idx;
         d_front=intf.d_front;
         d_back=intf.d_back;}
       ~intf_t(){}
       const intf_t& operator=(const intf_t& intf){
         d_begin_idx = intf.d_begin_idx;
         d_end_idx = intf.d_end_idx;
         d_front=intf.d_front;
         d_back=intf.d_back;
         return *this;
       }
       void set_front(const hdr_t& front){d_front = front;}
       void set_back(const hdr_t& back){d_back = back;}
       void set_begin(int idx){d_begin_idx = idx; if(d_end_idx<idx){d_end_idx = idx;}}
       void set_end(int idx){d_end_idx = idx;if(d_begin_idx>idx)d_begin_idx=idx;}
       void clear(){d_end_idx=0;d_begin_idx=0;d_front.reset();d_back.reset();}
       int begin()const{return d_begin_idx;}
       int end()const{return d_end_idx;}
       const intf_t& operator*(){return *this;}
       const hdr_t& front()const {return d_front;}
       const hdr_t& back()const {return d_back;}
       void increment(){d_end_idx++;}
       size_t size()const{
         if(d_end_idx==0 || d_end_idx==d_begin_idx){
           // if not complete, return 0
           return 0;
         }else{
           return d_end_idx-d_begin_idx+1;
         }
       }
       bool empty()const{return d_end_idx==0 && d_begin_idx==0 && d_front.empty() && d_back.empty();}
       bool front_tag_empty()const{return d_front.empty();}
       bool back_tag_empty()const{return d_back.empty();}
      private:
       int d_end_idx;
       int d_begin_idx;
       hdr_t d_front;
       hdr_t d_back;
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
      int d_cross_len;
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

      bool detect_ic_chance(const hdr_t& new_tag);
      void reset_retx();
      bool check_and_copy_retx(hdr_t& tag);

      void init_intf();
      bool new_intf();
      bool update_intf(int& residual);

      void do_ic();

     public:
      ic_critical_cc_impl(float thres,int cross_len,int sps,bool d_debug);
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

