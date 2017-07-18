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

 #ifndef INCLUDED_LSA_UTILS_H
 #define INCLUDED_LSA_UTILS_H
 
 #include <lsa/api.h>
 #include <gnuradio/config.h>
 #include <iostream>
 #include <pmt/pmt.h>
 #include <ctime>

 #define LSARETRYLIM 10
 #define LSATIMEOUT 5
 #define MAX_LSA_PAYLOAD 121
 #define MAX_PROU_PAYLOAD 127

 class block_t{
      public:
       friend std::ostream& operator<<(std::ostream& out, const block_t& block){
         out<<"block_id:"<<block.d_id<<" ,index:"<<block.d_idx;
         return out;
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
       friend std::ostream& operator<<(std::ostream& out,const intf_t& intf){
         out<<"total size:"<<intf.d_end_idx-intf.d_begin_idx+1
         <<" ,begin_idx:"<<intf.d_begin_idx<<" ,end_idx:"<<intf.d_end_idx<<std::endl
         <<" ,front tag:"<<intf.d_front<<std::endl
         <<" ,back tag:"<<intf.d_back<<std::endl
         <<" ,msg:"<<intf.d_msg;
         return out;
       }
       intf_t(){d_begin_idx=0;d_end_idx=0;d_front = hdr_t();d_front=hdr_t();d_msg=pmt::make_dict();}
       intf_t(const intf_t& intf){
         d_begin_idx = intf.d_begin_idx;
         d_end_idx=intf.d_end_idx;
         d_front=intf.d_front;
         d_back=intf.d_back;
         d_msg = intf.d_msg;}
       ~intf_t(){}
       const intf_t& operator=(const intf_t& intf){
         d_begin_idx = intf.d_begin_idx;
         d_end_idx = intf.d_end_idx;
         d_front=intf.d_front;
         d_back=intf.d_back;
         d_msg= intf.d_msg;
         return *this;
       }
       void set_front(const hdr_t& front){d_front = front;}
       void set_back(const hdr_t& back){d_back = back;}
       void set_begin(int idx){d_begin_idx = idx; if(d_end_idx<idx){d_end_idx = idx;}}
       void set_end(int idx){d_end_idx = idx;if(d_begin_idx>idx)d_begin_idx=idx;}
       void clear(){d_end_idx=0;d_begin_idx=0;d_front.reset();d_back.reset();d_msg=pmt::make_dict();}
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
       pmt::pmt_t msg(){return d_msg;}
       void add_msg(pmt::pmt_t k,pmt::pmt_t v){d_msg = pmt::dict_add(d_msg,k,v);}
       void delete_msg(pmt::pmt_t k){d_msg = pmt::dict_delete(d_msg,k);}
      private:
       pmt::pmt_t d_msg;
       int d_end_idx;
       int d_begin_idx;
       hdr_t d_front;
       hdr_t d_back;
    };

    class srArq_t{
      public:
      friend std::ostream & operator <<(std::ostream& out,const srArq_t& aq){
        out << "seq:"<<aq.d_noseq<<" ,created time:"<<std::difftime(std::time(NULL),aq.d_time)
        <<" ,retry:"<<aq.d_retry<<" ,blob_size:"<<pmt::blob_length(aq.d_msg);
        return out;
      }
      srArq_t(){ d_noseq=0; std::time(&d_time);d_retry=0;d_msg = pmt::PMT_NIL;}
      srArq_t(const srArq_t& aq){d_noseq = aq.d_noseq; d_time = aq.d_time; d_retry =aq.d_retry;d_msg = aq.d_msg;}
      srArq_t(uint16_t noseq,const pmt::pmt_t& msg){d_noseq = noseq; std::time(&d_time); d_retry = 0;d_msg= msg;}
      ~srArq_t(){}
      const srArq_t& operator=(const srArq_t& aq){
        d_noseq = aq.d_noseq; d_time = aq.d_time; d_retry = aq.d_retry; d_msg=aq.d_msg;
        return *this;
      }
      const srArq_t& operator*()const{return *this;}
      time_t time()const {return d_time;}
      uint16_t seq()const {return d_noseq;}
      uint32_t retry()const{return d_retry;}
      pmt::pmt_t msg()const{return d_msg;}
      bool inc_retry(){d_retry++; return d_retry>LSARETRYLIM;}
      bool timeout(){return std::difftime(std::time(NULL),d_time) >=LSATIMEOUT;}
      void reset(){d_retry = 0; std::time(&d_time);}
      void update_time(){std::time(&d_time);}
      void set_retry(uint32_t re){d_retry = re;}
      void set_time(time_t time){d_time = time;}
      void set_seq(uint16_t seq){d_noseq = seq;}
      void set_msg(pmt::pmt_t msg){d_msg = msg;}
      size_t blob_length(){return pmt::blob_length(d_msg);}
      private:
        uint16_t d_noseq;
        time_t d_time;
        uint32_t d_retry;
        pmt::pmt_t d_msg;
    };

 #endif /* INCLUDED_LSA_UTILS_H */