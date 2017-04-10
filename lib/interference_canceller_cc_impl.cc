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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "interference_canceller_cc_impl.h"
#include <algorithm>
#include <volk/volk.h>
#include <pmt/pmt.h>
#include <gnuradio/math.h>
#include <gnuradio/expj.h>
#include <map>


namespace gr {
  namespace lsa {

#define TWO_PI (2.0f*M_PI)


    interference_canceller_cc::sptr
    interference_canceller_cc::make(const std::vector<gr_complex>& clean_preamble,
      const std::string& sensing_tagname,
      int sps,
      int bps,
      int hdr_bits,
      bool debug)
    {
      return gnuradio::get_initial_sptr
        (new interference_canceller_cc_impl(clean_preamble,
          sensing_tagname,
          sps,
          bps,
          hdr_bits,
          debug));
    }

    /*
     * The private constructor
     */
    static int ios[]={sizeof(gr_complex),sizeof(float),sizeof(float)};
    static std::vector<int> iosig(ios,ios+sizeof(ios)/sizeof(int));

    interference_canceller_cc_impl::interference_canceller_cc_impl(const std::vector<gr_complex>& clean_preamble,
      const std::string& sensing_tagname,
      int sps,
      int bps,
      int hdr_bits,
      bool debug)
      : gr::block("interference_canceller_cc",
              gr::io_signature::makev(3, 3, iosig),
              gr::io_signature::make2(1, 2, sizeof(gr_complex), sizeof(float))),
      d_cap(1024*2048),
      d_clean_preamble(clean_preamble),
      d_sps(sps)
    {
      d_sensing_tagname = pmt::string_to_symbol(sensing_tagname);
      d_sample_buffer = new gr_complex[d_cap];
      d_sample_size =0;
      d_sync_size =0;

      d_sync_buffer = new gr_complex[d_cap];

      d_output_buffer = new gr_complex[d_cap];
      d_output_size =0;
      d_output_idx = 0;

      d_eng_buffer = (float*) volk_malloc( sizeof(float) * d_cap, volk_get_alignment());

      d_bps = bps;
      d_hdr_bits = hdr_bits;
      d_hdr_sample_len = d_hdr_bits /d_bps * d_sps;

      d_retx_buffer.clear(); 
      d_retx_pkt_size.clear();
      d_retx_pkt_index.clear();
      d_retx_info.clear();
      d_buffer_info.clear();
      d_debug = debug;
      set_tag_propagation_policy(TPP_DONT);
      set_output_multiple(d_sps);

      d_last_info_idx = 0;

      //for sync
      d_phase_buffer = new float[d_cap];
    }

    /*
     * Our virtual destructor.
     */
    interference_canceller_cc_impl::~interference_canceller_cc_impl()
    {
      delete [] d_sample_buffer;
      delete [] d_output_buffer;
      delete [] d_sync_buffer;
      for(int i=0;i<d_retx_buffer.size();++i){
        if(d_retx_buffer[i]!=NULL)
        delete [] d_retx_buffer[i];
      }
      d_retx_buffer.clear();

      volk_free(d_eng_buffer);
      delete [] d_phase_buffer;
    }

    void
    interference_canceller_cc_impl::retx_check(pmt::pmt_t hdr_info, int qindex,int qsize,int offset)
    {
      if(qindex == 0 && qsize==0){
        return;
      }
      if(qindex>=qsize){
        return;
      }
      int payload_size=0;
      int pkt_begin_idx = offset;
      if(pmt::dict_has_key(hdr_info, pmt::intern("payload"))){
        payload_size = pmt::to_long(pmt::dict_ref(hdr_info, pmt::intern("payload"), pmt::PMT_NIL));
      }
      if(d_retx_buffer.empty()){
        d_retx_buffer.resize(qsize,NULL);
        d_retx_pkt_size.resize(qsize);
        d_retx_pkt_index.resize(qsize);
        d_retx_count =0;
        d_retx_info.resize(qsize);
      }
      
      if(payload_size ==0){
        throw std::runtime_error("no payload length found");
      }
      if(d_retx_buffer[qindex]==NULL){
        // can reserve a sps for possible time offset
        d_retx_buffer[qindex] = new gr_complex[payload_size + d_hdr_sample_len + 2*d_sps];
        d_retx_count++;
        //this is naive sample length
        d_retx_pkt_size[qindex] = payload_size + d_hdr_sample_len;
        d_retx_pkt_index[qindex] = pkt_begin_idx;
        d_retx_info[qindex] = hdr_info;
      }
       
    }

    void
    interference_canceller_cc_impl::cfo_correction(
      int end_idx)
    {
      std::map<long int, int>::iterator samp_it, sync_it;
      std::map<long int, int>::iterator samp_beg, sync_beg;
      for(samp_it= d_samp_map.begin();samp_it!=d_samp_map.end();++samp_it){
        if(samp_it->second>=end_idx){
          samp_it--;
          break;
        }
        sync_it = d_sync_map.find(samp_it->first);
        if(sync_it==d_sync_map.end()){
          std::cerr<<"<error>not found:"<<samp_it->first<<std::endl; 
          throw std::runtime_error("Iterator of sync map did not find matched index");
        }
      }
      samp_beg = d_samp_map.begin();
      sync_beg = d_sync_map.find(samp_beg->first);
      if(sync_beg == d_sync_map.end()){
        throw std::runtime_error("Sync map does not have required sample index");
      }
      int samp_len = end_idx - samp_beg->second;
      if((sync_beg->second+samp_len) >d_sync_size){
        throw std::runtime_error("Synchronization information are not enough");
      }
      for(int i=0;i<samp_len;++i){
        gr_complex est = gr_expj(-1* d_phase_buffer[sync_beg->second +i]);
        d_sync_buffer[samp_beg->second+i]=d_sample_buffer[samp_beg->second+i] * est;
      }
      
    }

    void
    interference_canceller_cc_impl::sync_hdr_index(
      std::vector<int>& coerced_packet_len, 
      std::vector<pmt::pmt_t>& buffer_info,
      std::vector<int>& info_index,
      int end_idx)
    {
      if(d_info_index.empty()){
        throw std::runtime_error("Cancellation failed: no header info found");
      }
      int cur_pkt_begin,next_pkt_begin, retx_idx;
      int count =0;
      int test_pkt_len, cur_payload, fixed_pkt_len;
      gr_complex sample_fix;
      pmt::pmt_t tmp_dict;

      buffer_info.assign(d_buffer_info.begin(),d_buffer_info.begin()+d_last_info_idx+1);
      info_index.assign(d_info_index.begin(),d_info_index.begin()+d_last_info_idx+1);
      for(int i=0;i<buffer_info.size();++i){
        int test_pkt_len = pmt::to_long(pmt::dict_ref(buffer_info[i],pmt::intern("payload"),pmt::PMT_NIL));
        coerced_packet_len.push_back(test_pkt_len+d_hdr_sample_len);
      }
      
    }

    void
    interference_canceller_cc_impl::do_interference_cancellation()
    {
      if(d_debug)
      {
        std::cout<<"***************************************"<<std::endl;
        std::cout<<"retx_buffer_size:"<<d_retx_buffer.size()<<" ,d_retx_count:"<<d_retx_count<<std::endl;
        std::cout<<"sample size:"<<d_sample_size<<" ,sync_size:"<<d_sync_size<<std::endl;
        std::cout<<"Do interference cancellation:"<<std::endl;
        for(int i=0;i<(d_last_info_idx+1) ;++i){
          int ct, qi,qs;
          ct = pmt::to_long(pmt::dict_ref(d_buffer_info[i],pmt::intern("counter"),pmt::PMT_NIL));
          qi = pmt::to_long(pmt::dict_ref(d_buffer_info[i],pmt::intern("queue_index"),pmt::PMT_NIL));
          qs = pmt::to_long(pmt::dict_ref(d_buffer_info[i],pmt::intern("queue_size"),pmt::PMT_NIL));
          std::cout<<"["<<i<<"]"<<" ,index:"<<d_info_index[i]<<" counter:"<<ct<<" ,queue_index:"<<qi<<" ,queue_size:"<<qs<<std::endl;
        }
        std::map<long int, int>::iterator samp_it,sync_it;
        std::cout<<"----------------------------------------"<<std::endl;
        std::cout<<"Sample map:"<<std::endl;
        for(samp_it=d_samp_map.begin();samp_it!=d_samp_map.end();++samp_it){
          std::cout<<"time:"<<samp_it->first<<" ,index:"<<samp_it->second<<std::endl;
        }
        std::cout<<"---------------------------------------"<<std::endl;
        std::cout<<"Sync map;"<<std::endl;
        for(sync_it=d_sync_map.begin();sync_it!=d_sync_map.end();++sync_it){
          std::cout<<"time:"<<sync_it->first<<" ,index"<<sync_it->second<<std::endl;
        }
        std::cout<<"***************************************"<<std::endl;
      }

      int end_idx = d_end_index.front();
      d_end_index.erase(d_end_index.begin());
      //do signal processing here,
      std::vector<int> packet_len;
      std::vector<pmt::pmt_t> buffer_info;
      std::vector<int> info_index;
      int cei_pkt_counter, cei_sample_counter;
      int retx_size = d_retx_buffer.size();
      int total_size;
      int pld_len;
      gr_complex* retx, corrected_sample;
      pmt::pmt_t tmp_dict;

      // cfo, phase correction
      cfo_correction(end_idx);
      // calculate offset in sync and sample      
      // fixing packet len
      sync_hdr_index(packet_len, buffer_info, info_index, end_idx);

      //copy retransmission
      for(int i=0;i<d_retx_count;++i){
        memcpy(d_retx_buffer[i],d_sync_buffer+d_retx_pkt_index[i],
          sizeof(gr_complex)*(d_retx_pkt_size[i]) );
      }

      cei_pkt_counter = pmt::to_long(pmt::dict_ref(d_buffer_info[d_last_info_idx],pmt::intern("queue_index"),pmt::PMT_NIL));
      pld_len = pmt::to_long(pmt::dict_ref(d_buffer_info[d_last_info_idx],pmt::intern("payload"),pmt::PMT_NIL));
      
      cei_sample_counter = d_retx_pkt_size[cei_pkt_counter];
      retx = d_retx_buffer[cei_pkt_counter];

      total_size = d_info_index[d_last_info_idx]+pld_len+d_hdr_sample_len;

      if((d_output_size+total_size) > d_cap){
        //clear output_buffer before doing interference cancellation
        GR_LOG_WARN(d_logger,"output size greater than buffer capacity");
        d_output_size =0;
        d_output_idx =0;
        d_out_info.clear();
        d_out_info_idx.clear();
        d_out_index.clear();
      }
      d_out_index.push_back(d_output_size+total_size);
      // not neccessary, for debugging tags
      for(int i=0;i<info_index.size();++i){
        d_out_info.push_back(buffer_info[i]);
        d_out_info_idx.push_back(info_index[i]+d_output_size);
      }
      std::fill_n(d_output_buffer+d_output_size, total_size, gr_complex(0,0));

      while(total_size>0){
        total_size--;
        cei_sample_counter--;
        d_output_buffer[d_output_size+total_size] = d_sync_buffer[total_size] - retx[cei_sample_counter];

        if(!info_index.empty()){
          int idx = info_index.back();
          if(total_size==idx){
            //update to previous info
            cei_sample_counter=0;
            buffer_info.pop_back();
            info_index.pop_back();
            packet_len.pop_back();
          }
          else if((cei_sample_counter==0) && (idx<total_size) ){
            cei_sample_counter++;
          }
        }
        if(cei_sample_counter==0){
          cei_pkt_counter = (cei_pkt_counter-1)%retx_size;
          if(cei_pkt_counter<0)
            cei_pkt_counter+= retx_size;
          cei_sample_counter = d_retx_pkt_size[cei_pkt_counter];
          retx = d_retx_buffer[cei_pkt_counter];
        }
        
      }
      //fixing header part
      d_output_size = d_out_index.back();
      update_system_index(end_idx);
    }

    void
    interference_canceller_cc_impl::output_result(int noutput_items, gr_complex* out, float* eng)
    {
      int out_count =0;
      int end_idx = d_out_index.front();
      int nout = (noutput_items > (d_output_size-d_output_idx)) ? (d_output_size-d_output_idx) : noutput_items;
      
      if(nout==0)
        return;
      if(eng!=NULL && (nout!=0))
      {
        volk_32fc_magnitude_squared_32f(d_eng_buffer, d_output_buffer+d_output_idx, nout);
        for(int i=0;i<nout;++i){
          eng[i] = d_eng_buffer[i];
        }
        produce(1,nout);
      }
      while(!d_out_info.empty()){
        if(d_out_info_idx[0]>(d_output_idx+nout)){
          break;
        }
          int counter = pmt::to_long(pmt::dict_ref(d_out_info[0],pmt::intern("counter"),pmt::PMT_NIL));
          int offset = d_out_info_idx[0] - d_output_idx;
          int qsize = pmt::to_long(pmt::dict_ref(d_out_info[0],pmt::intern("queue_size"),pmt::PMT_NIL));
          int qidx =  pmt::to_long(pmt::dict_ref(d_out_info[0],pmt::intern("queue_index"),pmt::PMT_NIL));
          add_item_tag(0,nitems_written(0)+offset,pmt::intern("counter"),pmt::from_long(counter));
          if(qsize!=0){
            add_item_tag(0,nitems_written(0)+offset,pmt::intern("CEI_idx"),pmt::from_long(qidx));
            add_item_tag(0,nitems_written(0)+offset,pmt::intern("CEI_size"),pmt::from_long(qsize));
          }
          d_out_info.erase(d_out_info.begin());
          d_out_info_idx.erase(d_out_info_idx.begin());
      }
      memcpy(out,d_output_buffer+d_output_idx,sizeof(gr_complex)*nout);
      produce(0,nout);
      d_output_idx += nout;
      //update output buffer
      if(d_output_idx >= end_idx){
        int new_size = d_output_size - end_idx;
        memcpy(d_output_buffer, d_output_buffer+ end_idx, sizeof(gr_complex)* new_size);
        d_output_size-=end_idx;
        d_output_idx=0;
        d_out_index.erase(d_out_index.begin());
        for(int i=0;i<d_out_index.size();++i){
          d_out_index[i]-=end_idx;
          if(d_out_index[i]<0){
            throw std::runtime_error("output index cannot be negative");
          }
        }
        std::vector<int> fix_info_idx;
        std::vector<pmt::pmt_t> fix_info;
        for(int i=0;i<d_out_info_idx.size();++i){
          d_out_info_idx[i]-= end_idx;
          if(d_out_info_idx[i]>=0){
            fix_info_idx.push_back(d_out_info_idx[i]);
            fix_info.push_back(d_out_info[i]);
          }
          else{
            std::cerr<<"<WARNING>:output index correction encounter negative index:"<<
            "("<<d_out_info_idx[i]<<"-"<<end_idx<<")"<<std::endl;
          }
        }
        d_out_info=fix_info;
        d_out_info_idx=fix_info_idx;
      }
    }

    bool
    interference_canceller_cc_impl::cancellation_detector()
    {
      int qsize,qidx;
      int pld_len, sample_idx;
      int valid_size, sync_size;
      long int time_check;
      std::map<long int, int>::iterator it, samp_it;
      if(d_buffer_info.empty()){
        return false;
      }
      else{
        //update index
        while(d_last_info_idx<d_buffer_info.size()){
          time_check = pmt::to_long(pmt::dict_ref(d_buffer_info[d_last_info_idx],pmt::intern("ctime"),pmt::PMT_NIL));
          it = d_sync_map.find(time_check);
          if(it == d_sync_map.end()){
            //sync info not yet received
            return false;
          }
          samp_it = d_samp_map.find(time_check);
          if(samp_it == d_samp_map.end()){
            std::cout<<"error: time not found:"<<time_check<<"<--key"<<std::endl;
            throw std::runtime_error("Time not matched in sample map and buffer info");
          }

          pld_len = pmt::to_long(pmt::dict_ref(d_buffer_info[d_last_info_idx],pmt::intern("payload"),pmt::PMT_NIL));
          valid_size = d_info_index[d_last_info_idx] + pld_len + d_hdr_sample_len;
          //calculate the valid sync info size
          sync_size = d_info_index[d_last_info_idx] - samp_it->second + it->second + d_hdr_sample_len + pld_len;
          qidx = pmt::to_long(pmt::dict_ref(d_buffer_info[d_last_info_idx],pmt::intern("queue_index"),pmt::PMT_NIL));
          qsize = pmt::to_long(pmt::dict_ref(d_buffer_info[d_last_info_idx],pmt::intern("queue_size"),pmt::PMT_NIL));
          if( ((valid_size+1024) > d_sample_size) || ((sync_size+1024) > d_sync_size) ){
            //samples not enough
            return false;
          }
          if((qsize!=d_retx_buffer.size()) && !d_retx_buffer.empty()){
            // should update info and tracking for new size
            // reset HERE
            update_system_hdr();
            return false;
          }
          retx_check(d_buffer_info[d_last_info_idx],qidx,qsize,d_info_index[d_last_info_idx]);
          if(!d_retx_buffer.empty() && (d_retx_count == d_retx_buffer.size()) ){ 
            //interference cancellation available  
            int last_idx = d_info_index[d_last_info_idx] + d_hdr_sample_len + pld_len;
            d_end_index.push_back(last_idx);
            return true;
          }
          d_last_info_idx++;
        }//end while 
        
      }//else
      return false;
    }//function

    void
    interference_canceller_cc_impl::update_system_hdr()
    {
      //removed till d_last_info_idx;
      // updating sync info
      std::map<long int, int>::iterator samp_it, sync_it;
      long int update_time = pmt::to_long(
        pmt::dict_ref(d_buffer_info[d_last_info_idx],pmt::intern("ctime"),
          pmt::PMT_NIL));
      int update_idx;
      sync_it = d_sync_map.find(update_time);
      if(sync_it==d_sync_map.end()){
        throw std::runtime_error("find no time info");
      }
      update_idx = sync_it->second;
      d_sync_map.erase(d_sync_map.begin(),sync_it);
      for(sync_it =d_sync_map.begin();sync_it!=d_sync_map.end();++sync_it){
        d_sync_map[sync_it->first] = sync_it->second-update_idx;
      }
      memcpy(d_phase_buffer,d_phase_buffer+update_idx,sizeof(float)*(d_sync_size-update_idx));
      d_sync_size-=update_idx;

      // handling sample information
      // reset for sample
      samp_it = d_samp_map.find(update_time);
      if(samp_it == d_samp_map.end()){
        throw std::runtime_error("find no time info");
      }
      // reset update_idx for tracking index
      update_idx = samp_it->second;

      int rm_idx = d_info_index[d_last_info_idx];
      d_info_index.erase(d_info_index.begin(),d_info_index.begin()+d_last_info_idx);
      d_buffer_info.erase(d_buffer_info.begin(),d_buffer_info.begin()+d_last_info_idx);
      for(int k=0;k<d_info_index.size();++k){
        d_info_index[k]-= update_idx;
      }
      int rest_len = d_sample_size - update_idx;
      memcpy(d_sample_buffer, d_sample_buffer+update_idx, sizeof(gr_complex)*rest_len);
      d_sample_size-= update_idx;
      
      d_samp_map.erase(d_samp_map.begin(),samp_it);
      for(samp_it=d_samp_map.begin();samp_it!=d_samp_map.end();++samp_it){
        d_samp_map[samp_it->first] = samp_it->second - update_idx;
      }
      //reset retransmission info
      d_retx_count =0;
      d_retx_info.clear();
      d_retx_pkt_size.clear();
      d_retx_pkt_index.clear();
      for(int k=0;k<d_retx_buffer.size();++k){
        if(d_retx_buffer[k]!=NULL)
          delete [] d_retx_buffer[k];
      }
      d_retx_buffer.clear();
      d_last_info_idx = 0;
      d_end_index.clear();
    }

    void
    interference_canceller_cc_impl::update_system_index(int queue_index)
    {
      std::map<long int,int>::iterator samp_it,sync_it;
      int block_index;
      long int time_ref;
      for(samp_it=d_samp_map.begin();samp_it!=d_samp_map.end();++samp_it){
        if(samp_it->second > queue_index){
          break;  
        }
          block_index = samp_it->second;
          time_ref = samp_it->first;  
      }
      // track back by one block
      samp_it--;
      d_samp_map.erase(d_samp_map.begin(),samp_it);
      for(samp_it=d_samp_map.begin();samp_it!=d_samp_map.end();++samp_it){
        d_samp_map[samp_it->first] = samp_it->second-block_index;
      }
      sync_it = d_sync_map.find(time_ref);
      int sync_block_idx = sync_it->second;
      if(sync_it==d_sync_map.end()){
        throw std::runtime_error("searching time index in sync buffer failed");
      }
      d_sync_map.erase(d_sync_map.begin(),sync_it);
      for(sync_it=d_sync_map.begin();sync_it!=d_sync_map.end();++sync_it){
        d_sync_map[sync_it->first] = sync_it->second - sync_block_idx;
      }
      //DEBUG required      
      // move samples
      int new_size = d_sample_size-block_index;
      memcpy(d_sample_buffer, d_sample_buffer+block_index, sizeof(gr_complex)*new_size);
      memcpy(d_phase_buffer,d_phase_buffer+sync_block_idx,sizeof(float)*(d_sync_size-sync_block_idx));
      
      d_sync_size-=sync_block_idx;
      d_sample_size = new_size;

      std::vector<int> new_info_index;
      std::vector<int> new_end_idx;
      int rm_count=0;
      
      // update info index
      for(int i=0;i<d_info_index.size();++i){
        int tmp_idx = d_info_index[i] - queue_index;
        if(tmp_idx<0){
          rm_count++;
          continue;
        }
        new_info_index.push_back(d_info_index[i]-block_index);
      }
      d_info_index = new_info_index;
      // update buffer info
      if(rm_count>0){
        d_buffer_info.erase(d_buffer_info.begin(),d_buffer_info.begin()+rm_count);  
      }
      // update last info index
      d_last_info_idx = 0;
      // update end index
      for(int i=0;i<d_end_index.size();++i){
        d_end_index[i] -= block_index;
        if(d_end_index[i]>=0){
          new_end_idx.push_back(d_end_index[i]);
        }
      }
      d_end_index = new_end_idx;
      // update retx info
      // this is a passive method, resynchronization to maintain hdr info.
      d_retx_count = 0;
      d_retx_pkt_size.clear();
      d_retx_pkt_index.clear();
      d_retx_info.clear();
      for(int i=0;i<d_retx_buffer.size();++i){
        if(d_retx_buffer[i]!=NULL){
          delete [] d_retx_buffer[i];
        }
      }
      d_retx_buffer.clear();

    }
    void
    interference_canceller_cc_impl::tags_handler(std::vector<tag_t>& tags, int nin)
    {
      int qsize, qidx;
      int offset;
      int begin_hdr_idx;

      // this should be for new tags
      // using tag absolute offset is a terrible idea. 

      for(int i=0;i<tags.size();++i){
        std::vector<tag_t> tmp_tags;
        get_tags_in_range(tmp_tags, 0,tags[i].offset,tags[i].offset+d_sps);
        offset = tags[i].offset - nitems_read(0);
        if(!tmp_tags.empty()){
          pmt::pmt_t dict = pmt::make_dict();
          for(int j =0;j<tmp_tags.size();++j){
            dict = pmt::dict_add(dict, tmp_tags[j].key, tmp_tags[j].value);
          }
          d_buffer_info.push_back(dict);
          //hdr sample len is for preamble length
          begin_hdr_idx = d_sample_size + offset - d_hdr_sample_len; 
          if(begin_hdr_idx <0){
            GR_LOG_WARN(d_logger,"header begins at negative index");
            begin_hdr_idx = 0;
          }
          d_info_index.push_back(begin_hdr_idx);
        }
      }
    }

    void
    interference_canceller_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      int items_reqd=0;
      int sync_reqd =0;
      int space = d_cap -d_sample_size;
      int sync_space = d_cap - d_sync_size;
      items_reqd = (noutput_items>space) ? space : noutput_items;
      sync_reqd = (noutput_items>sync_space)?sync_space:noutput_items;
      ninput_items_required[0] = items_reqd;
      for(int i=1;i<ninput_items_required.size();++i){
             ninput_items_required[i] = sync_reqd;
          }    
    }

    int
    interference_canceller_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      const float* in_sync_p=(const float*) input_items[1];
      const float* in_sync_k=(const float*) input_items[2];
      gr_complex *out = (gr_complex *) output_items[0];
      bool have_eng = output_items.size()>=2;
      float* eng =(have_eng) ?  (float*) output_items[1] : NULL;
      int count = 0; //for outputs
      int nin = (ninput_items[0]>noutput_items) ? noutput_items : ninput_items[0];
      int nsync= (ninput_items[1]>noutput_items) ? noutput_items : ninput_items[1];
      // maintain queue size
      if( ( (d_cap-d_sample_size) < noutput_items) || ( (d_cap - d_sync_size) < noutput_items) ){
        update_system_index(d_cap/2); //FIXME
      }
      std::vector<tag_t> sample_time, sync_time;
      get_tags_in_range(sample_time,0,nitems_read(0),nitems_read(0)+nin,pmt::intern("ctime"));
      get_tags_in_range(sync_time,1,nitems_read(1),nitems_read(1)+nsync,pmt::intern("ctime"));

      for(int i=0;i<sample_time.size();++i){
        long int tmp_time = pmt::to_long(sample_time[i].value);
        int offset = (int) d_sample_size + sample_time[i].offset - nitems_read(0);
        d_samp_map.insert(std::pair<long int,int>(tmp_time,offset));
      }
      for(int i=0;i<sync_time.size();++i){
        long int tmp_time = pmt::to_long(sync_time[i].value);
        int offset = (int) d_sync_size + sync_time[i].offset - nitems_read(1);
        d_sync_map.insert(std::pair<long int,int>(tmp_time, offset));
      }
      memcpy(d_sample_buffer+d_sample_size, in, sizeof(gr_complex)*nin);
      memcpy(d_phase_buffer+d_sync_size, in_sync_p , sizeof(float)*nsync);
      
      std::vector<tag_t> tags;
      get_tags_in_window(tags, 0,0 ,nin, pmt::intern("header_found"));
      // insert tags as dictionaries. 
      tags_handler(tags, nin);
      // update sample size
      d_sample_size+=nin;
      d_sync_size+=nsync;
      // detecting interference cancellation availability 
      // is faster by checking tag-by-tag.  
      if(cancellation_detector()){
        do_interference_cancellation();
      }
      // output status checking
      if(!d_out_index.empty()){
        output_result(noutput_items, out, eng);
      }
      else{
        produce(0,0);
        if(have_eng)
          produce(1,0);
      }
      consume(0,nin);
      consume(1,nsync);
      consume(2,nsync);
      
      return WORK_CALLED_PRODUCE;
    }

  } /* namespace lsa */
} /* namespace gr */

