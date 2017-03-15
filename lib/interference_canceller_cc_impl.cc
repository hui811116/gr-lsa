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
    interference_canceller_cc_impl::interference_canceller_cc_impl(const std::vector<gr_complex>& clean_preamble,
      const std::string& sensing_tagname,
      int sps,
      int bps,
      int hdr_bits,
      bool debug)
      : gr::block("interference_canceller_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make2(1, 2, sizeof(gr_complex), sizeof(float))),
      d_cap(1024*4096),
      d_clean_preamble(clean_preamble),
      d_sps(sps)
    {
      d_sensing_tagname = pmt::string_to_symbol(sensing_tagname);
      d_sample_buffer = new gr_complex[d_cap];
      d_sample_size =0;
      d_sample_idx =0;
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

      d_last_info_idx = 0;
    }

    /*
     * Our virtual destructor.
     */
    interference_canceller_cc_impl::~interference_canceller_cc_impl()
    {
      delete [] d_sample_buffer;
      delete [] d_output_buffer;
      for(int i=0;i<d_retx_buffer.size();++i){
        if(d_retx_buffer[i]!=NULL)
        delete [] d_retx_buffer[i];
      }
      d_retx_buffer.clear();

      volk_free(d_eng_buffer);
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
      int pkt_begin_idx = offset - d_hdr_sample_len;
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
        d_retx_buffer[qindex] = new gr_complex[payload_size + d_hdr_sample_len];
        d_retx_count++;
        //this is naive sample length
        d_retx_pkt_size[qindex] = payload_size + d_hdr_sample_len;
        d_retx_pkt_index[qindex] = pkt_begin_idx;
        d_retx_info[qindex] = hdr_info;
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
      float phase_shift, freq_offset;
      gr_complex sample_fix;
      pmt::pmt_t tmp_dict;

      while(count < d_info_index.size()){
        tmp_dict = d_buffer_info[count];
        cur_payload = pmt::to_long(pmt::dict_ref(tmp_dict, pmt::intern("payload"), pmt::PMT_NIL));
        cur_pkt_begin = d_info_index[count];
        if(cur_pkt_begin + cur_payload + d_hdr_sample_len >= end_idx){
          break;
        }
        if(count == d_info_index.size()-1){
          next_pkt_begin = d_sample_size;
          GR_LOG_WARN(d_logger, "index info reaching end of available information");
        }
        else{
          next_pkt_begin = d_info_index[count+1];  
        }
        
        test_pkt_len = next_pkt_begin - cur_pkt_begin - (cur_payload + d_hdr_sample_len);
        if( abs(test_pkt_len)<=d_sps ){
          fixed_pkt_len = cur_payload + d_hdr_sample_len + test_pkt_len;
        }
        else{
          //not in the range of next header information
          fixed_pkt_len = cur_payload + d_hdr_sample_len;
        }
        coerced_packet_len.push_back(fixed_pkt_len);
        buffer_info.push_back(tmp_dict);
        info_index.push_back(cur_pkt_begin);
        count++;
      }
      
      //create copy of retransmission
      for(int i=0;i<d_retx_count;++i){
        //fix its own impairments first?
        phase_shift = pmt::to_float(pmt::dict_ref(d_retx_info[i],pmt::intern("phase_est"),pmt::PMT_NIL));
        freq_offset = pmt::to_float(pmt::dict_ref(d_retx_info[i],pmt::intern("freq_est"),pmt::PMT_NIL));
        freq_offset/= static_cast<float>(d_sps);
        //FIXME
        // carrier frequency and phase correction for retransmission.
        // this method performs so poor!
        /*
        for(int j=0;j<d_retx_pkt_size[i];++j){
          sample_fix = d_sample_buffer[d_retx_pkt_index[i]+j] * gr_expj(-phase_shift);
          d_retx_buffer[i][j] = sample_fix;
          phase_shift += freq_offset;
          while(phase_shift>TWO_PI){
            phase_shift-= TWO_PI;
          }
          while(phase_shift<-TWO_PI){
            phase_shift+= TWO_PI;
          }
        }*/
        // this is for non correcting case
        memcpy(d_retx_buffer[i],d_sample_buffer+d_retx_pkt_index[i], sizeof(gr_complex)* d_retx_pkt_size[i]);
      }

    }

    void
    interference_canceller_cc_impl::do_interference_cancellation()
    {
      
      int end_idx = d_end_index.front();
      d_end_index.erase(d_end_index.begin());
      //do signal processing here,
      //move result to output buffer
      //push back output_index
      
      //can initialize output buffer first
      std::vector<int> packet_len;
      std::vector<pmt::pmt_t> buffer_info;
      std::vector<int> info_index;
      //find last cei
      int last_cei_idx=0 , last_cei_qiter = 0;
      int total_retx_size = 0;
      int cei_pkt_counter, cei_sample_counter;
      int retx_size;
      int total_size;
      int offset =0;
      int sample_count=0;
      int begin_idx=0, next_pkt_begin=0, next_pkt_size=0;
      int test_index=0;
      float phase_shift, freq_offset;
      gr_complex* retx, fixed_sample;
      pmt::pmt_t tmp_dict;

      sync_hdr_index(packet_len, buffer_info, info_index, end_idx);

      for(int i=0;i<d_retx_pkt_index.size();++i){
        if(d_retx_pkt_index[i]>last_cei_idx){
          last_cei_idx = d_retx_pkt_index[i];
          last_cei_qiter = i;
        }
        total_retx_size += d_retx_pkt_size[i];
      }
      retx_size = d_retx_buffer.size();
      cei_pkt_counter = last_cei_qiter;
      cei_sample_counter = d_retx_pkt_size[last_cei_qiter];

      total_size = d_retx_pkt_index[cei_pkt_counter] + d_retx_pkt_size[cei_pkt_counter];

      //d_output_size += total_size;
      d_out_index.push_back(d_output_size+total_size);
      std::cout<<"output end size:"<<d_output_size+total_size<<std::endl;
      if(d_output_size+total_size >d_cap){
        throw std::runtime_error("output size greater than buffer capacity");
      }
      std::fill_n(d_output_buffer+d_output_size, total_size, gr_complex(0,0));

      /*
      while(total_size>0)
      {
        retx = d_retx_buffer[cei_pkt_counter];
        begin_idx = total_size - cei_sample_counter;
        if(begin_idx <0){
          // in case out of sample range
          // offset is the truncated size of first packet
          offset = d_retx_pkt_size[cei_pkt_counter] - total_size;
          sample_count = total_size;
          begin_idx =0;
        }
        else{
          // normal case, samples are enough so offset =0;
          offset = 0;
          sample_count = d_retx_pkt_size[cei_pkt_counter];
        }
        
        if(!d_info_index.empty()){
          // FIXME
          // copy info before doing this part
          tmp_dict = buffer_info.back();
          next_pkt_begin = info_index.back();
          next_pkt_size = packet_len.back();
          test_index = begin_idx - next_pkt_begin;
          // can modified to handle more complex time offset
          //FIXME
          // can adjust to accommodate offset of larger range
          if(abs(test_index)<= 2*d_sps){
            //FIXME
            phase_shift = pmt::to_float(pmt::dict_ref(tmp_dict, pmt::intern("phase_est"), pmt::PMT_NIL));
            freq_offset = pmt::to_float(pmt::dict_ref(tmp_dict, pmt::intern("freq_est"), pmt::PMT_NIL));
            freq_offset/= static_cast<float>(d_sps);//change to sample-wise frequency offset

            begin_idx = next_pkt_begin;
            buffer_info.pop_back();
            info_index.pop_back();
            packet_len.pop_back(); 
          }
        }//sync with known index of packet        
        for(int i=0;i<sample_count;++i){
          //NO CFO case
          fixed_sample = d_sample_buffer[begin_idx+i] - retx[i+offset];
          //First method , synchronize to SU signal
          //fixed_sample = d_sample_buffer[begin_idx+i] * gr_expj(-phase_shift) - retx[i+offset];
          //Second method, leave possible proU CFO unchanged 
          //NOTE: in expj the sign is the same as desired sample in order to synchronize the phase of two samples
          //fixed_sample = retx[i+offset]*gr_expj(phase_shift);
          //if there is cfo or time offset, fixed here?
          //d_output_buffer[begin_idx+i] = d_sample_buffer[begin_idx+i] - retx[i+offset];
          //d_output_buffer[begin_idx+i] = d_sample_buffer[begin_idx+i] - fixed_sample;
          d_output_buffer[d_output_idx + begin_idx+i] = fixed_sample;
          
          phase_shift += freq_offset;
          while(phase_shift>TWO_PI){
            phase_shift-=TWO_PI;
          }
          while(phase_shift<-TWO_PI){
            phase_shift+=TWO_PI;
          }
        }
        //abuse variable total_size to sync to the estimated begin of previous packet
        total_size = begin_idx;
        cei_pkt_counter = (cei_pkt_counter-1) % retx_size;
        if(cei_pkt_counter<0)
          cei_pkt_counter+= retx_size;
        cei_sample_counter = d_retx_pkt_size[cei_pkt_counter];
      }//end while

      */
      d_output_size = d_out_index.back();      
      update_system_index(end_idx);
    }

    void
    interference_canceller_cc_impl::output_result(int noutput_items, gr_complex* out, float* eng)
    {
      /*
      std::cout<<"*****************************************"<<std::endl;
      std::cout<<"<debug> output_result: begin"<<std::endl;
      std::cout<<"output index:"<<d_output_idx<<" output_size:"<<d_output_size<<std::endl;
      std::cout<<"end indexes:"<<std::endl;
      for(int i=0;i<d_out_index.size();++i){
        std::cout<<"["<<i<<"]"<<d_out_index[i]<<std::endl;
      }*/

      int out_count =0;
      int end_idx = d_out_index.front();
      int nout = (noutput_items > (d_output_size-d_output_idx)) ? (d_output_size-d_output_idx) : noutput_items;

      memcpy(out,d_output_buffer+d_output_idx,sizeof(gr_complex)*nout);
         
      produce(0,nout);
      if(eng!=NULL && (nout!=0))
      {
        volk_32fc_magnitude_squared_32f(d_eng_buffer, d_output_buffer+d_output_idx, nout);
        for(int i=0;i<nout;++i){
          eng[i] = d_eng_buffer[i];
        }
        produce(1,nout);
      }
      d_output_idx += nout;
      //update output buffer
      if(d_output_idx >= end_idx){
        int new_size = d_output_size - end_idx;
        memcpy(d_output_buffer, d_output_buffer+ end_idx, sizeof(gr_complex)* new_size);
        d_output_size-=end_idx;
        d_output_idx -= end_idx;
        d_out_index.erase(d_out_index.begin());
        for(int i=0;i<d_out_index.size();++i){
          d_out_index[i]-=end_idx;
          if(d_out_index[i]<0){
            throw std::runtime_error("output index cannot be negative");
          }
        }
      }
      /*
      std::cout<<"-----------------------------------------------"<<std::endl;
      std::cout<<"after update:"<<std::endl;
      std::cout<<"nout:"<<nout<<std::endl;
      std::cout<<"end idx:"<<std::endl;
      for(int i=0;i<d_out_index.size();++i){
        std::cout<<"["<<i<<"]"<<d_out_index[i]<<std::endl;
      }
      std::cout<<"output index:"<<d_output_idx<<" output_size:"<<d_output_size<<std::endl;
      std::cout<<"<debug> output_result ends"<<std::endl;
      std::cout<<"****************************************************"<<std::endl;
      */
    }

    void
    interference_canceller_cc_impl::cancellation_detector()
    {
      int qsize,qidx;
      int pld_len, sample_idx;
      int valid_size;
      bool success=false, failed=false;
      int end_iter = 0;
      if(d_buffer_info.empty()){
        return;
      }
      else{
        //update index
        while(d_last_info_idx<d_buffer_info.size()){
          pld_len = pmt::to_long(pmt::dict_ref(d_buffer_info[d_last_info_idx],pmt::intern("payload"),pmt::PMT_NIL));
          valid_size = d_info_index[d_last_info_idx] + pld_len;
          if(valid_size>d_sample_size){
            //samples not enough
            return;
          }
          if(pmt::dict_has_key(d_buffer_info[d_last_info_idx],pmt::intern("queue_index"))){
            qidx = pmt::to_long(pmt::dict_ref(d_buffer_info[d_last_info_idx],pmt::intern("queue_index"),pmt::PMT_NIL));
          }
          if(pmt::dict_has_key(d_buffer_info[d_last_info_idx],pmt::intern("queue_size"))){
            qsize = pmt::to_long(pmt::dict_ref(d_buffer_info[d_last_info_idx],pmt::intern("queue_size"),pmt::PMT_NIL));
          }
          if(qidx==0 && qsize==0 && !d_retx_buffer.empty()){
            //this case meant for end of retransmission.
            //should declare failed and fix queue and indexes
            end_iter = d_last_info_idx-1;
            if(end_iter<0){
              throw std::runtime_error("detection error, setting retransmission before a valid one received");
            }
            //NOTE: when this case happens, it had passed the desired position at least one pkt.
            failed = true;
            break;
          }
          if(qsize!=d_retx_buffer.size() && !d_retx_buffer.empty()){
            // this case means a new retransmission size encountered
            // should update info and tracking for new size
            end_iter = d_last_info_idx-1;
            if(end_iter<0){
              throw std::runtime_error("detection error, setting retransmission before a valid one received");
            }
            failed = true;
            break;
          }
          retx_check(d_buffer_info[d_last_info_idx],qidx,qsize,d_info_index[d_last_info_idx]);
          if(!d_retx_buffer.empty() && d_retx_count == d_retx_buffer.size()){
            //interference cancellation available
            end_iter = d_last_info_idx;
            success = true;
            d_last_info_idx++;
            break;
          }
          d_last_info_idx++;
        }//end while 
        
        sample_idx = pmt::to_long(pmt::dict_ref(d_buffer_info[end_iter],pmt::intern("payload"),pmt::PMT_NIL));
        sample_idx+= d_info_index[end_iter];
        if(success){
        // copy to another buffer?
        // do_interference cancellation 
        // move to output buffer?
          d_end_index.push_back(sample_idx);
          if(d_debug){
            std::cout<<"<debug>" <<"interference cancellation available!"<<std::endl;
            for(int i=0;i<d_retx_pkt_index.size();++i){
              std::cout<<"["<<i<<"] "<<d_retx_pkt_index[i]<< " ,pkt size:"<<d_retx_pkt_size[i]<< std::endl;
            }
            std::cout<<"************************************************"<<std::endl;
          }
        }
        else if(failed){
          if(d_debug)
            std::cout<<"<debug>: failed calling update system index to fix info"<<std::endl;
          update_system_index(sample_idx);
        }//other cases?
      }
    }

    void
    interference_canceller_cc_impl::update_system_index(int queue_index)
    {
      // move samples
      int new_size = d_sample_size-queue_index;
      memcpy(d_sample_buffer, d_sample_buffer+queue_index, sizeof(gr_complex)*new_size);

      d_sample_size = new_size;
      d_sample_idx = 0;

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
        new_info_index.push_back(d_info_index[i]-queue_index);
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
        d_end_index[i] -= queue_index;
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
            std::cout<<"<debug>"<<"sample_size:"<<d_sample_size<<" ,offset:"<<offset<<" ,d_hdr_sample_len:"<<d_hdr_sample_len<<std::endl;
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

      int space = d_cap -d_sample_size;

      items_reqd = (noutput_items>space) ? space : noutput_items;
      for(int i=0;i<ninput_items_required.size();++i){
             ninput_items_required[i] = items_reqd;
          }    
    }

    int
    interference_canceller_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
      bool have_eng = output_items.size()>=2;
      float* eng =(have_eng) ?  (float*) output_items[1] : NULL;
      int count = 0; //for outputs
      int nin = (ninput_items[0]>noutput_items) ? noutput_items : ninput_items[0];
      // maintain queue size
      if(d_cap-d_sample_size < noutput_items){
        update_system_index(d_cap/2);
      }
      nin = (d_cap - d_sample_size > nin) ? nin : (d_cap - d_sample_size);
      // handle sample_size properly;
      memcpy(d_sample_buffer+d_sample_size, in, sizeof(gr_complex)* nin);
      // Do <+signal processing+>
      std::vector<tag_t> tags;
      get_tags_in_window(tags, 0,0 ,nin, pmt::intern("header_found"));
      // insert tags as dictionaries. 
      tags_handler(tags, nin);
      // update sample size
      d_sample_size+=nin; 
      // checking interference cancellation availability
      cancellation_detector();
      if(!d_end_index.empty()){
        do_interference_cancellation();
        //do one end index
      }
      // output status checking
      if(!d_out_index.empty()){
        output_result(noutput_items, out, eng);
      }
      else{
        produce(0,0);
        produce(1,0);
      }
      consume_each(nin);

      return WORK_CALLED_PRODUCE;
    }

  } /* namespace lsa */
} /* namespace gr */

