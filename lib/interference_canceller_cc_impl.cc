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

namespace gr {
  namespace lsa {

    enum cancellerState{
      RECEIVE,
      OUTPUT
    };

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
      d_clean_preamble(clean_preamble),
      d_sps(sps)
    {
      d_sensing_tagname = pmt::string_to_symbol(sensing_tagname);
      const size_t capacity = 1024*4096;
      d_sample_buffer = new gr_complex[capacity];
      d_sample_size =0;
      d_sample_idx =0;
      d_output_buffer = new gr_complex[capacity];
      d_output_size =0;
      d_output_idx = 0;

      d_eng_buffer = (float*) volk_malloc( sizeof(float) * capacity, volk_get_alignment());

      d_bps = bps;
      d_hdr_bits = hdr_bits;
      d_hdr_sample_len = d_hdr_bits /d_bps * d_sps;

      d_retx_buffer.clear(); 
      d_retx_pkt_size.clear();
      d_buffer_info.clear();

      d_state = RECEIVE;
      d_debug = debug;
      set_tag_propagation_policy(TPP_DONT);
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
    interference_canceller_cc_impl::retx_handler(pmt::pmt_t hdr_info, int index)
    {
      int retx_size, retx_idx, payload_size=0;
      //float phase_est,freq_est, time_rate_est,time_k_est;
      int pkt_begin_idx = index - d_hdr_sample_len;
      if(pmt::dict_has_key(hdr_info, pmt::intern("retx_size"))){
        retx_size = pmt::to_long(pmt::dict_ref(hdr_info, pmt::intern("retx_size"), pmt::PMT_NIL));
      }
      if(pmt::dict_has_key(hdr_info, pmt::intern("retx_idx"))){
        retx_idx = pmt::to_long(pmt::dict_ref(hdr_info, pmt::intern("retx_idx"), pmt::PMT_NIL));
      }
      if(pmt::dict_has_key(hdr_info, pmt::intern("payload"))){
        payload_size = pmt::to_long(pmt::dict_ref(hdr_info, pmt::intern("payload"), pmt::PMT_NIL));
      }
      if(d_retx_buffer.empty()){
        d_retx_buffer.resize(retx_size,NULL);
        d_retx_pkt_size.resize(retx_size);
        d_retx_pkt_index.resize(retx_size);
        d_retx_count =0;
      }
      if(payload_size ==0){
        throw std::runtime_error("no payload length found");
      }
      d_retx_count++;
      //FIXME
      // can add a header length
      d_retx_pkt_size[retx_idx] = payload_size;
      d_retx_pkt_index[retx_idx] = pkt_begin_idx; 
    }

    void
    interference_canceller_cc_impl::header_handler(pmt::pmt_t hdr_info, int index)
    {
      int payload_len=0;
      //for debugging not relevent to interference cancellation
      int counter, qidx, qsize;
      float freq, phase, time_freq, time_phase;
      bool sensing_info=true;
      int pkt_begin_idx = index - d_hdr_sample_len;
      if(pmt::dict_has_key(hdr_info, d_sensing_tagname)){
        sensing_info = pmt::to_bool(pmt::dict_ref(hdr_info, d_sensing_tagname,pmt::PMT_NIL));
      }
      if(pmt::dict_has_key(hdr_info, pmt::intern("payload"))){
        payload_len = pmt::to_long(pmt::dict_ref(hdr_info, pmt::intern("payload"),pmt::PMT_NIL));
      }
      if(pmt::dict_has_key(hdr_info, pmt::intern("counter"))){
        counter = pmt::to_long(pmt::dict_ref(hdr_info, pmt::intern("counter"),pmt::PMT_NIL));
      }
      if(pmt::dict_has_key(hdr_info, pmt::intern("queue_index"))){
        qidx = pmt::to_long(pmt::dict_ref(hdr_info, pmt::intern("queue_index"),pmt::PMT_NIL));
      }
      if(pmt::dict_has_key(hdr_info, pmt::intern("queue_size"))){
        qsize = pmt::to_long(pmt::dict_ref(hdr_info, pmt::intern("queue_size"),pmt::PMT_NIL));
      }
      if(pmt::dict_has_key(hdr_info, pmt::intern("time_rate_f_est"))){
        time_freq = pmt::to_float(pmt::dict_ref(hdr_info, pmt::intern("time_rate_f_est"), pmt::PMT_NIL));
      }
      if(pmt::dict_has_key(hdr_info, pmt::intern("time_k_est"))){
        time_phase = pmt::to_float(pmt::dict_ref(hdr_info, pmt::intern("time_k_est"), pmt::PMT_NIL)); 
      }
      if(pmt::dict_has_key(hdr_info, pmt::intern("freq_est"))){
        freq = pmt::to_float(pmt::dict_ref(hdr_info, pmt::intern("freq_est"), pmt::PMT_NIL));
      }
      if(pmt::dict_has_key(hdr_info, pmt::intern("phase_est"))){
        phase = pmt::to_float(pmt::dict_ref(hdr_info, pmt::intern("phase_est"), pmt::PMT_NIL));  
      }

      d_buffer_info.push_back(hdr_info);
      d_info_index.push_back(pkt_begin_idx);

    }
    void
    interference_canceller_cc_impl::sync_hdr_index(std::vector<int>& coerced_packet_len)
    {
      if(d_info_index.empty())
        throw std::runtime_error("Cancellation failed: no header info found");
      int cur_pkt_begin;
      int count =0;
      int next_pkt_begin;
      int test_pkt_len;
      pmt::pmt_t tmp_dict;
      int cur_payload;
      int retx_idx;
      int fixed_pkt_len;
      while(count < d_info_index.size()-1){
        
        tmp_dict = d_buffer_info[count];
        cur_payload = pmt::to_long(pmt::dict_ref(tmp_dict, pmt::intern("payload"), pmt::PMT_NIL));
        cur_pkt_begin = d_info_index[count];
        next_pkt_begin = d_info_index[count+1];
        test_pkt_len = next_pkt_begin - cur_pkt_begin - (cur_payload + d_hdr_sample_len);
        
        if(test_pkt_len == 0){
          //fix normal case
          fixed_pkt_len = cur_payload + d_hdr_sample_len;
        }
        else if( abs(test_pkt_len)<=d_sps ){
            fixed_pkt_len = cur_payload + d_hdr_sample_len + test_pkt_len;
        }
        else{
          //not in the range of next header information
          fixed_pkt_len = cur_payload + d_hdr_sample_len;
        }
        if( pmt::dict_has_key(tmp_dict, pmt::intern("retx_idx"))){
          retx_idx = pmt::to_long(pmt::dict_ref(tmp_dict, pmt::intern("retx_idx"), pmt::PMT_NIL));
          d_retx_pkt_size[retx_idx] = fixed_pkt_len;
        }
        coerced_packet_len.push_back(fixed_pkt_len);
        count++;
      }
      // fix last pkt
      tmp_dict = d_buffer_info.back();
      cur_pkt_begin = d_info_index.back();
      cur_payload = pmt::to_long(pmt::dict_ref(tmp_dict, pmt::intern("payload"), pmt::PMT_NIL));
      if(cur_pkt_begin + cur_payload+ d_hdr_sample_len >= d_sample_size){
        fixed_pkt_len = d_sample_size - cur_pkt_begin;
      }
      else{
        fixed_pkt_len = cur_pkt_begin + cur_payload+ d_hdr_sample_len;
      }
      if( pmt::dict_has_key(tmp_dict, pmt::intern("retx_idx"))){
          retx_idx = pmt::to_long(pmt::dict_ref(tmp_dict, pmt::intern("retx_idx"), pmt::PMT_NIL));
          d_retx_pkt_size[retx_idx] = fixed_pkt_len; 
      }
      coerced_packet_len.push_back(fixed_pkt_len); 
      //create copy of retransmission
      for(int i=0;i<d_retx_count;++i){
        d_retx_buffer[i] = new gr_complex[d_retx_pkt_size[i]];
        memcpy(d_retx_buffer[i],d_sample_buffer+d_retx_pkt_index[i], sizeof(gr_complex)* d_retx_pkt_size[i]);
      }

    }

    void
    interference_canceller_cc_impl::do_interference_cancellation()
    {
      if(d_debug){
        std::cout<<"<Debug>: do interference cancellation"<<std::endl;
        std::cout<<"header info:"<<std::endl;
        for(int i=0;i<d_buffer_info.size();++i){
          pmt::pmt_t tmp_dict=d_buffer_info[i];
          tmp_dict = pmt::dict_delete(tmp_dict, pmt::intern("time_rate_f_est"));
          tmp_dict = pmt::dict_delete(tmp_dict, pmt::intern("time_k_est"));
          tmp_dict = pmt::dict_delete(tmp_dict, pmt::intern("freq_est"));
          tmp_dict = pmt::dict_delete(tmp_dict, pmt::intern("phase_est"));
          std::cout<<"index:"<<d_info_index[i]<<" content:"<<tmp_dict<<std::endl;
        }
        std::cout<<"retx count:"<<d_retx_count<<std::endl;  
      }
      //FIXME

      //can initialize output buffer first
      std::vector<int> packet_len;
      sync_hdr_index(packet_len);
      //find last cei
      int last_cei_idx=0;
      int last_cei_qiter = 0;
      int total_retx_size = 0;
      for(int i=0;i<d_retx_pkt_index.size();++i){
        if(d_retx_pkt_index[i]>last_cei_idx){
          last_cei_idx = d_retx_pkt_index[i];
          last_cei_qiter = i;
        }
        total_retx_size += d_retx_pkt_size[i];
      }
      int retx_size = d_retx_buffer.size();
      d_cei_pkt_counter = last_cei_qiter;
      d_cei_sample_counter = d_retx_pkt_size[last_cei_qiter];

      int total_size = d_retx_pkt_index[d_cei_pkt_counter] + d_retx_pkt_size[d_cei_pkt_counter];

      d_output_size = total_size;
      std::fill_n(d_output_buffer, d_output_size, gr_complex(0,0));

      int offset =0;
      int sample_count=0;
      int begin_idx=0;
      int next_pkt_begin=0;
      int next_pkt_size=0;
      int test_index=0;
      gr_complex* retx;
      pmt::pmt_t tmp_dict;
      
      while(total_size>0)
      {
        retx = d_retx_buffer[d_cei_pkt_counter];
        begin_idx = total_size - d_cei_sample_counter;
        if(begin_idx <0){
          offset = d_retx_pkt_size[d_cei_pkt_counter] - total_size;
          sample_count = total_size;
          begin_idx =0;
        }
        else{
          offset = 0;
          sample_count = d_retx_pkt_size[d_cei_pkt_counter];
        }
        
        if(!d_info_index.empty()){
          tmp_dict = d_buffer_info.back();

          next_pkt_begin = d_info_index.back();
          next_pkt_size = packet_len.back();
          test_index = begin_idx - next_pkt_begin;

          // can modified to handle more complex time offset
          //FIXME
          // can adjust to accommodate offset of larger range
          if(abs(test_index)<= d_sps){
            //begin_idx -= test_index;
            begin_idx = next_pkt_begin;
            d_buffer_info.pop_back();
            d_info_index.pop_back();
            packet_len.pop_back();
            
          }
        }//sync with known index of packet        
        for(int i=0;i<sample_count;++i){
          //if there is cfo or time offset, fixed here?
          d_output_buffer[begin_idx+i] = d_sample_buffer[begin_idx+i] - retx[i+offset];
        }
        total_size = begin_idx;
        d_cei_pkt_counter = (d_cei_pkt_counter-1) % retx_size;
        if(d_cei_pkt_counter<0)
          d_cei_pkt_counter+= retx_size;
        d_cei_sample_counter = d_retx_pkt_size[d_cei_pkt_counter];
      }
      
      d_output_idx =0;
    }

    void
    interference_canceller_cc_impl::output_result(int noutput_items, gr_complex* out, float* eng)
    {
      if(d_debug)
      {
        std::cout<<"<output_result>"<< " out_idx:"<<d_output_idx<<" ,out_size:"<<d_output_size<<std::endl;
      }
      int out_count =0;
      int begin_idx = d_output_idx;
      int nout = (noutput_items > (d_output_size-d_output_idx)) ? (d_output_size-d_output_idx) : noutput_items;
      for(int i=0;i<nout;++i){
        out[i] = d_output_buffer[d_output_idx++];
      }
      produce(0,nout);
      if(eng!=NULL && (nout!=0))
      {
        //GR_LOG_DEBUG(d_logger, "output samples with energy");
        volk_32fc_magnitude_squared_32f(d_eng_buffer, d_output_buffer+begin_idx, nout);
        for(int i=0;i<nout;++i){
          eng[i] = d_eng_buffer[i];
          //eng[i] = 8.7;
        }
        produce(1,nout);
      }
      if(d_output_idx == d_output_size){
        d_state = RECEIVE;
        d_output_idx = 0;
        d_output_size =0; 
      }
      
    }

    void
    interference_canceller_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      switch(d_state)
      {
        case RECEIVE:
          for(int i=0;i<ninput_items_required.size();++i){
             ninput_items_required[i] = noutput_items;
          }    
        break;
        case OUTPUT:
          for(int i=0;i<ninput_items_required.size();++i)
            ninput_items_required[i] = 0;
        break;
        default:
          std::runtime_error("Entering wrong state");
        break;
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
      int nin = (ninput_items[0]>noutput_items) ? noutput_items : ninput_items[0];
      //handle sample_size properly;
      // Do <+signal processing+>
      std::vector<tag_t> tags;
      std::vector<tag_t> end_tags;
      std::vector<tag_t> hdr_tags;
      get_tags_in_window(tags, 0, 0, nin, pmt::intern("begin_of_retx"));
      get_tags_in_window(end_tags, 0,0 ,nin, pmt::intern("end_of_retx"));
      get_tags_in_window(hdr_tags, 0,0 ,nin, pmt::intern("header_found"));
      
      int count = 0 ;

      switch(d_state)
      {
        case RECEIVE:
          for(int i=0;i<nin;++i){
        count++;
        if(!tags.empty()){
        int offset = tags.back().offset - nitems_read(0);
          if(offset == i){
            d_sample_idx = 0;
            d_sample_size = 0;
            tags.erase(tags.begin());
          }
        }
        
        if(!hdr_tags.empty()){
          int offset = hdr_tags[0].offset - nitems_read(0);
          if(i==offset){
            std::vector<tag_t> tmp_tags;
            //window size?
            get_tags_in_window(tmp_tags, 0,i, i+4);
            pmt::pmt_t tmp_dict=pmt::make_dict();
            for(int j=0;j<tmp_tags.size();++j){
              tmp_dict = pmt::dict_add(tmp_dict, tmp_tags[j].key, tmp_tags[j].value);
            }
            if(pmt::dict_has_key(tmp_dict, pmt::intern("retx_idx"))){
              retx_handler(tmp_dict, d_sample_idx);
            }
            if(pmt::dict_has_key(tmp_dict, pmt::intern("header_found"))){
              header_handler(tmp_dict, d_sample_idx);
            }
            hdr_tags.erase(hdr_tags.begin());
          }
        }
        d_sample_buffer[d_sample_idx++] = in[i];
        d_sample_size++;

        if(!end_tags.empty()){
        int offset = end_tags.back().offset - nitems_read(0);
        //output
        if(offset == i){
          do_interference_cancellation();
        d_retx_count=0;
        for(int i=0;i<d_retx_buffer.size();++i){
          if(d_retx_buffer[i]!=NULL)
          delete [] d_retx_buffer[i];
        }
          d_retx_buffer.clear();
          d_retx_pkt_size.clear();
          d_retx_pkt_index.clear();
          d_buffer_info.clear();
          d_info_index.clear();
          d_sample_idx = 0;
          d_sample_size =0;
          d_state = OUTPUT;  
          end_tags.erase(end_tags.begin());
          break;
        }
        }
      }
        consume_each(count);
        break;
        case OUTPUT:
          output_result(noutput_items, out, eng);
          consume_each(0);
        break;
      }
      // Tell runtime system how many input items we consumed on
      // each input stream.
      
      // Tell runtime system how many output items we produced.
      return WORK_CALLED_PRODUCE;
    }

  } /* namespace lsa */
} /* namespace gr */

