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
#include "prou_sample_queue_cc_impl.h"

#include <ctime>

namespace gr {
  namespace lsa {

    enum outputState{
      LOAD_SAMPLE,
      WAIT_INFO,
      OUTPUT_SAMPLE,
    };

    prou_sample_queue_cc::sptr
    prou_sample_queue_cc::make(const std::string& sensing_tagname, bool debug)
    {
      return gnuradio::get_initial_sptr
        (new prou_sample_queue_cc_impl(sensing_tagname, debug));
    }

    /*
     * The private constructor
     */
    prou_sample_queue_cc_impl::prou_sample_queue_cc_impl(const std::string& sensing_tagname, bool debug)
      : gr::block("prou_sample_queue_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(2, 2, sizeof(gr_complex))),
      d_sample_cap(4096*1024)
    {
      d_sample_idx = 0;
      d_sample_size =0;
      d_sample_buffer = new gr_complex[d_sample_cap];
      d_state = LOAD_SAMPLE;

      d_info_port = pmt::mp("info");
      message_port_register_in(d_info_port);
      set_msg_handler(d_info_port, boost::bind(&prou_sample_queue_cc_impl::info_msg_handler, this, _1));

      d_update_time = std::clock();
      d_timeout = 10.0;
      d_retx_count = 0;
      d_last_retx_idx = 0;
      d_sensing_tagname = pmt::string_to_symbol(sensing_tagname);
      
      d_debug = debug;
    }

    /*
     * Our virtual destructor.
     */
    prou_sample_queue_cc_impl::~prou_sample_queue_cc_impl()
    {
      delete [] d_sample_buffer;
    }

    void
    prou_sample_queue_cc_impl::info_msg_handler(pmt::pmt_t msg)
    {
      long int time;
      int counter=0, qidx, qsize, offset, sample_len;
      bool sensing_state=false;
      pmt::pmt_t tag_info;
      if(pmt::dict_has_key(msg, d_sensing_tagname)){
        sensing_state = pmt::to_bool(pmt::dict_ref(msg, d_sensing_tagname, pmt::PMT_NIL));
      }
      if(pmt::dict_has_key(msg, pmt::intern("ctime")))
      {
        time = pmt::to_long(pmt::dict_ref(msg, pmt::intern("ctime"), pmt::PMT_NIL));
      }
      if(time < d_update_time){
            //outdated info, skip
            return;
          }
      if(pmt::dict_has_key(msg, pmt::intern("queue_index")))
      {
        qidx = pmt::to_long(pmt::dict_ref(msg, pmt::intern("queue_index"), pmt::PMT_NIL));
      }
      if(pmt::dict_has_key(msg, pmt::intern("queue_size")))
      {
        qsize = pmt::to_long(pmt::dict_ref(msg, pmt::intern("queue_size"), pmt::PMT_NIL));
      }
      if(pmt::dict_has_key(msg, pmt::intern("buffer_offset")))
      {
        offset = pmt::to_long(pmt::dict_ref(msg, pmt::intern("buffer_offset"), pmt::PMT_NIL));
      }
      if(pmt::dict_has_key(msg, pmt::intern("payload")))
      {
        sample_len = pmt::to_long(pmt::dict_ref(msg, pmt::intern("payload"), pmt::PMT_NIL));
      }
      long int tmp_time;
      int tmp_idx;
      bool retx_update=false;
      if( (qsize!=0) && (d_retx_status.size()==0) ){
            d_retx_status.resize(qsize,false);
            d_retx_count = 0;
      } 
      if(!sensing_state){
            for(int i=0;i<d_buffer_info.size();++i){
              tmp_time = pmt::to_long(pmt::dict_ref(d_buffer_info[i], pmt::intern("ctime"),pmt::PMT_NIL));
              tmp_idx = pmt::to_long(pmt::dict_ref(d_buffer_info[i], pmt::intern("buffer_index"),pmt::PMT_NIL));
              if(time == tmp_time){
                //found corresponding time index
                // attach header info on sample
                pmt::pmt_t info_tag = msg;
                int info_index = tmp_idx + offset;
                //this tag should be different from feedback, since it will be used by down stream
                info_tag = pmt::dict_delete(info_tag, pmt::intern("buffer_offset"));
                info_tag = pmt::dict_add(info_tag, pmt::intern("header_found"), pmt::from_long(info_index));
                if((qsize!=0) &&(qsize == d_retx_status.size()) && 
                  (qidx < qsize) && (d_retx_status[qidx] == false) ){
                  retx_update = true;
                  d_retx_status[qidx] = true;
                  d_retx_count++;
                  d_last_retx_idx = info_index;
                  d_last_retx_samples = sample_len;
                  info_tag = pmt::dict_add(info_tag, pmt::intern("retx_idx"), pmt::from_long(qidx));
                  info_tag = pmt::dict_add(info_tag, pmt::intern("retx_size"), pmt::from_long(qsize));
                }
                  d_pkt_info.push_back(info_tag);
                  break;
                }
              }
            }
      switch(d_state){
        case LOAD_SAMPLE:
          if(sensing_state || (qsize!=0) ){
            d_state = WAIT_INFO;
          }
        break;
        case WAIT_INFO:
          if( (d_retx_count == d_retx_status.size()) && 
            ((d_last_retx_idx + d_last_retx_samples) <= d_sample_size) ){
            d_last_time = 0;
            d_state = OUTPUT_SAMPLE;
            d_end_retx_sample_size = d_last_retx_idx + d_last_retx_samples;
          }
          else if( (qsize == 0) && (qidx==0)
          &&( d_retx_count!= d_retx_status.size()) ){
            clear_queue_index_fix(time);
            d_state = LOAD_SAMPLE;
          }
        break;
        case OUTPUT_SAMPLE:
          if(d_last_time==0){
            d_last_time =time;
          }
        break;
        default:
          throw std::runtime_error("Entering wrong state");
        break;
      }
    }
    void 
    prou_sample_queue_cc_impl::append_samples(
      const gr_complex* in, 
      int ninput_items,
      int noutput_items, 
      int& consume_count, 
      long int time)
    {
      consume_count = 0;
      if(ninput_items == 0){
        return;
      }
      int space_left = d_sample_cap - d_sample_size;
      std::vector<tag_t> tags;
      get_tags_in_range(tags, 0, nitems_read(0),nitems_read(0)+noutput_items, d_sensing_tagname);
      pmt::pmt_t mark_sample = pmt::make_dict();
      switch(d_state)
      {
        case LOAD_SAMPLE:
          if(d_sample_size == d_sample_cap){
            reduce_samples();
            space_left = d_sample_cap - d_sample_size;
          }      
        break;
        case WAIT_INFO:
          // if timeout --> failed
          if(((time-d_update_time)/CLOCKS_PER_SEC)>=d_timeout){
            d_sample_idx = 0;
            d_sample_size = 0;
            d_state = LOAD_SAMPLE;
            d_retx_status.clear();
            d_retx_count = 0;
            if(d_debug){
              GR_LOG_DEBUG(d_logger,"timeout, resetting sample buffer");
            }
            consume_count = 0;
            return;
          }
        break;
        case OUTPUT_SAMPLE:
          consume_count = 0;
          return;
        break;
        default:
          std::runtime_error("Entering wrong state");
        break;
      }   
      if(ninput_items <= space_left){
            memcpy(d_sample_buffer+d_sample_size, in , sizeof(gr_complex)*ninput_items);
            consume_count = ninput_items;
          }
          else{
            memcpy(d_sample_buffer+d_sample_size, in, sizeof(gr_complex)*space_left);
            consume_count = space_left;
          }
          if( (consume_count !=0) ){
            //d_update_time = time;
            if((d_sample_size ==0)||(d_update_time==0) ){
              d_update_time =time;
            }
            mark_sample = pmt::dict_add(mark_sample, pmt::intern("ctime"), pmt::from_long(time));
            mark_sample = pmt::dict_add(mark_sample, pmt::intern("buffer_index"), pmt::from_long(d_sample_size));
            d_sample_size+= consume_count;
            d_buffer_info.push_back(mark_sample);
          }
    }

    void
    prou_sample_queue_cc_impl::reduce_samples()
    {
      int samples_left;
      int samples_rm;
      int idx;
      long int time;
      //
      int sample_count =0;
      int reduced_size = d_sample_size/2;
      for(int i=0;i<d_buffer_info.size();++i){
        int tmp_size = pmt::to_long(pmt::dict_ref(d_buffer_info[i],pmt::intern("buffer_index"),pmt::PMT_NIL));
        sample_count = tmp_size; // store in accumlate form
        if(sample_count >= reduced_size){
          break;
        } 
      }
      if(sample_count==0){
        sample_count = d_sample_size;
        d_buffer_info.clear();
      } 
      samples_left = d_sample_size - sample_count;
      samples_rm = sample_count;

      tags_offset_handler(samples_rm);
      
      for(int i=0;i<samples_left;++i){
        d_sample_buffer[i] = d_sample_buffer[samples_rm+i];
      }
      d_sample_idx = (d_sample_idx <samples_rm) ? 0 : (d_sample_idx - samples_rm);
      d_last_retx_idx = (d_last_retx_idx < samples_rm) ? 0 : (d_last_retx_idx - samples_rm);
      d_sample_size = samples_left;
    }

    void
    prou_sample_queue_cc_impl::tags_offset_handler(int offset)
    {
      int idx;
      long int time;
      if(d_buffer_info.empty()){
        return;
      }
      idx = pmt::to_long(pmt::dict_ref(d_buffer_info[0], pmt::intern("buffer_index"),pmt::PMT_NIL));
      while(idx < offset){
        d_buffer_info.erase(d_buffer_info.begin());
        if(d_buffer_info.empty()){
          break;
        }
        else
        {
          idx = pmt::to_long(pmt::dict_ref(d_buffer_info[0], pmt::intern("buffer_index"),pmt::PMT_NIL));
          time = pmt::to_long(pmt::dict_ref(d_buffer_info[0], pmt::intern("ctime"), pmt::PMT_NIL));
        }
      }
      if(d_buffer_info.empty()){
        d_update_time=0;
        return;
      }
      for(int i=0;i<d_buffer_info.size();++i){
        int tmp_idx = pmt::to_long(pmt::dict_ref(d_buffer_info[i], pmt::intern("buffer_index"),pmt::PMT_NIL));
        d_buffer_info[i] = pmt::dict_delete(d_buffer_info[i], pmt::intern("buffer_index"));
        d_buffer_info[i] = pmt::dict_add(d_buffer_info[i], pmt::intern("buffer_index"), pmt::from_long(tmp_idx- idx));
      }
      int count=0;
      for(int i=0;i<d_pkt_info.size();++i){
        int tmp_time = pmt::to_long(pmt::dict_ref(d_pkt_info[i],pmt::intern("ctime"),pmt::PMT_NIL));
        int tmp_idx = pmt::to_long(pmt::dict_ref(d_pkt_info[i],pmt::intern("header_found"),pmt::PMT_NIL));
        if(tmp_time<time){
          count++;
        }
        d_pkt_info[i] = pmt::dict_delete(d_pkt_info[i],pmt::intern("header_found"));
        d_pkt_info[i] = pmt::dict_add(d_pkt_info[i], pmt::intern("header_found"), pmt::from_long(tmp_idx - idx));
      }
      d_pkt_info.erase(d_pkt_info.begin(),d_pkt_info.begin()+count);
    }

    void
    prou_sample_queue_cc_impl::out_items_handler(
      gr_complex* out, 
      gr_complex* sample_out,
      const gr_complex* in, 
      int noutput_items, 
      int ninput_items)
    {
      int count =0;
      int output_size;
      int end_idx;
      switch(d_state)
      {
        case LOAD_SAMPLE:
          add_item_tag(0,nitems_written(0),pmt::intern("ctime"),pmt::from_long(d_current_time));
          memcpy(out,in,sizeof(gr_complex)*ninput_items);
          produce(0,ninput_items);
          produce(1,0);
        break;
        case WAIT_INFO:
          add_item_tag(0,nitems_written(0),pmt::intern("ctime"),pmt::from_long(d_current_time));
          memcpy(out,in,sizeof(gr_complex)*ninput_items);
          produce(0,ninput_items);
          produce(1,0);
        break;
        case OUTPUT_SAMPLE:
          output_size = d_end_retx_sample_size - d_sample_idx;
          end_idx = (noutput_items > (output_size))? output_size : noutput_items; 
          end_idx += d_sample_idx;

            //last block of retransmission
            for(d_sample_idx;d_sample_idx<end_idx;++d_sample_idx){
              if(!d_pkt_info.empty()){
                size_t offset = pmt::to_long(pmt::dict_ref(d_pkt_info[0],pmt::intern("header_found"),pmt::PMT_NIL));
                if(offset == d_sample_idx){
                  pmt::pmt_t tmp_info =  d_pkt_info[0];
                  tmp_info = pmt::dict_delete(tmp_info,pmt::intern("ctime"));
                  tmp_info = pmt::dict_delete(tmp_info,pmt::intern("buffer_index"));
                  pmt::pmt_t dict_items(pmt::dict_items(tmp_info));
                  while(!pmt::is_null(dict_items)){
                    pmt::pmt_t this_dict(pmt::car(dict_items));
                    add_item_tag(1,nitems_written(1)+count,pmt::car(this_dict),pmt::cdr(this_dict));
                    dict_items = pmt::cdr(dict_items);
                  }
                  d_pkt_info.erase(d_pkt_info.begin());
                }
              }
              if(d_sample_idx == 0){
                add_item_tag(1,nitems_written(1)+count, pmt::intern("begin_of_retx"), pmt::from_long(d_end_retx_sample_size));
              }
              if(d_sample_idx == d_end_retx_sample_size-1){
                add_item_tag(1,nitems_written(1)+count,pmt::intern("end_of_retx"),pmt::from_long(d_end_retx_sample_size));
              }
              out[count++] = d_sample_buffer[d_sample_idx];
            }
            produce(1,count);
            produce(0,0);
          
          if(d_sample_idx == d_end_retx_sample_size){
            //FIXME
            int tmp_time = std::clock();
            for(int i=0;i<d_buffer_info.size();++i){
              tmp_time = pmt::to_long(pmt::dict_ref(d_buffer_info[i],pmt::intern("ctime"),pmt::PMT_NIL));
              if(tmp_time>d_last_time){
                d_last_time = tmp_time;
                break;
              }
            }

            clear_queue_index_fix(d_last_time);
            //handle the samples, times, indexes, tags, of the samples left in buffer
            d_state = LOAD_SAMPLE;
            d_retx_count =0;
            d_retx_status.clear();
            d_end_retx_sample_size =0;
          }
        break;
        default:
          throw std::runtime_error("Entering wrong state");
        break;
      }
    }

    void
    prou_sample_queue_cc_impl::clear_queue_index_fix(long int time)
    {
      
      int tmp_index=0; 
      long int tmp_time;
      int count = 0;
      for(int i=0;i<d_buffer_info.size();++i)
      {
        tmp_time = pmt::to_long(pmt::dict_ref(d_buffer_info[i],pmt::intern("ctime"),pmt::PMT_NIL));
        if(tmp_time < time){
          count++;
        }
      }
      if(count!=0){
        d_buffer_info.erase(d_buffer_info.begin(),d_buffer_info.begin()+count);
        d_update_time = time;
      }
      count =0;
      for(int i=0;i<d_pkt_info.size();++i)
      {
        tmp_time = pmt::to_long(pmt::dict_ref(d_pkt_info[i], pmt::intern("ctime"),pmt::PMT_NIL));
        if(tmp_time < time){
          count++;
        }
      }
      if(count!=0){
        d_pkt_info.erase(d_pkt_info.begin(),d_pkt_info.begin()+count);
      }
      int sample_fixed= 0;
      if(!d_buffer_info.empty()){
        tmp_index = pmt::to_long(pmt::dict_ref(d_buffer_info[0],pmt::intern("buffer_index"),pmt::PMT_NIL));
        sample_fixed = d_sample_size - tmp_index;
        if(sample_fixed < 0){
          throw std::runtime_error("clear_queue_index_fix: remain samples cannot be negative number!");
        }
        for(int i=0;i<sample_fixed;++i){
            d_sample_buffer[i] = d_sample_buffer[tmp_index+i];
          }
        for(int i=0;i<d_buffer_info.size();i++){
          int buf_idx = pmt::to_long(pmt::dict_ref(d_buffer_info[i], pmt::intern("buffer_index"),pmt::PMT_NIL));
          d_buffer_info[i] = pmt::dict_delete(d_buffer_info[i], pmt::intern("buffer_index"));
          d_buffer_info[i] = pmt::dict_add(d_buffer_info[i], pmt::intern("buffer_index"), pmt::from_long(buf_idx-tmp_index));
        }
        for(int i=0;i<d_pkt_info.size();i++){
          int hdr_idx = pmt::to_long(pmt::dict_ref(d_pkt_info[i], pmt::intern("header_found"), pmt::PMT_NIL));
          d_pkt_info[i] = pmt::dict_delete(d_pkt_info[i], pmt::intern("header_found"));
          d_pkt_info[i] = pmt::dict_add(d_pkt_info[i], pmt::intern("header_found"), pmt::from_long(hdr_idx-tmp_index));
        }
      }
          
          d_sample_idx = 0;
          d_sample_size = sample_fixed;
          
    }

    void
    prou_sample_queue_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      int items_reqd=0;
      switch(d_state)
      {
        case LOAD_SAMPLE:
          items_reqd = d_sample_cap - d_sample_size;
          items_reqd = (items_reqd > noutput_items) ? noutput_items : items_reqd;
        break;
        case WAIT_INFO:
          items_reqd = noutput_items;
        break;
        case OUTPUT_SAMPLE:
          //FIXME
          items_reqd = 0;
        break;
        default:
          std::runtime_error("Entering wrong state");
        break;
      }
      ninput_items_required[0] = items_reqd;
    }

    int
    prou_sample_queue_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
      gr_complex *sample = (gr_complex *) output_items[1];

      // Do <+signal processing+>
      d_current_time = std::clock();
      int consume_count =0;

      append_samples(in,ninput_items[0],noutput_items,consume_count,d_current_time);
      // Tell runtime system how many input items we consumed on
      // each input stream.
      out_items_handler(out,sample,in,noutput_items, consume_count);
      consume_each(consume_count);
      // Tell runtime system how many output items we produced.
      return WORK_CALLED_PRODUCE;
    }

  } /* namespace lsa */
} /* namespace gr */

