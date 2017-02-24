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
    prou_sample_queue_cc::make(const std::string& sensing_tagname)
    {
      return gnuradio::get_initial_sptr
        (new prou_sample_queue_cc_impl(sensing_tagname));
    }

    /*
     * The private constructor
     */
    prou_sample_queue_cc_impl::prou_sample_queue_cc_impl(const std::string& sensing_tagname)
      : gr::block("prou_sample_queue_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(2, 2, sizeof(gr_complex))),
      d_sample_cap(1024*1024)
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
      d_sensing_tagname = pmt::string_to_symbol(sensing_tagname);
      
      //set_message_port_register_in(d_info_port);
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
      //pmt::pmt_t tmp;
      long int time;
      int counter=0, qidx, qsize, offset;
      bool sensing_state=false;
      pmt::pmt_t tag_info;
      if(pmt::dict_has_key(msg, d_sensing_tagname)){
        sensing_state = pmt::to_bool(pmt::dict_ref(msg, d_sensing_tagname, pmt::PMT_NIL));
      }
      if(pmt::dict_has_key(msg, pmt::intern("ctime")))
      {
        time = pmt::to_long(pmt::dict_ref(msg, pmt::intern("ctime"), pmt::PMT_NIL));
      }
      if(pmt::dict_has_key(msg, pmt::intern("queue_index")))
      {
        qidx = pmt::to_long(pmt::dict_ref(msg, pmt::intern("queue_index"), pmt::PMT_NIL));
      }
      if(pmt::dict_has_key(msg, pmt::intern("queue_size")))
      {
        qsize = pmt::to_long(pmt::dict_ref(msg, pmt::intern("queue_size"), pmt::PMT_NIL));
      }
      if(pmt::dict_has_key(msg, pmt::intern("index_offset")))
      {
        offset = pmt::to_long(pmt::dict_ref(msg, pmt::intern("index_offset"), pmt::PMT_NIL));
      }
      long int tmp_time;
      int tmp_idx;
      switch(d_state)
      {
        case LOAD_SAMPLE:
          if(sensing_state || qsize!=0 ){
            d_state = WAIT_INFO;
            d_retx_index_counter.resize(qsize);
            d_retx_status.resize(qsize,false);
            d_retx_count = 0;
          }
        break;
        case WAIT_INFO:
          //Job for prou queue:
          // 1. labeling the header info from downstream on sample index
          // 2. try to detect the availability of interference cancellation 
      
          // wait here until timeout
          if(time < d_update_time){
            //outdated info, skip
            return;
          }
          if(qsize != d_retx_index_counter.size()){
            GR_LOG_DEBUG(d_logger, "receiving queue size of different length");
          }
          if(d_retx_count != 0 && qsize == 0){
            GR_LOG_DEBUG(d_logger, "receiving new data, retransmission ends");
          }

          if(!sensing_state){
            for(int i=0;i<d_buffer_info.size();++i){
              tmp_time = pmt::to_long(pmt::dict_ref(d_buffer_info[i], pmt::intern("ctime"),pmt::PMT_NIL));
              tmp_idx = pmt::to_long(pmt::dict_ref(d_buffer_info[i], pmt::intern("qindex"),pmt::PMT_NIL));
              if(time == tmp_time){
                //found corresponding time index
                // attach header info on sample
                pmt::pmt_t info_tag = msg;
                int info_index = tmp_idx + offset;
                info_tag = pmt::dict_add(info_tag, pmt::intern("index_offset"), pmt::from_long(info_index));
                if(qsize!=0 &&(qsize == d_retx_status.size()) && (d_retx_status[qidx] == false) ){
                  d_retx_status[qidx] = true;
                  d_retx_count++;
                  info_tag = pmt::dict_add(info_tag, pmt::intern("retx_idx"), pmt::from_long(qidx));
                }
                d_pkt_info.push_back(info_tag);
              }
            }
          }
          if(!d_retx_status.empty() && d_retx_count==d_retx_status.size()){
            //retransmission complete
            d_retx_count =0;
            d_retx_status.clear();
            d_state = OUTPUT_SAMPLE;
          }
        break;
        case OUTPUT_SAMPLE:
          //in this state, samples are not passed to downstream. discard message?
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
      int space_left = d_sample_cap - d_sample_size;
      std::vector<tag_t> tags;
      get_tags_in_range(tags, 0, nitems_read(0),nitems_read(0)+noutput_items, d_sensing_tagname);
      pmt::pmt_t mark_sample = pmt::make_dict();
      switch(d_state)
      {
        case LOAD_SAMPLE:
          if(d_sample_size == d_sample_cap){
            reduce_samples();
          }
          if(ninput_items <= space_left){
            memcpy(d_sample_buffer+d_sample_size, in , sizeof(gr_complex)*ninput_items);
            consume_count = ninput_items;
          }
          else{
            memcpy(d_sample_buffer+d_sample_size, in, sizeof(gr_complex)*space_left);
            consume_count = space_left;
          }
            mark_sample = pmt::dict_add(mark_sample, pmt::intern("ctime"), pmt::from_long(time));
            mark_sample = pmt::dict_add(mark_sample, pmt::intern("qindex"), pmt::from_long(d_sample_size));
            d_sample_size+= consume_count;
            d_buffer_info.push_back(mark_sample);
        break;
        case WAIT_INFO:
          // if timeout --> failed
          if(((time-d_update_time)/CLOCKS_PER_SEC)>=d_timeout){
            d_sample_idx = 0;
            d_sample_size = 0;
            d_state = LOAD_SAMPLE;
          }
          // clear queue
          // back to LOAD_SAMPLE;
          if(ninput_items <= space_left){
            memcpy(d_sample_buffer+d_sample_size, in , sizeof(gr_complex)*ninput_items);
            consume_count = ninput_items;
          }
          else{
            memcpy(d_sample_buffer+d_sample_size, in, sizeof(gr_complex)*space_left);
            consume_count = space_left;
          }
            mark_sample = pmt::dict_add(mark_sample, pmt::intern("ctime"), pmt::from_long(time));
            mark_sample = pmt::dict_add(mark_sample, pmt::intern("qindex"), pmt::from_long(d_sample_size));
            d_sample_size+= consume_count;
            consume_count = ninput_items;
            d_buffer_info.push_back(mark_sample);
        break;
        case OUTPUT_SAMPLE:
          consume_count = 0;
        break;
        default:
          std::runtime_error("Entering wrong state");
        break;
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
      while(sample_count < d_sample_size/2){
        d_buffer_info.erase(d_buffer_info.begin());
        if(!d_buffer_info.empty()){
          sample_count = pmt::to_long(pmt::dict_ref(d_buffer_info[0],pmt::intern("qindex"),pmt::PMT_NIL));
        }
        else{
          //this case means no info found and sample count cannot be updated
          sample_count = d_sample_size; //remove all
          d_buffer_info.clear();
        }
      }
      samples_left = d_sample_size - sample_count;
      samples_rm = sample_count;
      for(int i=0;i<d_buffer_info.size();++i){
        pmt::pmt_t tmp = pmt::make_dict();
        idx = pmt::to_long(pmt::dict_ref(d_buffer_info[i], pmt::intern("qindex"),pmt::PMT_NIL));
        time = pmt::to_uint64(pmt::dict_ref(d_buffer_info[i], pmt::intern("ctime"),pmt::PMT_NIL));
        tmp = pmt::dict_add(tmp, pmt::intern("ctime"), pmt::from_long(time));
        tmp = pmt::dict_add(tmp, pmt::intern("qindex"), pmt::from_long(idx - samples_rm));
        d_buffer_info[i] = tmp;
        if(i==0){
          d_update_time = time;
        }
      }
      for(int i=0;i<samples_left;++i){
        d_sample_buffer[i] = d_sample_buffer[samples_rm+i];
      }
      d_sample_idx = (d_sample_idx <samples_rm) ? 0 : (d_sample_idx - samples_rm);
      d_sample_size = samples_left;
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
      switch(d_state)
      {
        case LOAD_SAMPLE:
          memcpy(out,in,sizeof(gr_complex)*ninput_items);
          produce(0,ninput_items);
        break;
        case WAIT_INFO:
          memcpy(out,in,sizeof(gr_complex)*ninput_items);
          produce(0,ninput_items);
        break;
        case OUTPUT_SAMPLE:
          if(noutput_items > (d_sample_size-d_sample_idx))
          {
            for(d_sample_idx;d_sample_idx<d_sample_size;++d_sample_idx){
              if(!d_pkt_info.empty()){
                size_t offset = pmt::to_long(pmt::dict_ref(d_pkt_info[0],pmt::intern("index_offset"),pmt::PMT_NIL));
                if(offset == d_sample_idx){
                  pmt::pmt_t dict_items(pmt::dict_items(d_pkt_info[0]));
                  while(!pmt::is_null(dict_items)){
                    pmt::pmt_t this_dict(pmt::car(dict_items));
                    add_item_tag(1,nitems_written(1)+count,pmt::car(this_dict),pmt::cdr(this_dict));
                    dict_items = pmt::cdr(dict_items);
                  }
                  d_pkt_info.erase(d_pkt_info.begin());
                }
              }
              out[count++] = d_sample_buffer[d_sample_idx];
            }
            produce(1,count);
          }
          else{
            for(count;count < noutput_items;++count){
              if(!d_pkt_info.empty()){
                size_t offset = pmt::to_long(pmt::dict_ref(d_pkt_info[0],pmt::intern("index_offset"),pmt::PMT_NIL));
                if(offset == d_sample_idx){
                  pmt::pmt_t dict_items(pmt::dict_items(d_pkt_info[0]));
                  while(!pmt::is_null(dict_items)){
                    pmt::pmt_t this_dict(pmt::car(dict_items));
                    add_item_tag(1,nitems_written(1)+count,pmt::car(this_dict),pmt::cdr(this_dict));
                    dict_items = pmt::cdr(dict_items);
                  }
                  d_pkt_info.erase(d_pkt_info.begin());
                }
              }
              out[count] = d_sample_buffer[d_sample_idx + count];
            }
            produce(1,count);
            d_sample_idx += count;
          }
          if(d_sample_idx == d_sample_size){
            d_sample_idx = 0;
            d_sample_size= 0;
            d_state = LOAD_SAMPLE;
          }
        break;
        default:
          std::runtime_error("Entering wrong state");
        break;
      }
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
          if(noutput_items >= d_sample_size){
            items_reqd = noutput_items - d_sample_size;
          }
        break;
        default:
          std::runtime_error("Entering wrong state");
        break;
      }
      ninput_items_required[0] = items_reqd;
      //ninput_items_required[0] = noutput_items;
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
      //int process_samples = noutput_items;
      long int cur_time = std::clock();
      int consume_count =0;

      append_samples(in,ninput_items[0],noutput_items,consume_count,cur_time);
      // Tell runtime system how many input items we consumed on
      // each input stream.
      out_items_handler(out,sample,in,noutput_items, consume_count);

      // Tell runtime system how many output items we produced.
      return WORK_CALLED_PRODUCE;
    }

  } /* namespace lsa */
} /* namespace gr */

