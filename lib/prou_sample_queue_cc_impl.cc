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
      d_sample_cap(128*1024)
    {
      d_sample_idx = 0;
      d_sample_size =0;
      d_sample_buffer = new gr_complex[d_sample_cap];

      d_info_port = pmt::mp("info");
      message_port_register_in(d_info_port);
      set_msg_handler(d_info_port, boost::bind(&prou_sample_queue_cc_impl::info_msg_handler, this, _1));

      d_update_time = std::clock();
      d_timeout = 5.0;
      d_last_time = d_update_time;
      d_current_time = d_update_time;
      //d_retx_count = 0;
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
                d_pkt_info.push_back(info_tag);
                break;
                }
              }
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
      pmt::pmt_t mark_sample = pmt::make_dict();
      if(ninput_items <= space_left){
            memcpy(d_sample_buffer+d_sample_size, in , sizeof(gr_complex)*ninput_items);
            consume_count = ninput_items;
          }
          else{
            memcpy(d_sample_buffer+d_sample_size, in, sizeof(gr_complex)*space_left);
            consume_count = space_left;
          }
          if( (consume_count !=0) ){
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
    prou_sample_queue_cc_impl::out_items_handler(
      gr_complex* out, 
      gr_complex* sample_out,
      const gr_complex* in, 
      int noutput_items, 
      int ninput_items)
    {
      int count =0;
      int output_size;
      int end_idx ,hdr_idx;
      int pkt_count;
      int tmp_pld; 
      long int tmp_time, hdr_time;
      if(ninput_items>0){
        memcpy(out,in, sizeof(gr_complex)*ninput_items);
        add_item_tag(0,nitems_written(0),pmt::intern("ctime"),pmt::from_long(d_current_time));
        produce(0,ninput_items);
      }
      
        end_idx = 0;
        //pkt_count = 0;
        while(pkt_count<d_pkt_info.size()){
          hdr_idx = pmt::to_long(pmt::dict_ref(d_pkt_info[pkt_count],pmt::intern("header_found"),pmt::PMT_NIL));
          tmp_pld = pmt::to_long(pmt::dict_ref(d_pkt_info[pkt_count],pmt::intern("payload"),pmt::PMT_NIL));
          end_idx = hdr_idx + tmp_pld-1;
          if(hdr_idx >= (d_sample_idx + output_size)){
            break;
          }
          //pkt_count++;
        }
        output_size = d_sample_size - d_sample_idx;
        output_size = (output_size > noutput_items) ? noutput_items : output_size;
        //if(d_debug){
          //std::cout<<"<debug>output:"<<output_size<<" ,d_sample_idx:"<<d_sample_idx<<" ,d_sample_size"
          //<<d_sample_size<<" ,end_idx:"<<end_idx<<std::endl;
        //}
        if(end_idx==0){
          produce(1,0);
          return;
        }

        while(!d_pkt_info.empty()){
          hdr_idx = pmt::to_long(pmt::dict_ref(d_pkt_info[0],pmt::intern("header_found"),pmt::PMT_NIL));
          if(hdr_idx >= (d_sample_idx + output_size)){
            break;
          }          
          d_last_time = pmt::to_long(pmt::dict_ref(d_pkt_info[0],pmt::intern("ctime"),pmt::PMT_NIL));
          d_pkt_info[0] = pmt::dict_delete(d_pkt_info[0],pmt::intern("ctime"));
          pmt::pmt_t dict = pmt::dict_items(d_pkt_info[0]);
          while(!pmt::is_null(dict)){
            pmt::pmt_t tmp_pair = pmt::car(dict);
            add_item_tag(1,nitems_written(1)+ hdr_idx-d_sample_idx,pmt::car(tmp_pair),pmt::cdr(tmp_pair));
            dict = pmt::cdr(dict);
          }
          d_pkt_info.erase(d_pkt_info.begin());
        }
        produce(1,output_size);
        memcpy(sample_out, d_sample_buffer+d_sample_idx, sizeof(gr_complex)*output_size);
        
        d_sample_idx+= output_size;
    }

    void
    prou_sample_queue_cc_impl::update_sample_buffer()
    {
      if(d_buffer_info.empty()){
          throw std::runtime_error("Cannot have empty buffer info but half of sample capacity");
        }
        // update buffer info according to time.
        // you cannot guarantee every packet header can be found!
        long int tmp_time;
        int tmp_idx, hdr_idx;
        int rm_count=0;
        while(!d_buffer_info.empty()){
          tmp_idx = pmt::to_long(pmt::dict_ref(d_buffer_info[rm_count],pmt::intern("buffer_index"),pmt::PMT_NIL));
          //4096 for default output buffer size
          if(tmp_idx +4096 >= d_sample_idx){
            break;
          }
          rm_count++;
        }
        if(rm_count==0)
          return;
        d_buffer_info.erase(d_buffer_info.begin(),d_buffer_info.begin()+rm_count);
        tmp_idx = pmt::to_long(pmt::dict_ref(d_buffer_info[0],pmt::intern("buffer_index"),pmt::PMT_NIL));
        for(int i=0;i<d_buffer_info.size();++i){
          int offset = pmt::to_long(pmt::dict_ref(d_buffer_info[i],pmt::intern("buffer_index"),pmt::PMT_NIL));
          d_buffer_info[i] = pmt::dict_delete(d_buffer_info[i],pmt::intern("buffer_index"));
          d_buffer_info[i] = pmt::dict_add(d_buffer_info[i],pmt::intern("buffer_index"),pmt::from_long(offset-tmp_idx));
        }
        std::vector<pmt::pmt_t> new_pkt_info;
        for(int i=0;i<d_pkt_info.size();++i){
          int offset = pmt::to_long(pmt::dict_ref(d_pkt_info[i],pmt::intern("header_found"),pmt::PMT_NIL));
          if(offset - tmp_idx >=0){
            d_pkt_info[i] = pmt::dict_delete(d_pkt_info[i],pmt::intern("header_found"));
            d_pkt_info[i] = pmt::dict_add(d_pkt_info[i],pmt::intern("header_found"),pmt::from_long(offset-tmp_idx));
            new_pkt_info.push_back(d_pkt_info[i]);
          }
        }
        d_update_time = pmt::to_long(pmt::dict_ref(d_buffer_info[0],pmt::intern("ctime"),pmt::PMT_NIL));
        d_pkt_info = new_pkt_info;
        memcpy(d_sample_buffer, d_sample_buffer+tmp_idx, sizeof(gr_complex)* (d_sample_size-tmp_idx));
        if(d_sample_idx-tmp_idx<0){
          std::cout<<"ERROR:"<<std::endl;
          std::cout<<"sample index:"<<d_sample_idx<<" ,sample_size:"<<d_sample_size<<" ,tmp_idx:"<<tmp_idx<<" ,rm_count:"<<rm_count<<std::endl;
          throw std::runtime_error("sample index cannot be negative");
        }
        //std::cout<<"sample index:"<<d_sample_idx<<" ,sample_size:"<<d_sample_size<<" ,tmp_idx:"<<tmp_idx<<" ,rm_count:"<<rm_count<<std::endl;
        //std::cout<<"buffer_info:"<<d_buffer_info[0]<<std::endl;
        //std::cout<<"pkt_info:"<<d_pkt_info[0]<<std::endl;
        d_sample_idx -= tmp_idx;//bug here
        d_sample_size-= tmp_idx;
        
    }

    void
    prou_sample_queue_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      int items_reqd=0;
      int space_left = d_sample_cap-d_sample_size;
      items_reqd = (space_left >noutput_items) ?noutput_items : space_left;
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
      
      d_current_time = std::clock();
      int consume_count =0;

      if( 2*d_sample_size >= d_sample_cap){
        if(d_debug){
          std::cout<<"before update:"<<std::endl;
          std::cout<<"buffer size:"<<d_buffer_info.size()<<" ,pkt_hdr_size:"<<d_pkt_info.size()<<std::endl;
          std::cout<<"sample_size:"<<d_sample_size<<" ,sample_idx:"<<d_sample_idx<<std::endl;
          std::cout<<"current time:"<<d_current_time<<" ,update_time:"<<d_update_time<<" ,last_time:"<<d_last_time<<std::endl;
        }
        update_sample_buffer();
        if(d_debug){
          std::cout<<"after update:"<<std::endl;
          std::cout<<"buffer size:"<<d_buffer_info.size()<<" ,pkt_hdr_size:"<<d_pkt_info.size()<<std::endl;
          std::cout<<"sample_size:"<<d_sample_size<<" ,sample_idx:"<<d_sample_idx<<std::endl;
          std::cout<<"current time:"<<d_current_time<<" ,update_time:"<<d_update_time<<" ,last_time:"<<d_last_time<<std::endl;
          std::cout<<"****************************************************************************************"<<std::endl;
        }
      }
      append_samples(in,ninput_items[0],noutput_items,consume_count,d_current_time);
      out_items_handler(out,sample,in,noutput_items, consume_count);
      consume_each(consume_count);

      return WORK_CALLED_PRODUCE;
    }

  } /* namespace lsa */
} /* namespace gr */

