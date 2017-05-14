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
#include <pmt/pmt.h>
#include <ctime>

namespace gr {
  namespace lsa {

    prou_sample_queue_cc::sptr
    prou_sample_queue_cc::make(const std::string& sensing_tagname,int bps, int sps,bool debug)
    {
      return gnuradio::get_initial_sptr
        (new prou_sample_queue_cc_impl(sensing_tagname,bps,sps,debug));
    }

    /*
     * The private constructor
     */

    prou_sample_queue_cc_impl::prou_sample_queue_cc_impl(
      const std::string& sensing_tagname, 
      int bps,
      int sps,
      bool debug)
      : gr::block("prou_sample_queue_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make2(2, 2, sizeof(gr_complex),sizeof(gr_complex))),
      d_sample_cap(2048*2048),
      d_min_output_items(4096)
    {
      d_sample_idx = 0;
      d_sample_size =0;
      d_sample_buffer = new gr_complex[d_sample_cap];

      d_info_port = pmt::mp("info");
      message_port_register_in(d_info_port);
      set_msg_handler(d_info_port, boost::bind(&prou_sample_queue_cc_impl::info_msg_handler, this, _1));

      if(sps<0){
        throw std::invalid_argument("Sample per symbol cannot be negative");
      }
      d_sps = sps;
      if(bps<0){
        throw std::invalid_argument("Bits per symbol cannot be negative");
      }
      d_bps = bps;
      d_output_count = d_min_output_items;

      d_update_time = std::clock();
      d_current_time = d_update_time;
      d_sensing_tagname = pmt::string_to_symbol(sensing_tagname);
      
      d_debug = debug;

      set_tag_propagation_policy(TPP_DONT);
      set_min_noutput_items(512);
      set_output_multiple(d_sps);
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
      int counter=0, qidx, qsize, offset=0;
      bool sensing_state=false;
      bool hdr = false;
      int output_len = 0;

      if(pmt::dict_has_key(msg, pmt::intern("ctime")))
      {
        time = pmt::to_long(pmt::dict_ref(msg, pmt::intern("ctime"), pmt::PMT_NIL));
      }
      if(time < d_update_time){
            //outdated info, skip
            return;
      }
      hdr = pmt::to_bool(pmt::dict_ref(msg,pmt::intern("LSA_hdr"),pmt::PMT_F));
      if(hdr){
        // not sync time info
        return;
      }
      long int tmp_time;
      int tmp_idx;
      
      for(int i=0;i<d_buffer_info.size();++i){
        tmp_time = pmt::to_long(pmt::dict_ref(d_buffer_info[i], pmt::intern("ctime"),pmt::PMT_NIL));
        tmp_idx = pmt::to_long(pmt::dict_ref(d_buffer_info[i], pmt::intern("buffer_index"),pmt::PMT_NIL));
        if(time == tmp_time){
        //found corresponding time index
        // attach header info on sample
        pmt::pmt_t info_tag = pmt::make_dict();

          info_tag = pmt::dict_add(info_tag, pmt::intern("buffer_index"),pmt::from_long(tmp_idx));
          info_tag = pmt::dict_add(info_tag, pmt::intern("ctime"),pmt::from_long(time));
          d_sync_info.push_back(info_tag);

        break;
        }
      }
            
    }


    void
    prou_sample_queue_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      int items_reqd=0;
      int space_left = d_sample_cap-d_sample_size;
      items_reqd = (space_left >noutput_items) ?noutput_items : space_left;

      for(int i=0;i<ninput_items_required.size();++i)
        ninput_items_required[i] = items_reqd;
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

      std::vector<tag_t> tags;
      
      int nin = d_sample_cap - d_sample_size;
      nin = (nin<ninput_items[0])? nin : ninput_items[0];
      nin = (nin<noutput_items)? nin:noutput_items;

      get_tags_in_range(tags,0,nitems_read(0),nitems_read(0)+nin);
      for(int i=0;i<tags.size();++i){
        int offset = tags[i].offset - nitems_read(0);
        add_item_tag(0,nitems_written(0)+offset,tags[i].key,tags[i].value);
      }
      if((d_output_count/d_min_output_items) !=0){
        add_item_tag(0,nitems_written(0),pmt::intern("ctime"),pmt::from_long(d_current_time));
        pmt::pmt_t dict = pmt::make_dict();
        dict = pmt::dict_add(dict,pmt::intern("ctime"),pmt::from_long(d_current_time));
        dict = pmt::dict_add(dict,pmt::intern("buffer_index"),pmt::from_long(d_sample_size));
        d_buffer_info.push_back(dict);
        d_output_count-= d_min_output_items;
      }
      d_output_count+=nin;
      memcpy(out,in,sizeof(gr_complex)*nin);
      consume(0,nin);
      produce(0,nin);
      
      if( d_sample_size == d_sample_cap){
        // update
        if(d_debug){
          std::cerr<<"<ProU Queue>reaching capacity, reduce size to half!"<<std::endl;
        }
        half_queue_cap();
      }
      //have space 
      memcpy(d_sample_buffer+d_sample_size,in,sizeof(gr_complex)*nin);
      d_sample_size+=nin;
      std::vector<pmt::pmt_t> p2_tags;
      int nout = (noutput_items< (d_sample_size-d_sample_idx ))?noutput_items: (d_sample_size-d_sample_idx);
      if(nout>0 && check_port1(nout,p2_tags)){
        //ready to send
        memcpy(sample,d_sample_buffer+d_sample_idx,sizeof(gr_complex)*nout);
        // tags and other information!!
        for(int i=0;i<p2_tags.size();++i){
          long int tmp_time = pmt::to_long(pmt::dict_ref(p2_tags[i],pmt::intern("ctime"),pmt::from_long(-1)));
          int offset = pmt::to_long(pmt::dict_ref(p2_tags[i],pmt::intern("buffer_index"),pmt::from_long(-1)));
          if(tmp_time<0 || offset <0){
            std::cerr<<"<ProU Queue>time:"<<tmp_time<<" ,offset:"<<offset<<std::endl;
            throw std::runtime_error("invalid tags info");
          }
          if(d_debug){
            std::cerr<<"<ProU Queue>adding time tag at:"<<nitems_written(1)+offset-d_sample_idx<<" ,time:"<<tmp_time<<std::endl;
          }
          add_item_tag(1,nitems_written(1)+offset-d_sample_idx,pmt::intern("ctime"),pmt::from_long(tmp_time));
        }
        d_sample_idx += nout;
      }
      else{
        nout =0;
      }
      produce(1,nout);
      return WORK_CALLED_PRODUCE;
      

    }

    bool
    prou_sample_queue_cc_impl::check_port1(int nout, std::vector<pmt::pmt_t>& tags)
    {
      long int tmp_time=0;
      int tmp_idx=0;
      if(d_sync_info.empty()){
        return false;
      }

      while(!d_sync_info.empty()){
        tmp_idx = pmt::to_long(pmt::dict_ref(d_sync_info[0],pmt::intern("buffer_index"),pmt::from_long(-1)));
        tmp_time= pmt::to_long(pmt::dict_ref(d_sync_info[0],pmt::intern("ctime"),pmt::from_long(-1)));
        if(tmp_idx<0 || tmp_time <0){
          throw std::runtime_error("failed at sync info");
        }
        if( (tmp_idx< (nout+d_sample_idx)) && (tmp_idx>= d_sample_idx) ){
          if(d_debug)
            std::cerr<<"<ProU Queue debug> adding time tag:"<<d_sync_info[0]<<std::endl;
          tags.push_back(d_sync_info[0]);
          d_sync_info.erase(d_sync_info.begin());
        }
        else if(tmp_idx>= (nout+d_sample_idx)){
          return true;
        }
      }
      
      return !tags.empty();
    }

    void
    prou_sample_queue_cc_impl::half_queue_cap()
    {
      int rm_idx = d_sample_size/2;
      int align_idx;
      if(d_sync_info.empty()){
        // all done
        d_sample_size =0;
      }
      else{
        int tmp_idx =0;
        while(!d_sync_info.empty()){
        tmp_idx = pmt::to_long(pmt::dict_ref(d_sync_info[0],pmt::intern("buffer_index"),pmt::from_long(-1)));
        if(tmp_idx>=rm_idx){
          align_idx = tmp_idx;
          break;
        }
        d_sync_info.erase(d_sync_info.begin());
        }
        memcpy(d_sample_buffer,d_sample_buffer+tmp_idx,sizeof(gr_complex)*(d_sample_size-tmp_idx));
        d_sample_size -= tmp_idx;
        for(int i=0;i<d_sync_info.size();++i){
          int fix_idx = pmt::to_long(pmt::dict_ref(d_sync_info[i],pmt::intern("buffer_index"),pmt::from_long(-1)));
          d_sync_info[i] = pmt::dict_delete(d_sync_info[i],pmt::intern("buffer_index"));
          d_sync_info[i] = pmt::dict_add(d_sync_info[i],pmt::intern("buffer_index"),pmt::from_long(fix_idx-tmp_idx));
        }
      }
      d_sample_idx = 0;
      d_update_time = std::clock();
    }

  } /* namespace lsa */
} /* namespace gr */

