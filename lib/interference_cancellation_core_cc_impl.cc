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
#include "interference_cancellation_core_cc_impl.h"

namespace gr {
  namespace lsa {

#define DEBUG d_debug && std::cout

    static const pmt::pmt_t d_ring_tag= pmt::intern("VoE_detected");
    static const pmt::pmt_t d_block_tag= pmt::intern("block_tag");
    static const int MAXLEN = (127+6)*8*8/2*4;
    static const int MEMCAP = MAXLEN*128;
    static const uint64_t MAXBASE = 16777216;

    interference_cancellation_core_cc::sptr
    interference_cancellation_core_cc::make(int sps,bool debug)
    {
      return gnuradio::get_initial_sptr
        (new interference_cancellation_core_cc_impl(sps,debug));
    }

    /*
     * The private constructor
     */
    interference_cancellation_core_cc_impl::interference_cancellation_core_cc_impl(int sps,bool debug)
      : gr::block("interference_cancellation_core_cc",
              gr::io_signature::make3(3, 3, sizeof(gr_complex),sizeof(float),sizeof(float)),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
              d_mem_cap(MEMCAP)
    {
      d_in_mem = new gr_complex[d_mem_cap];
      d_out_mem= new gr_complex[d_mem_cap];
      d_phase_mem=new float[d_mem_cap];
      if(sps<0){
        throw std::invalid_argument("Sps cannot be negative");
      }
      d_sps = sps;
      d_debug = debug;
      d_in_mem_idx =0;
      d_in_mem_size=0;
      d_phase_idx =0;
      d_phase_size=0;
      d_in_tlist.clear();
      d_out_tlist.clear();
    }

    /*
     * Our virtual destructor.
     */
    interference_cancellation_core_cc_impl::~interference_cancellation_core_cc_impl()
    {
      delete [] d_in_mem;
      delete [] d_out_mem;
      delete [] d_phase_mem;
    }

    bool
    interference_cancellation_core_cc_impl::tag_check()
    {
      DEBUG<<"<IC Core DEBUG>tag_check----tagObjects(raw):"<<d_in_tlist.size()<<std::endl;
      // before interference cancellation, checking all tags validity 
      // information to be checked:
      // 1. base counter (<MAXBASE)
      // 2. stream matching (block_id, block_offset)
      // 3. valid header (queue_index<queue_size)
      int qidx = -1, qsize=-1, block_offset = -1, pld = -1;
      uint64_t base=0, block_id = 0;
      std::map<uint64_t,int>::iterator map_it;
      std::list<tagObject_t>::iterator it;
      it = d_in_tlist.begin();
      while(it!=d_in_tlist.end()){
        // preparing available tags
        pmt::pmt_t dict = it->msg();
        if(pmt::dict_has_key(dict,pmt::intern("base"))){
          base = pmt::to_uint64(pmt::dict_ref(dict,pmt::intern("base"),pmt::from_uint64(MAXBASE)));
        }
        if(pmt::dict_has_key(dict,pmt::intern("queue_index"))){
          qidx = pmt::to_long(pmt::dict_ref(dict,pmt::intern("queue_index"),pmt::from_long(-1)));
        }
        if(pmt::dict_has_key(dict,pmt::intern("queue_size"))){
          qsize = pmt::to_long(pmt::dict_ref(dict,pmt::intern("queue_size"),pmt::from_long(-1)));
        }
        if(pmt::dict_has_key(dict,pmt::intern("block_id"))){
          block_id = pmt::to_uint64(pmt::dict_ref(dict,pmt::intern("block_id"),pmt::from_uint64(MAXBASE)));
        }
        if(pmt::dict_has_key(dict,pmt::intern("block_offset"))){
          block_offset = pmt::to_long(pmt::dict_ref(dict,pmt::intern("block_offset"),pmt::from_long(-1)));
        }
        if(pmt::dict_has_key(dict,pmt::intern("payload"))){
          // turn into sample based
          pld = d_sps * pmt::to_long(pmt::dict_ref(dict,pmt::intern("payload"),pmt::from_long(-1)));
        }
        // first step: checking tags' validity
        if(qidx<0 || qsize<0 || block_id == MAXBASE || block_offset<0 || pld<0){
          //useless tagobject
          it = d_in_tlist.erase(it);
          continue;
        }else if((qsize<=qidx) || (base> MAXBASE) ){
          it = d_in_tlist.erase(it);
          continue;
        }
        // second step: matching streams
        map_it = d_samp_map.find(block_id);
        if(map_it == d_samp_map.end()){
          // no matching stream tags
          it= d_in_tlist.erase(it);
          continue;
        }else{
          // matched stream tags
          it->delete_msg(pmt::intern("block_id"));
          it->delete_msg(pmt::intern("block_offset"));
          it->delete_msg(pmt::intern("LSA_hdr"));
          it->set_idx(map_it->second+block_offset);
          //it->add_msg(pmt::intern("sample_index"),pmt::from_long(map_it->second+block_offset));
        }
        it++;
      }
      for(it = d_in_tlist.begin();it!= d_in_tlist.end();++it){
        DEBUG<<"<IC Core DEBUG>tagObject---"<<*it<<std::endl;
      }
      DEBUG<<"<IC Core DEBUG>tag_check----tagObjects(Refined):"<<d_in_tlist.size()<<std::endl;
      DEBUG<<"------------------------------------------------"<<std::endl;
      // FIXME
      // current version only check tags
      return false;
    }

    void
    interference_cancellation_core_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      for(int i=0;i<ninput_items_required.size();++i){
        ninput_items_required[i] = noutput_items;
      }
    }

    int
    interference_cancellation_core_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      const float* phase_in= (const float*) input_items[1];
      const float* time_in = (const float*) input_items[2];
      gr_complex *out = (gr_complex*) output_items[0];
      int nin_s = std::min(ninput_items[0],(d_mem_cap-d_in_mem_size) );
      int nin_p = std::min(ninput_items[1],std::min(ninput_items[2],(d_mem_cap-d_phase_size)));
      int nout = 0;
      bool reset_s = false;
      bool reset_p = false;
      bool reset_ready = false;
      std::vector<tag_t> tags_s;
      std::vector<tag_t> tags_p;
      std::vector<tag_t> hdr_tag;
      
      std::vector<tag_t> block_s;
      std::vector<tag_t> block_p;
      
      get_tags_in_range(tags_s,0,nitems_read(0),nitems_read(0)+nin_s, d_ring_tag);
      get_tags_in_range(tags_p,1,nitems_read(1),nitems_read(1)+nin_p, d_ring_tag);
      if(!tags_s.empty()){
        int offset = tags_s[0].offset - nitems_read(0);
        if(offset < nin_s){
          nin_s = offset;
          reset_s = true;
        }
      }
      if(!tags_p.empty()){
        int offset = tags_p[0].offset - nitems_read(1);
        if(offset < nin_p){
          nin_p = offset;
          reset_p = true;
        }
      }
      reset_ready = (reset_s && reset_p) || (d_in_mem_size==d_mem_cap && d_phase_size==d_mem_cap);

      // parsing stream 0 tags
      std::map<uint64_t,int32_t>::iterator samp_it;
      get_tags_in_range(block_s,0,nitems_read(0),nitems_read(0)+nin_s,d_block_tag);
      for(int i=0;i<block_s.size();++i){
        d_samp_block_no = pmt::to_uint64(block_s[i].value);
        int offset = block_s[i].offset - nitems_read(0);
        // checking block id and insert
        samp_it = d_samp_map.find(d_samp_block_no);
        if(samp_it == d_samp_map.end()){
          d_samp_map.insert(std::pair<uint64_t,int>(d_samp_block_no,d_in_mem_size+offset) );
        }
      }
      
      // parsing stream 1 tags
      tagObject_t tmp_obj;
      get_tags_in_range(hdr_tag,1,nitems_read(1),nitems_read(1)+nin_p);
      while(!hdr_tag.empty()){
        int offset = hdr_tag[0].offset - nitems_read(1);
        // find block number and block offset in phase stream!!
        if(pmt::eqv(d_block_tag,hdr_tag[0].key)){
          d_sync_block_no = pmt::to_uint64(hdr_tag[0].value);
          d_sync_block_idx= offset + d_phase_size;
        }
        // exclude ring tag
        else if(!pmt::eqv(d_ring_tag,hdr_tag[0].key)){
          if(tmp_obj.index() == (offset+d_phase_size) ){
            // same objectblock_id
            tmp_obj.add_msg(hdr_tag[0].key,hdr_tag[0].value);
          }else{
            // new object
            if(!tmp_obj.empty()){
              // first store previous object
              //DEBUG<<"<IC Core>New tag object added---"<<tmp_obj<<std::endl;
              d_in_tlist.push_back(tmp_obj);
              tmp_obj.reset();
            }
            // create dictionary to store tags of new object
              tmp_obj.init_dict();
              tmp_obj.add_msg(pmt::intern("block_id"),pmt::from_uint64(d_sync_block_no));
              tmp_obj.add_msg(pmt::intern("block_offset"),pmt::from_long(d_phase_size+offset-d_sync_block_idx));
              tmp_obj.add_msg(hdr_tag[0].key,hdr_tag[0].value);
              tmp_obj.set_idx(offset+d_phase_size);
          }
        }
        hdr_tag.erase(hdr_tag.begin());
      }
      // clean up, in case there is one object
      if(!tmp_obj.empty()){
        //DEBUG<<"<IC Core>One tag object added---"<<tmp_obj<<std::endl;
        d_in_tlist.push_back(tmp_obj);
      }
      // copy the samples and sync values
      memcpy(d_in_mem+d_in_mem_size,in,sizeof(gr_complex)*nin_s);
      memcpy(d_phase_mem+d_phase_size,phase_in,sizeof(float)*nin_p);
      d_in_mem_size += nin_s;
      d_phase_size  += nin_p;
      // reset condition checking
      if(reset_ready){
        // end of a possible ic opportunities
        if(d_in_mem_size == d_mem_cap){
          DEBUG<<"<IC Core>signal buffer full..."<<std::endl;
        }else{
          DEBUG <<"<IC Core>Reset signal are all ready, clear queue... sig_size:"
          <<d_in_mem_size<<" ,phase_size:"<<d_phase_size<<std::endl;
          nin_s++;
          nin_p++;
        }
        // checking all tags
        if(tag_check()){
          // do interference cancellation
        }

        // handling output
        d_in_tlist.clear();
        d_in_mem_idx =0;
        d_in_mem_size=0;
        d_phase_idx =0;
        d_phase_size=0;
        d_samp_block_idx= 0;
        d_sync_block_idx= 0;
        d_samp_map.clear();
        d_sync_map.clear();
      }
      consume(0,nin_s);
      consume(1,nin_p);
      consume(2,nin_p);
      return nout;
    }

  } /* namespace lsa */
} /* namespace gr */

