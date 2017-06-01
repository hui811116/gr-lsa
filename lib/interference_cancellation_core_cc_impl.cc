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
#include <gnuradio/expj.h>
#include <gnuradio/math.h>

namespace gr {
  namespace lsa {

#define DEBUG d_debug && std::cout
#define TWO_PI M_PI*2.0f

    static const pmt::pmt_t d_ring_tag= pmt::intern("VoE_detected");
    static const pmt::pmt_t d_block_tag= pmt::intern("block_tag");
    static const int MAXLEN = (127+6)*8*8/2*4;
    static const int MEMCAP = MAXLEN*1024;
    static const uint64_t MAXBASE = 16777216;
    static const int LSAPHYLEN = 6;   // bytes: 0x00,0x00,0x00,0x00,0xe6,(0xLENGTH)
    static const int LSACODERATEINV=8;
    static const int LSAPHYSYMBOLLEN=LSAPHYLEN *8 *LSACODERATEINV/2;

    inline void phase_wrap(float& phase){
      while(phase>=TWO_PI)
        phase-=TWO_PI;
      while(phase<=-TWO_PI)
        phase+=TWO_PI;
    }

    interference_cancellation_core_cc::sptr
    interference_cancellation_core_cc::make(int sps,int cross_len,bool debug)
    {
      return gnuradio::get_initial_sptr
        (new interference_cancellation_core_cc_impl(sps,cross_len,debug));
    }

    /*
     * The private constructor
     */
    interference_cancellation_core_cc_impl::interference_cancellation_core_cc_impl(int sps,int cross_len,bool debug)
      : gr::block("interference_cancellation_core_cc",
              gr::io_signature::make3(3, 3, sizeof(gr_complex),sizeof(float),sizeof(float)),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
              d_mem_cap(MEMCAP)
    {
      d_in_mem = new gr_complex[d_mem_cap];
      d_out_mem= new gr_complex[d_mem_cap];
      d_phase_mem=new float[d_mem_cap];
      d_freq_mem = new float[d_mem_cap];
      d_retx_buffer = new gr_complex[d_mem_cap/2];
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
      if(cross_len<0){
        throw std::invalid_argument("Cross length cannot be negative");
      }
      d_cross_len = cross_len;
    }

    /*
     * Our virtual destructor.
     */
    interference_cancellation_core_cc_impl::~interference_cancellation_core_cc_impl()
    {
      delete [] d_in_mem;
      delete [] d_out_mem;
      delete [] d_phase_mem;
      delete [] d_freq_mem;
      delete [] d_retx_buffer;
    }

    bool
    interference_cancellation_core_cc_impl::tag_check()
    {
      //DEBUG<<"<IC Core DEBUG>tag_check begins"<<std::endl;
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
          // checking sample size
          if(map_it->second+block_offset >=d_in_mem_size){
            it = d_in_tlist.erase(it);
            continue;
          }
          // calculating average frequency offset, for IC resync
          float avg_freq = 0.0f;
          int pkt_len = (qidx==0)? pld + ( (LSAPHYSYMBOLLEN+d_cross_len)*d_sps) : pld + (LSAPHYSYMBOLLEN*d_sps);
          int sync_idx = it->index();
          for(int j=0;j<pkt_len;++j){
            avg_freq += d_freq_mem[sync_idx-j];
          }
          avg_freq/=(float)pkt_len;
          // matched stream tags
          it->delete_msg(pmt::intern("block_id"));
          it->delete_msg(pmt::intern("block_offset"));
          it->delete_msg(pmt::intern("LSA_hdr"));
          // NOTE: we should keep index in phase memory for further phase/freq tracking
          it->add_msg(pmt::intern("sample_index"),pmt::from_long(map_it->second+block_offset));
          it->add_msg(pmt::intern("avg_freq"),pmt::from_float(avg_freq));
        }
        it++;
      }
      // find su compensation or retry base
      if(d_in_tlist.empty()){
        return false;
      }
      int max_cnt=0;
      // fixing queue size for base candidate
      std::map<int,int> qmap;
      std::map<int,int>::iterator q_it;
      max_cnt = 0;
      int max_qsize=0;
      for(it = d_in_tlist.begin();it!=d_in_tlist.end();++it){
        pmt::pmt_t dict = it->msg();
        int qsize;
          qsize = pmt::to_long(pmt::dict_ref(dict,pmt::intern("queue_size"),pmt::from_long(-1)));
          if(qsize!=0){
            q_it = qmap.find(qsize);
          if(q_it == qmap.end()){
            qmap.insert(std::pair<int,int>(qsize,1));
            if(max_cnt==0){
              max_qsize = 1; max_cnt = 1;
            }
          }else{
            qmap[q_it->first]=q_it->second+1;
            if(q_it->second+1>max_cnt){
              max_cnt = q_it->second+1; max_qsize = q_it->first;
            }
          }
          }
      }
      // queue size candidate: max_qsize;
      std::vector<tagObject_t> update_tlist;
      std::list<tagObject_t>::reverse_iterator rit;
      if(max_qsize==0 || max_cnt==0){
        DEBUG<<"<IC Core DEBUG>Tag check return false, No valid queue_size found"<<std::endl;
        return false;
      }else{
        for(rit = d_in_tlist.rbegin();rit!=d_in_tlist.rend();++rit){
          pmt::pmt_t dict = rit->msg();
          int qsize = pmt::to_long(pmt::dict_ref(dict,pmt::intern("queue_size"),pmt::from_long(-1)));
          if(qsize == max_qsize){
            update_tlist.push_back(*rit);
          }
        }
      }
      // counting queue indexes
      // reset retx_table
      d_retx_table.clear();
      d_retx_table.resize(max_qsize);
      int retx_cnt = 0;
      for(int i=0;i<update_tlist.size();++i){
        pmt::pmt_t dict = update_tlist[i].msg();
        int qidx = pmt::to_long(pmt::dict_ref(dict,pmt::intern("queue_index"),pmt::from_long(-1)));
        if(d_retx_table[qidx].empty()){
          d_retx_table[qidx] = update_tlist[i];
          retx_cnt++;
          if(retx_cnt == max_qsize){
            break;
          }
        }
      }
      return retx_cnt==max_qsize;
    }

    bool
    interference_cancellation_core_cc_impl::do_interference_cancellation()
    {
      // ***********************************************************************************************************
      // Prequisite for entering this functtions:
      // 1. Tags been cleaned up
      // 2. retransmission tags are placed in d_retx_table<--vector of queue_size
      // 3. phase/freq tracking and waveform samples are stored in inner memory
      // Some notes:
      // a. The pair (qidx,qsize) = (0,X) has a dictinct preamble for cross correlation ahead of it <-- d_cross_len
      // b. The payload in tagObject_t is symbol-based and do no account for the preamble part of a packet.
      // c. The preamble length is defined as static const (LSAPHYSYMBOLLEN)
      // d. True samples are not copyed to other buffer. 
      // ***********************************************************************************************************
      // Step 0: reset retransmission buffer
      d_retx_buf_size = 0;
      d_retx_tags.clear();
      d_retx_tags.resize(d_retx_table.size());
      std::vector<int> retx_pkt_len; 
      // Step 1: Copy samples to other buffer
      const int reserved_length = d_sps*4; // magic number temporarily
      // NOTE: should reserved some samples in case timing offset
      for(int i=0;i<d_retx_table.size();++i){
        pmt::pmt_t dict = d_retx_table[i].msg();
        int32_t sync_idx  = d_retx_table[i].index();
        int32_t sample_idx= pmt::to_long(pmt::dict_ref(dict,pmt::intern("sample_index"),pmt::from_long(-1)));
        int pld = pmt::to_long(pmt::dict_ref(dict,pmt::intern("payload"),pmt::from_long(0)));
        int pkt_len = (pld+LSAPHYSYMBOLLEN)*d_sps;
        int copy_len= (pld+LSAPHYSYMBOLLEN)*d_sps;
        pmt::pmt_t avg_freq = pmt::dict_ref(dict,pmt::intern("avg_freq"),pmt::from_float(0.0f));
        if(pld==0 || sample_idx<0){
          throw std::runtime_error("<IC Core DEBUG>Interference cancellation core found invalid payload length or sample idx");
        }
        if(i==0){
          // handle cross correlation samples
          pkt_len += d_cross_len*d_sps;
          copy_len += (d_cross_len*d_sps+reserved_length);
        }else{
          copy_len += reserved_length;
        }
        if( (sample_idx+1<copy_len) || (sync_idx+1<copy_len) ){
          DEBUG<<"<IC Core DEBUG>do_ic: samples or sync less than required size, abort..."<<std::endl;
          return false;
        }else{
          if(d_retx_buf_size+copy_len>d_mem_cap){
            DEBUG<<"<IC Core DEBUG>do_ic: samples exceed retransmission buffer capacity"<<std::endl;
            return false;
          }
          memcpy(d_retx_buffer+d_retx_buf_size,d_in_mem+sample_idx-copy_len+1,sizeof(gr_complex)*copy_len);
          // sync phase
          float end_phase = d_phase_mem[sync_idx];
          for(int j =0;j<copy_len;++j){
            d_retx_buffer[sample_idx-j]*=gr_expj(-end_phase);
            end_phase-= d_freq_mem[sync_idx-j];
            phase_wrap(end_phase);
          }
          // record tag
          d_retx_tags[i].init_dict();
          d_retx_tags[i].add_msg(pmt::intern("packet_len"),pmt::from_long(pkt_len));
          d_retx_tags[i].add_msg(pmt::intern("copy_len"),pmt::from_long(copy_len));
          d_retx_tags[i].add_msg(pmt::intern("avg_freq"),avg_freq);
          d_retx_tags[i].set_idx(d_retx_buf_size);
          retx_pkt_len.push_back(pkt_len); // nominal length
          d_retx_buf_size+=copy_len;
          //DEBUG<<"<DO IC>Retx:"<<d_retx_tags[i];
        }        
      }
      // Step 2: collect those tags matched with retransmission base
      std::vector<tagObject_t> matched_tags;
      std::list<tagObject_t>::iterator it;
      int total_size;
      for(it=d_in_tlist.begin();it!=d_in_tlist.end();++it){
        int qsize,qidx,pld,sample_idx;
        uint64_t base;
        extract_tagObject(qidx,qsize,pld,sample_idx,base,*it);
        if( qsize == d_retx_table.size()){
          total_size = sample_idx+1;
          matched_tags.push_back(*it);
        }
      }
      std::vector<tagObject_t>::reverse_iterator vrit = matched_tags.rbegin();
      pmt::pmt_t last_msg = vrit->msg();
      int qidx_cnt = pmt::to_long(pmt::dict_ref(last_msg,pmt::intern("queue_index"),pmt::from_long(-1)));
      gr_complex * sample_ptr = d_retx_buffer + d_retx_tags[qidx_cnt].index();
      int copy_cnt = pmt::to_long(pmt::dict_ref(last_msg,pmt::intern("copy_len"),pmt::from_long(-1)));
      int phase_idx = vrit->index();
      int freq_idx = vrit->index();
      float d_retx_freq = pmt::to_float(pmt::dict_ref(last_msg,pmt::intern("avg_freq"),pmt::from_float(0.0f)));
      vrit++; // move on to next tagObject...
      // Step 3: Find a valid tagObject from behind
      // find total size to be cancelled
      d_out_mem_idx=0;d_out_mem_size=total_size;
      int nx_qidx,nx_qsize,nx_pld,nx_smp;
      uint64_t nx_bs;
      int nx_phase_idx;
      int nx_freq_idx;
      // main loop
      float d_phase = d_phase_mem[phase_idx], d_freq= d_freq_mem[freq_idx];
      bool good_track = true;
      while(total_size >0){
        // First step, find the begin of current block
        // Issues: Time sampling offset(+/-)
        // techniques: (+) align, (-) reserved_length 
        int begin_idx_tag=0;
        int begin_idx = total_size-retx_pkt_len[qidx_cnt];
        bool sync_failed = false;
        bool update_phase =false;
        if(vrit!=matched_tags.rend()){
          // there is tagObject for synchronization    
          extract_tagObject(nx_qidx,nx_qsize,nx_pld,nx_smp,nx_bs,*vrit);
          nx_phase_idx = vrit->index();
          nx_freq_idx  = vrit->index();
          begin_idx_tag = nx_smp+1;
          if(abs(begin_idx-begin_idx_tag)<reserved_length){
            update_phase = true;
            vrit++;
          }else if(nx_smp-begin_idx > reserved_length){
            // ****************************************************************
            // ISSUE:Block synchronization failure
            // ****************************************************************
            // This event may due to USRP overflow...
            // Result from slow interference cancellation process...
            // possible solution: considering reduce buffer size
            // ****************************************************************
            DEBUG<<"<IC Core DEBUG> Do_ic, Detect conflict tags:"<<std::endl;
            DEBUG<<*vrit<<*(vrit-1);
            // ****************************************************************
            // Handling Overflow, aggressively resync to available header info and continue
            // ----------------------------------------------------------------
            begin_idx = nx_smp+1;
            sync_failed = true;
          }
        }
        //Choose a proper begin index
        if(begin_idx_tag<begin_idx && (begin_idx-begin_idx_tag)<reserved_length){
          begin_idx = begin_idx_tag;
        }else{
          begin_idx = std::max(begin_idx_tag,begin_idx);
        }
        // cancel a block
        int block_length = total_size-begin_idx;
        for(int i=0;i<block_length;++i){
          // apply phase compansation
          d_out_mem[total_size-1-i] = d_in_mem[total_size-1-i] - sample_ptr[copy_cnt-1-i]*gr_expj(d_phase);
          d_phase-=d_freq;
          phase_wrap(d_phase);
          d_freq = (good_track)? d_freq_mem[freq_idx--] : d_freq;
          // Second method, use average CFO estimated by averaging over retransmission packet
          //d_freq = (good_track)? d_freq_mem[freq_idx--] : d_retx_freq;
        }
        total_size-=block_length;
        if(update_phase){
          d_phase = d_phase_mem[nx_phase_idx];
          d_freq = d_freq_mem[nx_freq_idx];
          good_track = true;
        }else{
          good_track = false;
        }
        if(sync_failed){
          DEBUG<<"<IC Core DEBUG>Resync to block index:"<<nx_qidx<< " (total:"<<total_size<<")"<<std::endl;
          qidx_cnt = nx_qidx;
          d_phase = d_phase_mem[nx_phase_idx];
          d_freq = d_freq_mem[nx_freq_idx];
          good_track = true;
          vrit++;
        }
        pmt::pmt_t tmp_msg = d_retx_tags[qidx_cnt].msg();
        copy_cnt = pmt::to_long(pmt::dict_ref(tmp_msg,pmt::intern("copy_len"),pmt::from_long(-1)));
        d_retx_freq =pmt::to_float(pmt::dict_ref(tmp_msg,pmt::intern("avg_freq"),pmt::from_float(0.0f)));
        sample_ptr = d_retx_buffer + d_retx_tags[qidx_cnt].index();
        qidx_cnt--;
        if(qidx_cnt<0)
          qidx_cnt+=d_retx_table.size();
      }
      return true;
    }

    void
    interference_cancellation_core_cc_impl::extract_tagObject(
                             int& qidx,int& qsize, 
                             int& payload, int& sample_idx,
                             uint64_t& base, const tagObject_t& obj)
    {
      pmt::pmt_t dict = obj.msg();
      qidx = pmt::to_long(pmt::dict_ref(dict,pmt::intern("queue_index"),pmt::from_long(-1)));
      qsize= pmt::to_long(pmt::dict_ref(dict,pmt::intern("queue_size"),pmt::from_long(-1)));
      payload=pmt::to_long(pmt::dict_ref(dict,pmt::intern("payload"),pmt::from_long(-1)));
      // Turn to sample based
      payload*=d_sps;
      sample_idx=pmt::to_long(pmt::dict_ref(dict,pmt::intern("sample_index"),pmt::from_long(-1)));
      base = pmt::to_uint64(pmt::dict_ref(dict,pmt::intern("base"),pmt::from_uint64(0)));
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
      const float* freq_in = (const float*) input_items[2];
      gr_complex *out = (gr_complex*) output_items[0];
      int nin_s = std::min(ninput_items[0],(d_mem_cap-d_in_mem_size) );
      int nin_p = std::min(ninput_items[1],std::min(ninput_items[2],(d_mem_cap-d_phase_size)));
      int nout = 0;
      bool reset_s = false;
      bool reset_p = false;
      bool reset_ready = false;
      std::vector<tag_t> tags_s;                          // collecting tags from ring queue, for reset purpose, stream 0
      std::vector<tag_t> tags_p;                          // collecting tags from ring queue, for reset prupose, stream 1
      std::vector<tag_t> hdr_tag;                         // collecting header tags from stream 1. should exclude ring tags
      std::vector<tag_t> block_s;                         // collecting block tags of stream 0, for stream sync purpose
      std::vector<tag_t> block_p;                         // collecting block tags of stream 1, for stream sync purpose
      const uint64_t nwrite = nitems_written(0);
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
      // clean up, in case there is only one object in tags
      if(!tmp_obj.empty()){
        d_in_tlist.push_back(tmp_obj);
      }
      // copy the samples and sync values
      memcpy(d_in_mem+d_in_mem_size,in,sizeof(gr_complex)*nin_s);
      memcpy(d_phase_mem+d_phase_size,phase_in,sizeof(float)*nin_p);
      memcpy(d_freq_mem+d_phase_size,freq_in,sizeof(float)*nin_p);
      d_in_mem_size += nin_s;
      d_phase_size  += nin_p;
      // reset condition checking
      if(reset_ready){
        // checking all tags
        if(tag_check()){
          DEBUG<<"<IC Core DEBUG>Tag checking success, calling do_interference_cancellation()..."<<std::endl;
          if(d_out_mem_size>0 && d_out_mem_idx!=d_out_mem_size){
            DEBUG<<"<IC Core warning> output of IC is overwitten...("<<d_out_mem_idx<<"/"<<d_out_mem_size<<")"<<std::endl;
          }
          if(do_interference_cancellation()){
            DEBUG<<"<IC Core DEBUG>Do_IC complete..."<<std::endl;
            add_item_tag(0,nwrite,pmt::intern("ic_out"),pmt::PMT_T);
          }else{
            DEBUG<<"<IC Core DEBUG>Do_IC failed..."<<std::endl;
          }
        }
        // end of a possible ic opportunities
        if(d_in_mem_size == d_mem_cap){
          DEBUG<<"<IC Core>signal buffer full..."<<std::endl;
        }else{
          DEBUG <<"<IC Core>Reset signal are all ready, clear queue... sig_size:"
          <<d_in_mem_size<<" ,phase_size:"<<d_phase_size<<std::endl;
          nin_s++;
          nin_p++;
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
        // FIXME
        // Current version will force to zero
        d_out_mem_idx =0;
        d_out_mem_size =0;
      }
      // output condition checking
      nout = std::min(noutput_items, d_out_mem_size-d_out_mem_idx);
      memcpy(out,d_out_mem+d_out_mem_idx,sizeof(gr_complex)*nout);
      d_out_mem_idx+=nout;
      // for GR schedualer maintenance
      consume(0,nin_s);
      consume(1,nin_p);
      consume(2,nin_p);
      return nout;
    }

  } /* namespace lsa */
} /* namespace gr */

