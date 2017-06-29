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
#include "ic_critical_cc_impl.h"
#include <volk/volk.h>
#include <gnuradio/expj.h>
#include <gnuradio/math.h>

namespace gr {
  namespace lsa {

#define DEBUG d_debug && std::cout
#define PHYLEN 6
#define LSACODERATEINV 8
#define MAXLEN (127+PHYLEN)*8*8/2*4
#define MAXCAP 256*MAXLEN
#define TWO_PI M_PI*2.0f
#define SAMPLE_PORT 0
#define PHASE_PORT 1
#define FREQ_PORT 2

  enum VOESTATE{
    FREE,
    SUFFERING,
    SEARCH_RETX
  };

    static const pmt::pmt_t d_block_tag = pmt::intern("block_tag");  // sample block tagger for stream matching
    static const pmt::pmt_t d_voe_tag = pmt::intern("voe_tag");      // variance of energy tagger
    static const pmt::pmt_t d_sfd_tag = pmt::intern("sfd_est");
    static const pmt::pmt_t d_lsa_tag = pmt::intern("LSA_hdr");
    static const int d_min_process = 16;                             // minimum labeled headers to process retransmissions
    static const int LSAPHYSYMBOLLEN = PHYLEN*8*LSACODERATEINV/2;    // LSA Physical layer symbol level length
    
    // helper function for phase correction
    inline void phase_wrap(float& phase){
      while(phase>=TWO_PI)
        phase-=TWO_PI;
      while(phase<=-TWO_PI)
        phase+=TWO_PI;
    }

    ic_critical_cc::sptr
    ic_critical_cc::make(int prelen, int sps,int block_size,bool debug)
    {
      return gnuradio::get_initial_sptr
        (new ic_critical_cc_impl(prelen,sps,block_size,debug));
    }

    static int ios[] = {sizeof(gr_complex),sizeof(float),sizeof(float)};
    static std::vector<int> iosig(ios,ios+sizeof(ios)/sizeof(int));
    ic_critical_cc_impl::ic_critical_cc_impl(int prelen,int sps,int block_size,bool debug)
      : gr::block("ic_critical_cc",
              gr::io_signature::makev(3, 3, iosig),
              gr::io_signature::make2(2, 2, sizeof(gr_complex),sizeof(gr_complex))),
              d_cap(MAXCAP),
              d_out_msg_port(pmt::mp("SINR"))
    {
      d_debug = debug;
      d_in_mem = new gr_complex[d_cap];
      d_comp_mem = new gr_complex[d_cap];
      d_phase_mem = new float[d_cap];
      d_freq_mem= new float[d_cap];
      d_out_mem = (gr_complex*) volk_malloc(sizeof(gr_complex)*d_cap,volk_get_alignment());
      d_retx_mem = (gr_complex*) volk_malloc(sizeof(gr_complex)*d_cap,volk_get_alignment());
      d_intf_mem = (gr_complex*) volk_malloc(sizeof(gr_complex)*d_cap,volk_get_alignment());
      d_intf_freq = (float*) volk_malloc(sizeof(float)*d_cap,volk_get_alignment());
      d_in_idx = 0;
      d_sync_idx =0;
      d_out_idx =0;
      d_out_size =0;
      d_state = FREE;
      d_voe_state = false;
      d_intf_idx =0;
      d_in_block_idx = 0;
      d_phase_block_idx =0;

      if(prelen<=0){
        throw std::invalid_argument("preamble length should be positive");
      }
      // sample based
      d_prelen = prelen;
      if(sps<=0){
        throw std::invalid_argument("Invalid samples per symbol");
      }
      d_sps = sps;
      if(block_size<=0){
        throw std::runtime_error("Invalid block size");
      }
      d_block_size = block_size;
      reset_retx();
      set_tag_propagation_policy(TPP_DONT);
      message_port_register_out(d_out_msg_port);
    }

    /*
     * Our virtual destructor.
     */
    ic_critical_cc_impl::~ic_critical_cc_impl()
    {
      delete [] d_in_mem;
      delete [] d_phase_mem;
      delete [] d_freq_mem;
      delete [] d_comp_mem;
      volk_free(d_out_mem);
      volk_free(d_retx_mem);
      volk_free(d_intf_mem);
      volk_free(d_intf_freq);
    }

    bool
    ic_critical_cc_impl::detect_ic_chance(const hdr_t& new_tag)
    {
      d_retx_candidate.push_back(new_tag);
      std::list<hdr_t>::reverse_iterator rit;
      std::map<uint32_t,int> count_map;
      std::map<uint32_t,int>::iterator map_it;
      if(d_retx_candidate.size()>=d_min_process){
        if(d_retx_tag.empty()){
          // initialization
          DEBUG<<"<IC Crit>Initialize retransmission collecting"<<std::endl;
          int max_qsize =0;
          int max_cnt =0;
          for(rit = d_retx_candidate.rbegin();rit!=d_retx_candidate.rend();++rit){
            int tmp_qsize = pmt::to_long(pmt::dict_ref(rit->msg(),pmt::intern("queue_size"),pmt::from_long(-1)));
            map_it = count_map.find(tmp_qsize);
            if(map_it==count_map.end()){
              count_map.insert(std::pair<uint32_t,int>(tmp_qsize,1));
            }else{
              int new_cnt = map_it->second +1;
              count_map[tmp_qsize] = new_cnt;
              if(new_cnt > max_cnt){
                max_cnt = new_cnt;
                max_qsize = map_it->first; 
              }
            }
          }
          if(max_cnt==0){
            return false;
          }
          d_retx_idx =0;
          d_retx_cnt=0;
          d_retx_tag.clear();
          d_retx_block.clear();
          d_retx_tag.resize(max_qsize);
          d_retx_block.resize(max_qsize);
          for(rit=d_retx_candidate.rbegin();rit!=d_retx_candidate.rend();++rit){
            int tmp_qsize = pmt::to_long(pmt::dict_ref(rit->msg(),pmt::intern("queue_size"),pmt::from_long(-1)));
            int tmp_qidx = pmt::to_long(pmt::dict_ref(rit->msg(),pmt::intern("queue_index"),pmt::from_long(-1)));
            if(tmp_qsize == max_qsize && d_retx_tag[tmp_qidx].empty()){
              hdr_t tmp_tag = *rit;
              if(check_and_copy_retx(tmp_tag)){
                d_retx_tag[tmp_qidx] = tmp_tag;
                d_retx_cnt++;
              }
            }
          }
        }else{
          // maintenance
          int qidx = pmt::to_long(pmt::dict_ref(new_tag.msg(),pmt::intern("queue_index"),pmt::from_long(-1)));
          int qsize= pmt::to_long(pmt::dict_ref(new_tag.msg(),pmt::intern("queue_size"),pmt::from_long(-1)));
          if(qsize!=d_retx_tag.size()){
            return false;
          }else if(d_retx_tag[qidx].empty()){
            hdr_t tmp_tag = new_tag;
            if(check_and_copy_retx(tmp_tag)){
              d_retx_tag[qidx]=tmp_tag;
              d_retx_cnt++;
            }
          }
        }
        if(!d_retx_tag.empty() && d_retx_tag.size()==d_retx_cnt){
          // received all retransmission !!
          // should prepare all required information
          DEBUG<<"<IC Crit> Retransmission received, retx_size="<<d_retx_tag.size()<<std::endl;
          return true;
        }
      }
      return false;
    }

    bool
    ic_critical_cc_impl::check_and_copy_retx(hdr_t& tag)
    {
      pmt::pmt_t msg = tag.msg();
      uint64_t block_id = pmt::to_uint64(pmt::dict_ref(msg,pmt::intern("sync_block_id"),pmt::from_uint64(0xffffffffffff)));
      int block_offset = pmt::to_long(pmt::dict_ref(msg,pmt::intern("sync_offset"),pmt::from_long(0)));
      int payload = pmt::to_long(pmt::dict_ref(msg,pmt::intern("payload"),pmt::from_long(-1)));
      int pkt_nominal = (payload + LSAPHYSYMBOLLEN)*d_sps;
      const int reserved_length = 8*d_sps;
      if(!buffer_index_check(tag.index(),pkt_nominal+reserved_length,SAMPLE)){
        //DEBUG<<"<RETX Check> Sample check failed"<<std::endl;
        //DEBUG<<"tag_index:"<<tag.index()<<" pkt_nominal+reserved_length:"<<pkt_nominal+reserved_length<<" ,d_in_idx:"<<d_in_idx<<std::endl;
        return false;
      }
      int bidx = search_id(block_id);
      if(bidx<0){
        return false;
      }
      int sync_idx = (bidx+block_offset)%d_cap;
      int sync_iter = sync_idx;
      if(!buffer_index_check(sync_idx,pkt_nominal,SYNC)){
        //DEBUG<<"<RETX Check> Sync check failed"<<std::endl;
        //DEBUG<<"bit_p->index():"<<bit_p->index()<<" offset:"<<block_offset<<" ,d sync_idx:"<<d_sync_idx<<std::endl;
        return false;
      }
      int copy_len = pkt_nominal+reserved_length;
      int samp_iter = tag.index();
      int retx_idx_begin = d_retx_idx;
      if(d_retx_idx+copy_len>=d_cap){
        // buffer size not enough
        DEBUG<<"<RETX Check> RETX overflow"<<std::endl;
        return false;
      }
      for(int i=0;i<pkt_nominal;++i){
        d_intf_freq[i] = d_freq_mem[sync_iter++];
        sync_iter%=d_cap;
      }
      float mean,stddev;
      volk_32f_stddev_and_mean_32f_x2(&stddev,&mean,d_intf_freq,pkt_nominal);
      // FIXME
      // Use init_phase to ratate phase offset
      float init_phase = pmt::to_float(pmt::dict_ref(msg,pmt::intern("init_phase"),pmt::from_float(0)));
      sync_iter = sync_idx; // reset
      for(int i=0;i<copy_len;++i){
        d_retx_mem[d_retx_idx++] = d_in_mem[samp_iter++] * gr_expj(-init_phase);
        init_phase+= d_freq_mem[sync_iter++];
        phase_wrap(init_phase);
        samp_iter%=d_cap;
      }
      tag.set_index(retx_idx_begin);
      tag.add_msg(pmt::intern("copy_len"),pmt::from_long(copy_len));
      tag.add_msg(pmt::intern("packet_len"),pmt::from_long(pkt_nominal));
      tag.add_msg(pmt::intern("freq_mean"),pmt::from_float(mean));
      tag.add_msg(pmt::intern("freq_std"),pmt::from_float(stddev));
      tag.delete_msg(pmt::intern("sync_block_id"));
      tag.delete_msg(pmt::intern("in_block_id"));
      tag.delete_msg(pmt::intern("sync_offset"));
      tag.delete_msg(pmt::intern("in_offset"));
      return true;
    }

    void
    ic_critical_cc_impl::reset_retx()
    {
      d_retx_candidate.clear();
      d_retx_tag.clear();
      d_retx_block.clear();
      d_retx_cnt =0;
      d_retx_idx=0;
    }

    void
    ic_critical_cc_impl::init_intf()
    {
      d_intf_idx =0;
      d_current_intf_tag = intf_t();
      d_intf_stack.clear();
    }

    bool
    ic_critical_cc_impl::new_intf()
    {
      if(!d_current_intf_tag.empty()){
        return false;
      }
      std::list<hdr_t>::reverse_iterator rit = d_tag_list.rbegin();
      int bit_p;
      int block_offset;
      int pkt_nominal;
      while(rit!=d_tag_list.rend()){
        pmt::pmt_t msg = rit->msg();
        uint64_t id = pmt::to_uint64(pmt::dict_ref(msg,pmt::intern("sync_block_id"),pmt::from_uint64(0xffffffffffff)));
        block_offset = pmt::to_long(pmt::dict_ref(msg,pmt::intern("sync_offset"),pmt::from_long(0)));
        int payload = pmt::to_long(pmt::dict_ref(msg,pmt::intern("payload"),pmt::from_long(-1)));
        pkt_nominal = (payload+LSAPHYSYMBOLLEN) *d_sps;
        bit_p =search_id(id);
        if(bit_p>=0 && id!=d_current_block){
          break;
        }
        rit++;
      }
      if(rit==d_tag_list.rend()||bit_p<0){
        return false;
      }
      // but notice the wrap around point!!
      int current_begin = d_in_idx;
      int current_end = (d_in_idx==0)? d_cap-1 : d_in_idx-1;
      if(current_begin>current_end){
        current_end+=d_cap;
      }
      int samp_idx = rit->index();
      samp_idx = (samp_idx<current_begin)?samp_idx+d_cap : samp_idx;
      int length_check = current_end - (samp_idx+pkt_nominal) ;
      if(length_check+d_intf_idx >=d_cap){
        DEBUG<<"<IC Crit>loading history will make buffer overflow"<<std::endl;
        return false;
      }
      int intf_begin = d_intf_idx;
      tag_t tmp_tag;
      tmp_tag.offset = d_intf_idx;
      tmp_tag.key = pmt::intern("Intf_begin");
      tmp_tag.value=pmt::PMT_T;
      samp_idx = rit->index();
      while(samp_idx!=d_in_idx){
        d_intf_mem[d_intf_idx++] = d_in_mem[samp_idx++];
        samp_idx %= d_cap;
      }
      // for phase stream
      int sync_idx = (bit_p+block_offset)%d_cap;
      if(!buffer_index_check( sync_idx, pkt_nominal,SYNC)){
        // reset to index before copying
        d_intf_idx = intf_begin;
        return false;
      }
      int count =0;
      for(count=0;count<pkt_nominal;++count){
        d_intf_freq[count] = d_freq_mem[sync_idx++];
        sync_idx%=d_cap;
      }
      float mean,stddev;
      volk_32f_stddev_and_mean_32f_x2(&stddev,&mean,d_intf_freq,count);
      hdr_t new_front = *rit;
      new_front.add_msg(pmt::intern("freq_mean"),pmt::from_float(mean));
      new_front.add_msg(pmt::intern("freq_std"),pmt::from_float(stddev)); 
      d_current_intf_tag.set_begin(intf_begin);
      d_current_intf_tag.set_front(new_front);
      // add tag only everything set
      d_out_tags.push_back(tmp_tag);
      return true;
    }

    bool
    ic_critical_cc_impl::update_intf()
    {
      if(d_current_intf_tag.empty() || !d_current_intf_tag.back_tag_empty()){
        return false;
      }
      pmt::pmt_t intf_msg = d_current_intf_tag.msg();
      int last_voe_end_idx = pmt::to_long(pmt::dict_ref(intf_msg,pmt::intern("voe_end_idx"),pmt::from_long(-1))) ;
      // guarantee for new tags
      // just need to check the last one
      std::list<hdr_t>::reverse_iterator rit = d_tag_list.rbegin();
      pmt::pmt_t msg = rit->msg();
      uint64_t id = pmt::to_uint64(pmt::dict_ref(msg,pmt::intern("sync_block_id"),pmt::from_uint64(0xffffffffffff)));
      int idx = pmt::to_long(pmt::dict_ref(msg,pmt::intern("sync_offset"),pmt::from_long(0)));
      int payload = pmt::to_long(pmt::dict_ref(msg,pmt::intern("payload"),pmt::from_long(-1)));
      int nominal_pkt = (payload+LSAPHYSYMBOLLEN)*d_sps;
      // guard length first guarantee not ends in interfering samples
      if(d_intf_idx-nominal_pkt+1 <last_voe_end_idx ){
        return false;
      }
      int bit_p = search_id(id);
      if(bit_p<0 || rit == d_tag_list.rend()){
        // failed
        return false;
      }
      hdr_t tmp_front = d_current_intf_tag.front();
      pmt::pmt_t fmsg = tmp_front.msg();
      uint64_t fid = pmt::to_uint64(pmt::dict_ref(fmsg,pmt::intern("sync_block_id"),pmt::from_uint64(0xffffffffffff)));
      int foff = pmt::to_long(pmt::dict_ref(fmsg,pmt::intern("sync_offset"),pmt::from_long(0)));
      if(fid>=id){
        return false;
      }else{
        long int fdis = (id-fid)*d_block_size + (idx-foff);
        if(fdis<d_block_size){
          return false;
        }
      }
      int end_idx_corrected =d_intf_idx-1;
      // success to record a valid header after interference occur
      //DEBUG<<"<Update Intf> Front tag:"<<d_current_intf_tag.front()<<std::endl;
      //DEBUG<<"<Update Intf> Insert back header:"<<*rit<<std::endl;
      hdr_t new_back = *rit;
      d_current_intf_tag.set_back(new_back);
      d_current_intf_tag.set_end(end_idx_corrected);
      // tag output
      tag_t tag;
      tag.offset = end_idx_corrected;
      tag.key = pmt::intern("Intf_end");
      tag.value = pmt::PMT_T;
      d_out_tags.push_back(tag);
      return true;
    }

    void
    ic_critical_cc_impl::do_ic()
    {
      // ic core here.
      std::map<int,int> base_map;
      std::map<int,int>::iterator map_it;
      int base;
      for(int i=0;i<d_retx_tag.size();++i){
        pmt::pmt_t msg = d_retx_tag[i].msg();
        base = pmt::to_long(pmt::dict_ref(msg,pmt::intern("base"),pmt::from_long(-1)));
        base_map.insert(std::pair<int,int>(base,i));
      }
      for(int i=0;i<d_intf_stack.size();++i){
        hdr_t front = d_intf_stack[i].front();
        hdr_t back = d_intf_stack[i].back();
        int intf_begin = d_intf_stack[i].begin();
        int intf_end = d_intf_stack[i].end();
        DEBUG<<"Processing intf tag<"<<i<<">:"<<std::endl<<d_intf_stack[i]<<std::endl;
        pmt::pmt_t front_msg = front.msg();
        int front_base = pmt::to_long(pmt::dict_ref(front_msg,pmt::intern("base"),pmt::from_long(-1)));
        // DEBUG
        // FOR output SINR
        // locate prou signal
        pmt::pmt_t intf_msg = d_intf_stack[i].msg();
        int voe_begin_idx = pmt::to_long(pmt::dict_ref(intf_msg,pmt::intern("voe_begin_idx"),pmt::from_long(0)));
        int voe_end_idx = pmt::to_long(pmt::dict_ref(intf_msg,pmt::intern("voe_end_idx"),pmt::from_long(0)));
        
        map_it = base_map.find(front_base);
        if(map_it==base_map.end()){
          DEBUG<<"\033[32m"<<"<IC Crit DEBUG>Intf tag <"<<i<<">find no matched base, ignore"<<"\033[0m"<<std::endl;
          continue;
        }
        int retx_id = map_it->second;
        int retx_idx = d_retx_tag[retx_id].index();
        pmt::pmt_t retx_msg = d_retx_tag[retx_id].msg();
        DEBUG<<"<IC Crit DEBUG>First retransmission tag:"<<d_retx_tag[retx_id]<<std::endl;
        int pkt_len = pmt::to_long(pmt::dict_ref(retx_msg,pmt::intern("packet_len"),pmt::from_long(0)));
        int pkt_cnt=0;
        int intf_cnt = intf_begin;
        float init_phase = pmt::to_float(pmt::dict_ref(front_msg,pmt::intern("init_phase"),pmt::from_float(0)));
        float freq_mean = pmt::to_float(pmt::dict_ref(front_msg,pmt::intern("freq_mean"),pmt::from_float(0)));
        // FOR SINR calculation
        // waste system resources, only for demo purpose
        gr_complex retx_eng;
        volk_32fc_x2_conjugate_dot_prod_32fc(&retx_eng,d_retx_mem+retx_idx,d_retx_mem+retx_idx,pkt_len);
        retx_eng/=(float)pkt_len; // avg power
        //
        while(intf_cnt<intf_end){
          // FIXME
          // try to improve phase correction
          // for DEMO
          d_comp_mem[d_out_size] = d_intf_mem[intf_cnt]*gr_expj(-init_phase);
          d_out_mem[d_out_size++] = d_intf_mem[intf_cnt++]*gr_expj(-init_phase) - d_retx_mem[retx_idx++];
          init_phase+=freq_mean;
          phase_wrap(init_phase);
          pkt_cnt++;
          if(pkt_cnt==pkt_len){
            // Update Retransmission 
            retx_id = (retx_id+1)%d_retx_tag.size();
            retx_idx = d_retx_tag[retx_id].index();
            retx_msg = d_retx_tag[retx_id].msg();
            pkt_len = pmt::to_long(pmt::dict_ref(retx_msg,pmt::intern("packet_len"),pmt::from_long(0)));
            pkt_cnt=0;
            // NOTE: maybe the freq est in retransmission helps?
            tag_t tmp_tag;
            tmp_tag.offset = d_out_size;
            tmp_tag.key = pmt::intern("retx");
            tmp_tag.value = pmt::from_long(retx_id);
            d_out_tags.push_back(tmp_tag);
          }
          if(d_out_size==d_cap){
            DEBUG<<"\033[32m"
            <<"<IC Crit DEBUG>Warning: output size reach capacity, reset index and size, some samples may be overwritten"
            <<"\033[0m"<<std::endl;
            d_out_idx=0;
            d_out_size=0;
            d_out_tags.clear();
          }
        }//while intf_idx
        DEBUG<<"\033[35;1m"<<"<IC Crit>Produced output samples:"<<d_out_size<<"\033[0m"<<std::endl;
        // SINR calculation
        // Waste of system resources, only for demo purpose
        pmt::pmt_t out_msg = pmt::make_dict();
        gr_complex sig_eng, ic_eng;
        int voe_len = voe_end_idx-voe_begin_idx+1;
        if(voe_len<=0 || (voe_begin_idx+voe_len>=d_cap) ){
          // save guard, avoid overflow, may require warning
          DEBUG<<"\033[36;1m"<<"<IC Crit DEBUG>When Calculating SINR, predict overflow event, avoid execution"
          <<"\033[0m"<<std::endl;
          continue;
        }
        volk_32fc_x2_conjugate_dot_prod_32fc(&sig_eng,d_intf_mem+voe_begin_idx,d_intf_mem+voe_begin_idx,voe_len);
        volk_32fc_x2_conjugate_dot_prod_32fc(&ic_eng,d_out_mem+voe_begin_idx,d_out_mem+voe_begin_idx,voe_len);
        sig_eng/=(float)voe_len;
        ic_eng/=(float)voe_len;
        // original-SINR: (sig_eng - retx_eng)/(retx_eng+gr_complex(1e-16,0));
        // canceled-SINR: (sig_eng - retx_eng)/(ic_eng-sig_eng+retx_eng+gr_complex(1e-16,0));
        double oSINR = abs(sig_eng - retx_eng)/(double)abs((retx_eng+gr_complex(1e-16,0)));
        double cSINR = abs(sig_eng - retx_eng)/(double)abs((ic_eng-sig_eng+retx_eng+gr_complex(1e-16,0)));
        out_msg = pmt::dict_add(out_msg,pmt::intern("original_SINR"),pmt::from_double(oSINR));
        out_msg = pmt::dict_add(out_msg,pmt::intern("canceled_SINR"),pmt::from_double(cSINR));
        message_port_pub(d_out_msg_port,out_msg);
      }// for d_intf_stack
      std::sort(d_out_tags.begin(),d_out_tags.end(),gr::tag_t::offset_compare);
    }

    int
    ic_critical_cc_impl::search_id(uint64_t id)
    {
      // ONLY Sync list need to search block id, since sample has header binding
      std::list< std::pair<block_t,int> >::reverse_iterator rit = d_sync_list.rbegin();
      while(rit!=d_sync_list.rend()){
        block_t tmp = std::get<0>(*rit);
        if(tmp.id()==id){
          return tmp.index();
        }
        rit++;
      }
      return -1;
    }

    bool
    ic_critical_cc_impl::buffer_index_check(int idxToCheck,int duration, BUFFERTYPE type)
    {
      int abs_begin,abs_end;
      int rel_end;
      int rel_toCheck;
      if(type == SAMPLE){
        abs_begin = d_in_idx;
      }else if(type == SYNC){
        abs_begin = d_sync_idx;
      }else if(type == INTF){
        abs_begin = d_intf_idx;
      }else{
        abs_begin = d_retx_idx;
      }
      switch(type){
        case SAMPLE:
        case SYNC:
          abs_end = (abs_begin==0)? d_cap-1 : abs_begin-1;
          rel_toCheck = (idxToCheck<abs_begin)? idxToCheck+d_cap : idxToCheck;
          rel_end = (abs_end<=abs_begin)? abs_end+d_cap : abs_end;
          if(rel_toCheck+duration-1>rel_end){
            return false;
          }
        break;
        case INTF:  
        case RETX:
          if(abs_begin+duration-1>=d_cap){
            return false;
          }
        break;
        default:
          throw std::runtime_error("Undefined state");
        break;
      }
      return true;
    }

    bool
    ic_critical_cc_impl::preprocess_hdr(hdr_t& raw_hdr)
    {
      pmt::pmt_t src = raw_hdr.msg();
      int qidx = pmt::to_long(pmt::dict_ref(src,pmt::intern("queue_index"),pmt::from_long(-1)));
      int qsize= pmt::to_long(pmt::dict_ref(src,pmt::intern("queue_size"),pmt::from_long(-1)));
      int base =pmt::to_long(pmt::dict_ref(src,pmt::intern("base"),pmt::from_long(-1)));
      if(qidx<0 || qsize<0 || base<0){
        return false;
      }else if(qsize!=0 && qidx>=qsize){
        return false;
      }
      if(!matching_header(raw_hdr)){
        //DEBUG<<"<IC Crit> header not match any candidates..."<<std::endl;
        return false;
      }
      return true;
    }

    void
    ic_critical_cc_impl::update_voe_state(int idx)
    {
      const uint64_t nread = nitems_read(SAMPLE_PORT);
      while(!d_voe_tags.empty()){
        int offset = d_voe_tags[0].offset - nread;
        if(offset == idx){
          d_voe_state = pmt::to_bool(d_voe_tags[0].value);
          d_voe_tags.erase(d_voe_tags.begin());
          break;
        }else if(idx>offset){
          throw std::runtime_error("WTF...idx>offset");
        }else{
          break;
        }
      }
    }

    std::pair<uint64_t,int>
    ic_critical_cc_impl::sync_block_offset_converter(uint64_t bid, int offset, int revdis)
    {
      std::list< std::pair<block_t,int> >::reverse_iterator rit;
      for(rit=d_sync_list.rbegin();rit!=d_sync_list.rend();++rit){
        uint64_t id = std::get<0>(*rit).id();
        if(id==bid){
          break;
        }
      }
      if(rit == d_sync_list.rend()){
        return std::make_pair(0,-1);
      }
      offset-=revdis;
      while(revdis<0){
        rit++;
        if(rit==d_sync_list.rend()){
          return std::make_pair(0,-1);
        }
        revdis += std::get<1>(*rit);
        assert(bid!=0);
        bid = (std::get<0>(*rit)).id();
      }
      return std::make_pair(bid,offset);
    }

    bool
    ic_critical_cc_impl::matching_header(hdr_t& header){
      // header is from sync stream
      pmt::pmt_t msg = header.msg();
      uint64_t block_id = pmt::to_uint64(pmt::dict_ref(msg,pmt::intern("sync_block_id"),pmt::from_uint64(0xfffffffffffe)));
      int block_offset = header.index();
      int payload = pmt::to_long(pmt::dict_ref(msg,pmt::intern("payload"),pmt::from_long(-1)));
      int pkt_nominal = (payload+LSAPHYSYMBOLLEN)*d_sps;
      std::list<hdr_t>::iterator it; 
      std::list<hdr_t>::iterator candidate_it= d_pending_list.end();
      hdr_t matched_hdr;
      pmt::pmt_t temp_msg;
      uint64_t pending_bid;
      int min_diff = d_block_size;
      int diff;
      int pending_idx;
      int pending_offset;
      // FOR DEBUGGING PURPOSE
      int min_pend_id;
      int min_pend_offset;
      pmt::pmt_t min_phase;
      // FIXME
      // Convert to begin index...
      std::pair<uint64_t, int> cvt = sync_block_offset_converter(block_id,block_offset,pkt_nominal);
      block_id = std::get<0>(cvt);
      block_offset = std::get<1>(cvt);
      if(block_offset<0){
        // conversion failed
        return false;
      }
      for(it= d_pending_list.begin();it!=d_pending_list.end();++it){
        temp_msg = it->msg();
        pending_bid = pmt::to_uint64(pmt::dict_ref(temp_msg,pmt::intern("in_block_id"),pmt::from_uint64(0xfffffffffffe)));
        pending_offset = pmt::to_long(pmt::dict_ref(temp_msg,pmt::intern("in_offset"),pmt::from_long(0)));
        pending_idx = it->index();
        // should compare distance in absolute sample index...
        // this is guarantee since it is in sample buffer.
        diff = (pending_bid>block_id)? (pending_bid-block_id)*d_block_size : (block_id-pending_bid)*d_block_size;
        diff += (pending_bid>block_id)? (pending_offset-block_offset) : (block_offset- pending_offset);
        if(diff<min_diff){
          matched_hdr = *it;
          min_diff = diff;
          candidate_it = it;
          min_pend_id = pending_bid;
          min_pend_offset = pending_offset;
          min_phase = pmt::dict_ref(temp_msg,pmt::intern("init_phase"),pmt::from_float(0));
        }
      }
      if(candidate_it==d_pending_list.end() || min_diff>= d_block_size/16){
        return false;
      }
      candidate_it++;
      d_pending_list.erase(d_pending_list.begin(),candidate_it);
      // update matched header
      header.delete_msg(pmt::intern("sync_block_id"));
      header.add_msg(pmt::intern("in_offset"),pmt::from_long(min_pend_offset));
      header.add_msg(pmt::intern("in_block_id"),pmt::from_uint64(min_pend_id));
      header.add_msg(pmt::intern("sync_offset"),pmt::from_long(block_offset));
      header.add_msg(pmt::intern("sync_block_id"),pmt::from_uint64(block_id));
      header.add_msg(pmt::intern("init_phase"),min_phase);
      // bind to ring queue...
      header.set_index(matched_hdr.index());
      return true;
    }

    void
    ic_critical_cc_impl::check_before_reset()
    {
      std::map<int,int> base_map;
      std::map<int,int>::iterator map_it;
      std::vector<intf_t> new_intf_stack;
      for(int i=0;i<d_retx_tag.size();++i){
        if(!d_retx_tag[i].empty()){
          pmt::pmt_t retx_msg = d_retx_tag[i].msg();
          int base = pmt::to_long(pmt::dict_ref(retx_msg,pmt::intern("base"),pmt::from_long(-1)));
          base_map.insert(std::pair<int,int>(base,i));
        }
      }
      for(int i=0;i<d_intf_stack.size();++i){
        pmt::pmt_t msg = d_intf_stack[i].msg();
        hdr_t front = d_intf_stack[i].front();
        hdr_t end = d_intf_stack[i].back();
        pmt::pmt_t front_msg = front.msg();
        pmt::pmt_t end_msg = end.msg();
        int front_base = pmt::to_long(pmt::dict_ref(front_msg,pmt::intern("base"),pmt::from_long(-1)));
        int end_base = pmt::to_long(pmt::dict_ref(end_msg,pmt::intern("base"),pmt::from_long(-1)));
        map_it = base_map.find(front_base);
        if(map_it==base_map.end() || (front_base == end_base) ){
          continue;
        }
        int front_idx = map_it->second;
        int count = 0;
        while(count<d_retx_tag.size() && front_base!=end_base){
          front_idx = (front_idx+1)%d_retx_tag.size();
          if(d_retx_tag[front_idx].empty()){
            break;
          }
          front_msg = d_retx_tag[front_idx].msg();
          front_base = pmt::to_long(pmt::dict_ref(front_msg,pmt::intern("base"),pmt::from_long(-1)));
          if(front_base == end_base){
            // can be canceled with incomplete retransmission
            break;
          }
          count++;
        }
        if(front_base == end_base){
          new_intf_stack.push_back(d_intf_stack[i]);
        }
      }
      d_intf_stack.clear();
      d_intf_stack = new_intf_stack;
    }

    void
    ic_critical_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      for(int i=0;i<ninput_items_required.size();++i){
        ninput_items_required[i] = noutput_items;
      }
    }

    int
    ic_critical_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[SAMPLE_PORT];
      const float* phase= (const float*) input_items[PHASE_PORT];
      const float* freq = (const float*) input_items[FREQ_PORT];
      gr_complex *out = (gr_complex *) output_items[0];
      gr_complex *demo= (gr_complex *) output_items[1];
      int nout = 0;
      int nin_s = ninput_items[SAMPLE_PORT];
      int nin_p = std::min(ninput_items[PHASE_PORT],ninput_items[FREQ_PORT]);
      bool d_do_ic = false;
      bool d_reset_retx=false;
      std::vector<tag_t> tags_s,tags_p,hdr_tags,cross_tags;
      d_voe_tags.clear(); //
      get_tags_in_window(tags_s,SAMPLE_PORT,0,nin_s,d_block_tag);
      get_tags_in_window(tags_p,PHASE_PORT,0,nin_p,d_block_tag);
      get_tags_in_window(d_voe_tags,SAMPLE_PORT,0,nin_s,d_voe_tag); //
      get_tags_in_window(cross_tags,SAMPLE_PORT,0,nin_s,pmt::intern("phase_est"));
      int next_block_id = d_current_block;
      if(!tags_s.empty()){
        nin_s = tags_s[0].offset-nitems_read(SAMPLE_PORT);
      }
      if(!tags_p.empty()){
        nin_p = tags_p[0].offset-nitems_read(PHASE_PORT);
      }
      int next_in_block_idx = d_in_block_idx + nin_s;
      int next_phase_block_idx = d_phase_block_idx + nin_p;
      if(!tags_s.empty() && !tags_p.empty()){
        //DEBUG<<"<Crit> next_in:"<<next_in_block_idx<<" ,next_phase:"<<next_phase_block_idx<<std::endl;
        // next block tags is ready
        uint64_t bid_s = pmt::to_uint64(tags_s[0].value);
        uint64_t bid_p = pmt::to_uint64(tags_p[0].value);
        nin_s++;
        nin_p++;
        d_smp_list.push_back(block_t(bid_s,d_in_idx));
        d_sync_list.push_back(std::pair<block_t,int>(block_t(bid_p,d_sync_idx),next_phase_block_idx+1));
        //d_sync_list.push_back(block_t(bid_p,d_sync_idx));
        assert(bid_s == bid_p);
        next_block_id = bid_s;
        next_in_block_idx= 0;
        next_phase_block_idx =0;
      }
      // state based method
      int count_s=0;
      int count_p=0;
      int next_state = d_state;
      int current_in_idx = d_in_idx;
      bool state_interrupt =false;
      std::list<block_t>::iterator it1 = d_smp_list.begin();
      std::list<std::pair<block_t,int> >::iterator it2 = d_sync_list.begin();
      std::list<hdr_t>::iterator hdr_it = d_tag_list.begin();
      switch(d_state){
        case FREE:
          for(count_s=0;count_s<nin_s;++count_s){
            assert(d_voe_state == false);
            update_voe_state(count_s);
            if(d_voe_state){
                next_state = SUFFERING;
                DEBUG<<"\033[33m"<<"<IC Crit>Detect interfering signals"<<"\033[0m"
                <<", block_id="<<d_current_block<<" ,block_idx="<<d_phase_block_idx
                <<" , d_in_idx="<<d_in_idx<<std::endl;
                init_intf();
                if(new_intf()){ 
                  d_current_intf_tag.add_msg(pmt::intern("voe_begin_idx"),pmt::from_long(d_intf_idx+count_s));
                  DEBUG<<"Initialize an interference object...d_intf_idx="<<d_intf_idx<<std::endl;
                }
                state_interrupt = true;
                break; // jump out from loop
            }
          }
        break;
        case SUFFERING:
          for(count_s=0;count_s <nin_s;++count_s){
            assert(d_voe_state == true);
            update_voe_state(count_s);
              if(!d_voe_state){
                reset_retx();
                next_state = SEARCH_RETX;
                DEBUG<<"\033[33m"<<"<IC Crit>Detect the end of interference"<<"\033[0m"
                <<" , block_id="<<d_current_block<<" ,block_idx="<<d_phase_block_idx
                <<" , d_in_idx="<<d_in_idx<<std::endl;
                state_interrupt = true;
                // FIXME
                if(!d_current_intf_tag.empty()){
                  d_current_intf_tag.add_msg(pmt::intern("voe_end_idx"),pmt::from_long(d_intf_idx+count_s));
                }
                break; // jump out from loop
              }
          }
        break;
        case SEARCH_RETX:
          for(count_s =0;count_s<nin_s;++count_s){
            // intf queue
            // add something to detect interference during retransmission process
            bool prev_state = d_voe_state;
            update_voe_state(count_s);
            if(d_voe_state && !prev_state){
                DEBUG<<"\033[33m"<<"<IC Crit> Detect an interfering event during retransmission state"<<"\033[0m"
                  <<" ,block_id="<<d_current_block<<" ,block_idx="<<d_phase_block_idx<<std::endl;
                if(d_current_intf_tag.empty()){
                  //already store at least one intf_t
                  if(new_intf()){
                    d_current_intf_tag.add_msg(pmt::intern("voe_begin_idx"),pmt::from_long(d_intf_idx+count_s));
                    DEBUG<<"<IC Crit> new intferference object Created"<<std::endl;
                  }
                }
            }else if(!d_voe_state && prev_state){
              if(!d_current_intf_tag.empty()){
                d_current_intf_tag.add_msg(pmt::intern("voe_end_idx"),pmt::from_long(d_intf_idx+count_s));
              }
                DEBUG<<"\033[33m"<<"<Ic Crit> Interfering event during retransmission ends"<<"\033[0m"
                <<" ,block_id="<<d_current_block<<" ,phase_block_idx="<<d_phase_block_idx<<std::endl;
            }
          }
        break;
        default:
          throw std::runtime_error("Undefined state");
        break;
      }
      for(int i=0;i<count_s;++i){
        d_in_mem[d_in_idx++] = in[i];
        d_in_idx %=d_cap;
        if(it1!=d_smp_list.end()){
          if(d_in_idx == it1->index()){
            // wrap around detected
            it1 = d_smp_list.erase(it1);
          }
        }
        // index check for detected preambles
        if(hdr_it != d_tag_list.end()){
          if(d_in_idx == hdr_it->index()){
            hdr_it = d_tag_list.erase(hdr_it);
          }
        }
        // intf signal collecting
        if(!d_current_intf_tag.empty() && d_current_intf_tag.back_tag_empty()){
          d_intf_mem[d_intf_idx++] = in[i];
          if(d_intf_idx==d_cap){
            DEBUG<<"\033[32m"<<"<IC Crit DEBUG> Additional interfering signal may cause overflow, clear tag..."<<"\033[0m"<<std::endl;
            d_intf_idx = d_current_intf_tag.begin();
            d_current_intf_tag.clear();
          }
        }
      }
      while(!cross_tags.empty()){
        int offset = cross_tags[0].offset - nitems_read(SAMPLE_PORT);
        if(offset<count_s){
          pmt::pmt_t tmp_msg = pmt::make_dict();
          // modify to sfd position, try to track back to begin of pkt,
          // since the difference of index is small, can greedyly assume exist...
          int idx_check = d_in_block_idx+offset - d_prelen;
          uint64_t block_check = d_current_block;
          while(idx_check<0){
            assert(block_check!=0);
            block_check = block_check-1;
            idx_check += d_block_size;
          }
          tmp_msg = pmt::dict_add(tmp_msg,pmt::intern("in_block_id"),pmt::from_uint64(block_check));
          tmp_msg = pmt::dict_add(tmp_msg,pmt::intern("in_offset"),pmt::from_long(idx_check));
          // this is sfd phase
          tmp_msg = pmt::dict_add(tmp_msg,pmt::intern("init_phase"),cross_tags[0].value);
          int samp_idx = current_in_idx+offset-d_prelen;
          while(samp_idx<0){
            samp_idx+=d_cap;
          }
          hdr_t tmp_hdr( (samp_idx)%d_cap,tmp_msg);
          //DEBUG<<"<IC Crit>Pending tag:"<<tmp_hdr<<std::endl;
          d_pending_list.push_back(tmp_hdr);
          cross_tags.erase(cross_tags.begin());
        }else{
          break;
        }
      }
      count_p = (state_interrupt)? std::min(nin_p,count_s) : nin_p;
      for(int i=0;i<count_p;++i){
        d_phase_mem[d_sync_idx] = phase[i];
        d_freq_mem[d_sync_idx] = freq[i];
        d_sync_idx = (d_sync_idx+1)%d_cap;
        if(it2!=d_sync_list.end()){
          int index = std::get<0>(*it2).index();
          if(d_sync_idx == index){
            // wrap around detected
            it2 = d_sync_list.erase(it2);
          }
        }
        // index check for header
      }
      // when state change, phase stream should follow count_s 
      // otherwise, just consume until a block tag found
      // handling tags
      hdr_t tmp_hdr;
      std::vector<hdr_t> new_tags;
      if(count_p!=0){
        get_tags_in_window(hdr_tags,PHASE_PORT,0,count_p,d_lsa_tag);
        for(int i=0;i<hdr_tags.size();++i){
          int offset = hdr_tags[i].offset - nitems_read(PHASE_PORT);
          std::vector<tag_t> other_tags;
          tmp_hdr.reset();
          tmp_hdr.init();
          tmp_hdr.set_index(d_phase_block_idx+offset); // offset only
          tmp_hdr.add_msg(pmt::intern("sync_block_id"),pmt::from_uint64(d_current_block));
          get_tags_in_window(other_tags,PHASE_PORT,offset,offset+1);
          for(int j=0;j<other_tags.size();++j){
            tmp_hdr.add_msg(other_tags[j].key,other_tags[j].value);
          }
          tmp_hdr.delete_msg(pmt::intern("pld_bytes"));
          tmp_hdr.delete_msg(pmt::intern("LSA_hdr"));
          new_tags.push_back(tmp_hdr);
        }
      }
      // tag size control
      for(int i=0;i<new_tags.size();++i){
        pmt::pmt_t temp_msg = new_tags[i].msg();
        int qsize = pmt::to_long(pmt::dict_ref(temp_msg,pmt::intern("queue_size"),pmt::from_long(-1)));
        bool valid_hdr = false;
        //DEBUG<<"<IC Crit>Raw header:"<<new_tags[i]<<std::endl;
        if(preprocess_hdr(new_tags[i])){
          d_tag_list.push_back(new_tags[i]);
          valid_hdr = true;
        }
        if(d_state == SEARCH_RETX && valid_hdr){
          if(qsize==0){
            if(!d_retx_tag.empty()){
              // retransmission ends
              d_tag_list.clear();
              // redo push back, for this new tag
              d_tag_list.push_back(new_tags[i]);
              d_reset_retx = true;
            }
          }else{
            if(d_do_ic == false)
              d_do_ic = detect_ic_chance(new_tags[i]);
          }
          if(update_intf()){
            // ready for one interference cancellation
            if(!d_current_intf_tag.empty())
              d_intf_stack.push_back(d_current_intf_tag);
            d_current_intf_tag.clear();
            DEBUG<<"<IC Crit>Interference object complete!"<<std::endl;
          }
        }
      }
      // increment block index
      if(d_do_ic){
        // do ic
        DEBUG<<"\033[31m"<<"<IC Crit>Calling DO IC "<<"\033[0m"<<std::endl;
        do_ic();
        reset_retx();
        d_tag_list.clear();
        next_state = FREE;
        d_voe_state = false;
        if(d_out_size!=0){
          add_item_tag(0,nitems_written(0),pmt::intern("ic_out"),pmt::PMT_T);
          add_item_tag(1,nitems_written(1),pmt::intern("ic_out"),pmt::PMT_T);
        }
      }
      if(d_reset_retx){
        DEBUG<<"\033[31m"<<"<IC Crit>Reset RETX, due to failure"<<"\033[0m"
        <<" ,expected:"<<d_retx_tag.size()<<" ,received:"<<d_retx_cnt<<std::endl;
        check_before_reset();
        if(!d_intf_stack.empty()){
          DEBUG<<"\033[31;1m"<<"<IC Crit>Available interfering object exists, calling do_ic before reset"
          <<"\033[0m"<<std::endl;
          do_ic();
        }
        reset_retx();
        next_state = FREE;
        d_voe_state = false;
        d_out_tags.clear();
      }
      // update indexes
      d_in_block_idx = next_in_block_idx;
      d_phase_block_idx = next_phase_block_idx;
      d_current_block = next_block_id;
      d_state = next_state;
      
      nout = std::min(d_out_size-d_out_idx,noutput_items);
      while(!d_out_tags.empty()){
        int offset = d_out_tags[0].offset;
        if(offset>=d_out_idx && offset<d_out_idx+nout){
          add_item_tag(0,nitems_written(0),d_out_tags[0].key,d_out_tags[0].value);
          d_out_tags.erase(d_out_tags.begin());
        }else if(offset>=d_out_idx+nout){
          break;
        }else{
          // Debugging tags, may have some mismatch
          d_out_tags.erase(d_out_tags.begin());
        }
      }
      memcpy(out,d_out_mem+d_out_idx,sizeof(gr_complex)*nout);
      // For DEMO
      memcpy(demo,d_comp_mem+d_out_idx,sizeof(gr_complex)*nout);
      d_out_idx += nout;
      if(d_out_idx == d_out_size){
        d_out_idx=0;
        d_out_size=0;
        d_out_tags.clear();
      }
      consume(SAMPLE_PORT,count_s);
      consume(PHASE_PORT,count_p);
      consume(FREQ_PORT,count_p);
      return nout;
    }

  } /* namespace lsa */
} /* namespace gr */

