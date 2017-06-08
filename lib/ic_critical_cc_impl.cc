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

  enum VOESTATE{
    FREE,
    SUFFERING,
    SEARCH_RETX
  };

    static const pmt::pmt_t d_block_tag = pmt::intern("block_tag");
    static const pmt::pmt_t d_voe_tag = pmt::intern("voe_tag");
    static const int d_voe_min = 128;
    static const int d_max_pending =256;
    static const int d_min_process = 16;
    static const int LSAPHYSYMBOLLEN = PHYLEN*8*LSACODERATEINV/2;

    inline void phase_wrap(float& phase){
      while(phase>=TWO_PI)
        phase-=TWO_PI;
      while(phase<=-TWO_PI)
        phase+=TWO_PI;
    }

    ic_critical_cc::sptr
    ic_critical_cc::make(int cross_len, int sps,bool debug)
    {
      return gnuradio::get_initial_sptr
        (new ic_critical_cc_impl(cross_len,sps,debug));
    }

    /*
     * The private constructor
     */
    static int ios[] = {sizeof(gr_complex),sizeof(float),sizeof(float)};
    static std::vector<int> iosig(ios,ios+sizeof(ios)/sizeof(int));
    ic_critical_cc_impl::ic_critical_cc_impl(int cross_len,int sps,bool debug)
      : gr::block("ic_critical_cc",
              gr::io_signature::makev(3, 3, iosig),
              gr::io_signature::make2(2, 2, sizeof(gr_complex),sizeof(gr_complex))),
              d_cap(MAXCAP)
    {
      d_debug = debug;
      d_in_mem = new gr_complex[d_cap];
      d_out_mem= new gr_complex[d_cap];
      d_comp_mem = new gr_complex[d_cap];
      d_phase_mem = new float[d_cap];
      d_freq_mem= new float[d_cap];
      d_in_idx = 0;
      d_sync_idx =0;
      d_out_idx =0;
      d_out_size =0;
      d_state = FREE;
      d_voe_state = false;
      //d_voe_cnt =0;

      d_retx_mem = new gr_complex[d_cap];
      d_intf_mem = new gr_complex[d_cap];
      d_intf_idx =0;
      d_intf_freq = (float*) volk_malloc(sizeof(float)*d_cap,volk_get_alignment());

      if(cross_len<=0){
        throw std::invalid_argument("Invalid cross length");
      }
      d_cross_len = cross_len;
      if(sps<=0){
        throw std::invalid_argument("Invalid samples per symbol");
      }
      d_sps = sps;
      reset_retx();
      //set_threshold(thres);
      //set_tag_propagation_policy(TPP_DONT);
    }

    /*
     * Our virtual destructor.
     */
    ic_critical_cc_impl::~ic_critical_cc_impl()
    {
      delete [] d_in_mem;
      delete [] d_out_mem;
      delete [] d_phase_mem;
      delete [] d_freq_mem;
      delete [] d_retx_mem;
      delete [] d_intf_mem;
      delete [] d_comp_mem;
      volk_free(d_intf_freq);
    }

    /*void
    ic_critical_cc_impl::set_threshold(float thres)
    {
      d_threshold = thres;
    }
    float
    ic_critical_cc_impl::threshold()const
    {
      return d_threshold;
    }*/

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
          //DEBUG<<"<IC Crit>Retransmission queue size maximum found:"<<max_qsize<<std::endl;
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
          uint64_t base=pmt::to_uint64(pmt::dict_ref(new_tag.msg(),pmt::intern("base"),pmt::from_uint64(0xffffffffffff)));
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
          //for(int i =0;i<d_retx_tag.size();++i){
            //DEBUG<<d_retx_tag[i]<<std::endl;
          //}
          //DEBUG<<"------------------------------------------"<<std::endl;
          return true;
        }
      }
      return false;
    }

    bool
    ic_critical_cc_impl::check_and_copy_retx(hdr_t& tag)
    {
      pmt::pmt_t msg = tag.msg();
      uint64_t block_id = pmt::to_uint64(pmt::dict_ref(msg,pmt::intern("block_id"),pmt::from_uint64(0xffffffffffff)));
      int block_offset = tag.index();
      int payload = pmt::to_long(pmt::dict_ref(msg,pmt::intern("payload"),pmt::from_long(-1)));
      int pkt_nominal = (payload+d_cross_len + LSAPHYSYMBOLLEN)*d_sps;
      const int reserved_length = 8*d_sps;
      std::list<block_t>::reverse_iterator bit_s = d_smp_list.rbegin(),bit_p = d_sync_list.rbegin();
      while(bit_s!=d_smp_list.rend()){
        if(bit_s->id()==block_id){
          break;
        }
        bit_s++;
      }
      while(bit_p!=d_sync_list.rend()){
        if(bit_p->id()==block_id){
          break;
        }
        bit_p++;
      }
      if(bit_s == d_smp_list.rend() || bit_p == d_sync_list.rend()){
        return false;
      }
      int begin = d_in_idx;
      int end = (d_in_idx==0)? d_cap-1: d_in_idx-1;
      if(end<begin)
        end+=d_cap;
      int index_check = (bit_s->index()<begin)? bit_s->index()+d_cap : bit_s->index();
      index_check+=block_offset;
      if( (index_check-pkt_nominal+1 < begin) || (index_check+reserved_length-1>end) ){
        return false;
      }
      int begin_p = d_sync_idx;
      int end_p = (d_sync_idx==0)? d_cap-1 : d_sync_idx-1;
      if(end_p<begin_p)
        end_p+=d_cap;
      int sync_idx_check = (bit_p->index()<begin_p)? bit_p->index() +d_cap : bit_p->index();
      sync_idx_check += block_offset;
      if( (sync_idx_check-pkt_nominal+1 < begin_p)|| (sync_idx_check+reserved_length-1>end_p) ){
        return false;
      }
      // valid retransmission size
      // NOTE: can add additional check
      // VoE of freq tracking signal
      // float mean, stddev
      // volk()...
      int samp_idx = index_check - pkt_nominal+1;
      samp_idx = (samp_idx>=d_cap)? samp_idx-d_cap : samp_idx;
      int sync_idx = sync_idx_check -pkt_nominal+1;
      sync_idx = (sync_idx>=d_cap)? sync_idx-d_cap : sync_idx;
      float init_phase = d_phase_mem[sync_idx];
      float init_freq = d_freq_mem[sync_idx];
      int copy_len = pkt_nominal+reserved_length;
      int samp_iter = samp_idx;
      int sync_iter = sync_idx;
      int retx_idx_begin = d_retx_idx;
      if(d_retx_idx+copy_len>=d_cap){
        throw std::runtime_error("<IC Crit>Fatal Error: Retransmission buffer is too small to acquire all retransmission....");
      }
      for(int i=0;i<copy_len;++i){
        d_retx_mem[d_retx_idx++] = d_in_mem[samp_iter++] * gr_expj(-d_phase_mem[sync_iter++]);
        //d_retx_mem[d_retx_idx++] = d_in_mem[samp_iter++] * gr_expj(-init_phase);
        //init_phase+=init_freq;
        //phase_wrap(init_phase);
        //init_freq = d_freq_mem[sync_iter++];
        samp_iter%=d_cap;
        sync_iter%=d_cap;
        if(d_retx_idx == d_cap){
          throw std::runtime_error("<IC Crit DEBUG>Fatal error: retransmission size is not enough... abort");
        }
      }
      tag.set_index(retx_idx_begin);
      tag.add_msg(pmt::intern("copy_len"),pmt::from_long(copy_len));
      tag.add_msg(pmt::intern("packet_len"),pmt::from_long(pkt_nominal));
      // tag.add_msg(pmt::intern("freq_mean"),pmt::from_float(mean));
      // tag.add_msg(pmt::intern("freq_std"),pmt::from_float(std));
      tag.delete_msg(pmt::intern("queue_index"));
      tag.delete_msg(pmt::intern("queue_size"));
      tag.delete_msg(pmt::intern("block_id"));
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
        //DEBUG<<"<IC Crit DEBUG>Not an empty interference tag"<<std::endl;
        return false;
      }
      std::list<hdr_t>::reverse_iterator rit = d_tag_list.rbegin();
      std::list<block_t>::iterator bit_s,bit_p;
      int block_offset;
      int pkt_nominal;
      while(rit!=d_tag_list.rend()){
        pmt::pmt_t msg = rit->msg();
        uint64_t id = pmt::to_uint64(pmt::dict_ref(msg,pmt::intern("block_id"),pmt::from_uint64(0xffffffffffff)));
        uint32_t idx= rit->index();
        int payload = pmt::to_long(pmt::dict_ref(msg,pmt::intern("payload"),pmt::from_long(-1)));
        pkt_nominal = (payload+LSAPHYSYMBOLLEN+d_cross_len) *d_sps;
        bit_s =d_smp_list.begin();
        bit_p =d_sync_list.begin();
        while(bit_s!=d_smp_list.end()){
          if(bit_s->id()==id){
            break;
          }
          bit_s++;
        }
        while(bit_p!=d_sync_list.end()){
          if(bit_p->id()==id){
            break;
          }
          bit_p++;
        }
        if(bit_p!=d_sync_list.end() && bit_s!=d_smp_list.end() && id!=d_current_block){
          // both sync and sample have matched block index
          // the header belongs to previous block
          block_offset = idx;
          break;
        }
        rit++;
      }
      if(rit==d_tag_list.rend() || bit_s==d_smp_list.end() || bit_p == d_sync_list.end()){
        // failed
        //DEBUG<<"<IC Crit>new_intf::no available front tag found, end function"<<std::endl;
        return false;
      }
      // but notice the wrap around point!!
      int current_begin = d_in_idx;
      int current_end = (d_in_idx==0)? d_cap-1 : d_in_idx-1;
      if(current_begin>current_end){
        current_end+=d_cap;
      }
      int samp_base = (bit_s->index()<current_begin)? bit_s->index()+d_cap : bit_s->index();
      if(samp_base + block_offset-pkt_nominal+1<current_begin){
        //DEBUG<<"<IC Crit>new_intf::signal already removed..."<<std::endl;
        return false;
      }
      int samp_idx = bit_s->index()+block_offset - pkt_nominal+1;
      // +1-1 cancel the last term
      int length_check = current_end - (samp_base + block_offset- pkt_nominal) ;
      if(length_check+d_intf_idx >=d_cap){
        DEBUG<<"<IC Crit>loading history will make buffer overflow"<<std::endl;
        return false;
      }
      int intf_begin = d_intf_idx;
      samp_idx = (samp_idx<0)? samp_idx+d_cap: samp_idx;
      while(samp_idx!=d_in_idx){
        d_intf_mem[d_intf_idx++] = d_in_mem[samp_idx++];
        samp_idx %= d_cap;
      }
      // for phase stream
      current_begin = d_sync_idx;
      current_end = (d_sync_idx==0)? d_cap-1 : d_sync_idx-1;
      if(current_begin>current_end){
        current_end+=d_cap;
      }
      float end_phase = d_phase_mem[bit_p->index()+block_offset];
      //float end_freq = d_freq_mem[bit_p->index()+block_offset];
      int sync_idx = bit_p->index()+block_offset - pkt_nominal+1;
      int sync_base = (bit_p->index()<current_begin)? bit_p->index()+d_cap : bit_p->index();
      if( (sync_base + block_offset -pkt_nominal+1) <current_begin){
        //DEBUG<<"<IC Crit>new_intf:: the begin of packet already been removed(sync)...end function"<<std::endl;
        return false;
      }
      float clean_phase  = d_phase_mem[sync_idx];
      int count =0;
      while(sync_idx!=d_sync_idx){
        // FOR DEBUG
        //if(count<d_intf_idx){
          //d_intf_mem[count]*= gr_expj(-d_phase_mem[sync_idx]);
        //}
        d_intf_freq[count++] = d_freq_mem[sync_idx++];
        sync_idx%=d_cap;
      }
      float mean,stddev;
      volk_32f_stddev_and_mean_32f_x2(&stddev,&mean,d_intf_freq,count);
      // recored begin idx
      // avg freq?
      hdr_t new_front = *rit;
      new_front.add_msg(pmt::intern("freq_mean"),pmt::from_float(mean));
      new_front.add_msg(pmt::intern("freq_std"),pmt::from_float(stddev));
      new_front.add_msg(pmt::intern("phase_init"),pmt::from_float(clean_phase));
      new_front.add_msg(pmt::intern("phase_end"),pmt::from_float(end_phase));
      //new_front.add_msg(pmt::intern("freq_end"),pmt::from_float(end_freq));
      // record clean phase?
      d_current_intf_tag.set_begin(intf_begin);
      d_current_intf_tag.set_front(new_front);
      return true;
    }

    bool
    ic_critical_cc_impl::update_intf(int& residual)
    {
      if(d_current_intf_tag.empty() || !d_current_intf_tag.back_tag_empty()){
        //DEBUG<<"<IC Crit DEBUG>current intf tag is empty or end tag already found... abort"<<std::endl;
        return false;
      }
      // guarantee for new tags
      // just need to check the last one
      // available info
      // d_block_idx <--sync to phase offset
      // d_current_block <--burrent processing block, which is this tag belongs
      std::list<hdr_t>::reverse_iterator rit = d_tag_list.rbegin();
      std::list<block_t>::iterator bit_s,bit_p;
      int block_offset=0;
      int nominal_pkt =0;
      pmt::pmt_t msg = rit->msg();
      uint64_t id = pmt::to_uint64(pmt::dict_ref(msg,pmt::intern("block_id"),pmt::from_uint64(0xffffffffffff)));
      int32_t idx = rit->index();
      int payload = pmt::to_long(pmt::dict_ref(msg,pmt::intern("payload"),pmt::from_long(-1)));
      nominal_pkt = (payload+LSAPHYSYMBOLLEN+d_cross_len)*d_sps;
      bit_s = d_smp_list.begin();
      bit_p = d_sync_list.begin();
      while(bit_s!=d_smp_list.end()){
        if(id==bit_s->id()){
          break;
        }
        bit_s++;
      }
      while(bit_p!=d_sync_list.end()){
        if(id==bit_p->id()){
          break;
        }
        bit_p++;
      }
      if(bit_s==d_smp_list.end() || bit_p==d_sync_list.end() || rit == d_tag_list.rend()){
        // failed
        //DEBUG<<"<IC Crit>update_intf::no available end tag found, end function"<<std::endl;
        return false;
      }
      block_offset = idx;
      int current_begin = d_in_idx;
      int current_end = (d_in_idx==0)? d_cap-1 : d_in_idx-1;
      current_end = (current_end<current_begin)? current_end+d_cap: current_end;
      int intf_samp_check = bit_s->index();
      int end_idx_corrected;
      if(intf_samp_check<current_begin){
        intf_samp_check+=d_cap;
      }
      if(intf_samp_check+block_offset<=current_end){
        //already stored
        residual =0;
        end_idx_corrected = d_intf_idx-1;
      }else if(intf_samp_check+block_offset-current_end+1<=residual){
        residual = intf_samp_check + block_offset -current_end+1;
        end_idx_corrected = d_intf_idx+residual-1;
      }else{
        DEBUG<<"<IC Crit>update_intf:: not already stored nor will be stored in current samples...end function"<<std::endl;
        return false;
      }
      // success to record a valid header after interference occur
      current_begin = d_sync_idx;
      current_end = (d_sync_idx==0)? d_cap-1 : d_sync_idx-1;
      if(current_begin>current_end){
        current_end+=d_cap;
      }
      int sync_base = (bit_p->index()<current_begin)? bit_p->index()+d_cap:bit_p->index();
      int sync_idx = bit_p->index()+block_offset - nominal_pkt+1;
      sync_idx = (sync_idx<0)? sync_idx+d_cap:sync_idx;
      if( (sync_base+block_offset-nominal_pkt+1) <current_begin){
        //sync_idx = current_begin;
        return false;
      }
      //float end_phase = d_phase_mem[bit_p->index()+block_offset];
      //float begin_phase = d_phase_mem[sync_idx];
      //float end_freq = d_freq_mem[sync_idx];
      int count =0;
      while(sync_idx!=d_sync_idx){
        d_intf_freq[count++] = d_freq_mem[sync_idx++];
        sync_idx%=d_cap;
      }
      float mean,stddev;
      volk_32f_stddev_and_mean_32f_x2(&stddev,&mean,d_intf_freq,count);
      // fix intf_size
      // or ignore, and do ic from begin tag
      // set end tag
      // set size
      hdr_t new_back = *rit;
      new_back.add_msg(pmt::intern("freq_mean"),pmt::from_float(mean));
      new_back.add_msg(pmt::intern("freq_std"),pmt::from_float(stddev));
      //new_back.add_msg(pmt::intern("phase_init"),pmt::from_float(begin_phase));
      //new_back.add_msg(pmt::intern("phase_end"),pmt::from_float(end_phase));
      //new_back.add_msg(pmt::intern("freq_end"),pmt::from_float(end_freq));
      d_current_intf_tag.set_back(new_back);
      // NOTE:
      // due to difficulty in handling state transition and header tags
      // the interfering samples may exceed required.
      // trim those not neccessary 
      d_current_intf_tag.set_end(end_idx_corrected);
      // average of the freq
      // and record the end phase
      //DEBUG<<"Interfering tag added:"<<std::endl
      //<<d_current_intf_tag<<std::endl;
      return true;
    }

    void
    ic_critical_cc_impl::do_ic()
    {
      // ic core here.
      std::map<uint64_t,int> base_map;
      std::map<uint64_t,int>::iterator map_it;
      uint64_t base;
      for(int i=0;i<d_retx_tag.size();++i){
        pmt::pmt_t msg = d_retx_tag[i].msg();
        base = pmt::to_uint64(pmt::dict_ref(msg,pmt::intern("base"),pmt::from_uint64(0xffffffffffff)));
        base_map.insert(std::pair<uint64_t,int>(base,i));
      }
      for(int i=0;i<d_intf_stack.size();++i){
        hdr_t front = d_intf_stack[i].front();
        hdr_t back = d_intf_stack[i].back();
        int intf_begin = d_intf_stack[i].begin();
        int intf_end = d_intf_stack[i].end();
        DEBUG<<"Processing intf tag<"<<i<<">:"<<std::endl
        <<" front tag="<<front<<std::endl
        <<" back tag="<<back<<std::endl;
        DEBUG<<"Intf indexes: begin="<<intf_begin<<", end="<<intf_end<<std::endl;
        pmt::pmt_t front_msg = front.msg();
        uint64_t front_base = pmt::to_uint64(pmt::dict_ref(front_msg,pmt::intern("base"),pmt::from_uint64(0xfffffffffffe)));
        // FIXME
        // this tag is added at the end of signal
        // Take this into consideration!!
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
        float init_phase = pmt::to_float(pmt::dict_ref(front_msg,pmt::intern("phase_init"),pmt::from_float(0)));
        float freq_mean = pmt::to_float(pmt::dict_ref(front_msg,pmt::intern("freq_mean"),pmt::from_float(0)));
        int intf_cnt = intf_begin;

        int debug_retx_id = (retx_id+1)%d_retx_tag.size();
        int debug_retx_idx = d_retx_tag[debug_retx_id].index();
        while(intf_cnt<intf_end){
          //NOTE: sync to interfering signal
          if(intf_cnt>=d_cap){
            throw std::runtime_error("inft cnt exceed maximum, boom");
          }
          if(retx_idx>=d_cap){
            throw std::runtime_error("retx idx exceed maximum, boom");
          }
          // for DEMO
          d_comp_mem[d_out_size] = d_retx_mem[debug_retx_idx++]-d_retx_mem[retx_idx];
          //d_comp_mem[d_out_size] = d_intf_mem[intf_cnt]*gr_expj(-init_phase);
          //d_comp_mem[d_out_size] = d_retx_mem[retx_idx];
          d_out_mem[d_out_size++] = d_intf_mem[intf_cnt++] - d_retx_mem[retx_idx++]*gr_expj(init_phase);
          // FOR DEBUG
          //d_out_mem[d_out_size++] = d_intf_mem[intf_cnt++];
          //retx_idx++;
          //d_out_mem[d_out_size++] = d_intf_mem[intf_cnt++] - d_retx_mem[retx_idx++];
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
            // freq_mean
            debug_retx_id = (retx_id+1)%d_retx_tag.size();
            debug_retx_idx= d_retx_tag[debug_retx_id].index();
            //DEBUG<<"Update retx: index="<<retx_id<<" pkt_nominal="<<pkt_len<<std::endl
            //<<"msg:"<<retx_msg<<std::endl;
          }
          if(d_out_size==d_cap){
            DEBUG<<"\033[32m"
            <<"<IC Crit DEBUG>Warning: output size reach capacity, reset index and size, some samples may be overwritten"
            <<"\033[0m"<<std::endl;
            d_out_idx=0;
            d_out_size=0;
          }
        }
        DEBUG<<"\033[35;1m"<<"<IC Crit>Produced output samples:"<<d_out_size<<"\033[0m"<<std::endl;
        // creat output object?
      }
    }
    void
    ic_critical_cc_impl::update_voe_state(int idx)
    {
      const uint64_t nread = nitems_read(0);
      while(d_voe_tags.empty()){
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
      const gr_complex *in = (const gr_complex *) input_items[0];
      //const float* voe = (const float*) input_items[1];
      const float* phase= (const float*) input_items[2];
      const float* freq = (const float*) input_items[3];
      gr_complex *out = (gr_complex *) output_items[0];
      // For DEMO
      gr_complex *demo= (gr_complex *) output_items[1];
      int nout = 0;
      int nin_s = std::min(ninput_items[0],ninput_items[1]);
      int nin_p = std::min(ninput_items[2],ninput_items[3]);
      bool d_do_ic = false;
      bool d_reset_retx=false;
      std::vector<tag_t> tags_s;
      std::vector<tag_t> tags_p;
      std::vector<tag_t> hdr_tags;
      d_voe_tags.clear(); //
      get_tags_in_window(tags_s,0,0,nin_s,d_block_tag);
      get_tags_in_window(tags_p,2,0,nin_p,d_block_tag);
      get_tags_in_window(d_voe_tags,0,0,nin_s,d_voe_tag); //
      if(!tags_s.empty()){
        nin_s = tags_s[0].offset-nitems_read(0);
      }
      if(!tags_p.empty()){
        nin_p = tags_p[0].offset-nitems_read(2);
      }
      // state based method
      int count_s=0;
      int count_p=0;
      int next_state = d_state;
      bool state_interrupt =false;
      std::list<block_t>::iterator it1 = d_smp_list.begin(),it2 = d_sync_list.begin();
      switch(d_state){
        case FREE:
          for(count_s=0;count_s<nin_s;++count_s){
            // ring queue
            d_in_mem[d_in_idx++] = in[count_s];
            d_in_idx %=d_cap;
            if(it1!=d_smp_list.end()){
              if(d_in_idx == it1->index()){
                // wrap around detected
                it1 = d_smp_list.erase(it1);
              }
            }
            assert(d_voe_state == false);
            update_voe_state(count_s);
            if(d_voe_state){
                next_state = SUFFERING;
                DEBUG<<"\033[33m"<<"<IC Crit>Detect interfering signals"<<"\033[0m"
                <<", block_id="<<d_current_block<<" ,block_idx="<<d_block_idx<<std::endl;
                init_intf();
                if(new_intf()){ 
                  //DEBUG<<"Initialize an interference object..."<<std::endl;
                }
                state_interrupt = true;
                break; // jump out from loop
            }
          }
        break;
        case SUFFERING:
          for(count_s=0;count_s <nin_s;++count_s){
            // ring queue
            d_in_mem[d_in_idx++] = in[count_s];
            d_in_idx %=d_cap;
            if(it1!=d_smp_list.end()){
              if(d_in_idx == it1->index()){
                // wrap around detected
                it1 = d_smp_list.erase(it1);
              }
            }
            // intf queue
            d_intf_mem[d_intf_idx++] = in[count_s];
            if(d_intf_idx==d_cap){
              DEBUG<<"\033[32m"<<"<IC Crit DEBUG> Additional interfering signal may cause overflow"<<"\033[0m"<<std::endl;
              state_interrupt = true;
              next_state = FREE;
              //d_voe_cnt =0;
              d_voe_state = false;
              break;
            }
            assert(d_voe_state == true);
            update_voe_state(count_s);
              if(!d_voe_state){
                reset_retx();
                next_state = SEARCH_RETX;
                DEBUG<<"\033[33m"<<"<IC Crit>Detect the end of interference"<<"\033[0m"
                <<" , block_id="<<d_current_block<<" ,block_idx="<<d_block_idx<<std::endl;
                state_interrupt = true;
                break; // jump out from loop
              }  
          }
        break;
        case SEARCH_RETX:
          for(count_s =0;count_s<nin_s;++count_s){
            //ring queue
            d_in_mem[d_in_idx++] = in[count_s];
            d_in_idx %=d_cap;
            if(it1!=d_smp_list.end()){
              if(d_in_idx == it1->index()){
                // wrap around detected
                it1 = d_smp_list.erase(it1);
              }
            }
            // intf queue
            if(!d_current_intf_tag.empty() && d_current_intf_tag.back_tag_empty()){
              d_intf_mem[d_intf_idx++] = in[count_s];
              if(d_intf_idx==d_cap){
                DEBUG<<"\033[32m"<<"<IC Crit DEBUG> Additional interfering signal may cause overflow, clear tag..."<<"\033[0m"<<std::endl;
                d_intf_idx = d_current_intf_tag.begin();
                d_current_intf_tag.clear();
              }
            }
            // add something to detect interference during retransmission process
            bool prev_state = d_voe_state;
            update_voe_state(count_s);
            if(!d_voe_state && prev_state){
                DEBUG<<"\033[33m"<<"<IC Crit> Detect an interfering event during retransmission state"<<"\033[0m"
                  <<" ,block_id="<<d_current_block<<" ,block_idx="<<d_block_idx<<std::endl;
                if(d_current_intf_tag.empty()){
                  //already store at least one intf_t
                  if(new_intf()){
                    DEBUG<<"\033[35m"<<"<IC Crit> new intferference object Created"<<"\033[0m"<<std::endl;
                  }
                }
            }else if(d_voe_state && !prev_state){
                DEBUG<<"\033[33m"<<"<Ic Crit> Interfering event during retransmission ends"<<"\033[0m"
                <<" ,block_id="<<d_current_block<<" ,block_idx="<<d_block_idx<<std::endl;
            }
          }
        break;
        default:
          throw std::runtime_error("Undefined state");
        break;
      }
      count_p = (state_interrupt)? std::min(nin_p,count_s) : nin_p;
      for(int i=0;i<count_p;++i){
        d_phase_mem[d_sync_idx] = phase[i];
        d_freq_mem[d_sync_idx] = freq[i];
        d_sync_idx = (d_sync_idx+1)%d_cap;
        if(it2!=d_sync_list.end()){
          if(d_sync_idx == it2->index()){
            // wrap around detected
            it2 = d_sync_list.erase(it2);
          }
        }
      }
        // when state change, phase stream should follow count_s 
        // otherwise, just consume until a block tag found
      // handling tags
      hdr_t tmp_hdr;
      std::vector<hdr_t> new_tags;
      if(count_p!=0){
        get_tags_in_window(hdr_tags,2,0,count_p);
      for(int i=0;i<hdr_tags.size();++i){
        if(!pmt::eqv(d_block_tag,hdr_tags[i].key)){
          int offset = hdr_tags[i].offset - nitems_read(2);
          if(tmp_hdr.index() == (offset + d_block_idx) ){
            tmp_hdr.add_msg(hdr_tags[i].key,hdr_tags[i].value);
          }else{
            // insert new tag
            if(!tmp_hdr.empty()){
              tmp_hdr.delete_msg(pmt::intern("pld_bytes"));
              tmp_hdr.delete_msg(pmt::intern("LSA_hdr"));
              new_tags.push_back(tmp_hdr);
              tmp_hdr.reset();
              //DEBUG<<"<DEBUG>New tag:"<<tmp_hdr<<std::endl;
            }
            // new tag
            tmp_hdr.init();
            tmp_hdr.set_index(d_block_idx+offset);
            tmp_hdr.add_msg(hdr_tags[i].key,hdr_tags[i].value);
            tmp_hdr.add_msg(pmt::intern("block_id"),pmt::from_uint64(d_current_block));
          }
        }
      }
      if(!tmp_hdr.empty()){
        tmp_hdr.delete_msg(pmt::intern("pld_bytes"));
        tmp_hdr.delete_msg(pmt::intern("LSA_hdr"));
        new_tags.push_back(tmp_hdr);
        //DEBUG<<"<DEBUG>New tag:"<<tmp_hdr<<std::endl;
      }
      }
      // FIXME
      // not necessary anymore
      int residual = count_p;
      // tag size control
      for(int i=0;i<new_tags.size();++i){
        int qidx = pmt::to_long(pmt::dict_ref(new_tags[i].msg(),pmt::intern("queue_index"),pmt::from_long(-1)));
        int qsize= pmt::to_long(pmt::dict_ref(new_tags[i].msg(),pmt::intern("queue_size"),pmt::from_long(-1)));
        uint64_t base =pmt::to_uint64(pmt::dict_ref(new_tags[i].msg(),pmt::intern("base"),pmt::from_uint64(0xffffffffffff)));
        if(qidx<0 || qsize<0 || base==0xffffffffffff){
          continue;
        }else if(qsize!=0 && qidx>=qsize){
          continue;
        }
        switch(d_state){
          case FREE:
          d_tag_list.push_back(new_tags[i]);
            if(d_tag_list.size()>d_max_pending){
              while(d_tag_list.size()>d_max_pending/2){
                d_tag_list.pop_front();
              }
            }
          break;
          case SUFFERING:
          d_tag_list.push_back(new_tags[i]);
            if(d_tag_list.size()>d_max_pending){
              while(d_tag_list.size()>d_max_pending/2){
                d_tag_list.pop_front();
              }
            }
          break;
          case SEARCH_RETX:
            if(qsize==0){
              if(!d_retx_tag.empty()){
                // this means retransmission is end
                // declare retransmission failed
                d_tag_list.clear();
                // reset retransmission information
                d_reset_retx = true;
              }
            }else{
              d_do_ic = detect_ic_chance(new_tags[i]);
            }
            d_tag_list.push_back(new_tags[i]);
            //FIXME
            // same tag event for front and back tags may occur 
            if(update_intf(residual)){
              // ready for one interference cancellation block
              if(!d_current_intf_tag.empty())
                d_intf_stack.push_back(d_current_intf_tag);
              d_current_intf_tag.clear();
              DEBUG<<"\033[35m"<<"<IC Crit>Interference object complete!"<<"\033[0m"<<std::endl;
            }
          break;
          default:
            throw std::runtime_error("undefined state");
          break;
        }
      }
      // increment block index
      for(int i=0;i<count_p;++i){
        d_block_idx++;
      }
      if(d_do_ic){
        // do ic
        DEBUG<<"\033[31m"<<"<IC Crit>Calling DO IC "<<"\033[0m"<<std::endl;
        do_ic();
        reset_retx();
        d_tag_list.clear();
        next_state = FREE;
        //d_voe_cnt =0;
        d_voe_state = false;
        if(d_out_size!=0){
          add_item_tag(0,nitems_written(0),pmt::intern("ic_out"),pmt::PMT_T);
          add_item_tag(1,nitems_written(1),pmt::intern("ic_out"),pmt::PMT_T);
        }
          
      }
      if(d_reset_retx){
        //
        DEBUG<<"\033[31m"<<"<IC Crit>Reset RETX, due to failure"<<"\033[0m"<<std::endl;
        reset_retx();
        next_state = FREE;
        //d_voe_cnt =0;
        d_voe_state = false;
      }
      d_state = next_state;
      if(!tags_s.empty() && count_s==nin_s && !tags_p.empty() && count_p==nin_p){
        // next block tags is ready
        uint64_t bid_s = pmt::to_uint64(tags_s[0].value);
        uint64_t bid_p = pmt::to_uint64(tags_p[0].value);
        count_s++;
        count_p++;
        d_smp_list.push_back(block_t(bid_s,d_in_idx));
        d_sync_list.push_back(block_t(bid_p,d_sync_idx));
        d_block_idx = 0;
        d_current_block = bid_s;
      }
      nout = std::min(d_out_size-d_out_idx,noutput_items);
      memcpy(out,d_out_mem+d_out_idx,sizeof(gr_complex)*nout);
      // For DEMO
      memcpy(demo,d_comp_mem+d_out_idx,sizeof(gr_complex)*nout);
      d_out_idx += nout;
      if(d_out_idx == d_out_size){
        d_out_idx=0;
        d_out_size=0;
      }

      consume(0,count_s);
      consume(1,count_p);
      consume(2,count_p);
      return nout;
    }

  } /* namespace lsa */
} /* namespace gr */

