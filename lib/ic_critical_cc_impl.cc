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

namespace gr {
  namespace lsa {

#define DEBUG d_debug && std::cout
#define PHYLEN 6
#define MAXLEN (127+PHYLEN)*8*8/2*4
#define MAXCAP 128*MAXLEN

  enum VOESTATE{
    FREE,
    SUFFERING,
    SEARCH_RETX
  };

    static const pmt::pmt_t d_block_tag = pmt::intern("block_tag");
    static const int d_voe_min = 128;
    static const int d_max_pending =256;
    static const int d_min_process = 16;

    ic_critical_cc::sptr
    ic_critical_cc::make(float thres,bool debug)
    {
      return gnuradio::get_initial_sptr
        (new ic_critical_cc_impl(thres,debug));
    }

    /*
     * The private constructor
     */
    static int ios[] = {sizeof(gr_complex),sizeof(float),sizeof(float),sizeof(float)};
    static std::vector<int> iosig(ios,ios+sizeof(ios)/sizeof(int));
    ic_critical_cc_impl::ic_critical_cc_impl(float thres,bool debug)
      : gr::block("ic_critical_cc",
              gr::io_signature::makev(4, 4, iosig),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
              d_cap(MAXCAP)
    {
      d_debug = debug;
      d_in_mem = new gr_complex[d_cap];
      d_out_mem= new gr_complex[d_cap];
      d_phase_mem = new float[d_cap];
      d_freq_mem= new float[d_cap];
      d_in_idx = 0;
      d_sync_idx =0;
      d_out_idx =0;
      d_out_size =0;
      d_state = FREE;
      d_voe_state = false;
      d_voe_cnt =0;

      d_retx_mem = new gr_complex[d_cap];
      reset_retx();
      set_threshold(thres);
      set_tag_propagation_policy(TPP_DONT);
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
    }

    void
    ic_critical_cc_impl::set_threshold(float thres)
    {
      d_threshold = thres;
    }
    float
    ic_critical_cc_impl::threshold()const
    {
      return d_threshold;
    }

    bool
    ic_critical_cc_impl::detect_ic_chance(const hdr_t& new_tag)
    {
      d_retx_candidate.push_back(new_tag);
      std::list<hdr_t>::reverse_iterator rit;
      std::map<uint32_t,int> count_map;
      std::map<uint32_t,int>::iterator map_it;
      //DEBUG<<"<IC Crit>Processing tag--"<<new_tag<<std::endl;
      if(d_retx_candidate.size()>=d_min_process){
        //DEBUG<<"<IC Crit>Processing Retransmission blocks"<<std::endl;
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
              d_retx_tag[tmp_qidx] = *rit;
              d_retx_cnt++;
              //DEBUG<<"<IC Crit>Retx tags handling---add idx:"<<tmp_qidx<<" ,expected size:"<<d_retx_tag.size()<<" ,current received:"<<d_retx_cnt<<std::endl;
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
            d_retx_tag[qidx]=new_tag;
            d_retx_cnt++;
            //DEBUG<<"<IC Crit>Retx tags handling---add idx:"<<qidx<<" ,expected size:"<<d_retx_tag.size()<<" ,current received:"<<d_retx_cnt<<std::endl;
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
      const float* voe = (const float*) input_items[1];
      const float* phase= (const float*) input_items[2];
      const float* freq = (const float*) input_items[3];
      gr_complex *out = (gr_complex *) output_items[0];
      int nout = 0;
      int nin_s = std::min(ninput_items[0],ninput_items[1]);
      int nin_p = std::min(ninput_items[2],ninput_items[3]);
      bool d_do_ic = false;
      bool d_reset_retx=false;
      std::vector<tag_t> tags_s;
      std::vector<tag_t> tags_p;
      std::vector<tag_t> hdr_tags;
      get_tags_in_window(tags_s,0,0,nin_s,d_block_tag);
      get_tags_in_window(tags_p,2,0,nin_p,d_block_tag);
      if(!tags_s.empty()){
        nin_s = tags_s[0].offset-nitems_read(0);
      }
      if(!tags_p.empty()){
        nin_p = tags_p[0].offset-nitems_read(2);
      }
      if(!tags_s.empty() && !tags_p.empty()){
        uint64_t bid_s = pmt::to_uint64(tags_s[0].value);
        uint64_t bid_p = pmt::to_uint64(tags_p[0].value);
        nin_s++;
        nin_p++;
        d_smp_list.push_back(block_t(bid_s,d_in_idx));
        d_sync_list.push_back(block_t(bid_p,d_sync_idx));
        d_block_idx = 0;
        d_current_block = bid_s;
        //DEBUG<<"<IC Crit>current block id="<<d_current_block<<std::endl;
      }
      get_tags_in_window(hdr_tags,2,0,nin_p);
      hdr_t tmp_hdr;
      std::vector<hdr_t> new_tags;
      for(int i=0;i<hdr_tags.size();++i){
        if(!pmt::eqv(d_block_tag,hdr_tags[i].key)){
          int offset = hdr_tags[i].offset - nitems_read(2);
          if(tmp_hdr.index() == (offset + d_block_idx) ){
            tmp_hdr.add_msg(hdr_tags[i].key,hdr_tags[i].value);
          }else{
            // insert new tag
            if(!tmp_hdr.empty()){
              new_tags.push_back(tmp_hdr);
              tmp_hdr.reset();
            }
            // new tag
            tmp_hdr.init();
            tmp_hdr.set_index(d_block_idx+offset);
            tmp_hdr.add_msg(hdr_tags[i].key,hdr_tags[i].value);
            tmp_hdr.add_msg(pmt::intern("block_id"),pmt::from_uint64(d_current_block));
            //tmp_hdr.add_msg(pmt::intern("nin_p"),pmt::from_long(nin_p));
          }
        }
      }
      if(!tmp_hdr.empty()){
        new_tags.push_back(tmp_hdr);
      }
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
        //DEBUG<<"<DEBUG> new tag--"<<i<<"--"<<new_tags[i]<<std::endl;
        switch(d_state){
          case FREE:
          d_tag_list.push_back(new_tags[i]);
            if(d_tag_list.size()>d_max_pending){
              for(int i=0;i<d_max_pending/2;++i){
                d_tag_list.pop_front();
              }
            }
          break;
          case SUFFERING:
          d_tag_list.push_back(new_tags[i]);
            if(d_tag_list.size()>d_max_pending){
              for(int i=0;i<d_max_pending/2;++i){
                d_tag_list.pop_front();
              }
            }
          break;
          case SEARCH_RETX:
            //do nothing
            if(qsize==0){
              if(!d_retx_tag.empty()){
                // this means retransmission is end
                // declare retransmission failed
                DEBUG<<"<IC Crit>Retransmission failed! (clear tag found)"<<std::endl;
                // reset retransmission information
                //d_state = FREE;
                d_reset_retx = true;
              }
              d_tag_list.push_back(new_tags[i]);
            }else{
              d_do_ic = detect_ic_chance(new_tags[i]);
            }
          break;
          default:
            throw std::runtime_error("undefined state");
          break;
        }
      }
      std::list<block_t>::iterator it1 = d_smp_list.begin(),it2 = d_sync_list.begin();
      for(int j=0;j<nin_p;++j){
        d_phase_mem[d_sync_idx] = phase[j];
        d_freq_mem[d_sync_idx] = freq[j];
        d_sync_idx = (d_sync_idx+1)%d_cap;
        if(it2!=d_sync_list.end()){
          if(d_sync_idx == it2->index()){
            // wrap around detected
            it2 = d_sync_list.erase(it2);
          }
        }
        d_block_idx++;
      }
      for(int i=0;i<nin_s;++i){
        switch(d_state){
          case FREE:
            assert(d_voe_state == false);
            if(voe[i]>d_threshold){
              d_voe_cnt++;
              if(d_voe_cnt>=d_voe_min){
                d_voe_state = true;
                d_voe_cnt=0;
                d_state = SUFFERING;
                DEBUG<<"<IC Crit>Detect interfering signals"<<std::endl;
              }
            }else{
              d_voe_cnt =0;
            }
          break;
          case SUFFERING:
            assert(d_voe_state == true);
            if(voe[i]<d_threshold){
              d_voe_cnt++;
              if(d_voe_cnt>=d_voe_min){
                d_voe_state = false;
                d_voe_cnt =0;
                reset_retx();
                d_state = SEARCH_RETX;
                DEBUG<<"<IC Crit>Detect the end of interference"<<std::endl;
              }
            }else{
              d_voe_cnt=0;
            }
          break;
          case SEARCH_RETX:
            // do nothing
            // FIXME
            // add something to detect interference during retransmission process
            if(!d_voe_state && voe[i]>d_threshold){
              d_voe_cnt++;
              if(d_voe_cnt>=d_voe_min){
                d_voe_state = true;
                d_voe_cnt =0;
                DEBUG<<"<IC Crit> Detect an interfering event during retransmission state"<<std::endl;
              }
            }else if(d_voe_state && voe[i]<d_threshold){
              d_voe_cnt++;
              if(d_voe_cnt>=d_voe_min){
                d_voe_state = false;
                d_voe_cnt=0;
                DEBUG<<"<Ic Crit> Interfering event during retransmission ends"<<std::endl;
              }
            }else{
              d_voe_cnt=0;
            }
          break;
          default:
            throw std::runtime_error("Undefined state");
          break;
        }
        d_in_mem[d_in_idx++] = in[i];
        d_in_idx%=d_cap;
        if(it1!=d_smp_list.end()){
          if(d_in_idx == it1->index()){
            // wrap around detected
            it1 = d_smp_list.erase(it1);
          }
        }
      }
      if(d_do_ic){
        // do ic
        DEBUG<<"<IC Crit>Calling DO IC "<<std::endl;
        reset_retx();
        d_state = FREE;
      }
      if(d_reset_retx){
        //
        DEBUG<<"<IC Crit>Reset RETX, due to failure"<<std::endl;
        reset_retx();
        d_state = FREE;  
      }
      nout = d_out_size-d_out_idx;
      memcpy(out,d_out_mem,sizeof(gr_complex)*nout);
      d_out_idx += nout;

      consume(0,nin_s);
      consume(1,nin_s);
      consume(2,nin_p);
      consume(3,nin_p);
      return nout;
    }

  } /* namespace lsa */
} /* namespace gr */

