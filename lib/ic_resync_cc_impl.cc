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
#include "ic_resync_cc_impl.h"
#include <volk/volk.h>

namespace gr {
  namespace lsa {
    #define d_debug true
    #define DEBUG d_debug && std::cout
    #define CAPACITY 128*1024*1024
    #define FIRCAPACITY 256*128
    #define CHIPRATE 8
    #define MODBPS 2
    #define LSAPHYLEN 6
    static int d_sps = 4;
    static int d_prelen = 128; // symbols 16*8
    static int d_phylen = 192; // 0x00,0x00,0x00,0x00,0xe6,0xXX
    static int d_protect_len = 512;
    enum VOESTATE{
      VOE_CLEAR,
      VOE_TRIGGERED
    };
    enum ICSTATE{
      CLEAR,
      COLLECT,
      RESET
    };

    ic_resync_cc::sptr
    ic_resync_cc::make(const std::vector<float>& taps)
    {
      return gnuradio::get_initial_sptr
        (new ic_resync_cc_impl(taps));
    }

    /*
     * The private constructor
     */
    ic_resync_cc_impl::ic_resync_cc_impl(const std::vector<float>& taps)
      : gr::block("ic_resync_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(2, 2, sizeof(gr_complex))),
              d_in_port(pmt::intern("pkt_in")),
              d_cap(CAPACITY)
    {
      message_port_register_in(d_in_port);
      set_msg_handler(d_in_port,boost::bind(&ic_resync_cc_impl::msg_in,this,_1));
      d_fir_buffer = (gr_complex*)volk_malloc(sizeof(gr_complex)*FIRCAPACITY,volk_get_alignment());
      d_in_mem = (gr_complex*)volk_malloc(sizeof(gr_complex)*d_cap,volk_get_alignment());
      d_in_idx =0;
      d_out_idx=0;
      d_out_size=0;
      d_intf_idx=0;
      d_intf_mem = new gr_complex[d_cap];
      d_out_mem = new gr_complex[d_cap];
      d_demo_mem = new gr_complex[d_cap];
      d_state = VOE_CLEAR;
      d_intf_protect = false;
      d_protect_cnt=0;
      d_cur_intf.clear();
      d_intf_list.clear();
    }

    /*
     * Our virtual destructor.
     */
    ic_resync_cc_impl::~ic_resync_cc_impl()
    {
      volk_free(d_fir_buffer);
      volk_free(d_in_mem);
      delete [] d_intf_mem;
      delete [] d_out_mem;
      delete [] d_demo_mem;
    }

    bool
    ic_resync_cc_impl::voe_update(int idx)
    {
      bool result = false;
      if(!d_voe_tags.empty()){
        int offset = d_voe_tags[0].offset - nitems_read(0);
        if(offset == idx){
          result = pmt::to_bool(d_voe_tags[0].value);
          if(d_state == VOE_CLEAR && result){
            result= true;
          }else if(d_state == VOE_TRIGGERED && !result){
            result = true;
          }
          d_voe_tags.erase(d_voe_tags.begin());
        }
      }
      return result;
    }
    void
    ic_resync_cc_impl::system_update(int idx)
    {
      d_offset++;
      if(!d_sfd_list.empty()){
        if(std::get<0>(d_sfd_list.front())==idx){
          d_sfd_list.pop_front();
        }
      }
      if(!d_block_list.empty()){
        if(std::get<1>(d_block_list.front())==idx){
          d_block_list.pop_front();
        }
      }
      if(!d_pkt_history.empty()){
        if(d_pkt_history.front().index()==idx){
          d_pkt_history.pop_front();
        }
      }
    }
    void
    ic_resync_cc_impl::tags_update(int idx)
    {
      if(!d_block_tags.empty()){
        int offset = d_block_tags[0].offset-nitems_read(0);
        if(offset == idx){
          d_offset =0;
          d_block = pmt::to_uint64(d_block_tags[0].value);
          d_block_list.push_back(std::make_pair(d_block,d_in_idx));
          d_block_tags.erase(d_block_tags.begin());
        }
      }
      if(!d_sfd_tags.empty()){
        int offset = d_sfd_tags[0].offset-nitems_read(0);
        if(offset == idx){
          pmt::pmt_t sfd_msg = pmt::make_dict();
          sfd_msg = pmt::dict_add(sfd_msg,pmt::intern("init_phase"),d_sfd_tags[0].value);
          int idx_fix = d_in_idx-d_prelen*d_sps;
          idx_fix = (idx_fix<0)? idx_fix+d_cap : idx_fix;
          hdr_t sfd_hdr(idx_fix,sfd_msg);
          d_sfd_list.push_back(std::make_pair(idx_fix,sfd_hdr) );
          d_sfd_tags.erase(d_sfd_tags.begin());
          //DEBUG<<"<Resync>\033[33;1m"<<"found sfd at block:"<<d_block<<" ,offset:"<<d_offset<<"\033[0m"<<std::endl;
        }
      }
    }

    void
    ic_resync_cc_impl::msg_in(pmt::pmt_t msg)
    {
      gr::thread::scoped_lock guard(d_mutex);
      pmt::pmt_t k = pmt::car(msg);
      pmt::pmt_t v = pmt::cdr(msg);
      assert(pmt::is_dict(k));
      assert(pmt::is_blob(v));
      size_t io(0);
      const uint8_t* uvec = u8vector_elements(v,io);
      uint64_t block = pmt::to_uint64(pmt::dict_ref(k,pmt::intern("block_id"),pmt::from_uint64(0)));
      int offset = pmt::to_long(pmt::dict_ref(k,pmt::intern("offset"),pmt::from_long(0)));
      offset*= d_sps;
      int pktlen = (io+LSAPHYLEN)*CHIPRATE*8/MODBPS*d_sps;
      uint16_t base,base_crc, qsize,qidx;
      qidx = uvec[0]<<8;
      qidx|= uvec[1];
      qsize= uvec[2]<<8;
      qsize|=uvec[3];
      base = uvec[4]<<8;
      base|= uvec[5];
      base_crc=uvec[6]<<8;
      base_crc|=uvec[7];
      if( (qsize!=0 && qidx>=qsize) || (base!=base_crc) ){
        DEBUG<<"<Msg in>Low quality header!"<<std::endl;
        return; // low quality packets
      }
      hdr_t hdr;
      if(!pkt_validate(hdr,block,offset,pktlen, qidx,qsize,base)){
        // invalid
        DEBUG<<"<Msg in>Invalid packet!"<<std::endl;
        return;
      }
      // critical information added
      if(!matching_pkt(hdr)){
        // not matched
        DEBUG<<"<Msg in>Not matched with correlation tag!"<<std::endl;
        return;
      }
      d_pkt_history.push_back(hdr);
      retx_detector(qidx,qsize,base,v,pktlen);
      //intf_detector();
    }

    void
    ic_resync_cc_impl::retx_detector(uint16_t qidx,uint16_t qsize,uint16_t base,pmt::pmt_t blob,int pktlen)
    {
      if(qsize!=d_retx_stack.size()){
        d_retx_stack.clear();
        if(qsize==0){
          // reset signal
          d_retx_cnt=0;
        }else{
          // direct change of retransmission
          d_retx_stack.resize(qsize,std::make_tuple(0,pmt::PMT_NIL,0));
          d_retx_stack[qidx] = std::make_tuple(pktlen,blob,base);
          d_retx_cnt=1;
        }
      }else{
        if(qsize==0){
          // clean and also clean
          return;
        }
        // equal retransmissions
        if(pmt::is_null(std::get<1>(d_retx_stack[qidx]))){
          d_retx_stack[qidx] = std::make_tuple(pktlen,blob,base);
          d_retx_cnt++;
          //DEBUG<<"<RETX> found a new one: base="<<base<<" ,idx="<<qidx<<" ,count="<<d_retx_cnt<<" ,total="<<qsize<<std::endl;
        }
      }
    }

    void
    ic_resync_cc_impl::intf_detector()
    {
      if(d_retx_stack.empty()){
        return;
      }
      bool all_done = (d_retx_stack.size()==d_retx_cnt);
      std::list<intf_t>::iterator it;
      for(it=d_intf_list.begin();it!=d_intf_list.end();++it){
        bool do_ic = false;
        std::vector<int> idx_stack;
        // remove those intf that the front base not present in retransmission
        pmt::pmt_t front_msg = (*it).front().msg();
        uint16_t front_base = pmt::to_long(pmt::dict_ref(front_msg,pmt::intern("base"),pmt::from_long(-1)));
        //DEBUG<<"<INTF>search base="<<front_base<<std::endl;
        for(int i=0;i<d_retx_stack.size();++i){
          uint16_t retx_base = std::get<2>(d_retx_stack[i]);
          if(front_base == retx_base){
            // matched in base
            // check whether retransmission is enough for cancellation
            int total_size = (*it).size();
            int idx_iter = i;
            while(total_size>0){
              if(pmt::is_null(std::get<1>(d_retx_stack[idx_iter]))){
                break;
              }
              total_size-=std::get<0>(d_retx_stack[idx_iter]);
              idx_stack.push_back(idx_iter);
              idx_iter=(idx_iter+1)%d_retx_stack.size();
            }
            if(total_size<0){
              do_ic = true;
              break;
            }
          }
        }
        if(do_ic){
          // indication of ic avalability
          DEBUG<<"<INTF DETECTOR>An intf object ready to do ic!"<<std::endl;
          d_ic_list.push_back(std::make_pair(*it,idx_stack));
          // DO NOT remove to prevent cleaning up before ic
          //it = d_intf_list.erase(it);
        }else if(all_done){
          // outdated intf object
          DEBUG<<"<INTF DETECTOR>An outdated intf object removed!"<<std::endl;
          it = d_intf_list.erase(it);
        }
      }
    }

    bool
    ic_resync_cc_impl::pkt_validate(hdr_t& hdr,uint64_t bid, int offset, int pktlen,uint16_t qidx,uint16_t qsize,uint16_t base)
    {
      std::list< std::pair<uint64_t, int> >::reverse_iterator rit;
      for(rit = d_block_list.rbegin();rit!=d_block_list.rend();++rit){
        if(std::get<0>(*rit)==bid){
          break;
        }
      }
      if(rit==d_block_list.rend()){
        return false;
      }
      // bid and offset at PKTLEN, should track back to preamble
      int begin = (std::get<1>(*rit) + offset)%d_cap;
      for(int i=0;i<(d_prelen+16*2)*d_sps;++i){
        begin = (begin==0)? d_cap-1 : begin-1;
        if(begin==d_in_idx){
          DEBUG<<"<PKTVALID>failed at front"<<std::endl;
          return false;
        }
      }
      pmt::pmt_t msg = pmt::make_dict();
      msg = pmt::dict_add(msg,pmt::intern("packet_len"),pmt::from_long(pktlen));
      msg = pmt::dict_add(msg,pmt::intern("queue_index"),pmt::from_long(qidx));
      msg = pmt::dict_add(msg,pmt::intern("queue_size"),pmt::from_long(qsize));
      msg = pmt::dict_add(msg,pmt::intern("base"),pmt::from_long(base));
      hdr_t tmp_hdr(begin,msg);
      hdr = tmp_hdr;
      return true;
    }

    bool
    ic_resync_cc_impl::matching_pkt(hdr_t& hdr)
    {
      const int min_dis = 8192;
      std::list< std::pair<int,hdr_t> >::reverse_iterator rit;
      int idx = hdr.index();
      for(rit=d_sfd_list.rbegin();rit!=d_sfd_list.rend();rit++){
        int distance1 = std::abs(std::get<0>(*rit)-idx);
        int distance2 = std::abs(std::get<0>(*rit)+d_cap-idx);
        if(std::min(distance1,distance2)<=min_dis){
          //DEBUG<<"<Resync>\033[34;1mMatching packets: dist1="<<distance1<<" ,dist2="<<distance2<<"\033[0m"<<std::endl;
          // possible matched case
          // remove other headers?
          pmt::pmt_t tmp = (std::get<1>(*rit)).msg();
          hdr.set_index(std::get<1>(*rit).index());
          hdr.add_msg(pmt::intern("init_phase"),pmt::dict_ref(tmp,pmt::intern("init_phase"),pmt::from_float(0)));
          return true;
        }
      }
      return false;
    }

    bool
    ic_resync_cc_impl::create_intf()
    {
      if(!d_cur_intf.empty()){
        return false;
      }else if(d_intf_list.empty()){
        DEBUG<<"Empty intference object state, reset intf index..."<<std::endl;
        d_intf_idx=0;
      }
      std::list<hdr_t>::reverse_iterator rit=d_pkt_history.rbegin();
      if(rit==d_pkt_history.rend()){
        return false;
      }
      intf_t obj;
      obj.set_begin(d_intf_idx);
      obj.set_front((*rit));
      int pkt_begin= (*rit).index();
      int length=0;
      if(d_in_idx<pkt_begin){
        length = d_in_idx+d_cap-pkt_begin;
      }else{
        length = d_in_idx-pkt_begin;
      }
      if(d_intf_idx+length>=d_cap){
        return false;
      }
      while(pkt_begin!=d_in_idx){
        d_intf_mem[d_intf_idx++] = d_in_mem[pkt_begin++];
        pkt_begin%=d_cap;
      }
      obj.set_end(d_intf_idx-1);
      d_cur_intf= obj;
      return true;
    }

    void
    ic_resync_cc_impl::do_ic(std::pair<intf_t,std::vector<int> > obj)
    {
      int begin = std::get<0>(obj).begin();
      int size = std::get<0>(obj).size();
      std::vector<int> retx_idx = std::get<1>(obj);
      // result are stored in d_intf_idx[begin] up to d_intf_idx[begin+size-1];
      DEBUG<<"Calling do ic:"<<std::endl;
      DEBUG<<"front tag:"<<std::get<0>(obj).front()<<std::endl;
      DEBUG<<"intf_begin="<<begin<<" ,intf_size="<<size<<std::endl;
      DEBUG<<"Retransmissions:"<<std::endl;
      // required retransmissions 
      for(int i=0;i<retx_idx.size();++i){
        DEBUG<<"pkt_len:"<<std::get<0>(d_retx_stack[retx_idx[i]])<<" ,base:"<<std::get<2>(d_retx_stack[retx_idx[i]])<<std::endl;
        pmt::pmt_t blob = std::get<1>(d_retx_stack[retx_idx[i]]);
      }
      DEBUG<<"End of do ic"<<std::endl;
    }

    void
    ic_resync_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = noutput_items;
    }

    int
    ic_resync_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
      gr_complex *demo= (gr_complex *) output_items[1];
      int nin = ninput_items[0];
      int nout = 0;
      int count =0;
      d_voe_tags.clear();
      d_block_tags.clear();
      d_sfd_tags.clear();
      get_tags_in_window(d_block_tags,0,0,nin,pmt::intern("block_tag"));
      get_tags_in_window(d_voe_tags,0,0,nin,pmt::intern("voe_tag"));
      get_tags_in_window(d_sfd_tags,0,0,nin,pmt::intern("phase_est"));
      while(count<nin){
        switch(d_state)
        {
          case VOE_CLEAR:
            while(count<nin){
              tags_update(count);
              if(voe_update(count)){
                d_state = VOE_TRIGGERED;
                d_latest_voe_begin = d_in_idx;
                DEBUG<<"<Resync>\033[36;1m"<<"Detecting start of VoE signal...at:"<<d_in_idx<<"\033[0m"<<std::endl;
                if(create_intf()){
                  DEBUG<<"<Resync>\033[34;1m"<<"Create New Interference tag..."<<"\033[0m"<<std::endl;
                  d_intf_protect = false;
                }else{
                  // somthing wrong
                  DEBUG<<"<Resync>\033[34;1m"<<"Something wrong, clear intf obj..."<<"\033[0m"<<std::endl;
                  d_intf_idx = d_cur_intf.begin();
                  d_intf_protect=false;
                  d_protect_cnt=0;
                  d_cur_intf.clear();
                }
                break;
              }
              if(!d_cur_intf.front_tag_empty() && d_intf_protect){
                d_intf_mem[d_intf_idx++] = in[count];
                d_cur_intf.increment();
                d_protect_cnt++;
                if(d_protect_cnt==d_protect_len){
                  d_intf_protect = false;
                  d_protect_cnt=0;
                  DEBUG<<"<Resync>complete a intf tag,...total size="<<d_cur_intf.size()<<std::endl;
                  //DEBUG<<"front header:"<<d_cur_intf.front()<<std::endl;
                  d_intf_list.push_back(d_cur_intf);
                  d_cur_intf.clear();
                }
                if(d_intf_idx==d_cap){
                  d_intf_idx = d_cur_intf.begin();
                  d_cur_intf.clear();
                  d_intf_protect = false;
                  d_protect_cnt=0;
                }
              }
              d_in_mem[d_in_idx++] = in[count++];
              d_in_idx%=d_cap;
              system_update(d_in_idx);
            }
          break;
          case VOE_TRIGGERED:
            while(count<nin){
              tags_update(count);
              if(voe_update(count)){
                d_state = VOE_CLEAR;
                // should record the block and idx of the event for back tracking
                // change state to receive a end header
                DEBUG<<"<Resync>\033[36;1m"<<"Detecting end of VoE signal...at:"<<d_in_idx<<"\033[0m"<<std::endl;
                d_latest_voe_end = d_in_idx;
                d_intf_protect = true;
                d_protect_cnt=0;
                break;
              }
              if(!d_cur_intf.front_tag_empty()){
                d_intf_mem[d_intf_idx++] = in[count];
                d_cur_intf.increment();
                if(d_intf_idx==d_cap){
                  d_intf_idx = d_cur_intf.begin();
                  d_cur_intf.clear();
                }
              }
              d_in_mem[d_in_idx++] = in[count++];
              d_in_idx%=d_cap;
              system_update(d_in_idx);
            }
          break;
          default:
            throw std::runtime_error("Undefined state");
          break;
        }
      }
      //DEBUG required
      intf_detector();
      std::list<intf_t>::iterator intf_it;
      while(!d_ic_list.empty()){
        do_ic(d_ic_list.front());
        for(intf_it=d_intf_list.begin();intf_it!=d_intf_list.end();++intf_it){
          if((std::get<0>(d_ic_list.front())).begin() == (*intf_it).begin() ){
            intf_it = d_intf_list.erase(intf_it);
            break;
          }
        }
        d_ic_list.pop_front();
      }
      nout = std::min(noutput_items,std::max(d_out_size-d_out_idx,0));
      memcpy(out,d_out_mem+d_out_idx,sizeof(gr_complex)*nout);
      memcpy(demo,d_demo_mem+d_out_idx,sizeof(gr_complex)*nout);
      d_out_idx+=nout;
      consume_each (count);
      return nout;
    }

  } /* namespace lsa */
} /* namespace gr */

