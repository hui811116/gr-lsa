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
    enum VOESTATE{
      VOE_CLEAR,
      VOE_TRIGGERED
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
        DEBUG<<"<Low quality>"<<std::endl;
        return; // low quality packets
      }
      hdr_t hdr;
      if(!pkt_validate(hdr,block,offset,pktlen, qidx,qsize,base)){
        // invalid
        DEBUG<<"<Invalid packet>"<<std::endl;
        return;
      }
      // critical information added
      if(!matching_pkt(hdr)){
        // not matched
        DEBUG<<"<Not matched>"<<std::endl;
        return;
      }
      d_pkt_history.push_back(hdr);
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
        // still finding end tag
        return false;
      }
      std::list<hdr_t>::reverse_iterator rit;
      intf_t obj;
      for(rit = d_hdr_list.rbegin();rit!=d_hdr_list.rend();++rit){
        int count=(*rit).index();
        pmt::pmt_t msg = (*rit).msg();
        int pktlen = pmt::to_long(pmt::dict_ref(msg,pmt::intern("packet_len"),pmt::from_long(0)));
        for(count;count!=d_in_idx;++count){
          if(count==d_cap)
            count=0;
        }
        if(count<pktlen){
          return false;
        }else{
          break;
        }
      }
      obj.set_begin(d_intf_idx);
      obj.set_front((*rit));
      int pkt_begin= (*rit).index();
      for(pkt_begin;pkt_begin!=d_in_idx;++pkt_begin){
        d_intf_mem[d_intf_idx++] = d_in_mem[pkt_begin++];
        pkt_begin%=d_cap;
        if(d_intf_idx==d_cap){
          d_intf_idx=d_cap-1;
          return false;
        }
      }
      d_cur_intf= obj;
      return true;
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
                // try to locate a header in advance?
                // use SFD? Correlation?
                // should record the block and idx of the event for back tracking
                // create intf object here. 
                /*if(create_intf()){

                }else{
                  // somthing wrong
                }*/
                DEBUG<<"<Resync>\033[36;1m"<<"Detecting start of VoE signal...at:"<<d_in_idx<<"\033[0m"<<std::endl;
                break;
              }
              if(!d_cur_intf.front_tag_empty() && d_cur_intf.back_tag_empty()){
                d_intf_mem[d_intf_idx++] = in[count];
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
          case VOE_TRIGGERED:
            while(count<nin){
              tags_update(count);
              if(voe_update(count)){
                d_state = VOE_CLEAR;
                // should record the block and idx of the event for back tracking
                // change state to receive a end header
                DEBUG<<"<Resync>\033[36;1m"<<"Detecting end of VoE signal...at:"<<d_in_idx<<"\033[0m"<<std::endl;
                d_latest_voe_end = d_in_idx;
                break;
              }
              if(!d_cur_intf.front_tag_empty() && d_cur_intf.back_tag_empty()){
                d_intf_mem[d_intf_idx++] = in[count];
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
      nout = std::min(noutput_items,std::max(d_out_size-d_out_idx,0));
      memcpy(out,d_out_mem+d_out_idx,sizeof(gr_complex)*nout);
      memcpy(demo,d_demo_mem+d_out_idx,sizeof(gr_complex)*nout);
      d_out_idx+=nout;
      consume_each (count);
      return nout;
    }

  } /* namespace lsa */
} /* namespace gr */

