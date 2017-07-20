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
#include <gnuradio/math.h>
#include <gnuradio/expj.h>
#include <gnuradio/blocks/count_bits.h>

namespace gr {
  namespace lsa {
    #define d_debug false
    #define DEBUG d_debug && std::cout
    #define CAPACITY 128*1024*1024
    #define BUFCAP 1024*1024
    #define CHIPRATE 8
    #define MODBPS 2
    #define LSAPHYLEN 6
    #define TWO_PI M_PI*2.0F
    static int d_sps = 4;
    static int d_prelen = 128; // symbols 16*8
    static int d_phylen = 192; // 0x00,0x00,0x00,0x00,0xe6,0xXX
    static int d_protect_len = 512;
    static const gr_complex d_pu_taps[] = {gr_complex(0,0),gr_complex(std::sin(M_PI/4.0F),0),gr_complex(1.0F,0),gr_complex(std::sin(3*M_PI/4.0F))};
    // parameters for sync
    static float d_gain_mu = 0.03;
    static float d_gain_omega = 2.25e-4;
    static float d_omega_relative_limit = 2e-4;
    static float d_ori_mu = 0.5;
    static float d_ori_omega = 2;
    static inline float slice(float x){
      return x<0?-1.0F:1.0F;
    }
    static inline unsigned char byte_slice(const float& f){
      return (f>0)? 0x01 : 0x00;
    }
    static inline void phase_wrap(float& phase){
      while(phase>TWO_PI)
        phase-=TWO_PI;
      while(phase<-TWO_PI)
        phase+=TWO_PI;
    }
    
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
              d_out_port(pmt::intern("pdu_out")),
              d_cap(CAPACITY),
              d_buf_lim(BUFCAP),
              d_interp(new filter::mmse_fir_interpolator_ff())
    {
      set_tag_propagation_policy(TPP_DONT);
      message_port_register_in(d_in_port);
      message_port_register_out(d_out_port);
      set_msg_handler(d_in_port,boost::bind(&ic_resync_cc_impl::msg_in,this,_1));
      d_fir_buffer = (gr_complex*)volk_malloc(sizeof(gr_complex)*d_buf_lim/d_sps,volk_get_alignment());
      d_in_mem = (gr_complex*)volk_malloc(sizeof(gr_complex)*d_cap,volk_get_alignment());
      d_ic_mem = (gr_complex*)volk_malloc(sizeof(gr_complex)*d_buf_lim,volk_get_alignment());
      d_mm_mem = (float*)volk_malloc(sizeof(float)*d_buf_lim,volk_get_alignment());
      d_qmod_mem = (float*)volk_malloc(sizeof(float)*d_buf_lim,volk_get_alignment());
      d_intf_mem = (gr_complex*)volk_malloc(sizeof(gr_complex)*d_cap,volk_get_alignment());
      d_out_mem = (gr_complex*)volk_malloc(sizeof(gr_complex)*d_buf_lim,volk_get_alignment());
      d_demo_mem = (gr_complex*)volk_malloc(sizeof(gr_complex)*d_buf_lim,volk_get_alignment());
      d_in_idx =0;
      d_out_idx=0;
      d_out_size=0;
      d_intf_idx=0;
      d_state = VOE_CLEAR;
      d_intf_protect = false;
      d_protect_cnt=0;
      d_cur_intf.clear();
      d_intf_list.clear();
      if(taps.empty()){
        throw std::invalid_argument("No filter taps given");
      }
      d_taps.clear();
      for(int i=0;i<taps.size();++i){
        d_taps.push_back(gr_complex(taps[i],0));
      }
      d_chunk_size=64;
      d_gain_gain = 0.02;
      d_tracking_gain = 0.00628;
      d_pole_alpha = 160e-6;
      d_pole_one_alpha=1-d_pole_alpha;
      d_dec_threshold = 10;
      d_pu_gain_gain = 0.02;
      d_pu_cfo_gain = 0.00628;
      d_tap_buffer = std::vector<gr_complex>(taps.size());
      d_pu_rebuild = std::vector<gr_complex>(64);
      d_su_rebuild = std::vector<gr_complex>(d_buf_lim);
      d_kay_taps = std::vector<float>(64);
      d_kay_tmp = std::vector<gr_complex>(64);
      d_qmod_tmp = std::vector<gr_complex>(d_chunk_size);
      d_pu_tmp = std::vector<gr_complex>(64);
      d_pu_cancel_buf = std::vector<gr_complex>(64);
      float kay_len = d_chunk_size-1;
      float kay_const = 1.5*(kay_len)/(kay_len*kay_len-1);
      for(int i=0;i<kay_len;++i){
        d_kay_taps.push_back(1.0-( (2.0* ((float)i+1.0)-kay_len)/kay_len )*( (2* ((float)i+1.0)-kay_len)/kay_len ));
      }
    }

    /*
     * Our virtual destructor.
     */
    ic_resync_cc_impl::~ic_resync_cc_impl()
    {
      volk_free(d_fir_buffer);
      volk_free(d_in_mem);
      volk_free(d_ic_mem);
      volk_free(d_mm_mem);
      volk_free(d_out_mem);
      volk_free(d_demo_mem);
      volk_free(d_qmod_mem);
      volk_free(d_intf_mem);
      //delete [] d_intf_mem;
      d_su_rebuild.clear();
      d_pu_rebuild.clear();
      d_qmod_tmp.clear();
      d_tap_buffer.clear();
      d_kay_taps.clear();
      d_kay_tmp.clear();
      d_qmod_tmp.clear();
      d_pu_tmp.clear();
      d_pu_cancel_buf.clear();
      delete d_interp;
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
      for(rit=d_block_list.rbegin();rit!=d_block_list.rend();++rit){
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
      int intf_begin = d_intf_idx;
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
      // record voe tag begin for ease of cancellation
      obj.add_msg(pmt::intern("voe_begin"),pmt::from_long(d_intf_idx-intf_begin));
      obj.set_end(d_intf_idx-1);
      d_cur_intf= obj;
      return true;
    }

    void
    ic_resync_cc_impl::rebuild_su(bool retx, const std::vector<int>& retx_idx,std::vector<int>& pkt_len)
    {
      DEBUG<<"Rebuild SU samples: retransmission header?"<<retx<<std::endl;
      // reset fir buffer;
      std::fill(d_su_rebuild.begin(),d_su_rebuild.end(),gr_complex(0,0));
      // d_lsaphy_idx [] in total 10 elements
      int hdr_idx[retx_idx.size()][8];
      if(!retx){
        for(int i=0;i<retx_idx.size();++i){
          for(int j=0;j<8;++j)
            hdr_idx[i][j]=0;
        }
      }else{
       for(int i=0;i<retx_idx.size();++i){
         uint16_t qsize = d_retx_stack.size();
         uint16_t qidx = retx_idx[i];
         uint8_t * qs8 = (uint8_t*)&qsize;
         uint8_t * qi8 = (uint8_t*)&qidx;
         hdr_idx[i][0] = (qi8[1]>>4) &0x0f;
         hdr_idx[i][1] = qi8[1]&0x0f;
         hdr_idx[i][2] = (qi8[0]>>4) &0x0f;
         hdr_idx[i][3] = qi8[0]&0x0f;
         hdr_idx[i][4] = (qs8[1]>>4) &0x0f;
         hdr_idx[i][5] = qs8[1]&0x0f;
         hdr_idx[i][6] = (qs8[0]>>4) &0x0f;
         hdr_idx[i][7] = qs8[0]&0x0f;
       }
      }
      // use fir buffer to rebuild su
      int size_cnt =0;
      for(int i=0;i<retx_idx.size();++i){
        pmt::pmt_t blob = std::get<1>(d_retx_stack[retx_idx[i]]);
        size_t io(0);
        // copy preamble
        for(int k=0;k<10;++k){
          memcpy(d_fir_buffer+size_cnt+k*16,d_map[d_lsaphy_idx[k]],sizeof(gr_complex)*16);
        }
        size_cnt+=160;
        const uint8_t* uvec = pmt::u8vector_elements(blob,io);
        pkt_len.push_back((io+6)*8*8*2);
        uint8_t u8_io = (uint8_t)io;
        // copy size field
        memcpy(d_fir_buffer+size_cnt,d_map[(u8_io>>4)&0x0f],sizeof(gr_complex)*16);
        memcpy(d_fir_buffer+size_cnt+16,d_map[u8_io&0x0f],sizeof(gr_complex)*16);
        size_cnt+=32;
        // paste header according to retx type
        for(int h=0;h<8;++h){
          memcpy(d_fir_buffer+size_cnt+16*h,d_map[hdr_idx[i][h]],sizeof(gr_complex)*16);
        }
        // rebuild payload
        for(int j=4;j<io;++j){
          memcpy(d_fir_buffer+size_cnt+j*32,d_map[(uvec[j]>>4)&0x0f],sizeof(gr_complex)*16);
          memcpy(d_fir_buffer+size_cnt+j*32+16,d_map[uvec[j]&0x0f],sizeof(gr_complex)*16);
        }
        // 4 for subtracting header length
        size_cnt+=(32*io);
      }
      DEBUG<<"REbuild SU, generated symbols:"<<size_cnt<<std::endl;
      // fir filter
      for(int i=0;i<size_cnt;++i){
        volk_32fc_s32fc_multiply_32fc(d_tap_buffer.data(),d_taps.data(),d_fir_buffer[i],d_taps.size());
        for(int j=0;j<d_taps.size();++j)
          d_su_rebuild[d_sps*i+j]+=d_tap_buffer[j];
      }
      // additional taps for fir 
      memcpy(d_su_rebuild.data(),d_su_rebuild.data()+20,sizeof(gr_complex)*size_cnt*d_sps);
    }

    void
    ic_resync_cc_impl::rebuild_pu(int chip_id)
    {
      const gr_complex* chip_ptr = d_map[chip_id];
      for(int i=0;i<16;++i){
        volk_32fc_s32fc_multiply_32fc(d_pu_tmp.data()+d_sps*i,d_pu_taps,chip_ptr[i],4);
      }
      for(int i=0;i<64;++i){
        d_pu_rebuild[i] = gr_complex(d_pu_tmp[i].real(),d_offset_d1);
        d_offset_d1 = d_offset_d2;
        d_offset_d2 = std::imag(d_pu_tmp[i]);
      }
    }

    void
    ic_resync_cc_impl::reset_sync()
    {
      // for mm
      d_last_sample=0;
      d_mu = d_ori_mu;
      d_omega = d_ori_omega;
      d_omega_mid = d_ori_omega;
      d_omega_lim = d_omega_mid * d_omega_relative_limit;
      // counters and registers
      d_cancel_idx=0;
      d_mm_size=0;
      d_mm_cnt=0;
      // mm
      d_mm_size=0;
      d_mm_cnt=0;
      d_mm_consume=0;
      // pole
      d_pole_prevo=0;
      // qmod
      d_qmod_cnt=0;
      // dec
      enter_search();
      // pu
      d_offset_d1 = 0;
      d_offset_d2 = 0;
      d_found_first_pu = false;
      
    }

    void
    ic_resync_cc_impl::do_ic(std::pair<intf_t,std::vector<int> > obj)
    {
      int begin = std::get<0>(obj).begin();
      const int size = std::get<0>(obj).size();
      pmt::pmt_t intf_msg = std::get<0>(obj).msg();
      int voe_begin = pmt::to_long(pmt::dict_ref(intf_msg,pmt::intern("voe_begin"),pmt::from_long(-1)));
      if(size>d_buf_lim){
        return;
      }else if(size>(d_buf_lim-d_out_size) ){
        d_out_size=0;
        d_out_idx=0;
        d_out_tags.clear();
      }
      memcpy(d_ic_mem,d_intf_mem+begin,sizeof(gr_complex)*size);
      std::vector<int> retx_idx = std::get<1>(obj);
      // result are stored in d_intf_idx[begin] up to d_intf_idx[begin+size-1];
      DEBUG<<"Calling do ic:"<<std::endl;
      DEBUG<<"front tag:"<<std::get<0>(obj).front()<<std::endl;
      DEBUG<<"intf_begin="<<begin<<" ,intf_size="<<size<<std::endl;
      DEBUG<<"voe begin="<<voe_begin<<std::endl;
      // required retransmissions 
      int length_cnt=0;
      for(int i=0;i<retx_idx.size();++i){
        DEBUG<<"pkt_len:"<<std::get<0>(d_retx_stack[retx_idx[i]])<<" ,base:"<<std::get<2>(d_retx_stack[retx_idx[i]])<<std::endl;
        pmt::pmt_t blob = std::get<1>(d_retx_stack[retx_idx[i]]);
        length_cnt+= std::get<0>(d_retx_stack[retx_idx[i]]);
      }
      if(length_cnt>=d_buf_lim){
        DEBUG<<"DO IC ERROR: rebuild length greater than available memory size"<<std::endl;
        return;
      }
      pmt::pmt_t front_msg = (std::get<0>(obj)).front().msg();
      bool is_retx = pmt::to_long(pmt::dict_ref(front_msg,pmt::intern("queue_size"),pmt::from_long(-1)))!=0;
      std::vector<int> pkt_len;
      rebuild_su(is_retx,retx_idx,pkt_len); // store samples of su in d_su_rebuild
      reset_sync();
      // note that there are residual samples due to fir interpolation
      // intf samples: d_intf_mem[begin] and size;
      // autocorrelation for first cfo estimate, search e6 for phase, use e6 for gain estimation
      gr_complex cross_corr, corr_eng, auto_corr, su_eng, diff;
      uint16_t max_idx = 0;
      const int delay = 128;
      int pkt_begin=0;
      /*for(int i=0;i<pkt_len.size();++i){
        if(pkt_begin+pkt_len[i]<voe_begin-640){
          pkt_begin+=pkt_len[i];
        }else{
          break;
        }
      }*/
      //DEBUG<<"Coarse estimated begin index:"<<pkt_begin<<std::endl;
      // coarse estimate of pkt_begin
      gr_complex auto_eng;
      for(int i=0;i<512;i++){
        int auto_idx = pkt_begin+i;
        volk_32fc_x2_conjugate_dot_prod_32fc(&auto_corr,d_ic_mem+auto_idx,d_ic_mem+auto_idx+delay,delay);
        volk_32fc_x2_conjugate_dot_prod_32fc(&corr_eng,d_ic_mem+auto_idx,d_ic_mem+auto_idx,delay);
        volk_32fc_x2_conjugate_dot_prod_32fc(&auto_eng,d_ic_mem+auto_idx+delay,d_ic_mem+auto_idx+delay,delay);
        d_corr_test[i] = auto_corr/std::sqrt(corr_eng*auto_eng);
      }
      volk_32fc_index_max_16u(&max_idx,d_corr_test,512);
      if(std::abs(d_corr_test[max_idx])>0.9){
        DEBUG<<"Step1 passed: Autocorrelation found: idx="<<pkt_begin+max_idx<<" cfo:"<<d_su_cfo<<std::endl;
      }else{
        // warning autocorrelation is weak
        DEBUG<<"Step1 failed: Autocorrelation of first 1024 samples does not reach threshold 0.9, abort---"<<std::abs(d_corr_test[max_idx])<<std::endl;
        return;
      }
      d_su_cfo = fast_atan2f(d_corr_test[max_idx].imag(),d_corr_test[max_idx].real())/(float)delay;
      uint16_t sfd_idx = pkt_begin+512;
      gr_complex init_phase(1,0);
      volk_32fc_s32fc_x2_rotator_32fc(d_chunk_buf,d_su_rebuild.data()+sfd_idx,gr_expj(d_su_cfo),&init_phase,d_chunk_size);
      volk_32fc_x2_conjugate_dot_prod_32fc(&su_eng,d_chunk_buf,d_chunk_buf,d_chunk_size);
      volk_32fc_x2_conjugate_dot_prod_32fc(&cross_corr,d_ic_mem+sfd_idx,d_chunk_buf,d_chunk_size);
      volk_32fc_x2_conjugate_dot_prod_32fc(&corr_eng,d_ic_mem+sfd_idx,d_ic_mem+sfd_idx,d_chunk_size);
      d_corr_test[sfd_idx] = cross_corr/std::sqrt(corr_eng*su_eng);
      if(std::abs(d_corr_test[sfd_idx])>0.9){
        DEBUG<<"Step2 passed: Cross correlation found SFD(0xE6) idx="<<sfd_idx+pkt_begin<<" ,correlation="<<std::abs(d_corr_test[sfd_idx])<<std::endl;
      }else{
        DEBUG<<"Step2 failed: Cross correlation of SFD (0xE6) does not show up at expected value... abort val:"<<std::abs(d_corr_test[sfd_idx])<<std::endl;
        return;
      }
      d_su_phase = fast_atan2f(d_corr_test[sfd_idx].imag(),d_corr_test[sfd_idx].real());
      d_su_gain = std::real(std::sqrt(corr_eng/su_eng));
      // begin to cancel
      d_cancel_idx = sfd_idx;
      // debug tag
      tag_t tmp_tag;
      tmp_tag.offset = d_out_size;
      tmp_tag.key = pmt::intern("ic_out");
      tmp_tag.value = pmt::PMT_T;
      d_out_tags.push_back(tmp_tag);
      // debugging auto correlation
      float auto_val, cross_val;
      while(d_cancel_idx<(d_su_rebuild.size()-d_chunk_size) && sfd_idx<(size-d_chunk_size) && (sfd_idx+2*d_chunk_size<voe_begin) ){
        volk_32fc_s32fc_x2_rotator_32fc(d_chunk_buf,d_su_rebuild.data()+d_cancel_idx,gr_expj(d_su_cfo),&init_phase,d_chunk_size);
        for(size_t t=0;t<d_chunk_size;++t){
          d_demo_mem[d_out_size] =d_ic_mem[sfd_idx+t]; // for demo purpose 
          d_out_mem[d_out_size++]=d_ic_mem[sfd_idx+t] - d_su_gain*gr_expj(d_su_phase)*d_chunk_buf[t];
        }
        // update phase, gain and cfo
        volk_32fc_x2_conjugate_dot_prod_32fc(&auto_corr,d_ic_mem+sfd_idx,d_ic_mem+sfd_idx+delay,delay); // debug
        volk_32fc_x2_conjugate_dot_prod_32fc(&corr_eng,d_ic_mem+sfd_idx,d_ic_mem+sfd_idx,delay); // debug
        auto_corr = auto_corr/corr_eng;
        volk_32fc_x2_conjugate_dot_prod_32fc(&cross_corr,d_ic_mem+sfd_idx,d_chunk_buf,d_chunk_size);
        volk_32fc_x2_conjugate_dot_prod_32fc(&corr_eng,d_ic_mem+sfd_idx,d_ic_mem+sfd_idx,d_chunk_size);
        volk_32fc_x2_conjugate_dot_prod_32fc(&su_eng,d_chunk_buf,d_chunk_buf,d_chunk_size);
        diff = cross_corr * gr_expj(-d_su_phase);
        cross_val = std::abs(cross_corr/std::sqrt(corr_eng*su_eng));
        bool loosing_track = (std::abs(auto_corr)>0.95 && cross_val<0.9);
        d_su_phase = (cross_val>0.9)? fast_atan2f(cross_corr.imag(),cross_corr.real()) : d_su_phase+d_su_cfo*d_chunk_size;
        d_su_cfo  = (loosing_track)? fast_atan2f(auto_corr.imag(),auto_corr.real())/(float)delay :
         (d_su_cfo + d_tracking_gain * fast_atan2f(diff.imag(),diff.real())/(float)d_chunk_size);
        d_su_gain = std::exp(std::log(d_su_gain)+d_gain_gain*(std::log(std::real(std::sqrt(corr_eng/su_eng)))-std::log(d_su_gain)));
        d_cancel_idx+=d_chunk_size;
        sfd_idx+=d_chunk_size;
      }
      d_last_su_sync_idx = sfd_idx;
      // next chunk contains interfering signal
      tmp_tag.offset = d_out_size;
      tmp_tag.key = pmt::intern("voe_begin");
      tmp_tag.value = pmt::PMT_T;
      d_out_tags.push_back(tmp_tag);

      while(d_cancel_idx<(d_su_rebuild.size()-d_chunk_size) && sfd_idx<(size-d_chunk_size)){
        bool found_pu_symbol = false;
        volk_32fc_s32fc_x2_rotator_32fc(d_chunk_buf,d_su_rebuild.data()+d_cancel_idx,gr_expj(d_su_cfo),&init_phase,d_chunk_size);
        for(size_t t=0;t<d_chunk_size;++t){
          d_demo_mem[d_out_size+t]=d_ic_mem[sfd_idx+t];
          d_out_mem[d_out_size+t]=d_ic_mem[sfd_idx+t]-d_su_gain*gr_expj(d_su_phase)*d_chunk_buf[t];
        }
        // qmod and single pole iir filter
        volk_32fc_x2_multiply_conjugate_32fc(&d_qmod_tmp[0],&d_out_mem[d_out_size],&d_out_mem[d_out_size-1],d_chunk_size);
        for(size_t p=0;p<d_chunk_size;++p){
          float qmod_out = gr::fast_atan2f(imag(d_qmod_tmp[p]),real(d_qmod_tmp[p]));
          d_pole_prevo = d_pole_alpha*qmod_out + d_pole_one_alpha*d_pole_prevo;
          d_qmod_mem[d_qmod_cnt+p] = qmod_out - d_pole_prevo;
        }
        d_qmod_cnt+=d_chunk_size;
        // MM clock recovery on qmod output
        float mm_val;
        int prev_mm_size = d_mm_size;
        while(d_mm_consume<(d_qmod_cnt-d_interp->ntaps())){
          d_mm_mem[d_mm_size] = d_interp->interpolate(&d_qmod_mem[d_mm_consume],d_mu);
          mm_val = slice(d_last_sample)*d_mm_mem[d_mm_size]-slice(d_mm_mem[d_mm_size])*d_last_sample;
          d_last_sample = d_mm_mem[d_mm_size];
          d_omega = d_omega + d_gain_omega * mm_val;
          d_omega = d_omega_mid + gr::branchless_clip(d_omega-d_omega_mid,d_omega_lim);
          d_mu = d_mu + d_omega + d_gain_mu * mm_val;
          d_mm_consume += (int)floor(d_mu);
          d_mu = d_mu - floor(d_mu);
          d_mm_size++;
        }
        // check accumulated zig_diff bits according to state
        // if unfound, search on one bit basis
        // if found, move on for every 32 bits
        // once a symbol detected, rebuild MSK and cancel back
        // SU should record the latest channel estimators. update accordingly.
        while(d_mm_cnt<d_mm_size){
          switch(d_dec_state){
            case SEARCH:
              while(d_mm_cnt<d_mm_size){
                d_dec_data_reg = (d_dec_data_reg<<1) | (byte_slice(d_mm_mem[d_mm_cnt++])&0x01);
                if(d_dec_pre_cnt>0){
                  d_dec_chip_cnt++;
                }
                if(d_dec_pre_cnt==0){
                  int thres = gr::blocks::count_bits32( (d_dec_data_reg&d_mask)^(CHIPSET[0] & d_mask));
                  if(thres<d_dec_threshold){
                    d_dec_pre_cnt++;
                    // found first zero, double check to do first cancellation
                    rebuild_pu(0);
                    cancel_pu_and_resync(d_out_size,d_cancel_idx,sfd_idx,prev_mm_size,d_mm_cnt);
                    found_pu_symbol = true;
                    DEBUG<<"<GOOD>found a zigbee symbol-->"<<0<<" at search state"<<std::endl;
                  }
                }else{
                  // have some preamble found
                  if(d_dec_chip_cnt==32){
                    d_dec_chip_cnt=0;
                    if(d_dec_byte_reg==0x00){
                      if(gr::blocks::count_bits32((d_dec_data_reg&d_mask)^(CHIPSET[0]&d_mask))<=d_dec_threshold){
                        d_dec_pre_cnt++;
                        d_dec_byte_reg = 0x00;
                        rebuild_pu(0);
                        cancel_pu_and_resync(d_out_size,d_cancel_idx,sfd_idx,prev_mm_size,d_mm_cnt);
                        DEBUG<<"<GOOD>found consecutive zeros at search state"<<std::endl;
                    found_pu_symbol = true;
                      }else if(gr::blocks::count_bits32((d_dec_data_reg&d_mask)^(CHIPSET[7]&d_mask))<d_dec_threshold){
                        d_dec_byte_reg = 0x70;
                        rebuild_pu(7);
                        cancel_pu_and_resync(d_out_size,d_cancel_idx,sfd_idx,prev_mm_size,d_mm_cnt);
                        found_pu_symbol = true;
                        DEBUG<<"<GOOD> found first SHR byte-->"<<7<<std::endl;
                      }else{
                        enter_search();
                        break;
                      }
                    }else{
                      if(gr::blocks::count_bits32((d_dec_data_reg&d_mask)^(CHIPSET[10]&d_mask))<d_dec_threshold){
                        d_dec_byte_reg |= 0x0A;
                        enter_sync();
                        rebuild_pu(10);
                        cancel_pu_and_resync(d_out_size,d_cancel_idx,sfd_idx,prev_mm_size,d_mm_cnt);
                        found_pu_symbol = true;
                        DEBUG<<"<GOOD> found second SHR byte-->"<<10<<std::endl;
                        break;
                      }else{
                        enter_search();
                        break;
                      }
                    }
                  }
                }
              }
            break;
            case SYNC:
              while(d_mm_cnt<d_mm_size){
                d_dec_data_reg = (d_dec_data_reg<<1) | (byte_slice(d_mm_mem[d_mm_cnt++])&0x01);
                d_dec_chip_cnt++;
                if(d_dec_chip_cnt==32){
                  d_dec_chip_cnt=0;
                  int quality=33;
                  unsigned char c = chip_decoder(d_dec_data_reg,quality);
                  if(quality>=d_dec_threshold){
                    // low quality warning
                    DEBUG<<"<warning> PU synchronization losing tracks...state=SYNC, quality="<<quality<<std::endl;
                  }
                  rebuild_pu((int)c);
                  cancel_pu_and_resync(d_out_size,d_cancel_idx,sfd_idx,prev_mm_size,d_mm_cnt);
                  found_pu_symbol = true;
                  if(d_dec_symbol_cnt==0){
                    d_dec_byte_reg = c<<4;
                    d_dec_symbol_cnt++;
                  }else{
                    d_dec_byte_reg |= c;
                    if(d_dec_byte_reg <= MAXPLD){
                      DEBUG<<"<GOOD>At sync state, found a valid pkt length="<<(int)d_dec_byte_reg<<std::endl;
                      enter_payload(d_dec_byte_reg);
                      break;
                    }else{
                      DEBUG<<"<BAD>Invalid packet length, return to search state"<<std::endl;
                      enter_search();
                      break;
                    }
                  }
                }
              }
            break;
            case PAYLOAD:
              while(d_mm_cnt<d_mm_size){
                d_dec_data_reg = (d_dec_data_reg<<1)| (byte_slice(d_mm_mem[d_mm_cnt++])&0x01);
                d_dec_chip_cnt++;
                if(d_dec_chip_cnt==32){
                  d_dec_chip_cnt=0;
                  int quality=33;
                  unsigned char c = chip_decoder(d_dec_data_reg,quality);
                  if(quality>=d_dec_threshold){
                    // low quality warning
                    DEBUG<<"<warning> PU synchronization losing tracks...state=PAYLOAD, quality="<<quality<<std::endl;
                  }
                  rebuild_pu((int)c);
                  cancel_pu_and_resync(d_out_size,d_cancel_idx,sfd_idx,prev_mm_size,d_mm_cnt);
                  found_pu_symbol = true;
                  if(d_dec_symbol_cnt%2==0){
                    d_dec_buf[d_dec_symbol_cnt/2] = c<<4;
                  }else{
                    d_dec_buf[d_dec_symbol_cnt/2] |=c;
                  }
                  d_dec_symbol_cnt++;
                  if(d_dec_symbol_cnt/2>=d_dec_pld_len){
                    DEBUG<<"<GOOD>Complete a decoding of PU!..."<<std::endl;
                    pmt::pmt_t blob = pmt::make_blob(d_dec_buf,d_dec_pld_len);
                    message_port_pub(d_out_port,pmt::cons(pmt::intern("IC_out"),blob));
                    enter_search();
                    break;
                  }
                }
              }
            break;
            default:
              throw std::runtime_error("Undefined state of inner decoder");
            break;
          }
        }
        if(!found_pu_symbol){
          d_su_phase = d_su_phase + (sfd_idx-d_last_su_sync_idx)*d_su_cfo;
          phase_wrap(d_su_phase);
          d_last_su_sync_idx = sfd_idx;
          // other estimator use that obtained before voe tag found...
        }
        d_out_size+=d_chunk_size;
        d_cancel_idx+=d_chunk_size;
        sfd_idx+=d_chunk_size;
      }
    }

    void
    ic_resync_cc_impl::cancel_pu_and_resync(int cur_sync_idx,int ic_mem_idx,int su_mem_idx,int prev_mm_size,int cur_mm_idx)
    {
      //32 for a symbol of chips
      int offset = cur_mm_idx-prev_mm_size-32;
      int symbol_begin = cur_sync_idx+offset*2;  // x/32*64
      int ic_begin = ic_mem_idx+offset*2;
      int su_begin = su_mem_idx+offset*2;
      memcpy(d_pu_cancel_buf.data(),d_out_mem+symbol_begin,sizeof(gr_complex)*64);
      // kay cfo
      volk_32fc_x2_multiply_conjugate_32fc(d_kay_tmp.data(),d_pu_cancel_buf.data(),d_pu_rebuild.data(),64);
      std::vector<gr_complex> kay_diff(64); // phase difference
      volk_32fc_x2_multiply_conjugate_32fc(kay_diff.data(),d_kay_tmp.data()+1,d_kay_tmp.data(),63);
      float kay_cfo=0;
      // use precalculated kay averaging taps and the fact that msk E[ci*conj(ci)] =1
      for(int i=0;i<63;++i){
        kay_cfo+= fast_atan2f(kay_diff[i].imag(),kay_diff[i].real()) * d_kay_taps[i];
      }
      d_pu_cfo = (d_found_first_pu)? kay_cfo : d_pu_cfo+(kay_cfo-d_pu_cfo)*d_pu_cfo_gain;
      gr_complex pu_eng, ori_eng, cross, init_phase(1,0);
      volk_32fc_x2_conjugate_dot_prod_32fc(&pu_eng,d_pu_cancel_buf.data(),d_pu_cancel_buf.data(),64);
      // already call rebuild_pu in do_ic!!
      volk_32fc_x2_conjugate_dot_prod_32fc(&ori_eng,d_pu_rebuild.data(),d_pu_rebuild.data(),64);
      float tmp_gain = std::real(std::sqrt(pu_eng/ori_eng));
      d_pu_gain = (d_found_first_pu)? tmp_gain  : std::exp(std::log(d_pu_gain)+d_pu_gain_gain*(std::log(tmp_gain)-std::log(d_pu_gain)));
      d_found_first_pu = (d_found_first_pu)? true : false;
      // cross correlation estimator
      volk_32fc_s32fc_x2_rotator_32fc(d_pu_cancel_buf.data(),d_pu_rebuild.data(),gr_expj(d_pu_cfo),&init_phase,d_chunk_size);
      volk_32fc_x2_conjugate_dot_prod_32fc(&cross,d_out_mem+symbol_begin,d_pu_cancel_buf.data(),64);
      // phase
      d_pu_phase = fast_atan2f(cross.imag(),cross.real());
      for(int i=0;i<64;++i){
        d_ic_mem[ic_begin+i] = d_ic_mem[ic_begin+i] - d_pu_gain*gr_expj(d_pu_phase)* d_pu_cancel_buf[i];
      }
      // su resync
      int distance = su_begin-d_last_su_sync_idx;
      float phase_est = d_su_phase + distance*d_su_cfo;
      phase_wrap(phase_est);
      std::vector<gr_complex> su_buf(64);
      gr_complex su_eng, diff;
      volk_32fc_s32fc_x2_rotator_32fc(su_buf.data(),d_su_rebuild.data()+su_begin,gr_expj(d_su_cfo),&init_phase,64);
      volk_32fc_x2_conjugate_dot_prod_32fc(&cross,d_ic_mem+ic_begin,su_buf.data(),64);
      volk_32fc_x2_conjugate_dot_prod_32fc(&su_eng,d_ic_mem+ic_begin,d_ic_mem+ic_begin,64);
      volk_32fc_x2_conjugate_dot_prod_32fc(&ori_eng,d_su_rebuild.data(),d_su_rebuild.data(),64);
      tmp_gain  = std::real(su_eng/ori_eng);
      diff = cross * gr_expj(-phase_est);
      float cross_val = std::abs(cross/std::sqrt(su_eng*ori_eng));
      // update according to cross correlation value
      d_su_cfo =(cross_val>0.9)? d_su_cfo+d_tracking_gain*fast_atan2f(diff.imag(),diff.real())/(float)d_chunk_size : d_su_cfo;
      d_su_gain = (cross_val>0.9)? std::exp(std::log(d_su_gain)+d_gain_gain*(std::log(tmp_gain)-std::log(d_su_gain))) : d_su_gain;
      d_su_phase = (cross_val>0.9)? fast_atan2f(cross.imag(),cross.real()) : d_su_phase;
      d_last_su_sync_idx = su_begin;
    }

    unsigned char
    ic_resync_cc_impl::chip_decoder(const unsigned int& c, int& quality)
    {
      unsigned char d;
      int min_thres = 33;
      int thres;
      for(int i=0;i<16;++i){
        thres = gr::blocks::count_bits32( (c&d_mask)^(CHIPSET[i]&d_mask) );
        if(thres<min_thres){
          min_thres = thres;
          d = (unsigned char)i;
        }
      }
      quality = min_thres;
      return d&0x0f;
    }
    void
    ic_resync_cc_impl::enter_search()
    {
      d_dec_state = SEARCH;
      d_dec_pre_cnt=0;
      d_dec_data_reg = 0x00000000;
      d_dec_chip_cnt=0;
      d_dec_byte_reg = 0x00;
    }
    void
    ic_resync_cc_impl::enter_sync()
    {
      d_dec_state = SYNC;
      d_dec_pre_cnt=0;
      d_dec_data_reg = 0x00000000;
      d_dec_symbol_cnt=0;
      d_dec_pld_len =0;
      d_dec_byte_reg= 0x00;
    }
    void
    ic_resync_cc_impl::enter_payload(const unsigned char& pld_len)
    {
      d_dec_pld_len = pld_len;
      d_dec_state = PAYLOAD;
      d_dec_data_reg = 0x00000000;
      d_dec_chip_cnt=0;
      d_dec_symbol_cnt=0;
      d_dec_byte_reg =0x00;
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
                if(d_intf_idx==d_cap){
                  d_intf_idx = d_cur_intf.begin();
                  d_cur_intf.clear();
                  d_intf_protect = false;
                  d_protect_cnt=0;
                }
                d_protect_cnt++;
                if(d_protect_cnt==d_protect_len){
                  d_intf_protect = false;
                  d_protect_cnt=0;
                  DEBUG<<"<Resync>complete a intf tag,...total size="<<d_cur_intf.size()<<std::endl;
                  d_intf_list.push_back(d_cur_intf);
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
      std::list<tag_t>::iterator oit=d_out_tags.begin();
      while(oit!=d_out_tags.end()){
        tag_t check_tag = *oit;
        if(check_tag.offset>=d_out_idx && check_tag.offset<d_out_idx+nout){
          add_item_tag(0,nitems_written(0)+check_tag.offset-d_out_idx,check_tag.key,check_tag.value);
          oit = d_out_tags.erase(oit);
        }else{
          break;
        }
      }
      d_out_idx+=nout;
      if( d_out_idx!=0 && (d_out_idx==d_out_size)){
        oit = d_out_tags.begin();
        while(oit!=d_out_tags.end()){
          tag_t tmp_tag = *oit;
          tmp_tag.offset =(long int)tmp_tag.offset- d_out_size;
          *oit=tmp_tag;
          oit++;
        }
        d_out_idx=0;
        d_out_size=0;
      }
      consume_each (count);
      return nout;
    }

  } /* namespace lsa */
} /* namespace gr */

