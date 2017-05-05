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
#include "su_transmitter_bb_impl.h"
#include <ctime>

namespace gr {
  namespace lsa {

    // LSA SU MAC HEADER DEFINITION
    // -------------------------------------
    //              BIT FIELD
    // -------------------------------------
    // queue idx| queue size| 
    //    1byte |    1byte  |
    // -------------------------------------
    // STATES: 1) CLEAR_TO_SEND, 2) RETRANSMISSION
    // BEHAVIOR:     queue size =0,    queue_size = buf size

    // for simplicity, PHY preamble and SFD are also prefixed on received stream

    enum SUTXMODE{
      SUCCESSIVE=0,
      NOQUEUE=1
    };
    enum SUTXSTATE{
      CLEAR_TO_SEND,
      RETRANSMISSION
    };

    static const int MAX_PLD_LEN = 125;  //(127-2);
    static const unsigned char LSA_PHY[] ={0x00,0x00,0x00,0x00,0x7A};
    static const int PHY_LEN = 5;
    

    su_transmitter_bb::sptr
    su_transmitter_bb::make(const std::string& tagname, int mode, bool debug)
    {
      return gnuradio::get_initial_sptr
        (new su_transmitter_bb_impl(tagname,mode,debug));
    }

    /*
     * The private constructor
     */
    su_transmitter_bb_impl::su_transmitter_bb_impl(const std::string& tagname, int mode, bool debug)
      : gr::tagged_stream_block("su_transmitter_bb",
              gr::io_signature::make(1, 1, sizeof(char)),
              gr::io_signature::make(1, 1, sizeof(char)), tagname),
              d_lentag(pmt::intern(tagname)),
              d_queue_cap(256)
    {
      d_msg_in = pmt::mp("msg_in");
      message_port_register_in(d_msg_in);
      set_msg_handler(d_msg_in,boost::bind(&su_transmitter_bb_impl::msg_in,this,_1));

      d_debug = debug;
      switch(mode){
        case 0:
          d_mode = SUCCESSIVE;
        break;
        case 1:
          d_mode = NOQUEUE;
        break;
        default:
          d_mode = NOQUEUE;
        break;
      }
      d_state = CLEAR_TO_SEND;
      d_queue_buf.resize(d_queue_cap,NULL);
      d_pkt_len_buf.resize(d_queue_cap,0);
      d_time_buf.resize(d_queue_cap,0);
      for(int i=0;i<d_queue_cap;++i){
        d_queue_buf[i] = new unsigned char[256];
      }
      d_latest_idx = 0;
      d_current_idx = 0;
      d_current_time = std::clock();
      d_update_time = std::clock();

      d_retx_cnt=0;
      d_retx_idx=0;
      
    }

    /*
     * Our virtual destructor.
     */
    su_transmitter_bb_impl::~su_transmitter_bb_impl()
    {
      for(int i=0;i<d_queue_cap;++i){
        delete [] d_queue_buf[i];
      }
      d_queue_buf.clear();
    }

    int
    su_transmitter_bb_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
    {
      assert(ninput_items <= MAX_PLD_LEN);
      int noutput_items = ninput_items[0]+2+PHY_LEN;
      switch(d_mode){
        case NOQUEUE:
        break;
        case SUCCESSIVE:
          if(d_state == CLEAR_TO_SEND){
            noutput_items = ninput_items[0]+2+PHY_LEN;
          }
          else if(d_state == RETRANSMISSION){
            assert(!d_retx_idx_buf.empty());
            int idx_mapping = d_retx_idx_buf[d_retx_idx];
            noutput_items = d_pkt_len_buf[idx_mapping]+2+PHY_LEN;
          }
        break;
        default:
          std::runtime_error("Wrong state");
        break;
      }
      
      return noutput_items;
    }

    void
    su_transmitter_bb_impl::msg_in(pmt::pmt_t msg)
    {
      assert(pmt::is_dict(msg));
      bool sen_result = false;
      int qidx=0;
      int qsize=0;
      switch(d_mode)
      {
        case NOQUEUE:
          if(pmt::dict_has_key(msg,pmt::intern("LSA_hdr"))){
        assert(pmt::dict_has_key(msg,pmt::intern("queue_index")));
        assert(pmt::dict_has_key(msg,pmt::intern("queue_size")));
        qidx = pmt::to_long(pmt::dict_ref(msg,pmt::intern("queue_index"),pmt::from_long(-1)));
        qsize= pmt::to_long(pmt::dict_ref(msg,pmt::intern("queue_size"),pmt::from_long(-1)));
        if(qidx<0 || qsize<0){
          throw std::runtime_error("Undefined MAC fields");
        }
        assert(qidx < d_queue_cap);
        if(d_time_buf[qidx]==0){
          // outdated ack
          return;
        }
        d_time_buf[qidx]=0;
        d_latest_idx= qidx;
        d_update_time = std::clock();
        // can add statistics counter here
      }
        break;
        case SUCCESSIVE:
        if(pmt::dict_has_key(msg,pmt::intern("sensing"))){
        sen_result = pmt::to_bool(pmt::dict_ref(msg,pmt::intern("sensing"),pmt::PMT_T));
        // retransmission no matter what
        if( (d_state == CLEAR_TO_SEND) && (sen_result) ){
          prepare_retx();
          d_state = RETRANSMISSION;
          return;
        }
      }
        if(pmt::dict_has_key(msg,pmt::intern("LSA_hdr"))){
        assert(pmt::dict_has_key(msg,pmt::intern("queue_index")));
        assert(pmt::dict_has_key(msg,pmt::intern("queue_size")));
        qidx = pmt::to_long(pmt::dict_ref(msg,pmt::intern("queue_index"),pmt::from_long(-1)));
        qsize= pmt::to_long(pmt::dict_ref(msg,pmt::intern("queue_size"),pmt::from_long(-1)));
        if(qidx<0 || qsize<0){
          throw std::runtime_error("Undefined MAC fields");
        }
        assert(qidx < d_queue_cap);
        switch(d_state)
        {
          case CLEAR_TO_SEND:
          if(d_time_buf[qidx]==0){
            // outdated ack
            return;
          }
          d_time_buf[qidx]=0;
          d_latest_idx= qidx;
          d_update_time = std::clock();
          // can add statistics counter here
          break;
          case RETRANSMISSION:
            if(sen_result){
              return;
            }
            if(d_time_buf[d_retx_idx_buf[qidx]]!=0){
              d_time_buf[d_retx_idx_buf[qidx]]=0;
              d_retx_cnt++;
              d_update_time = std::clock();
            }
            assert(!d_retx_idx_buf.empty());
            if(d_retx_cnt == d_retx_idx_buf.size()){
              if(d_debug){
                std::cerr<<"<STATE>Retransmission complete, reset queue and change state"<<std::endl;
              }
              reset_queue();
              d_state = CLEAR_TO_SEND;
            }
          break;
          default:
            throw std::runtime_error("Undefined state");
          break;
        }
        
      }
        break;
        default:
          throw std::runtime_error("Undefined state");
        break;
      }
    }

    void
    su_transmitter_bb_impl::prepare_retx()
    {
      // limit: the limited delay is 256
      // this is due to the mac field design
      assert(d_latest_idx!=d_current_idx);
      int delay = d_current_idx-d_latest_idx;
      if(d_latest_idx>d_current_idx){
        delay = d_current_idx + d_queue_cap- d_latest_idx;
      }
      d_retx_idx_buf.resize(delay,0);
      int idx_iter = d_latest_idx;
      for(int i=0;i<delay;++i){
        d_retx_idx_buf[i] = idx_iter++;
        if(idx_iter>=d_queue_cap){
          idx_iter%= d_queue_cap;
        }
      }
      d_retx_cnt=0;
      d_retx_idx=0;
      // do not change state here!!!!
    }

    void
    su_transmitter_bb_impl::reset_queue()
    {
      memset(d_pkt_len_buf.data(),0,sizeof(size_t)*d_queue_cap);
      memset(d_time_buf.data(),0,sizeof(long int)*d_queue_cap);
      d_latest_idx =0;
      d_current_idx =0;
      d_current_time = std::clock();
    }

    int
    su_transmitter_bb_impl::work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const unsigned char *in = (const unsigned char *) input_items[0];
      unsigned char *out = (unsigned char *) output_items[0];
      d_current_time = std::clock();
      int nout=ninput_items[0]+2;
      switch(d_mode){
        case NOQUEUE:
          out[PHY_LEN+0] = (unsigned char)d_current_idx;
          out[PHY_LEN+1] = 0x00;
          d_current_idx%=d_queue_cap;
          memcpy(out,LSA_PHY,sizeof(char)*PHY_LEN);
          memcpy(out+PHY_LEN+2,in,sizeof(char)*ninput_items[0]);
          d_current_idx = (d_current_idx+1) % d_queue_cap;
          nout = ninput_items[0]+PHY_LEN+2;
        break;
        case SUCCESSIVE:
          switch(d_state){
            case CLEAR_TO_SEND:
            {
              if(d_time_buf[d_current_idx]!=0){
                  // feedback too slow, will be overwritten
                  if(d_debug){
                    std::cerr<<"<WARNING> Feedback too slow, overwrite buffer to continue"<<std::endl;
                  }
                  reset_queue();
              }
              out[PHY_LEN+0] = (unsigned char)d_current_idx;
              out[PHY_LEN+1] = 0x00;  
              memcpy(out,LSA_PHY,sizeof(char)*PHY_LEN);
              memcpy(out+PHY_LEN+2,in,sizeof(char)*ninput_items[0]);
              memcpy(d_queue_buf[d_current_idx],in,sizeof(char)*ninput_items[0]);
              d_pkt_len_buf[d_current_idx] = ninput_items[0];
              d_time_buf[d_current_idx] = d_current_time;
              d_current_idx = (d_current_idx+1)%d_queue_cap;
              nout = ninput_items[0]+PHY_LEN+2;
            }
            break;
            case RETRANSMISSION:
            {
              assert(!d_retx_idx_buf.empty());
              out[0+PHY_LEN] = (unsigned char)d_retx_idx;
              out[1+PHY_LEN] = (unsigned char)d_retx_idx_buf.size();
              int idx_mapping = d_retx_idx_buf[d_retx_idx++];
              d_retx_idx%=d_retx_idx_buf.size();
              memcpy(out,LSA_PHY,sizeof(char)*PHY_LEN);
              memcpy(out+PHY_LEN+2, d_queue_buf[idx_mapping],sizeof(char)*d_pkt_len_buf[idx_mapping]);
              nout = d_pkt_len_buf[idx_mapping]+2+PHY_LEN;
            break;
            }
            default:
              throw std::runtime_error("Worng state");
            break;
          }
        break;
        default:
          throw std::runtime_error("Undefined mode");
        break;
      }
      return nout;
    }

  } /* namespace lsa */
} /* namespace gr */

