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
#include <lsa/su_mac.h>
#include <gnuradio/block_detail.h>
#include <thread>

namespace gr {
  namespace lsa {

    static const int LSAMACLEN = 6;

    class su_mac_impl : public su_mac
    {
      public:
      su_mac_impl(int bytes_per_packet):block("su_mac",
                  gr::io_signature::make(0,0,0),
                  gr::io_signature::make(0,0,0)),
                  d_app_in_port(pmt::mp("app_in")),
                  d_app_out_port(pmt::mp("app_out")),
                  d_mac_in_port(pmt::mp("mac_in")),
                  d_mac_out_port(pmt::mp("mac_out"))
      {
        message_port_register_in(d_app_in_port);
        message_port_register_in(d_mac_in_port);
        message_port_register_out(d_mac_out_port);
        message_port_register_out(d_app_out_port);
        set_msg_handler(d_app_in_port,boost::bind(&su_mac_impl::app_in,this,_1));
        set_msg_handler(d_mac_in_port,boost::bind(&su_mac_impl::mac_in,this,_1));
        if(bytes_per_packet<0 || bytes_per_packet >128){
          throw std::invalid_argument("Bytes per packet out of range");
        }
        d_bytes_per_packet = bytes_per_packet;
        d_current_base = -1;
        d_ack_cnt=0;
      }
      ~su_mac_impl()
      {

      }
      void app_in(pmt::pmt_t msg)
      {
        gr::thread::scoped_lock lock(d_mutex);
        assert(pmt::is_pair(msg));
        pmt::pmt_t k = pmt::car(msg);
        pmt::pmt_t v = pmt::cdr(msg);
        if(!pmt::is_blob(v)){
          throw std::runtime_error("not a blob!");
        }
        int new_base = pmt::to_long(k);
        if(new_base != d_current_base){
          // reaching retry limit
          d_current_base = new_base;
          generate_pdu(v);
          std::cerr<<"<SU MAC DEBUG> generate complete"<<std::endl;
        }
        // processing payload and base counter
        message_port_pub(d_mac_out_port,d_current_pld);
      }

      void mac_in(pmt::pmt_t msg)
      {
        gr::thread::scoped_lock lock(d_mutex);
        // TODO: still developing
        assert(pmt::is_pair(msg));
        pmt::pmt_t k = pmt::car(msg);
        pmt::pmt_t v = pmt::cdr(msg);
        if(!pmt::is_blob(v)){
          throw std::runtime_error("SU MAC input not a blob");
        }
        size_t io(0);
        const uint8_t* uvec = pmt::u8vector_elements(v,io);
        // parse
        if(io==2){
          // length reserved for sensing
          std::cerr<<"<SU MAC>received feedback of positive sensing information!"<<std::endl;
          return;
        }else{
          // queue size and queue index and base point
          unsigned int received_base = uvec[2]<<24;
          received_base |= uvec[3] << 16;
          received_base |= uvec[4] << 8;
          received_base |= uvec[5];
          // checking queue index and queue size here!
          int qidx = uvec[0];
          int qsize= uvec[1];
          if(io==6){
            // only control message
            // more likely for TX
            if(received_base == d_current_base){
              std::cerr<<"<SU MAC>found a matched base point:(expected)"<<d_current_base<<std::endl;
              if(qidx<d_ack_table.size()){
                if(d_ack_table[qidx]==false){
                  std::cerr<<"<SU MAC>segment:"<<qidx<<" ,acked!"<<std::endl;
                  d_ack_table[qidx]=true;
                  d_ack_cnt++;
                  if(d_ack_cnt==d_ack_table.size()){
                    //transmission complete
                    std::cerr<<"<SU MAC>all segments acked!"<<std::endl;
                    // report success to app layer?
                  }
                }
              }
            }
          }else{
              // contain payload
              if( (received_base != d_rx_base) && (qidx ==0) && (qsize!=0) ){
              // detect an initial frame
              d_current_base = received_base;
              d_blob_buf.clear();
              d_blob_buf.resize(qsize,pmt::PMT_NIL);
              d_blob_cnt=0;
              }
              if(pmt::is_null(d_blob_buf[qidx])){
                d_blob_buf[qidx] = pmt::make_blob(uvec,io-6); 
                d_blob_cnt++;
                // ack one, 
                pmt::pmt_t ack_frame = generate_ack_frame(qidx,qsize,received_base);
                std::cerr<<"<SU MAC DEBUG>acking segment:"<<qidx<<" of base:"<<received_base<<std::endl;
                message_port_pub(d_mac_out_port,ack_frame);
              }
              if(d_blob_cnt == d_blob_buf.size()){
                // all segments received
                std::cerr<<"<SU MAC>Received a block (base point:"<<d_current_base<<")"<<std::endl;
                int rebuild_cnt=0;
                for(int i=0;i<d_blob_buf.size();++i){
                  size_t tmp_io(0);
                  const uint8_t* tmp_uvec = pmt::u8vector_elements(d_blob_buf[i],tmp_io);
                  memcpy(&d_rx_buf[rebuild_cnt],tmp_uvec,sizeof(char)*tmp_io);
                  rebuild_cnt+=tmp_io;
                }
                message_port_pub(d_app_out_port,pmt::cons(pmt::from_long(d_rx_base),pmt::make_blob(d_rx_buf,rebuild_cnt)));
              }
          } 
        }
      }
      

      void generate_pdu(pmt::pmt_t v)
      {
        gr::thread::scoped_lock lock(d_mutex);
        size_t io(0);
        const uint8_t* uvec = pmt::u8vector_elements(v,io);
        // for integer part
        size_t npacket = io/d_bytes_per_packet;
        npacket = (io%d_bytes_per_packet==0)? npacket:npacket+1;
        // NOTE: only record base point;
        pmt::pmt_t msg_out = pmt::make_dict();
        size_t len_per_packet = 2+4+d_bytes_per_packet;
        unsigned int base = (unsigned int) d_current_base;
        unsigned char* u8_base = (unsigned char*) &base;
        d_buf[2] = u8_base[3];
        d_buf[3] = u8_base[2];
        d_buf[4] = u8_base[1];
        d_buf[5] = u8_base[0];
        for(int i=0;i<io/d_bytes_per_packet;++i){
          pmt::pmt_t tmp_msg;
          d_buf[0] = (unsigned char) i;
          d_buf[1] = (unsigned char) npacket;  
          memcpy(d_buf+6,uvec+i*d_bytes_per_packet,sizeof(char)*d_bytes_per_packet);
          tmp_msg = pmt::make_blob(d_buf,d_bytes_per_packet+6);
          msg_out = pmt::dict_add(msg_out,pmt::from_long(i),tmp_msg);
        }
        int residual = io%d_bytes_per_packet;
        if(residual!=0){
          d_buf[0] = (unsigned char)npacket-1;
          d_buf[1] = (unsigned char)npacket;
          memcpy(d_buf+6,uvec+d_bytes_per_packet*(npacket-1),sizeof(char)*residual);
          pmt::pmt_t tmp_msg = pmt::make_blob(d_buf,residual+6);
          msg_out = pmt::dict_add(msg_out,pmt::from_long(npacket-1),tmp_msg);
        }
        d_ack_table.clear();
        d_ack_table.resize(npacket,false);
        d_ack_cnt =0;
        d_current_pld = msg_out;
      }

      pmt::pmt_t generate_ack_frame(int qidx, int qsize,unsigned int base)
      {
        gr::thread::scoped_lock lock(d_mutex);
        // this is intended for receiver--> d_buf not used in rx mode
        d_buf[0] = (unsigned char) qidx;
        d_buf[1] = (unsigned char) qsize;
        unsigned char* base_u8 = (unsigned char*)&base;
        d_buf[2] = base_u8[3];
        d_buf[3] = base_u8[2];
        d_buf[4] = base_u8[1];
        d_buf[5] = base_u8[0];
        return pmt::cons(pmt::intern("LSA_ACK"),pmt::make_blob(d_buf,LSAMACLEN));
      }

      private:
        const pmt::pmt_t d_app_in_port;
        const pmt::pmt_t d_app_out_port;
        const pmt::pmt_t d_mac_in_port;
        const pmt::pmt_t d_mac_out_port;
        
        gr::thread::mutex d_mutex;
        //gr::thread::condition_variable d_cond_var
        std::vector<bool> d_ack_table;          // transmitter acking table
        int d_ack_cnt;                          // transmitter ack counter
        int d_current_base;                     // base point for transmitter
        pmt::pmt_t d_current_pld;               // temporary buffer for transmitter
        int d_bytes_per_packet;                 // transmission parameters for TX

        std::vector<pmt::pmt_t> d_blob_buf;     // receiver buffer for multiple segments
        int d_blob_cnt;                         // receiver segment buffer
        int d_rx_base;                          // base point for receiver
        
        //bool d_compensate;
        // NOTE: trying to accommondate multiple payloads
        unsigned char d_buf[2048];              // transmitter buffer
        unsigned char d_rx_buf[2048];           // receiver rebuild buffer

    };

    su_mac::sptr
    su_mac::make(int bytes_per_packet)
    {
      return gnuradio::get_initial_sptr(new su_mac_impl(bytes_per_packet));
    }

  } /* namespace lsa */
} /* namespace gr */

