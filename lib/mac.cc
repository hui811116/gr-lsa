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
#include <lsa/mac.h>
#include <gnuradio/block_detail.h>
#include <ctime>

namespace gr {
  namespace lsa {

    enum LSAMAC{
      IDLE,
      BUSY_HAND,
      BUSY_WAIT_DATA,
      BUSY_SEND_DATA
    };
    enum LSAOUT{
      SUCCESS,
      MAC_BUSY,
      FAILED,
      DATA
    };
    enum LSACTRL{
      ACK,
      NACK,
      REQ,
      SEN
    };

    class mac_impl : public mac{
      public: 

      mac_impl(unsigned int addr):block ("mac",
                        gr::io_signature::make(0,0,0),
                        gr::io_signature::make(0,0,0)),
                        d_timeout_clocks(CLOCKS_PER_SEC*2.0),
                        d_addr(addr)
      {
          d_to_mac_port = pmt::mp("msg_in");
          d_from_mac_port = pmt::mp("msg_out");
          d_to_phy_port = pmt::mp("to_phy");
          d_from_phy_port = pmt::mp("phy_in");
          d_to_ctrl_port = pmt::mp("to_ctrl");
          d_from_ctrl_port = pmt::mp("from_ctrl");

          message_port_register_out(d_to_phy_port);
          message_port_register_out(d_from_mac_port);
          message_port_register_in(d_to_mac_port);
          message_port_register_in(d_from_phy_port);
          message_port_register_in(d_from_ctrl_port);
          message_port_register_out(d_to_ctrl_port);

          set_msg_handler(d_from_phy_port, boost::bind(&mac_impl::from_phy, this , _1));
          set_msg_handler(d_to_mac_port, boost::bind(&mac_impl::to_mac, this , _1));
          set_msg_handler(d_from_ctrl_port, boost::bind(&mac_impl::from_ctrl, this, _1));

          d_hdr_buf = new unsigned char [1024];
          d_ctrl_buf = new unsigned char [256];
          d_mac_state = IDLE;
          d_current_clocks = std::clock();
          d_dest = 0;
          d_hdr_buf[1] = d_addr;
          d_ctrl_buf[1]= d_addr;
      }

      ~mac_impl(){
          delete [] d_hdr_buf;
          delete [] d_ctrl_buf;
      }

      void 
      to_mac(pmt::pmt_t msg){
        long int duration;
        pmt::pmt_t k= pmt::car(msg);
        pmt::pmt_t v= pmt::cdr(msg);
        if(!pmt::is_blob(v)){
          throw std::runtime_error("Input to mac not a blob");
        }
        assert(pmt::is_dict(k));
        switch(d_mac_state){
          case BUSY_HAND:
          case BUSY_WAIT_DATA:
          case BUSY_SEND_DATA:
          duration = std::clock() - d_current_clocks;
            if(duration<d_timeout_clocks){
              from_mac(MAC_BUSY,pmt::PMT_NIL);
              return;
            }
          case IDLE:
            // first try to handshake destination
            assert(pmt::dict_has_key(pmt::intern("DEST_ADDR")));
            d_dest = pmt::to_long(pmt::dict_ref(k,pmt::intern("DEST_ADDR"),pmt::from_long(0x00)));
            d_mac_state = BUSY_HAND;
            d_current_clocks = std::clock();
            //std::cout<<"<debug>addr="<<(int)d_addr<<" ,change to state BUSY_HAND"<<std::endl;
            to_ctrl(REQ,d_dest);
            //std::cout<<"<debug>addr="<<(int)d_addr<<"storing data"<<std::endl;
            store_data(v);
          break;
          default:
            throw std::runtime_error("<LSA MAC>ERROR: to_mac bad state");
          break;
        }
      }

      void 
      from_mac(LSAOUT mac_ctrl, pmt::pmt_t blob){
        pmt::pmt_t content=pmt::PMT_NIL;
        pmt::pmt_t dict=pmt::make_dict();
        switch(mac_ctrl)
        {
          case SUCCESS:
            dict = pmt::dict_add(dict,pmt::intern("LSA_MAC"),pmt::intern("SUCCESS"));
            d_mac_state = IDLE;
            d_current_clocks = std::clock();
            d_pdu = pmt::PMT_NIL;
          break;
          case FAILED:
            dict = pmt::dict_add(dict,pmt::intern("LSA_MAC"),pmt::intern("FAILED"));
            d_mac_state = IDLE;
            d_current_clocks = std::clock();
            d_pdu = pmt::PMT_NIL;
          break;
          case MAC_BUSY:
            dict = pmt::dict_add(dict,pmt::intern("LSA_MAC"),pmt::intern("BUSY"));
          break;
          case DATA:
            assert(pmt::is_blob(blob));
            assert(d_mac_state == BUSY_WAIT_DATA);
            content = blob;
            dict = pmt::dict_add(dict,pmt::intern("LSA_MAC"),pmt::intern("DATA"));
            d_mac_state = IDLE;
            d_current_clocks = std::clock();
            // current testing version do not print out pdu
          break;
          default:
            throw std::runtime_error("<LSA MAC>ERROR: from_mac bad state");
          break;
        }
        message_port_pub(d_from_mac_port,pmt::cons(dict,content));
      }

      void 
      from_phy(pmt::pmt_t msg){
        long int duration = std::clock()-d_current_clocks;
        if(duration > d_timeout_clocks)
            from_mac(FAILED,pmt::PMT_NIL);
        return ;
        pmt::pmt_t k = pmt::car(msg);
        pmt::pmt_t v = pmt::cdr(msg);
        if(pmt::eqv(pmt::intern("LSA_DATA"),k) && (d_mac_state==BUSY_WAIT_DATA) ){
          
          assert(pmt::is_blob(v));
          //std::cout<<"<DEBUG>addr="<<(int)d_addr<<" ,passing blob check"<<std::endl;
          parse_mac_hdr(v);
          unsigned char to_addr=d_mac_buf[0], from_addr=d_mac_buf[1];
            if((to_addr == d_addr) &&(from_addr == d_dest) ){
              //std::cout<<"<DEBUG>addr="<<(int)d_addr<<" Receive from <<"<<from_addr<<", ack back..."<<std::endl;
              d_current_clocks = std::clock();
              to_ctrl(ACK,d_dest);
              size_t io(0); 
              const uint8_t* uvec= pmt::u8vector_elements(v,io);
              assert(io>=2);
              from_mac(DATA,pmt::make_blob(uvec+2,io-2));
            }
        }
        // other case for future development
        else if(pmt::eqv(pmt::intern("LSA_DROP"),k)){
          //to_ctrl(NACK,d_dest);
        }
        else if(pmt::eqv(pmt::intern("SENSE"),k)){

        }
      }

      void 
      to_phy(unsigned char dest){
        assert(!pmt::is_null(d_pdu));
        assert(pmt::is_pair(d_pdu));
        pmt::pmt_t dest_addr = pmt::car(d_pdu);
        pmt::pmt_t blob = pmt::cdr(d_pdu);
        size_t io(0);
        const uint8_t* uvec = pmt::u8vector_elements(blob,io);
        memcpy(d_hdr_buf+2,uvec,sizeof(char)*io);
        d_hdr_buf[0] = pmt::to_long(dest_addr);
        d_hdr_buf[1] = d_addr;
        //std::cout<<"<DEBUG>addr="<<(int)d_addr<<" ,sending pdu phy..."<<std::endl;
        message_port_pub(d_to_phy_port, pmt::cons(pmt::intern("DATA"),pmt::make_blob(d_hdr_buf,io+2)) );
      }

      void
      to_ctrl(LSACTRL type, unsigned char dest)
      {
        pmt::pmt_t ctrl;
        std::string ctr_type;
        const unsigned char* ptr;
        d_ctrl_buf[0] = dest;
        d_ctrl_buf[1] = d_addr;
        switch(type)
        {
          case REQ:
            //ctr_type = "REQ";
          case ACK:
            ctr_type = "ACK";
          break;
          case NACK:
            ctr_type = "NACK";
          break;
          case SEN:
            ctr_type = "SEN";
          break;
        }
        ctrl = pmt::cons(pmt::string_to_symbol(ctr_type),pmt::init_u8vector(2,d_ctrl_buf));
        message_port_pub(d_to_ctrl_port,ctrl);
      }

      void
      from_ctrl(pmt::pmt_t msg)
      {
        pmt::pmt_t ctrl = pmt::car(msg);
        pmt::pmt_t blob = pmt::cdr(msg);
        assert(pmt::is_blob(blob));
        parse_mac_hdr(blob);
        // checking address
        if(d_mac_buf[0] != d_addr)
          return;
        // suppose address checking passed
        long int duration = std::clock() - d_current_clocks;
        if(d_mac_state==IDLE && pmt::eqv(pmt::intern("ACK"),ctrl)){
          // someone try to connect you
          // first check address and change state
            //std::cout<<"<DEBUG>addr="<<(int)d_addr << " ,receiving request to send"<<std::endl;
            d_dest = d_mac_buf[1];
            d_mac_state = BUSY_WAIT_DATA;
            d_current_clocks = std::clock();
            to_ctrl(ACK,d_dest);
          //assume address checked
        }
        else if( (d_mac_state == BUSY_HAND) && (d_dest == d_mac_buf[1])){
          if(duration > d_timeout_clocks){
            from_mac(FAILED,pmt::PMT_NIL);
            return;
          }
          if(pmt::eqv(pmt::intern("ACK"),ctrl)){
            // hand shaking success, ready to send
            //std::cout<<"<DEBUG>addr="<<(int)d_addr << " ,Handshaking complete send data"<<std::endl;
            d_mac_state = BUSY_SEND_DATA;
            d_current_clocks = std::clock();
            to_phy(d_dest);
          }
        }
        else if( (d_mac_state == BUSY_SEND_DATA) && (d_dest == d_mac_buf[1])){
          if(pmt::eqv(pmt::intern("ACK"),ctrl)){
            // data been received, report success and change state
            //std::cout<<"<DEBUG>addr="<<(int)d_addr << " ,Transmit complete, resetting mac"<<std::endl;
            from_mac(SUCCESS,pmt::PMT_NIL);
          }
          else if(pmt::eqv(pmt::intern("NACK"),ctrl)){
            // data failed
            if(duration<d_timeout_clocks){
              // retransmission go!
              to_phy(d_dest);
            }
            else{
              //std::cout<<"<DEBUG>addr="<<(int)d_addr << " ,timeout reporting failed"<<std::endl;
              from_mac(FAILED,pmt::PMT_NIL);
            }
          }
          else if(pmt::eqv(pmt::intern("SENSE"),ctrl)){
            // for future development;
            // depends on scheme
            // retransmit or stop n wait
          }
        }
      }

      void
      store_data(pmt::pmt_t blob)
      {
        assert(pmt::is_blob(blob));
        //size_t io(0);
        //const unsigned char* uvec = (unsigned char*) pmt::u8vector_elements(blob,io);
        //memcpy(d_hdr_buf+2,uvec,sizeof(char)*io);
        d_pdu = pmt::cons(pmt::from_long(d_dest),blob);
      }
      void
      parse_mac_hdr(pmt::pmt_t blob)
      {
        // parse MAC header
        size_t vlen = pmt::blob_length(blob)/sizeof(char);
        size_t io(0);
        const uint8_t* uvec = pmt::u8vector_elements(blob,io);
        assert(io>=2);
        memcpy(d_mac_buf, uvec, sizeof(char)*2);
      }

      private:
        pmt::pmt_t d_to_mac_port;
        pmt::pmt_t d_from_mac_port;
        pmt::pmt_t d_to_phy_port;
        pmt::pmt_t d_from_phy_port;

        pmt::pmt_t d_from_ctrl_port;
        pmt::pmt_t d_to_ctrl_port;

        pmt::pmt_t d_pdu;

        unsigned char* d_hdr_buf;
        unsigned char* d_ctrl_buf;
        unsigned char d_mac_buf[256];
        unsigned char d_addr;
        unsigned char d_dest;

        int d_mac_state;
        const long int d_timeout_clocks;
        long int d_current_clocks;
    };

    mac::sptr
    mac::make(unsigned int addr){
      return gnuradio::get_initial_sptr(new mac_impl(addr));
    }

  } /* namespace lsa */
} /* namespace gr */

