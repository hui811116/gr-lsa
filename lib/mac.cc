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

    /*mac::mac()
    {
    }

    mac::~mac()
    {
    }*/
    static const unsigned char lsa_SFD[] = {0xa7, 0xff};
    
    static const unsigned char lsa_ACK[] = {0x00, 0x00};
    static const unsigned char lsa_NACK[] = {0x01,0x01};

    enum LSAMAC{
      IDLE,
      BUSY_HAND,
      BUSY_DATA,
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

      mac_impl():block ("mac",
                        gr::io_signature::make(0,0,0),
                        gr::io_signature::make(0,0,0)),
                        d_timeout_clocks(CLOCKS_PER_SEC*0.01) {
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
          //pre setting SFD
          memcpy(d_hdr_buf,&lsa_SFD,sizeof(char)*2);
          memcpy(d_ctrl_buf,&lsa_SFD,sizeof(char)*2);
          d_mac_state = IDLE;
          d_current_clocks = std::clock();
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
        switch(d_mac_state){
          case IDLE:
            // first try to handshake destination
            d_mac_state = BUSY_HAND;
            //to_phy(k,v);
            d_current_clocks = std::clock();
            to_ctrl(REQ,k);
            copy_data(v,k);
          break;
          case BUSY_HAND:
          case BUSY_DATA:
            duration = std::clock() - d_current_clocks;
            if(duration>d_timeout_clocks){
              from_mac(FAILED,pmt::PMT_NIL);
            }
            else
            {
              from_mac(MAC_BUSY,pmt::PMT_NIL);
            }
          break;
          default:
            throw std::runtime_error("<LSA MAC>ERROR: to_mac bad state");
          break;
        }
      }

      void 
      from_mac(int mac_ctrl, pmt::pmt_t blob){
        pmt::pmt_t dict=pmt::make_dict();
        switch(mac_ctrl)
        {
          case SUCCESS:
            dict = pmt::dict_add(dict,pmt::intern("LSA_MAC"),pmt::intern("SUCCESS"));
            d_mac_state = IDLE;
            d_current_clocks = std::clock();
          break;
          case FAILED:
            dict = pmt::dict_add(dict,pmt::intern("LSA_MAC"),pmt::intern("FAILED"));
            d_mac_state = IDLE;
            d_current_clocks = std::clock();
          break;
          case MAC_BUSY:
            dict = pmt::dict_add(dict,pmt::intern("LSA_MAC"),pmt::intern("BUSY"));
          break;
          case DATA:
            assert(pmt::is_blob(blob));
            // temporary reporting result only, not data
            //dict = pmt::cons(pmt::intern("LSA_DATA"),blob);
            dict = pmt::dict_add(dict,pmt::intern("LSA_MAC"),pmt::intern("DATA"));
          break;
          default:
            throw std::runtime_error("<LSA MAC>ERROR: from_mac bad state");
          break;
        }
        message_port_pub(d_from_mac_port,dict);
      }

      void 
      from_phy(pmt::pmt_t msg){
        long int duration = std::clock()-d_current_clocks;
        pmt::pmt_t k = pmt::car(msg);
        pmt::pmt_t v = pmt::cdr(msg);
        //std::cout<<"time duration:"<<duration<< " ,timeout:"<<d_timeout_clocks<<std::endl;
        //if(pmt::eqv(pmt::intern("LSA_ACK"),k)){
          //from_mac(SUCCESS,pmt::PMT_NIL);
        //}
        //else if(pmt::eqv(pmt::intern("LSA_NACK"),k)){
          //if(duration<d_timeout_clocks){
            //retransmission since not timeout yet
            //message_port_pub(d_to_phy_port,d_prefix_pdu);
          //}
          //else{
            //timeout report failed
            //from_mac(FAILED,pmt::PMT_NIL);
          //}
        //}
        //else if(pmt::eqv(pmt::intern("LSA_DATA"),k)){
        if(pmt::eqv(pmt::intern("LSA_DATA"),k)){
          //to_phy(pmt::intern("ACK"),pmt::PMT_NIL);
          //FIXME
          // find a way to deliver mac address
          to_ctrl(ACK,pmt::intern("ADDR"));
          // collected payload pass to upper layer
          size_t vlen = pmt::blob_length(v)/sizeof(char);
          // length check can be placed here
          if(pmt::is_blob(v)){
            from_mac(DATA,v);
          }
          // 
        }
        // other case for future development
        else if(pmt::eqv(pmt::intern("LSA_DROP"),k)){
          to_ctrl(NACK,pmt::intern("ADDR"));
        }
        else if(pmt::eqv(pmt::intern("SENSE"),k)){

        }
        
      }

      void 
      to_phy(){
        assert(!pmt::is_null(d_prefix_pdu));
        message_port_pub(d_to_phy_port, d_prefix_pdu);
      }

      void
      to_ctrl(LSACTRL type, pmt::pmt_t addr)
      {
        pmt::pmt_t ctrl;
        std::string ctr_type;
        const unsigned char* ptr;
        // insert address (self and dest)
        d_ctrl_buf[4] = 0x00;
        d_ctrl_buf[5] = 0x00;
        switch(type)
        {
          case REQ:
            ctr_type = "REQ";
          case ACK:
            d_ctrl_buf[2] = 0x00;
            d_ctrl_buf[3] = 0x00;
            ctr_type = "ACK";
          break;
          case NACK:
            d_ctrl_buf[2] = 0x01;
            d_ctrl_buf[3] = 0x01;
            ctr_type = "NACK";
          break;
          case SEN:
            d_ctrl_buf[2] = 0xff;
            d_ctrl_buf[3] = 0xff;
            ctr_type = "SEN";
          break;
        }
        ctrl = pmt::cons(pmt::string_to_symbol(ctr_type),pmt::init_u8vector(6,d_ctrl_buf));
        message_port_pub(d_to_ctrl_port,ctrl);
      }

      void
      from_ctrl(pmt::pmt_t msg)
      {
        pmt::pmt_t ctrl = pmt::car(msg);
        pmt::pmt_t addr = pmt::cdr(msg);
        // FIXME checking address here
        // suppose address checking passed
        if(d_mac_state==IDLE && pmt::eqv(pmt::intern("REQ"),ctrl)){
          // someone try to connect you
          // first check address and change state
          // if(pmt::eqv(addr,d_addr))
          //FIXME
          pmt::pmt_t dest_addr=pmt::intern("DEST_ADDR");
          //assume address checked
          d_mac_state = BUSY_HAND;
          d_current_clocks = std::clock();
          to_ctrl(ACK,dest_addr);
        }
        else if(d_mac_state == BUSY_HAND){
          if(pmt::eqv(pmt::intern("ACK"),ctrl)){
            // hand shaking success, ready to send
            d_mac_state = BUSY_DATA;
            to_phy();
          }
        }
        else if(d_mac_state == BUSY_DATA){
          if(pmt::eqv(pmt::intern("ACK"),ctrl)){
            // data been received, report success and change state
            from_mac(SUCCESS,pmt::PMT_NIL);
          }
          else if(pmt::eqv(pmt::intern("NACK"),ctrl)){
            // data failed
            long int duration = std::clock() - d_current_clocks;
            if(duration<d_timeout_clocks){
              // retransmission go!
              to_phy();
            }
            else{
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
      copy_data(pmt::pmt_t blob, pmt::pmt_t dest)
      {
        assert(pmt::is_blob(blob));
        
        size_t io(0);
        const unsigned char* uvec = (unsigned char*) pmt::u8vector_elements(blob,io);
        
        uint8_t* pld_MSB = (uint8_t*) &io;
        d_hdr_buf[2] = pld_MSB[1];
        d_hdr_buf[3] = pld_MSB[0];
        d_hdr_buf[4] = pld_MSB[1];
        d_hdr_buf[5] = pld_MSB[0];
        // FIXME
        // insert detination
        memcpy(d_hdr_buf+6,uvec,sizeof(char)*io);
        d_prefix_pdu = pmt::cons(dest,pmt::init_u8vector(io+6,d_hdr_buf));
        //message_port_pub(d_to_phy_port,d_prefix_pdu);
      }

      private:
        pmt::pmt_t d_to_mac_port;
        pmt::pmt_t d_from_mac_port;
        pmt::pmt_t d_to_phy_port;
        pmt::pmt_t d_from_phy_port;

        pmt::pmt_t d_from_ctrl_port;
        pmt::pmt_t d_to_ctrl_port;

        pmt::pmt_t d_prefix_pdu;

        unsigned char* d_hdr_buf;
        unsigned char* d_ctrl_buf;
        
        int d_mac_state;
        const long int d_timeout_clocks;
        long int d_current_clocks;
    };

    mac::sptr
    mac::make(){
      return gnuradio::get_initial_sptr(new mac_impl());
    }

  } /* namespace lsa */
} /* namespace gr */

