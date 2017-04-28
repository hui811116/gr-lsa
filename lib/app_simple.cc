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
#include <lsa/app_simple.h>
#include <gnuradio/block_detail.h>

namespace gr {
  namespace lsa {

    class app_simple_impl:public app_simple{
      public:
        app_simple_impl(unsigned int dest, bool debug):block("app_simple",
                        gr::io_signature::make(0,0,0),
                        gr::io_signature::make(0,0,0)),
                        d_dest(dest)
        {
          d_debug = debug;
          d_msg_out_port = pmt::mp("app_out");
          d_msg_in_port = pmt::mp("app_in");
          d_mac_in_port = pmt::mp("mac_in");
          d_mac_out_port= pmt::mp("mac_out");
          message_port_register_in(d_msg_in_port);
          message_port_register_in(d_mac_in_port);
          message_port_register_out(d_msg_out_port);
          message_port_register_out(d_mac_out_port);
          set_msg_handler(d_msg_in_port,boost::bind(&app_simple_impl::msg_handler,this,_1));
          set_msg_handler(d_mac_in_port,boost::bind(&app_simple_impl::mac_handler,this,_1));
        }
        
        ~app_simple_impl()
        {

        }
        
        void
        msg_handler(pmt::pmt_t msg)
        {
          pmt::pmt_t dict = pmt::make_dict();
          assert(!pmt::is_null(msg));
          if(pmt::is_blob(msg)){
            dict = pmt::dict_add(dict,pmt::intern("DEST_ADDR"),pmt::from_long(d_dest));
            message_port_pub(d_mac_out_port,pmt::cons(dict,msg));
          }
          else if(pmt::is_pair(msg)){
            pmt::pmt_t k = pmt::car(msg);
            pmt::pmt_t v = pmt::cdr(msg);
            assert(pmt::is_blob(v));
            dict = pmt::dict_add(dict,pmt::intern("MSG_KEY"),k);
            dict = pmt::dict_add(dict,pmt::intern("DEST_ADDR"),pmt::from_long(d_dest));
            message_port_pub(d_mac_out_port, pmt::cons(dict,v));
          }
          else if(pmt::is_symbol(msg)){
            std::string str = pmt::symbol_to_string(msg);
            size_t io = str.length();
            const uint8_t* uvec = reinterpret_cast<const uint8_t*>(str.c_str());
            dict = pmt::dict_add(dict,pmt::intern("DEST_ADDR"),pmt::from_long(d_dest));
            pmt::pmt_t blob = pmt::init_u8vector(io,uvec);
            //pmt::pmt_t blob = pmt::make_blob(uvec,io);
            message_port_pub(d_mac_out_port, pmt::cons(dict,blob));
          }
          else{
            // unrecognized type
            throw std::runtime_error("APP: unrecognized message type");
          }
        }

        void
        mac_handler(pmt::pmt_t msg)
        {
          assert(pmt::is_pair(msg));
          pmt::pmt_t dict= pmt::car(msg);
          pmt::pmt_t blob= pmt::cdr(msg);
          assert(pmt::dict_has_key(dict,pmt::intern("LSA_MAC")));
          pmt::pmt_t mac_info = pmt::dict_ref(dict,pmt::intern("LSA_MAC"),pmt::intern("ERROR"));
          message_port_pub(d_msg_out_port, mac_info);

          if(d_debug){
            std::string str = pmt::symbol_to_string(mac_info);
            std::cout<<"<APP SIMPLE> received message from MAC---->"<<str<<std::endl;
          if(pmt::is_blob(blob)){
            size_t io(0);
            const uint8_t* uvec = pmt::u8vector_elements(blob,io);
            //const char* msg_ptr = reinterpret_cast<const char*>(uvec);
            std::string temp((const char*)uvec,io);
            std::cout<<"Length:"<<io<<std::endl;
            std::cout<<"String form:"<<temp<<std::endl;
            std::cout<<"Content:"<<std::endl;
            //reinterpret_cast<const uint8_t*>(uvec);
            std::cout<<std::hex;
            for(int i=0;i<io;++i){
              std::cout<<(int)uvec[i];
              if((i+1)%10==0)
                std::cout<<std::endl;
              else
                std::cout<<" ";
            }
            std::cout<<std::dec<<std::endl;
            std::cout<<"--------------------------"<<std::endl;
          }
          
          }
        }
      private:
      unsigned int d_dest;
      pmt::pmt_t d_msg_out_port;
      pmt::pmt_t d_msg_in_port;
      pmt::pmt_t d_mac_out_port;
      pmt::pmt_t d_mac_in_port;
      bool d_debug;
    };

    app_simple::sptr
    app_simple::make(unsigned int dest,bool debug){
      return gnuradio::get_initial_sptr(new app_simple_impl(dest,debug));
    }

  } /* namespace lsa */
} /* namespace gr */

