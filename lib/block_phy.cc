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
#include <lsa/block_phy.h>
#include <gnuradio/block_detail.h>

namespace gr {
  namespace lsa {
    
#define DEBUG d_debug && std::cerr

    static const unsigned char LSAPREAMBLE[]={0x00,0x00,0x00,0x00,0xE6,0x00}; // last one for pkt len
    static const int LSAPHYLEN = 6;
    static const int LSAMAXLEN = 127-2;

  class block_phy_impl:public block_phy
  {
    public:
    block_phy_impl(bool debug) : block("block_phy",
                gr::io_signature::make(0,0,0),
                gr::io_signature::make(0,0,0)),
                d_mac_in_port(pmt::mp("mac_in")),
                d_mac_out_port(pmt::mp("mac_out")),
                d_phy_in_port(pmt::mp("phy_in")),
                d_phy_out_port(pmt::mp("phy_out"))
    {
      message_port_register_in(d_mac_in_port);
      message_port_register_in(d_phy_in_port);
      message_port_register_out(d_mac_out_port);
      message_port_register_out(d_phy_out_port);
      set_msg_handler(d_mac_in_port,boost::bind(&block_phy_impl::mac_in,this,_1));
      set_msg_handler(d_phy_in_port,boost::bind(&block_phy_impl::phy_in,this,_1));
      d_debug = debug;
    }
    ~block_phy_impl()
    {
    }
    void mac_in(pmt::pmt_t msg)
    {
      if(!pmt::is_dict(msg)){
        // maybe is an ack frame
        throw std::runtime_error("<BLOCK PHY> input not a dictionary");
      }
      if(pmt::dict_has_key(msg,pmt::intern("LSA_ACK"))){
        pmt::pmt_t dict_item = pmt::car(msg);
        pmt::pmt_t k = pmt::car(dict_item);
        pmt::pmt_t v = pmt::cdr(dict_item);
        assert(pmt::is_blob(v));
        size_t oo(0);
        const uint8_t* ackvec = pmt::u8vector_elements(v,oo);
        memcpy(d_buf,LSAPREAMBLE,sizeof(char)*LSAPHYLEN);
        d_buf[LSAPHYLEN-1] = (unsigned char)oo;
        memcpy(d_buf+LSAPHYLEN,ackvec,sizeof(char)*oo);
        message_port_pub(d_phy_out_port,pmt::cons(pmt::PMT_NIL,pmt::make_blob(d_buf,LSAPHYLEN+oo)));
        return;
      }
      else{
        d_buf_cnt =0;
        std::vector<pmt::pmt_t> blob_stack;
        pmt::pmt_t dict_item(pmt::dict_items(msg));
        while(!pmt::is_null(dict_item)){
          pmt::pmt_t this_dict(pmt::car(dict_item));
          pmt::pmt_t k = pmt::car(this_dict);
          pmt::pmt_t v = pmt::cdr(this_dict);
          dict_item = pmt::cdr(dict_item);
          // NOTE the order is reverse
          blob_stack.push_back(v);
        }
        for(int i = blob_stack.size()-1;i>=0;--i){
          size_t io(0);
          const uint8_t* uvec = pmt::u8vector_elements(blob_stack[i],io);
          assert(io<LSAMAXLEN);
          memcpy(&d_buf[d_buf_cnt],LSAPREAMBLE,sizeof(char)*LSAPHYLEN);
          d_buf[d_buf_cnt+LSAPHYLEN-1] = (unsigned char) io;
          memcpy(&d_buf[d_buf_cnt+LSAPHYLEN],uvec,sizeof(char)*io);
          d_buf_cnt+= (io+LSAPHYLEN);
        }
        pmt::pmt_t blob = pmt::make_blob(d_buf,d_buf_cnt);
        message_port_pub(d_phy_out_port,pmt::cons(pmt::PMT_NIL,blob));
      }
      
    }
    void phy_in(pmt::pmt_t msg)
    {
      // developing for now
      assert(pmt::is_pair(msg));
      pmt::pmt_t k = pmt::car(msg);
      pmt::pmt_t v = pmt::cdr(msg);
      size_t io(0);
      const uint8_t* uvec = pmt::u8vector_elements(v,io);
      if(io==2 || io>=4){
        message_port_pub(d_mac_out_port,msg);
      }
    }
    private:
      const pmt::pmt_t d_mac_in_port;
      const pmt::pmt_t d_mac_out_port;
      const pmt::pmt_t d_phy_in_port;
      const pmt::pmt_t d_phy_out_port;
      unsigned char d_buf[8192];
      int d_buf_cnt;
      bool d_debug;
  };

  block_phy::sptr
  block_phy::make(bool debug)
  {
    return gnuradio::get_initial_sptr(new block_phy_impl(debug));
  }

  } /* namespace lsa */
} /* namespace gr */

