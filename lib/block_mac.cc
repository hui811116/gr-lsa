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
#include <lsa/block_mac.h>
#include <gnuradio/block_detail.h>
#include <ctime>

namespace gr {
  namespace lsa {


    // LSA MAC DESIGN:
    // BIT FIELD:
    // ||DESTINATION||MAC ADDRESS||BLOCK ID||
    //      1 byte      1 byte       2 bytes

    static const long int d_timeout_clocks = CLOCKS_PER_SEC*4.0;

    class block_mac_impl:public block_mac{
      public: 
        block_mac_impl(unsigned char addr,int block_len ,bool debug,bool verbose):block("block_mac",
                  gr::io_signature::make(0,0,0),
                  gr::io_signature::make(0,0,0)),
                  d_block_size(block_len),
                  d_addr(addr),
                  d_verbose(verbose)
        {
          if(block_len <0){
            throw std::invalid_argument("block length cannot be negative");
          }
          d_block_count = 0;
          d_block_clocks.resize(d_block_size, 0);
          d_current_clocks = std::clock();
          d_mac_in = pmt::mp("mac_in");
          //d_mac_out= pmt::mp("mac_out");
          d_phy_in = pmt::mp("phy_in");
          d_phy_out= pmt::mp("phy_out");
          //message_port_register_out(d_mac_out);
          message_port_register_out(d_phy_out);
          message_port_register_in(d_mac_in);
          message_port_register_in(d_phy_in);
          set_msg_handler(d_mac_in,boost::bind(&block_mac_impl::mac_in,this,_1));
          set_msg_handler(d_phy_in,boost::bind(&block_mac_impl::phy_in,this,_1));
          d_debug = debug;

          d_success_pkt = 0;
          d_sum_of_rtt_clocks =0;
          
          d_mac_buf[1] = d_addr;
        }

        ~block_mac_impl()
        {

        }
        void mac_in(pmt::pmt_t msg){
          long int duration = std::clock() - d_current_clocks;
          if(d_block_count<d_block_size){
            assert(pmt::is_pair(msg));
            pmt::pmt_t k = pmt::car(msg);
            pmt::pmt_t v = pmt::cdr(msg);
            assert(pmt::is_blob(v));
            size_t io(0);
            const unsigned char* uvec = pmt::u8vector_elements(v,io);
            assert(io<2024);
            memcpy(d_mac_buf+4,uvec,sizeof(char)*io);
            // first thing to do is filling the buffer
            assert(pmt::is_dict(k));
            assert(pmt::dict_has_key(k,pmt::intern("DEST_ADDR")));
            d_dest = pmt::to_long(pmt::dict_ref(k,pmt::intern("DEST_ADDR"),pmt::from_long(0)));
            d_current_clocks = std::clock();
            d_block_clocks[d_block_count] = d_current_clocks;
            uint16_t bc_u16 = (uint16_t) d_block_count;
            unsigned char * bc_u8arr = (unsigned char*)&bc_u16;
            d_mac_buf[0] = d_dest;
            d_mac_buf[2] = bc_u8arr[1];
            d_mac_buf[3] = bc_u8arr[0];
            d_block_count++;
            message_port_pub(d_phy_out,pmt::cons(pmt::intern("DATA"),pmt::make_blob(d_mac_buf,io+4)));
          }
          else if(duration > d_timeout_clocks){
            // timeout, summarize the result if verbose
            if(d_verbose){
              report();
            }
            // reset before new block transmission
            reset();
            return;
          }
          else{
            // not timeout yet, do not take extra data
            // 
            return;
          }
          
        }

        //void mac_out(){
          //message_port_pub(d_mac_out,);
        //}

        void phy_in(pmt::pmt_t msg)
        {
          assert(pmt::is_pair(msg));
          pmt::pmt_t k = pmt::car(msg);
          pmt::pmt_t v = pmt::cdr(msg);
          assert(pmt::is_blob(v));
            size_t io(0);
            const uint8_t* u8 = pmt::u8vector_elements(v,io);
            assert(io>=4);
            if(d_addr != u8[0]){
              // not intended for this user, may be cross talk from other antenna
                return ;
              }
          if(pmt::eqv(pmt::intern("LSA_DATA"),k))
          {
              d_mac_buf[0] = u8[1];
              d_mac_buf[2] = u8[2];
              d_mac_buf[3] = u8[3];
              message_port_pub(d_phy_out,pmt::cons(pmt::intern("ACK"),pmt::make_blob(d_mac_buf,4))); 
          }
          else if(pmt::eqv(pmt::intern("ACK"),k))
          {
            uint16_t block_num = 0x00;
            block_num = u8[2]<<8 | u8[3];
            if(block_num >= d_block_count){
              return;
            }
            d_success_pkt++;
            d_sum_of_rtt_clocks += std::clock() - d_block_clocks[block_num];
          }
        }

        void reset(){
          d_block_count = 0;
          d_current_clocks=std::clock();
          d_block_clocks.resize(d_block_size,0);
          d_success_pkt = 0;
          d_sum_of_rtt_clocks =0;
          d_dest = 0;
        }
        void report(){
          double avg_rtt_time = 0;
          if(d_success_pkt==0){
            std::cout<<"<BLOCK MAC>NO successful transmission, cannot calculate RTT!"<<std::endl;
          }
          else{
            avg_rtt_time = d_sum_of_rtt_clocks/(double)d_success_pkt/(double)CLOCKS_PER_SEC*1000;
            std::cout<<"-----------------<BLOCK MAC>(Delay statistics)------------------"<<std::endl;
            std::cout<<"Successive Packets: "<<d_success_pkt<<std::endl;
            std::cout<<"Average Round Trip Time(ms):"<<avg_rtt_time<<std::endl;
            std::cout<<"****************************************************************"<<std::endl;
          }
        }
        

      private:
        pmt::pmt_t d_mac_out;
        pmt::pmt_t d_mac_in;
        pmt::pmt_t d_phy_out;
        pmt::pmt_t d_phy_in;

        size_t d_block_count;
        long int d_current_clocks;
        
        unsigned char d_addr;
        unsigned char d_dest;

        std::vector<long int> d_block_clocks;
        size_t d_success_pkt;
        uint64_t d_sum_of_rtt_clocks;
        
        bool d_debug;
        bool d_verbose;
        const size_t d_block_size;

        unsigned char d_mac_buf[2048];
    };

    block_mac::sptr
    block_mac::make(unsigned char addr,int block_len,bool debug,bool verbose){
      return gnuradio::get_initial_sptr(new block_mac_impl(addr,block_len,debug,verbose));
    }

  } /* namespace lsa */
} /* namespace gr */

