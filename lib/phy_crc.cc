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
#include <lsa/phy_crc.h>
#include <gnuradio/block_detail.h>

namespace gr {
  namespace lsa {
    #define d_debug false
    #define DEBUG d_debug && std::cout
    #define LSAMAXLEN 121
    #define SNSMAXLEN 123
    #define PROUMAXLEN 123
    #define PROUMINLEN 4
    #define LSAMINLEN 8
    #define SNSMINLEN 4
    enum USERTYPE{
      PROU=0,
      LSA=1,
      SNS=2
    };

    class phy_crc_impl : public phy_crc
    {
      public:
        phy_crc_impl(int user):block("phy_crc",
                gr::io_signature::make(0,0,0),
                gr::io_signature::make(0,0,0)),
                d_in_port(pmt::mp("phy_in")),
                d_out_port(pmt::mp("pdu_out")),
                d_thr_port(pmt::mp("thr_out"))
        {
          message_port_register_in(d_in_port);
          message_port_register_out(d_out_port);
          message_port_register_out(d_thr_port);
          set_msg_handler(d_in_port,boost::bind(&phy_crc_impl::msg_in,this,_1));
          switch(user){
            case PROU:
              d_min_len = PROUMINLEN;
              d_max_len = PROUMAXLEN;
            break;
            case LSA:
              d_min_len = LSAMINLEN;
              d_max_len = LSAMAXLEN;
              d_lsa_queue_table.clear();
            break;
            case SNS:
              d_min_len = SNSMINLEN;
              d_max_len = SNSMAXLEN;
            break;
            default:
              throw std::invalid_argument("Undefined user type");
            break;
          }
          d_user = user;
        }
        ~phy_crc_impl(){}

        void msg_in(pmt::pmt_t msg)
        {
          gr::thread::scoped_lock guard(d_mutex);
          pmt::pmt_t k = pmt::car(msg);
          pmt::pmt_t v = pmt::cdr(msg);
          assert(pmt::is_blob(v));
          size_t io(0);
          const uint8_t* uvec = pmt::u8vector_elements(v,io);
          if(io<=d_min_len || io>d_max_len){
            return;
          }
          // crc for different user type
          if(phy_crc(uvec,io)){
            // crc passed
            pmt::pmt_t thr_msg;
            if(d_user == LSA){
              if(d_qsize!=d_lsa_queue_table.size()){
                d_lsa_queue_table.clear();
                d_lsa_queue_table.resize(d_qsize,false);
              }
              if(d_qsize!=0){
                if(d_lsa_queue_table[d_qidx]==false){
                  // a new retransmission
                  d_lsa_queue_table[d_qidx] = true;
                  thr_msg = pmt::make_blob(uvec+d_min_len,io-d_min_len);
                  message_port_pub(d_thr_port,pmt::cons(pmt::from_long(d_seq),thr_msg));
                }
              }else if(d_qsize==0 && d_qidx==0){
                thr_msg = pmt::make_blob(uvec+d_min_len,io-d_min_len);
                message_port_pub(d_thr_port,pmt::cons(pmt::from_long(d_seq),thr_msg));
              }
            }else{
              thr_msg = pmt::make_blob(uvec+d_min_len,io-d_min_len);
              // for throughput measurement and ber calculation
              message_port_pub(d_thr_port,pmt::cons(pmt::from_long(d_seq),thr_msg));
            }
            message_port_pub(d_out_port,pmt::cons(pmt::from_long(d_seq),v));
          }
        }

      private:
        bool phy_crc(const uint8_t* uvec, size_t io)
        {
          uint16_t base1, base2;
          uint16_t qidx,qsize;
          switch(d_user){
            case PROU:
              base1 = uvec[0]<<8;
              base1|= uvec[1];
              base2 = uvec[2]<<8;
              base2|= uvec[3];
              if(base1!=base2){
                return false;
              }
              d_seq = base1;
            break;
            case LSA:
              qidx = uvec[0]<<8;
              qidx|= uvec[1];
              qsize= uvec[2]<<8;
              qsize|=uvec[3];
              base1 =uvec[4]<<8;
              base1|=uvec[5];
              base2 =uvec[6]<<8;
              base2|=uvec[7];
              if( (qsize!=0 && qidx>=qsize) || base1!=base2 ){
                return false;
              }
              d_seq = base1;
              d_qidx = qidx;
              d_qsize =qsize;
            break;
            case SNS:
              base1 = uvec[0]<<8;
              base1|= uvec[1];
              base2 = uvec[2]<<8;
              base2|= uvec[3];
              if(base1!=base2){
                return false;
              }
              d_seq = base1;
            break;
            default:
              throw std::runtime_error("Undefined user type");
            break;
          }
          return true;
        }
        gr::thread::mutex d_mutex;
        const pmt::pmt_t d_in_port;
        const pmt::pmt_t d_out_port;
        const pmt::pmt_t d_thr_port;
        int d_user;
        std::vector<bool> d_lsa_queue_table;
        int d_min_len;
        int d_max_len;
        uint16_t d_qidx;
        uint16_t d_qsize;
        uint16_t d_seq;
    };

    phy_crc::sptr
    phy_crc::make(int user)
    {
      return gnuradio::get_initial_sptr(new phy_crc_impl(user));
    }

  } /* namespace lsa */
} /* namespace gr */

