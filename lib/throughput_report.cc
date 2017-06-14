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
#include <lsa/throughput_report.h>
#include <gnuradio/block_detail.h>

namespace gr {
  namespace lsa {

    #define PROUMAXLEN 127

    enum USERTYPE{
      PROU = 0,
      SU=1
    };

    inline bool lsa_crc(const uint8_t * uvec)
    {
      uint16_t qidx, qsize, base;
      qidx  = uvec[0]<<8;
      qidx |= uvec[1];
      qsize = uvec[2]<<8;
      qsize|= uvec[3];
      // can check base
      base  = uvec[4]<<8;
      base |= uvec[5];
      return qidx==0 && qsize ==0;
    }

    class throughput_report_impl:public throughput_report
    {
      public:
       throughput_report_impl(float ms, int hisLen, int user) : block("throughput_report",
                        gr::io_signature::make(0,0,0),
                        gr::io_signature::make(0,0,0)),
                        d_in_port(pmt::mp("pkt_in")),
                        d_out_port(pmt::mp("tp_out"))
       {
        message_port_register_in(d_in_port);
        set_msg_handler(d_in_port,boost::bind(&throughput_report_impl::msg_in,this,_1));
        message_port_register_out(d_out_port);
        if(ms<=0){
          throw std::invalid_argument("period cannot be negative or zero");
        }
        d_period_ms = ms;
        if(hisLen <0){
          throw std::invalid_argument("history length cannot be negative");
        }
        d_history_len = hisLen;
        d_history.clear();
        d_history.resize(d_history_len,0.0f);
        d_pkt_cnt=0;
        d_byte_cnt=0;
        d_pkt_acc=0;
        d_byte_acc=0;
        switch(user){
          case PROU:
            d_user = PROU;
          break;
          case SU:
            d_user = SU;
          break;
          default:
            throw std::runtime_error("unrecognized user type");
          break;
        }
       }
       ~throughput_report_impl(){}
       void 
       msg_in(pmt::pmt_t msg)
       {
        assert(pmt::is_pair(msg));
        pmt::pmt_t k = pmt::car(msg);
        pmt::pmt_t v = pmt::cdr(msg);
        if(pmt::is_blob(v)){
          size_t io(0);
          const uint8_t* uvec = pmt::u8vector_elements(v,io);
          switch(d_user){
            case PROU:
              if(io<=PROUMAXLEN){
                d_byte_cnt+=io;
                d_byte_acc+=io;
                d_pkt_cnt++;
                d_pkt_acc++;
              }
            break;
            case SU:
              // cyclic check
              if(io>=6){
                if(lsa_crc(uvec)){
                  // valid pkt
                  // remove header
                  d_byte_cnt+= (io-6);
                  d_byte_acc+= (io-6);
                  d_pkt_cnt++;
                  d_pkt_acc++;
                }
              }
            break;
            default:
              std::runtime_error("Undefined User Type");
            break;
          }
        }
       }
       bool 
       start()
       {
        d_finished = false;
        d_thread = boost::shared_ptr<gr::thread::thread>
          (new gr::thread::thread(boost::bind(&throughput_report_impl::run,this)));
        return block::start();
       }
       bool 
       stop()
       {
        d_finished = true;
        d_thread->interrupt();
        d_thread->join();
        return block::stop();
       }

      private:
      void
      run()
      {
        while(!d_finished){
          boost::this_thread::sleep(boost::posix_time::milliseconds(d_period_ms));
          if(d_finished){
            return;
          }
          gen_throughput();
        }
      }
      void
      gen_throughput()
      {
        float instant_throughput = d_byte_cnt/d_period_ms*1000; // unit in second
        d_history.push_back(instant_throughput);
        d_history.erase(d_history.begin());
        // can report pkts received
        if(d_history_len!=0){
          message_port_pub(d_out_port,pmt::cons(pmt::PMT_NIL,pmt::init_f32vector(d_history_len,d_history.data())));
        }else{
          message_port_pub(d_out_port,pmt::cons(pmt::PMT_NIL,pmt::init_f32vector(1,&instant_throughput)));
        }
        d_byte_cnt=0;
        d_pkt_cnt=0;
      }

       const pmt::pmt_t d_in_port;
       const pmt::pmt_t d_out_port;
       std::vector<float> d_history;
       int d_history_len;
       int d_byte_cnt;
       int d_pkt_cnt;
       long int d_pkt_acc;
       long int d_byte_acc;
       boost::shared_ptr<gr::thread::thread> d_thread;
       float d_period_ms;
       bool d_finished;
       int d_user;
    };
    throughput_report::sptr
    throughput_report::make(float ms, int hisLen, int user){
      return gnuradio::get_initial_sptr(new throughput_report_impl(ms,hisLen,user));
    }

  } /* namespace lsa */
} /* namespace gr */

