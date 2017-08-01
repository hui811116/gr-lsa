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
#include <lsa/dump_tx.h>
#include <gnuradio/block_detail.h>
#include <fstream>

namespace gr {
  namespace lsa {
    #define d_debug false
    #define DEBUG d_debug && std::cout
    #define VERBOSE d_verb && std::cout
    static int d_retry_lim = 10;
    static int d_queue_lim = 8192;

    class dump_tx_impl : public dump_tx
    {
      public:
        dump_tx_impl(const std::string& filename,int avg_size, float timeout, bool verb): block("dump_tx",
                  gr::io_signature::make(0,0,0),
                  gr::io_signature::make(0,0,0)),
                  d_in_port(pmt::mp("strobe")),
                  d_ack_port(pmt::mp("ack_in")),
                  d_out_port(pmt::mp("pdu_out"))
        {
          d_data_src.clear();
          if(!read_data(filename)){
            throw std::invalid_argument("filename invalid");
          }
          if(avg_size<=0 || timeout<=0){
            throw std::invalid_argument("Invalid system parameters");
          }
          set_avg_size(avg_size);
          set_timeout(timeout);
          message_port_register_in(d_ack_port);
          message_port_register_in(d_in_port);
          message_port_register_out(d_out_port);
          set_msg_handler(d_ack_port,boost::bind(&dump_tx_impl::ack_in,this,_1));
          set_msg_handler(d_in_port,boost::bind(&dump_tx_impl::strobe_in,this,_1));
          d_queue.clear();
          d_pending.clear();
          d_seqno =0;
          d_verb = verb;
          d_pkt_cnt=0;
        }
        ~dump_tx_impl(){}
        bool start()
        {
          d_finished = false;
          d_thread = boost::shared_ptr<gr::thread::thread>
            (new gr::thread::thread(boost::bind(&dump_tx_impl::run,this)));
          d_timeout_thread = boost::shared_ptr<gr::thread::thread>
            (new gr::thread::thread(boost::bind(&dump_tx_impl::check_timeout,this)));
          return block::start();
        }

        bool stop()
        {
          d_finished = true;
          d_thread->interrupt();
          d_timeout_thread->interrupt();
          d_thread->join();
          d_timeout_thread->join();
          return block::stop();
        }
        void ack_in(pmt::pmt_t msg)
        {
          gr::thread::scoped_lock guard(d_mutex);
          pmt::pmt_t k = pmt::car(msg);
          pmt::pmt_t v = pmt::cdr(msg);
          uint16_t base1, base2;
          std::list< std::tuple<uint16_t,pmt::pmt_t,boost::posix_time::ptime,int> >::iterator it;
          boost::posix_time::time_duration diff;
          int channel_use;
          assert(pmt::is_blob(v));
          size_t io(0);
          const uint8_t* uvec = pmt::u8vector_elements(v,io);
          // first step is CRC
          if(io==4){
            // ack
            base1 = uvec[0]<<8;
            base1|= uvec[1];
            base2 = uvec[2]<<8;
            base2|= uvec[3];
            if(base1==base2){
              // matched ack
              for(it=d_pending.begin();it!=d_pending.end();++it){
                if(std::get<0>(*it)==base1){
                  // get packet information here!
                  diff =boost::posix_time::microsec_clock::local_time()-std::get<2>(*it);
                  channel_use = std::get<3>(*it);
                  // milliseconds
                  VERBOSE<<"result:"<<d_pkt_cnt<<"channel use:"<<channel_use+1<<" ,RTT(msg):"<<diff.total_milliseconds()+d_timeout*channel_use<<std::endl;
                  d_pkt_cnt++;
                }
              }
            }
          }
        }
        void strobe_in(pmt::pmt_t msg)
        {
          gr::thread::scoped_lock guard(d_mutex);
          std::list< std::tuple<uint16_t,pmt::pmt_t,boost::posix_time::ptime,int> >::iterator it=d_pending.begin();
          boost::posix_time::ptime cur_time;  
          if(d_pending.size()>=d_queue_lim){
            DEBUG<<"<DUMP TX debug> pending queue reache limit"<<d_queue_lim<<std::endl;
            return;
          }
          const uint8_t* u8_seq = (const uint8_t*) &d_seqno;
          d_buf[0] = u8_seq[1];
          d_buf[1] = u8_seq[0];
          d_buf[2] = u8_seq[1];
          d_buf[3] = u8_seq[0];
          memcpy(d_buf+4,d_data_src[d_seqno].data(),sizeof(char)*d_data_src[d_seqno].size());
          pmt::pmt_t pdu = pmt::make_blob(d_buf,d_data_src[d_seqno].size()+4);
          cur_time = boost::posix_time::microsec_clock::local_time();
          d_queue.push_back(std::make_pair(d_seqno,pdu));
          d_pending.push_back(std::make_tuple(d_seqno,pdu,cur_time,0));
          d_seqno = (d_seqno==0xffff)? 0:d_seqno+1;
          d_queue_filled.notify_one();
        }
        void set_timeout(float timeout)
        {
          gr::thread::scoped_lock guard(d_mutex);
          d_timeout =(timeout<=0)?1000: timeout;
        }
        float timeout()const
        {
          return d_timeout;
        }
        void set_avg_size(int avg_size)
        {
          gr::thread::scoped_lock guard(d_mutex);
          d_avg_size = (avg_size<=0)? 10 : avg_size;
        }
        int avg_size()const
        {
          return d_avg_size;
        }
      private:
        pmt::pmt_t queue_pop()
        {
          gr::thread::scoped_lock lock(d_mutex);
          while(d_queue.size()==0){
            d_queue_filled.wait(lock);
          }
          pmt::pmt_t msg = std::get<1>(d_queue.front());
          d_queue.pop_front();
          return msg;
        }
        void check_timeout()
        {
          uint16_t id;
          pmt::pmt_t blob;
          int retry;
          boost::posix_time::ptime time_update;
          std::list<std::tuple<uint16_t, pmt::pmt_t, boost::posix_time::ptime,int> >::iterator it;
          boost::posix_time::time_duration diff;
          while(!d_finished){
            boost::this_thread::sleep(boost::posix_time::milliseconds(d_timeout/2.0));
            if(d_finished){
              return;
            }
            it=d_pending.begin();
            if(it!=d_pending.end()){
              diff = boost::posix_time::microsec_clock::local_time()-std::get<2>(*it);
              if(diff.total_milliseconds()>=d_timeout){
                id = std::get<0>(*it);
                blob = std::get<1>(*it);
                retry = std::get<3>(*it)+1;
                time_update = boost::posix_time::microsec_clock::local_time();
                d_pending.push_back(std::make_tuple(id,blob,time_update,retry));
                d_pending.pop_front();
                d_queue.push_back(std::make_pair(id,blob));
                DEBUG<<"<Dump TX>Timeout found---seq="<<id<<" ,retry cnt:"<<retry<<std::endl;
                d_queue_filled.notify_one();
              }
            }
          }
        }
        void run()
        {
          while(!d_finished){
            pmt::pmt_t pdu = queue_pop();
            //DEBUG<<"<DUMP TX DEBUG>pop from queue, transmit message"<<std::endl;
            message_port_pub(d_out_port,pmt::cons(pmt::PMT_NIL,pdu));
            if(d_finished){
              return;
            }
          }
        }
        bool read_data(const std::string& filename)
        {
          gr::thread::scoped_lock guard(d_mutex);
          std::string str,line;
          d_file.open(filename.c_str(),std::fstream::in);
          if(!d_file.is_open()){
            return false;
          }
          while(getline(d_file,line,'\n')){
            std::istringstream temp(line);
            std::vector<uint8_t> u8;
            while(getline(temp,str,',')){
              int tmp = std::atoi(str.c_str());
              if(u8.size()>127){
                throw std::runtime_error("message exceed maximum size");
              }
              u8.push_back((uint8_t)tmp);
            }
            d_data_src.push_back(u8);
          }
          d_file.close();
          return true;
        }
        const pmt::pmt_t d_in_port;
        const pmt::pmt_t d_ack_port;
        const pmt::pmt_t d_out_port;
        gr::thread::mutex d_mutex;
        boost::shared_ptr<gr::thread::thread> d_thread;
        boost::shared_ptr<gr::thread::thread> d_timeout_thread;
        bool d_verb;
        bool d_finished;
        gr::thread::condition_variable d_queue_filled;
        float d_timeout;
        int d_avg_size;
        int d_pkt_cnt;
        unsigned char d_buf[256];
        std::vector< std::vector<unsigned char> > d_data_src;
        std::fstream d_file;
        uint16_t d_seqno;
        std::list<std::pair<uint16_t,pmt::pmt_t> > d_queue;
        std::list<std::tuple<uint16_t,pmt::pmt_t,boost::posix_time::ptime,int> > d_pending;
    };

    dump_tx::sptr dump_tx::make(const std::string& filename,int avg_size, float timeout, bool verb)
    {
      return gnuradio::get_initial_sptr(new dump_tx_impl(filename,avg_size,timeout,verb));
    }

  } /* namespace lsa */
} /* namespace gr */

