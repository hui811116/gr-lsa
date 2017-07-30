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
#include <lsa/sns_tx.h>
#include <gnuradio/block_detail.h>
#include <fstream>

namespace gr {
  namespace lsa {
    #define DEBUG false && std::cout
    #define VERBOSE d_verb && std::cout
    class sns_tx_impl: public sns_tx
    {
      public:
        sns_tx_impl(const std::string& filename,int send, float timeout, bool verb):block("sns_tx",
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
          if(send<=0 || timeout<=0){
            throw std::invalid_argument("Invalid system parameters");
          }
          set_send(send);
          set_timeout(timeout);
          message_port_register_in(d_ack_port);
          message_port_register_in(d_in_port);
          message_port_register_out(d_out_port);
          set_msg_handler(d_ack_port,boost::bind(&sns_tx_impl::ack_in,this,_1));
          set_msg_handler(d_in_port,boost::bind(&sns_tx_impl::strobe_in,this,_1));
          d_queue.clear();
          d_seqno =0;
          d_verb = verb;
        }
        ~sns_tx_impl(){}
        bool start()
        {
          d_finished = false;
          d_thread = boost::shared_ptr<gr::thread::thread>
            (new gr::thread::thread(boost::bind(&sns_tx_impl::run,this)));
          return block::start();
        }

        bool stop()
        {
          d_finished = true;
          d_ack_received.notify_one();
          d_thread->interrupt();
          d_thread->join();
          return block::stop();
        }
        void ack_in(pmt::pmt_t msg)
        {
          gr::thread::scoped_lock guard(d_mutex);
          pmt::pmt_t k = pmt::car(msg);
          pmt::pmt_t v = pmt::cdr(msg);
          assert(pmt::is_blob(v));
          size_t io(0);
          const uint8_t* uvec = pmt::u8vector_elements(v,io);
          // first step is CRC
          uint16_t base1, base2;
          if(io==2){
            // positive sensing
            //VERBOSE<<"<SNS TX>receive sensing positive"<<std::endl;
            //d_wait = true;
          }else if(io==3){
            // clear
            VERBOSE<<"<SNSTX>receive sensing clear"<<std::endl;
            d_wait = false;
            d_wait_prou.notify_one();
          }else if(io==4){
            // ack
            base1 = uvec[0]<<8;
            base1|= uvec[1];
            base2 = uvec[2]<<8;
            base2|= uvec[3];
            if(base1==base2 && base1==d_cur_seqno){
              // matched ack
              //DEBUG<<"<SNSTX>successfullly ACKed base="<<base1<<std::endl;
              d_acked = true;
              d_ack_received.notify_one();
            }
          }
          
        }
        void strobe_in(pmt::pmt_t msg)
        {
          gr::thread::scoped_lock guard(d_mutex);
          if(!d_queue.empty()){
            // BUSY
            return ;
          }
          for(int i=0;i<d_send;++i){
            const uint8_t* u8_seq = (const uint8_t*) &d_seqno;
            d_buf[0] = u8_seq[1];
            d_buf[1] = u8_seq[0];
            d_buf[2] = u8_seq[1];
            d_buf[3] = u8_seq[0];
            memcpy(d_buf+4,d_data_src[d_seqno].data(),sizeof(char)*d_data_src[d_seqno].size());
            pmt::pmt_t pdu = pmt::make_blob(d_buf,d_data_src[d_seqno].size()+4);
            d_queue.push_back(std::make_pair(d_seqno,pdu));
            d_seqno = (d_seqno==0xffff)? 0:d_seqno+1;
          }
          d_queue_filled.notify_one();
        }
        void set_timeout(float timeout)
        {
          gr::thread::scoped_lock guard(d_mutex);
          d_timeout =(timeout<=0)?1000: timeout;
        }
        void set_send(int send)
        {
          gr::thread::scoped_lock guard(d_mutex);
          d_send = (send<=0)? 10 : send;
        }
      private:
        void wait_prou()
        {
          gr::thread::scoped_lock lock(d_mutex);
          while(d_wait==true){
            d_wait_prou.wait(lock);
          }
        }
        pmt::pmt_t queue_pop()
        {
          gr::thread::scoped_lock lock(d_mutex);
          while(d_queue.size()==0){
            d_queue_filled.wait(lock);
          }
          pmt::pmt_t msg = std::get<1>(d_queue.front());
          d_cur_seqno = std::get<0>(d_queue.front());
          return msg;
        }
        void run()
        {
          while(!d_finished){
            pmt::pmt_t pdu = queue_pop();
            int tx_count=0;
            d_acked =false;
            d_wait = false;
            d_start_time = boost::posix_time::microsec_clock::local_time();
            while(!d_acked && !d_wait){
              message_port_pub(d_out_port,pmt::cons(pmt::PMT_NIL,pdu));
              tx_count++;
              gr::thread::scoped_lock lock(d_mutex);
              d_ack_received.timed_wait(lock,boost::posix_time::milliseconds(d_timeout));
              lock.unlock();
              if(d_finished){
                return;
              }
              if(d_acked){
                // no retry limit
                d_queue.pop_front();
                if(d_queue.size()!=0){
                  d_acked = false;
                  pdu = std::get<1>(d_queue.front());
                  d_cur_seqno = std::get<0>(d_queue.front());
                  DEBUG<<"change current seqno to"<<d_cur_seqno<<std::endl;
                }else{
                  // complete task, calculate time spent, channel use
                  boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::local_time()-d_start_time;
                  VERBOSE<<"sns tx:"<<"complete transmission task <ms,tx_cnt>="<<diff.total_milliseconds()<<","<<tx_count<<std::endl;
                }
              }else if(d_wait){
                DEBUG<<"<SNS TX>Interfering ProU, stop and wait"<<std::endl;
                wait_prou();
              }else{
                // timeout
                //DEBUG<<"<SNS TX>timeout, retry"<<std::endl;
              }
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
        bool d_verb;
        bool d_finished;
        bool d_acked;
        bool d_wait;
        gr::thread::condition_variable d_ack_received;
        gr::thread::condition_variable d_queue_filled;
        gr::thread::condition_variable d_wait_prou;
        boost::posix_time::ptime d_start_time;
        float d_timeout;
        int d_send;
        unsigned char d_buf[256];
        std::vector< std::vector<unsigned char> > d_data_src;
        std::fstream d_file;
        uint16_t d_seqno;
        uint16_t d_cur_seqno;
        std::list<std::pair<uint16_t,pmt::pmt_t> > d_queue;
    };

    sns_tx::sptr sns_tx::make(const std::string& filename,int send, float timeout,bool verb)
    {
      return gnuradio::get_initial_sptr(new sns_tx_impl(filename,send,timeout,verb));
    }

  } /* namespace lsa */
} /* namespace gr */

