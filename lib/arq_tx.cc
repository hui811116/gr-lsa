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
#include <lsa/arq_tx.h>
#include <gnuradio/block_detail.h>
#include <fstream>

namespace gr {
  namespace lsa {
    #define MAXLEN 127
    #define d_debug false
    #define DEBUG d_debug && std::cout
    #define VERBOSE d_verb && std::cout
    #define SEQLEN 4

    class arq_tx_impl : public arq_tx
    {
      public:
        arq_tx_impl(const std::string& filename,int timeout,int period, int avg_size,bool verb): block("arq_tx",
                  gr::io_signature::make(0,0,0),
                  gr::io_signature::make(0,0,0)),
                  d_timeout((long)timeout),
                  d_period((long)period),
                  d_ack_port(pmt::mp("ack_in")),
                  d_pdu_port(pmt::mp("pdu_out")),
                  d_avg_size(avg_size)
        {
          if(timeout<0 || period<0 || avg_size<=0){
            throw std::invalid_argument("timeout or period is invalid");
          }
          d_data_src.clear();
          if(!read_data(filename)){
            throw std::runtime_error("Failed when reading data source");
          }
          message_port_register_in(d_ack_port);
          message_port_register_out(d_pdu_port);
          set_msg_handler(d_ack_port,boost::bind(&arq_tx_impl::msg_in,this,_1));
          d_seq_no =0;
          d_ack_no =0;
          d_verb = verb;
          d_success_cnt=0;
          d_channel_use=0;
          d_acc_delay=0;
        }
        ~arq_tx_impl(){

        }
        bool start()
        {
          d_finished = false;
          d_system_time = boost::posix_time::second_clock::local_time();
          d_thread = boost::shared_ptr<gr::thread::thread>
            (new gr::thread::thread(boost::bind(&arq_tx_impl::run,this)));
          return block::start();
        }
        bool stop()
        {
          d_finished = true;
          d_thread->interrupt();
          d_thread->join();
          return block::stop();
        }
        void msg_in(pmt::pmt_t msg)
        {
          gr::thread::scoped_lock guard(d_mutex);
          pmt::pmt_t k = pmt::car(msg);
          pmt::pmt_t v = pmt::cdr(msg);
          assert(pmt::is_blob(v));
          size_t io(0);
          const uint8_t* uvec = pmt::u8vector_elements(v,io);
          std::list<std::tuple<uint16_t,boost::posix_time::ptime,int> >::iterator it=d_pending.begin();
          if(io==4){
            uint16_t base1, base2;
            base1 = uvec[0]<<8;
            base1|= uvec[1];
            base2 = uvec[2]<<8;
            base2|= uvec[3];
            if(base1==base2){
              // crc_passed
              while(it!=d_pending.end()){
                if(std::get<0>(*it)==base1){
                  d_channel_use += std::get<2>(*it);
                  boost::posix_time::time_duration diff= boost::posix_time::microsec_clock::local_time()-std::get<1>(*it);
                  d_acc_delay += diff.total_milliseconds();
                  it=d_pending.erase(it);
                  DEBUG<<"<ARQ TX>Acked base="<<base1<<std::endl;
                  d_success_cnt++;
                  break;
                }
                it++;
              }
              if(d_success_cnt%d_avg_size == 0){
                // accumulate a averaging length
                VERBOSE << "<ARQ TX> ,acc_size="<<d_avg_size<<" ,acc_delay="<<d_acc_delay<<" ,acc_channel_use="<<d_channel_use<<" ,total_success="<<d_success_cnt<<std::endl;
                // reset
                d_acc_delay =0;
                d_channel_use=0;
              }
              if(base1 == d_ack_no){
                DEBUG<<"<ARQ TX>Acked base point, increment to next number, current:"<<d_ack_no<<std::endl;
                if(d_pending.empty()){
                  d_ack_no = d_seq_no;
                }else{
                  it = d_pending.begin();
                  boost::posix_time::ptime min_time = std::get<1>(*it);
                  uint16_t min_seq = std::get<0>(*it);
                  while(it!=d_pending.end()){
                    if(std::get<1>(*it)<min_time){
                      min_time = std::get<1>(*it);
                      min_seq = std::get<0>(*it);
                    }
                    ++it;
                  }
                  d_ack_no = min_seq;
                } 
              }
            }// crc passed
          }
        }
      private:
        void run()
        {
          uint16_t ack_image;
          while(!d_finished){
            // generate next message
            ack_image = d_ack_no;
            generate_msg();
            message_port_pub(d_pdu_port,d_current_msg);
            gr::thread::scoped_lock lock(d_mutex);
            d_ack_received.timed_wait(lock,boost::posix_time::milliseconds(d_period));
            lock.unlock();
            if(d_finished){
              return;
            }
            /*if(ack_image!=d_ack_no){
              // acked
            }else{
              // timeout
            }*/
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
              if(u8.size()>MAXLEN){
                throw std::runtime_error("message exceed maximum size");
              }
              u8.push_back((uint8_t)tmp);
            }
            d_data_src.push_back(u8);
          }
          d_file.close();
          return true;
        }
        void generate_msg()
        {
          gr::thread::scoped_lock guard(d_mutex);
          uint16_t seq = d_seq_no;
          std::list<std::tuple<uint16_t,boost::posix_time::ptime,int> >::iterator it=d_pending.begin();
          boost::posix_time::time_duration diff;
          if(!d_pending.empty()){
            diff = boost::posix_time::microsec_clock::local_time()-std::get<1>(*it);
            if(diff.total_milliseconds()>=d_timeout){
              seq = std::get<0>(*it);
              d_pending.push_back(std::make_tuple(seq,boost::posix_time::microsec_clock::local_time(),std::get<2>(*it)+1));
              d_pending.pop_front();
              DEBUG<<"<ARQ TX>Found timeout retry:"<<seq<<" ,current seqno="<<d_seq_no<<std::endl;
            }else{
              d_pending.push_back(std::make_tuple(seq,boost::posix_time::microsec_clock::local_time(),1));
              d_seq_no = (d_seq_no==0xffff)? 0 : d_seq_no+1;
              DEBUG<<"<ARQ TX>Not timeout ,current seqno="<<d_seq_no<<std::endl;
            }
          }else{
            d_pending.push_back(std::make_tuple(seq,boost::posix_time::microsec_clock::local_time(),1));
            d_seq_no = (d_seq_no==0xffff)? 0 : d_seq_no+1;
            DEBUG<<"<ARQ TX>Empty pending queue, current seqno="<<d_seq_no<<std::endl;
          }
          const uint8_t* u8_seq = (const uint8_t*) &seq;
          d_buf[0] = u8_seq[1];
          d_buf[1] = u8_seq[0];
          d_buf[2] = u8_seq[1];
          d_buf[3] = u8_seq[0];
          memcpy(d_buf+SEQLEN,d_data_src[seq].data(),sizeof(char)*d_data_src[seq].size());
          d_current_msg = pmt::cons(pmt::PMT_NIL,pmt::make_blob(d_buf,SEQLEN+d_data_src[seq].size()));
        }

        const pmt::pmt_t d_ack_port;
        const pmt::pmt_t d_pdu_port;
        boost::shared_ptr<gr::thread::thread> d_thread;
        gr::thread::mutex d_mutex;
        gr::thread::condition_variable d_ack_received;
        int d_window_size;
        uint16_t d_seq_no;
        uint16_t d_ack_no;
        boost::posix_time::ptime d_system_time;
        std::list<std::tuple<uint16_t,boost::posix_time::ptime,int> > d_pending;
        long d_period;
        long d_timeout;
        bool d_finished;
        int d_channel_use;
        bool d_verb;
        long int d_success_cnt;
        long int d_avg_size;
        long int d_acc_delay;
        std::fstream d_file;
        std::vector< std::vector<unsigned char> > d_data_src;
        unsigned char d_buf[256];
        pmt::pmt_t d_current_msg;
    };

    arq_tx::sptr
    arq_tx::make(const std::string& filename, int timeout,int period, int avg_size,bool verb)
    {
      return gnuradio::get_initial_sptr(new arq_tx_impl(filename,timeout,period,avg_size,verb));
    }

  } /* namespace lsa */
} /* namespace gr */

