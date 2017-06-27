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
#include <lsa/simple_tx.h>
#include <gnuradio/block_detail.h>
#include <fstream>

namespace gr {
  namespace lsa {
    #define d_debug false
    #define DEBUG d_debug && std::cout
    #define SEQLEN 4
    #define MAXLEN 123
    #define RETRYLIMIT 10
    static const unsigned char d_seq_field[] = {0x00,0x00,0x00,0x00};
    class simple_tx_impl : public simple_tx
    {
      public:
        simple_tx_impl(const std::string& filename, float timeout, bool slow): block("simple_tx",
                  gr::io_signature::make(0,0,0),
                  gr::io_signature::make(0,0,0)),
                  d_timeout(timeout),
                  d_retry_limit(RETRYLIMIT),
                  d_in_port(pmt::mp("ack_in")),
                  d_out_port(pmt::mp("pdu_out"))
        {
          if(timeout<=0){
            throw std::invalid_argument("Timeout should be positive");
          }
          d_data_src.clear();
          if(!read_data(filename)){
            throw std::runtime_error("File cannot be opened");
          }
          message_port_register_in(d_in_port);
          message_port_register_out(d_out_port);
          set_msg_handler(d_in_port,boost::bind(&simple_tx_impl::msg_in,this,_1));
          d_seqno = 0;
          d_slow = slow;
        }
        ~simple_tx_impl(){}
        bool start()
        {
          d_finished = false;
          d_thread = boost::shared_ptr<gr::thread::thread>
            (new gr::thread::thread(boost::bind(&simple_tx_impl::run,this)));
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
          if(io==4){
            uint16_t base1, base2;
            base1 = uvec[0]<<8;
            base1|= uvec[1];
            base2 = uvec[2]<<8;
            base2|= uvec[3];
            if(base1==base2){
              // crc passed
              if(base1 == d_seqno){
                DEBUG<<"<SIMPLE TX>successfullt acked:"<<base1<<std::endl;
                if(!d_slow){
                  d_ack_received.notify_one();
                }
                d_acked = true;
              }
            }
          }
        }
      private:
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
                throw std::runtime_error("message to be transmitted exceed the maximum payload length");
              }
              u8.push_back((uint8_t)tmp);
            }
            d_data_src.push_back(u8);
          }
          d_file.close();
          return true;
        }
        void run()
        {
          int retry =0;
          while(!d_finished){
            DEBUG<<"<SIMPLE TX>sending seq:"<<d_seqno<<std::endl;
            d_acked = false;
            generate_msg();
            message_port_pub(d_out_port,d_current_msg);
            retry++;
            gr::thread::scoped_lock lock(d_mutex);
            d_ack_received.timed_wait(lock,boost::posix_time::milliseconds(d_timeout));
            lock.unlock();
            //boost::this_thread::sleep(boost::posix_time::milliseconds(d_timeout));
            if(d_finished){
              //DEBUG<<"<SIMPLE TX>Finished called"<<std::endl;
              return;
            }
            if(d_acked || retry>d_retry_limit){
              // successfully acked or exceed retry limit
              if(d_acked){
                DEBUG<<"<SIMPLE TX>Acked"<<std::endl;
              }else{
                DEBUG<<"Exceed retry limit"<<std::endl;
              }
              retry=0;
              d_seqno = (d_seqno == 0xffff)? 0 : d_seqno+1;
            }
          }
        }
        
        void generate_msg(){
          gr::thread::scoped_lock guard(d_mutex);
          const uint8_t* u8_seq = (const uint8_t*) &d_seqno;
          d_buf[0] = u8_seq[1];
          d_buf[1] = u8_seq[0];
          d_buf[2] = u8_seq[1];
          d_buf[3] = u8_seq[0];
          memcpy(d_buf+SEQLEN,d_data_src[d_seqno].data(),sizeof(char)*d_data_src[d_seqno].size());
          pmt::pmt_t blob = pmt::make_blob(d_buf,SEQLEN+d_data_src[d_seqno].size());
          d_current_msg = pmt::cons(pmt::PMT_NIL,blob);
        }
        gr::thread::mutex d_mutex;
        gr::thread::condition_variable d_ack_received;
        boost::shared_ptr<gr::thread::thread> d_thread;
        bool d_finished;
        bool d_acked;
        float d_timeout;
        int d_retry_cnt;
        const int d_retry_limit;
        std::vector< std::vector<unsigned char> > d_data_src;
        std::fstream d_file;
        const pmt::pmt_t d_in_port;
        const pmt::pmt_t d_out_port;
        uint16_t d_seqno;
        pmt::pmt_t d_current_msg;
        unsigned char d_buf[256];
        bool d_slow;
    };
    simple_tx::sptr
    simple_tx::make(const std::string& filename,float timeout,bool slow)
    {
      return gnuradio::get_initial_sptr(new simple_tx_impl(filename,timeout,slow));
    }
  } /* namespace lsa */
} /* namespace gr */

