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
    #define SEQLEN 4
    #define MAXLEN 123
    #define RETRYLIMIT 10
    #define PKTCOUNTER d_countsize
    #define DEBUG d_debug && std::cout
    static const unsigned char d_seq_field[] = {0x00,0x00,0x00,0x00};
    class simple_tx_impl : public simple_tx
    {
      public:
        simple_tx_impl(const std::string& filename, float timeout,int cnt,bool slow): block("simple_tx",
                  gr::io_signature::make(0,0,0),
                  gr::io_signature::make(0,0,0)),
                  d_timeout(timeout),
                  d_countsize(cnt),
                  d_retry_limit(RETRYLIMIT),
                  d_in_port(pmt::mp("ack_in")),
                  d_out_port(pmt::mp("pdu_out")),
                  d_thr_port(pmt::mp("thr_out"))
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
          message_port_register_out(d_thr_port);
          set_msg_handler(d_in_port,boost::bind(&simple_tx_impl::msg_in,this,_1));
          d_seqno = 0;
          d_slow = slow;
          d_pkt_cnt=0;
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
                DEBUG<<"<SIMPLE TX>Acked successfully:"<<base1<<std::endl;
                if(!d_slow){
                  d_ack_received.notify_one();
                }
                d_acked = true;
                d_pkt_cnt++;
                if(d_pkt_cnt==PKTCOUNTER){
                  d_pkt_cnt=0;
                  boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::local_time()-d_start_time;
                  pmt::pmt_t dict = pmt::make_dict();
                  dict = pmt::dict_add(dict,pmt::intern("pkt_count"),pmt::from_long(d_countsize));
                  dict = pmt::dict_add(dict,pmt::intern("duration"),pmt::from_long(diff.total_milliseconds()));
                  message_port_pub(d_thr_port,pmt::cons(dict,pmt::PMT_NIL));
                  DEBUG <<"[Event]"<<"\n<Count>\n"<<d_countsize<<"\n<Duration>\n"
                  << diff.total_milliseconds()<<"\n[Event*]" <<std::endl;
                }
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
            if(d_pkt_cnt==0){
              d_start_time = boost::posix_time::microsec_clock::local_time();
            }
            message_port_pub(d_out_port,d_current_msg);
            retry++;
            gr::thread::scoped_lock lock(d_mutex);
            d_ack_received.timed_wait(lock,boost::posix_time::milliseconds(d_timeout));
            lock.unlock();
            if(d_finished){
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
        const int d_retry_limit;
        const int d_countsize;
        const pmt::pmt_t d_in_port;
        const pmt::pmt_t d_out_port;
        const pmt::pmt_t d_thr_port;
        gr::thread::mutex d_mutex;
        gr::thread::condition_variable d_ack_received;
        boost::shared_ptr<gr::thread::thread> d_thread;
        boost::posix_time::ptime d_start_time;
        bool d_finished;
        bool d_acked;
        bool d_slow;
        bool d_verb;
        int d_retry_cnt;
        int d_pkt_cnt;
        uint16_t d_seqno;
        float d_timeout;
        pmt::pmt_t d_current_msg;
        std::fstream d_file;
        std::vector< std::vector<unsigned char> > d_data_src;
        unsigned char d_buf[256];
    };
    simple_tx::sptr
    simple_tx::make(const std::string& filename,float timeout,int cnt,bool slow)
    {
      return gnuradio::get_initial_sptr(new simple_tx_impl(filename,timeout,cnt,slow));
    }
  } /* namespace lsa */
} /* namespace gr */

