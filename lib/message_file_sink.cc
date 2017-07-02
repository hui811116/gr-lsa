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
#include <lsa/message_file_sink.h>
#include <gnuradio/block_detail.h>
#include <fstream>
#include <ctime>
#include <iomanip>

namespace gr {
  namespace lsa {
    #define LSAMAXLEN 121
    #define SNSMAXLEN 123
    #define PROUMAXLEN 127
    #define LSAMACLEN 6
    #define SNSMACLEN 4
    #define BYTES_PER_LINE 20
    #define VERBOSEMS 1000
    enum SYSTEM{
      LSA=0,
      SNS=1,
      PROU=2
    };
    class message_file_sink_impl : public message_file_sink
    {
      public:
        message_file_sink_impl(const std::string& filename, int sys, bool append, bool verb) : block("message_file_sink",
                      gr::io_signature::make(0,0,0),
                      gr::io_signature::make(0,0,0)),
                      d_in_port(pmt::mp("msg_in"))
        {
          d_file = NULL;
          d_append = append;
          d_verb = verb;
          message_port_register_in(d_in_port);
          set_msg_handler(d_in_port,boost::bind(&message_file_sink_impl::msg_in,this,_1));
          update_file(filename);
          std::time(&d_timer); // start of this block
          switch(sys){
            case LSA:
            case SNS:
            case PROU:
              d_sys = sys;
            break;
            default:
              throw std::runtime_error("Undefined system");
            break;
          }
          d_pkt_cnt =0;
          d_byte_cnt=0;
          d_acc_pkt =0;
          d_start_time = boost::posix_time::second_clock::local_time();
        }
        ~message_file_sink_impl()
        {
          if(d_file->is_open()){
            d_file->close();
          }
          delete d_file;
        }
        void update_file(const std::string& filename)
        {
          gr::thread::scoped_lock guard(d_mutex);
          if(d_file!=NULL && d_file->is_open()){
            d_file->close();
            delete d_file;
          }
          if(d_append){
            d_file = new std::fstream(filename.c_str(),std::fstream::out|std::fstream::app);
          }else{
            d_file = new std::fstream(filename.c_str(),std::fstream::out|std::fstream::trunc);
          }
          if(!d_file->is_open()){
            throw std::runtime_error("<FILE SINK>cannot open file...file exist or cannot be opened");
          }
          // record the time when file started!
          d_timer = std::time(NULL);
          *d_file <<"\n[Event]\n<Time>\n"<< std::ctime(&d_timer)
          <<"<Bytes>\n"<<std::dec<<0<<"\n<Packets>\n"<<0<<"\n[Event*]"<<std::endl;
        }
        bool start()
        {
          d_finished = false;
          d_thread = boost::shared_ptr<gr::thread::thread>
          (new gr::thread::thread(boost::bind(&message_file_sink_impl::run,this)));
          return block::start();
        }
        bool stop()
        {
          d_finished = true;
          d_thread->interrupt();
          d_thread->join();
          return block::stop();
        }
      private:
        void run()
        {
          while(!d_finished){
            boost::this_thread::sleep(boost::posix_time::milliseconds(VERBOSEMS));
            if(d_finished){
              return;
            }
            boost::posix_time::time_duration diff = boost::posix_time::second_clock::local_time() - d_start_time;
            if(d_verb){
              std::printf("<Message File Sink> Accumulated results:%d, elapsed time:%d\n",d_acc_pkt,diff.total_seconds());
              std::fflush(stdout);
            }
          }
        }
        void msg_in(pmt::pmt_t msg)
        {
          gr::thread::scoped_lock guard(d_mutex);
          // change to local time, only have precision to second
          // maybe need to use clock to find higher precision throughput
          if(std::difftime(std::time(NULL),d_timer)>0){
            // at least a second passed
            std::time(&d_timer);
            *d_file <<"\n[Event]\n<Time>\n"<< std::ctime(&d_timer)
            <<"<Bytes>\n"<<std::dec<<d_byte_cnt<<"\n<Packets>\n"<<d_pkt_cnt
            <<"\n[Event*]"<<std::endl;
            d_pkt_cnt=0;
            d_byte_cnt=0;
          }
          // seq number can be stored here
          pmt::pmt_t k = pmt::car(msg); // pwr and seqno is hide in this key
          pmt::pmt_t v = pmt::cdr(msg);
          if(!pmt::is_blob(v) || !pmt::is_dict(k)){
            return;
          }
          uint16_t seq=0;
          float pwr=0;
          seq = pmt::to_long(pmt::dict_ref(k,pmt::intern("seqno"),pmt::from_long(0)));
          pwr = pmt::to_float(pmt::dict_ref(k,pmt::intern("pwr"),pmt::from_float(0)));
          size_t io(0);
          const uint8_t* uvec = pmt::u8vector_elements(v,io);
          d_pkt_cnt++;
          d_byte_cnt+=io;
          switch(d_sys){
            case LSA:
                *d_file << "[LSA]"<<std::endl;
            break;
            case SNS:
                *d_file << "[SNS]"<<std::endl;
            break;
            case PROU:
                *d_file << "[PROU]"<<std::endl;
            break;
            default:
              throw std::runtime_error("Undefined system");
            break;
          }
          *d_file<<"<seq>\n"<<std::dec<<seq<<std::endl
          <<"<pwr>\n"<<std::dec<<std::fixed<<std::setprecision(2)<<pwr<<std::endl
          <<"<size>\n"<<std::dec<<io<<std::endl
          <<"[Hex]"<<std::endl;
          int i=0;
          for(i=0;i<io;++i){
            *d_file<<std::setfill('0')<<std::setw(2)<<std::hex<<(int)uvec[i];
            if(i!=io-1){
              *d_file<<",";
            }
          }
          *d_file<<"\n[Hex*]"<<std::endl;
          switch(d_sys){
            case LSA:
                *d_file << "[LSA*]"<<std::endl;
            break;
            case SNS:
                *d_file << "[SNS*]"<<std::endl;
            break;
            case PROU:
                *d_file << "[PROU*]"<<std::endl;
            break;
            default:
              throw std::runtime_error("Undefined system");
            break;
          }
          d_acc_pkt++;
        }
        const pmt::pmt_t d_in_port;
        std::fstream* d_file;
        gr::thread::mutex d_mutex;
        boost::shared_ptr<gr::thread::thread> d_thread;
        boost::posix_time::ptime d_start_time;
        time_t d_timer;
        int d_sys;
        int d_pkt_cnt;
        int d_byte_cnt;
        unsigned int d_acc_pkt;
        bool d_append;
        bool d_verb;
        bool d_finished;
    };

    message_file_sink::sptr 
    message_file_sink::make(const std::string& filename, int sys, bool append,bool verb)
    {
      return gnuradio::get_initial_sptr(new message_file_sink_impl(filename,sys,append,verb));
    }

  } /* namespace lsa */
} /* namespace gr */

