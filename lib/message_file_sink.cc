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
    enum SYSTEM{
      LSA=0,
      SNS=1,
      PROU=2
    };
    class message_file_sink_impl : public message_file_sink
    {
      public:
        message_file_sink_impl(const std::string& filename, int sys, bool append) : block("message_file_sink",
                      gr::io_signature::make(0,0,0),
                      gr::io_signature::make(0,0,0)),
                      d_in_port(pmt::mp("msg_in"))
        {
          d_file = NULL;
          d_append = append;
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
        }
      private:
        void msg_in(pmt::pmt_t msg)
        {
          gr::thread::scoped_lock guard(d_mutex);
          // change to local time, only have precision to second
          // maybe need to use clock to find higher precision throughput
          if(std::difftime(std::time(NULL),d_timer)>0){
            // at least a second passed
            std::time(&d_timer);
            *d_file <<"\n[Event]\n <Time>\n"<< std::ctime(&d_timer);
            *d_file <<"\n<Bytes>\n"<<std::dec<<d_byte_cnt<<"\n<Packets>\n"<<d_pkt_cnt
            <<"\n[Event*]"<<std::endl;
            d_pkt_cnt=0;
            d_byte_cnt=0;
          }
          // seq number can be stored here
          pmt::pmt_t k = pmt::car(msg);
          pmt::pmt_t v = pmt::cdr(msg);
          if(!pmt::is_blob(v)){
            return;
          }
          size_t io(0);
          const uint8_t* uvec = pmt::u8vector_elements(v,io);
          uint16_t seq = pmt::to_long(k);
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
          *d_file<<"<seq>\n"<<std::dec<<seq<<std::endl;
          *d_file<<"<size>\n"<<std::dec<<io<<std::endl;
          *d_file<<"[Hex]"<<std::endl;
          int i=0;
          for(i=0;i<io;++i){
            *d_file<<" "<<std::setfill('0')<<std::setw(2)<<std::hex<<(int)uvec[i];
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
        }
        const pmt::pmt_t d_in_port;
        std::fstream* d_file;
        gr::thread::mutex d_mutex;
        time_t d_timer;
        int d_sys;
        int d_pkt_cnt;
        int d_byte_cnt;
        bool d_append;
    };

    message_file_sink::sptr 
    message_file_sink::make(const std::string& filename, int sys, bool append)
    {
      return gnuradio::get_initial_sptr(new message_file_sink_impl(filename,sys,append));
    }

  } /* namespace lsa */
} /* namespace gr */

