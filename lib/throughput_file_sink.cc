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
#include <lsa/throughput_file_sink.h>
#include <gnuradio/block_detail.h>
#include <fstream>

namespace gr {
  namespace lsa {

    #define PERIOD 1000
    enum SYSTEM{
      PROU=0,
      SU=1
    };
    class throughput_file_sink_impl : public throughput_file_sink
    {
      private:
        const pmt::pmt_t d_in_port;
        std::fstream d_file;
        bool d_verb;
        bool d_finished;
        gr::thread::mutex d_mutex;
        int d_system;
        int d_iter_cnt;
        boost::posix_time::ptime d_start_time;
        boost::shared_ptr<gr::thread::thread> d_thread;
        
        void msg_in(pmt::pmt_t msg)
        {
          gr::thread::scoped_lock guard(d_mutex);
          pmt::pmt_t k = pmt::car(msg);
          pmt::pmt_t v = pmt::cdr(msg);
          long int acc_delay,acc_ch_use,acc_size,total_suc;
          switch(d_system)
          {
            case PROU:
              assert(pmt::is_dict(k));
              acc_delay = pmt::to_long(pmt::dict_ref(k,pmt::intern("acc_delay"),pmt::from_long(0)));
              acc_ch_use= pmt::to_long(pmt::dict_ref(k,pmt::intern("acc_ch_use"),pmt::from_long(0)));
              acc_size  = pmt::to_long(pmt::dict_ref(k,pmt::intern("acc_size"),pmt::from_long(0)));
              total_suc = pmt::to_long(pmt::dict_ref(k,pmt::intern("total_suc"),pmt::from_long(0)));
              d_file<<acc_size<<","<<acc_delay<<","<<acc_ch_use<<","<<total_suc<<std::endl;
              d_file<<std::flush;
              d_iter_cnt++;
            break;
            case SU:
              // leave blank for future development
            break;
            default:
              throw std::runtime_error("Undefined system type");
            break;
            d_start_time = boost::posix_time::second_clock::local_time();
          }
        }
        void run()
        {
          while(!d_finished){
            boost::this_thread::sleep(boost::posix_time::milliseconds(PERIOD));
            if(d_finished){
              return;
            }
            boost::posix_time::time_duration diff = boost::posix_time::second_clock::local_time()-d_start_time;
            if(d_verb){
              std::printf("<Throughput file sink> Accumulated results:%d ,Execution time:%d secs\n",d_iter_cnt,diff.total_seconds());
              std::fflush(stdout);
            }
          }
        }

      public:
        bool start()
        {
          d_finished = false;
          d_start_time = boost::posix_time::second_clock::local_time();
          d_thread = boost::shared_ptr<gr::thread::thread>(
            new gr::thread::thread(boost::bind(&throughput_file_sink_impl::run,this)));
          return block::start();
        }
        bool stop()
        {
          d_finished = true;
          d_thread->interrupt();
          d_thread->join();
          return block::stop();
        }
        throughput_file_sink_impl(const std::string& filename,int sys, bool verbose) : block("throughput_file_sink",
                gr::io_signature::make(0,0,0),
                gr::io_signature::make(0,0,0)),
                d_in_port(pmt::mp("thr_in"))
        {
          message_port_register_in(d_in_port);
          set_msg_handler(d_in_port,boost::bind(&throughput_file_sink_impl::msg_in,this,_1));
          d_file.open(filename.c_str(),std::fstream::out|std::fstream::trunc);
          if(!d_file.is_open()){
            throw std::invalid_argument("Throughput file cannot be opened, abort execution...");
          }
          d_file<<"acc_size,acc_delay,acc_channel_use,total_success"<<std::endl;
          switch(sys){
            case PROU:
            case SU:
              d_system = sys;
            break;
            default:
              throw std::runtime_error("Undefined system type");
            break;
          }
          d_iter_cnt=0;
        }
        ~throughput_file_sink_impl()
        {
          if(d_file.is_open()){
            d_file.close();
          }
        }
    };
    throughput_file_sink::sptr
    throughput_file_sink::make(const std::string& filename, int sys, bool verbose)
    {
      return gnuradio::get_initial_sptr(new throughput_file_sink_impl(filename,sys,verbose));
    }

  } /* namespace lsa */
} /* namespace gr */

