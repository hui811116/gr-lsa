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
#include <lsa/detection_tx.h>
#include <gnuradio/block_detail.h>
#include <fstream>
#include <cstdlib>
#include <cstdio>
#include <ctime>

namespace gr {
  namespace lsa {
  class detection_tx_impl : public detection_tx
  {
    public:
      detection_tx_impl(const std::string& filename, float time_slot):block("detection_tx",
              gr::io_signature::make(0,0,0),
              gr::io_signature::make(0,0,0)),
              d_in_port(pmt::mp("fb_in")),
              d_out_port(pmt::mp("pdu_out"))
      {
        message_port_register_in(d_in_port);
        message_port_register_out(d_out_port);
        set_msg_handler(d_in_port,boost::bind(&detection_tx_impl::msg_in,this,_1));
        d_seq = 0x0000;
        if(!read_data(filename)){
          throw std::invalid_argument("Data file unrecognized, abort");
        }
        if(time_slot<=0){
          throw std::invalid_argument("Time slot should be positive");
        }
        d_time_slot = time_slot;
        std::srand(std::time(NULL));
        d_h0_cnt=0;
        d_h1_cnt=0;
        d_md_cnt=0;
        d_fa_cnt=0;
        d_start_time = boost::posix_time::second_clock::local_time();
      }
      ~detection_tx_impl(){}
      bool start()
      {
        d_finished = false;
        d_thread = boost::shared_ptr<gr::thread::thread>
          (new gr::thread::thread(&detection_tx_impl::run,this));
        d_report = boost::shared_ptr<gr::thread::thread>
          (new gr::thread::thread(&detection_tx_impl::report,this));
        return block::start();
      }
      bool stop()
      {
        d_finished = true;
        d_thread->interrupt();
        d_report->interrupt();
        d_thread->join();
        d_report->join();
        return block::stop();
      }
      void msg_in(pmt::pmt_t msg)
      {
        gr::thread::scoped_lock guard(d_mutex);
        pmt::pmt_t k = pmt::car(msg);
        pmt::pmt_t v = pmt::cdr(msg);
        size_t io(0);
        const uint8_t* uvec = pmt::u8vector_elements(v,io);
        if(io==2){
          if(uvec[0]==0xff && uvec[1]==0x00){
            // positive sensing result
            d_detect = true;
          }
        }
      }
    private:
      void report()
      {
        while(!d_finished){
          boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
          if(d_finished){
            return;
          }
          boost::posix_time::time_duration diff = boost::posix_time::second_clock::local_time() -d_start_time;
          //std::cout<<"<DETECTOR Report> Time spent:"<<diff.total_seconds()
          //<<" ,H1 event:"<<d_h0_cnt<<",H0 event:"<<d_h1_cnt<<" ,md_cnt="<<d_md_cnt<<" ,fa_cnt="<<d_fa_cnt<<std::endl;
          std::cout<<"<Detector report>(Time, H1, H0, MD, FA)=("<<diff.total_seconds()<<","<<d_h1_cnt<<","
          <<d_h0_cnt<<","<<d_md_cnt<<","<<d_fa_cnt<<")"<<std::endl;
        }
      }
      void run()
      {
        while(!d_finished)
        {
          d_detect = false;
          bool d_ground_truth = (std::rand()%2==1);
          if(d_ground_truth){
            memcpy(d_buf,d_data_src[d_seq].data(),sizeof(char)*d_data_src[d_seq].size());
            message_port_pub(d_out_port, pmt::cons(pmt::PMT_NIL, pmt::make_blob(d_buf,d_data_src[d_seq].size())));
            d_seq = (d_seq==0xffff)? 0x0000 : d_seq+1;
          }
          boost::this_thread::sleep(boost::posix_time::milliseconds(d_time_slot));
          if(d_finished){
            return;
          }
          if(d_ground_truth){
            d_h1_cnt++;
            d_md_cnt = (!d_detect)? d_md_cnt+1 : d_md_cnt;
          }else{
            d_h0_cnt++;
            d_fa_cnt = (d_detect)? d_fa_cnt+1 : d_fa_cnt;
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
              if(u8.size()>128){
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
      const pmt::pmt_t d_out_port;
      boost::shared_ptr<gr::thread::thread> d_thread;
      boost::shared_ptr<gr::thread::thread> d_report;
      boost::posix_time::ptime d_start_time;
      gr::thread::mutex d_mutex;
      bool d_finished;
      bool d_detect;
      uint16_t d_seq;
      std::fstream d_file;
      std::vector< std::vector<unsigned char> > d_data_src;
      unsigned char d_buf[256];
      uint32_t d_md_cnt;
      uint32_t d_fa_cnt;
      uint32_t d_h0_cnt;
      uint32_t d_h1_cnt;
      float d_time_slot;
  };

  detection_tx::sptr detection_tx::make(const std::string& filename,float time_slot)
  {
    return gnuradio::get_initial_sptr(new detection_tx_impl(filename,time_slot));
  }
    

  } /* namespace lsa */
} /* namespace gr */

