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
#include <lsa/file_downloader_tx.h>
#include <gnuradio/block_detail.h>
#include <fstream>
#include <boost/random/variate_generator.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/poisson_distribution.hpp>

namespace gr {
  namespace lsa {
    #define d_debug false
    #define DEBUG d_debug && std::cout

    class file_downloader_tx_impl : public file_downloader_tx
    {
      public:
        file_downloader_tx_impl(const std::string&filename,float mean):block("file_downloader_tx",
            gr::io_signature::make(0,0,0),
            gr::io_signature::make(0,0,0)),
            d_ack_port(pmt::mp("ack_in")),
            d_pdu_port(pmt::mp("pdu_out")),
            d_rng(),
            d_mean_ms(mean)
        {
          message_port_register_in(d_ack_port);
          message_port_register_out(d_pdu_port);
          set_msg_handler(d_ack_port,boost::bind(&file_downloader_tx_impl::ack_in,this,_1));
          // read file source
          d_total_bytes = 0;
          if(!read_data(filename)){
            throw std::invalid_argument("File does not exsists");
          }
          if(d_data_src.empty()){
            throw std::invalid_argument("File empty or not recognized");
          }
          boost::poisson_distribution<> pd(d_mean_ms); // poisson variable is characteried by mean
          d_variate_poisson = boost::shared_ptr< boost::variate_generator<boost::mt19937, boost::poisson_distribution<> > >(
            new boost::variate_generator <boost::mt19937, boost::poisson_distribution<> >(d_rng,pd) );
          reset_downloader();
        }
        ~file_downloader_tx_impl()
        {

        }
        bool start()
        {
          d_finished = false;
          d_thread = boost::shared_ptr<gr::thread::thread>(
            new gr::thread::thread(&file_downloader_tx_impl::run,this));
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
          // crc for ack
          size_t io(0);
          const uint8_t* uvec = pmt::u8vector_elements(v,io);
          if(io==4){
            uint16_t base1,base2;
            base1 = uvec[0]<<8;
            base1|= uvec[1];
            base2 = uvec[2]<<8;
            base2|= uvec[3];
            if(base1==base2 && base1==d_process_seq){
              // correctly acked
              // for fast delivery: notify d_acked_received
              // however, in our experiment, prou expected rate should not be large.
              d_acked = true;
            }else{
              return;
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
              if(u8.size()>123){
                throw std::runtime_error("message exceed maximum size");
              }
              u8.push_back((uint8_t)tmp);
            }
            d_data_src.push_back(u8);
            d_total_bytes+=u8.size(); // record total bytes
          }
          d_file.close();
          return true;
        }
        float next_delay()
        {
          return d_variate_poisson->operator()();
        }
        void generate_pdu(uint16_t seq)
        {
          if(seq==d_process_seq){
            return;
          }
          d_process_seq=seq; // sync to new seq number
          const uint8_t* u8_seq = (const uint8_t*) &seq;
          d_buf[0] = u8_seq[1];
          d_buf[1] = u8_seq[0];
          d_buf[2] = u8_seq[1];
          d_buf[3] = u8_seq[0];
          memcpy(d_buf+4,d_data_src[seq].data(),sizeof(char)*d_data_src[seq].size());
          d_current_pdu = pmt::make_blob(d_buf,4+d_data_src[seq].size()); // hold pdu
        }
        void run()
        {
          int retry=0;
          while(!d_finished)
          {
            generate_pdu(d_seq);
            d_acked = false;
            message_port_pub(d_pdu_port,pmt::cons(pmt::PMT_NIL,d_current_pdu));
            gr::thread::scoped_lock lock(d_mutex);
            d_ack_received.timed_wait(lock,boost::posix_time::milliseconds(next_delay()));
            lock.unlock();
            if(d_finished){
              return;
            }else if(d_acked){
              //acked
              d_seq++;
              if(d_seq==d_data_src.size()){
                // download complete....
                record_result();
                boost::this_thread::sleep(boost::posix_time::milliseconds(10000));
                reset_downloader();
              }else{
                retry=0;
                // if verbose, report download progress?
              }
            }else{
              //timeout
              retry++;
            }
          }
        }
        void record_result()
        {
          gr::thread::scoped_lock guard(d_mutex);
          boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::local_time()-d_start_time;
          std::cout<<"total_bytes,download_time(ms):"<<d_total_bytes<<","<<diff.total_milliseconds()<<std::endl;
          // may record more detailed message such as: maximum delay, retransmission times,...etc.
        }
        void reset_downloader()
        {
          d_seq = 0;
          d_process_seq=0xffff;
          d_start_time = boost::posix_time::microsec_clock::local_time();
        }
        const pmt::pmt_t d_ack_port;
        const pmt::pmt_t d_pdu_port;
        float d_mean_ms;
        bool d_finished;
        bool d_acked;
        long int d_total_bytes;
        boost::shared_ptr<gr::thread::thread> d_thread;
        boost::mt19937 d_rng;
        boost::shared_ptr< boost::variate_generator <boost::mt19937, boost::poisson_distribution<> > > d_variate_poisson;
        gr::thread::mutex d_mutex;
        boost::posix_time::ptime d_start_time;
        gr::thread::condition_variable d_ack_received;
        unsigned char d_buf[256];
        std::vector< std::vector<unsigned char> > d_data_src;
        std::fstream d_file;
        pmt::pmt_t d_current_pdu;
        uint16_t d_process_seq;
        uint16_t d_seq;
    };
    file_downloader_tx::sptr file_downloader_tx::make(const std::string& filename,float mean)
    {
      return gnuradio::get_initial_sptr(new file_downloader_tx_impl(filename,mean));
    }    

  } /* namespace lsa */
} /* namespace gr */

