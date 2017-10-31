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

#ifndef INCLUDED_LSA_PWR_SENSE_TX_BB_IMPL_H
#define INCLUDED_LSA_PWR_SENSE_TX_BB_IMPL_H

#include <lsa/pwr_sense_tx_bb.h>
#include <fstream>
#include "boost/tuple/tuple.hpp"
namespace gr {
  namespace lsa {
    #define d_debug 1
    #define dout d_debug && std::cout
    #define EVENT_COLLISION 2
    #define EVENT_CLEAR 3
    #define ACK_TIMEOUT 5000
    #define PHYLEN 6
    #define HDRLEN 4
    class pwr_sense_tx_bb_impl : public pwr_sense_tx_bb
    {
     private:
      gr::thread::condition_variable d_sense_received;
      gr::thread::condition_variable d_burst_ctrl;
      gr::thread::mutex d_mutex;
      bool d_finished;
      bool d_useFile;
      bool d_sense_collision;
      bool d_sense_clear;
      //bool d_first_burst;
      //bool d_last_burst;
      int d_state;
      double d_high_power;
      double d_low_power;
      float d_sense_time;
      boost::posix_time::ptime d_sysTime;
      boost::shared_ptr<gr::thread::thread> d_thread;
      unsigned char d_buf[256];
      const pmt::pmt_t d_fb_port;
      const pmt::pmt_t d_pwr_port;
      const pmt::pmt_t d_name;
      pmt::pmt_t d_pwr_msg;
      uint16_t d_seqno;
      std::fstream d_file;
      std::vector< std::vector<unsigned char> > d_data_src;
      std::list<boost::tuples::tuple<uint16_t,boost::posix_time::ptime,int> > d_ack_pending;
      void run();
      void fb_in(pmt::pmt_t msg);
      bool read_data(const std::string& filename);
      bool check_timeout();

     protected:
      int calculate_output_stream_length(const gr_vector_int &ninput_items);

     public:
      enum TXSTATE{
        SENSING,
        COLLISION,
        CLEAR
      };
      pwr_sense_tx_bb_impl(
        const std::string& lentag, 
        const std::string& filename, 
        bool useFile,
        float highPower,
        float lowPower,
        float senseTime);
      ~pwr_sense_tx_bb_impl();
      bool start();
      bool stop();
      void set_power(float high, float low);
      void set_sense_time(float senseTime);
      float get_high_power() const;
      float get_low_power() const;
      // Where all the action really happens
      int work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_PWR_SENSE_TX_BB_IMPL_H */

