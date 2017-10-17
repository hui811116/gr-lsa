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

#ifndef INCLUDED_LSA_SU_PWRCTRL_STREAM_TX_BB_IMPL_H
#define INCLUDED_LSA_SU_PWRCTRL_STREAM_TX_BB_IMPL_H

#include <lsa/su_pwrCtrl_stream_tx_bb.h>
#include "boost/tuple/tuple.hpp"
#include <fstream>

namespace gr {
  namespace lsa {

    class su_pwrCtrl_stream_tx_bb_impl : public su_pwrCtrl_stream_tx_bb
    {
     private:
      const pmt::pmt_t d_fb_port;
      const pmt::pmt_t d_pwr_out_port;
      gr::thread::mutex d_mutex;
      boost::shared_ptr<gr::thread::thread> d_thread;
      gr::thread::condition_variable d_update_pwr;
      std::fstream d_file;
      std::vector< std::vector<unsigned char> > d_data_src;
      std::list<boost::tuples::tuple<uint16_t,boost::posix_time::ptime,int> > d_ack_pending;
      uint16_t d_seqno;
      int d_fileSize;
      bool d_collision_state;
      bool d_useFile;
      bool d_finished;
      long int d_success_pkt;
      long int d_failed_pkt;
      boost::posix_time::ptime d_sys_time;
      unsigned char d_buf[256];
      float d_pwr_low;
      float d_pwr_high;
      // helper functions
      bool checkTimeout();
      bool read_data(const std::string& filename);
      void run();

     protected:
      int calculate_output_stream_length(const gr_vector_int &ninput_items);

     public:
      su_pwrCtrl_stream_tx_bb_impl(
        const std::string& tagname,
        const std::string& filename,
        bool useFile,
        float pwr1,
        float pwr2);
      ~su_pwrCtrl_stream_tx_bb_impl();
      void fb_in(pmt::pmt_t msg);

      void set_power(float pwr1, float pwr2);
      float power_low() const;
      float power_high() const;

      bool start();
      bool stop();

      // Where all the action really happens
      int work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_SU_PWRCTRL_STREAM_TX_BB_IMPL_H */

