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

#ifndef INCLUDED_LSA_PWR_RX_STATE_CTRL_CC_IMPL_H
#define INCLUDED_LSA_PWR_RX_STATE_CTRL_CC_IMPL_H

#include <lsa/pwr_rx_state_ctrl_cc.h>

namespace gr {
  namespace lsa {

    class pwr_rx_state_ctrl_cc_impl : public pwr_rx_state_ctrl_cc
    {
     private:
      pmt::pmt_t d_fb_port;
      gr::thread::mutex d_mutex;
      std::vector<gr_complex> d_samples;
      gr_complex d_sEng;
      gr_complex* d_corr_buf;
      const int d_max_out;
      int d_collision_cnt;
      int d_state;
      int d_gap_cnt;
      int d_gapLen;
      int d_report_event;
      bool d_voe_state;
      float d_threshold;
      const pmt::pmt_t d_colli_blob;
      const pmt::pmt_t d_clear_blob;
      boost::shared_ptr<gr::thread::thread> d_thread;
      bool d_finished;
      gr::thread::condition_variable d_pub_state;
      void enter_idle();
      void enter_collision();
      void pub_msg(int event);
      void run();

     public:
      enum RXSTATE{
        IDLE,
        COLLISION
      };
      
      bool start();
      bool stop();
      pwr_rx_state_ctrl_cc_impl(
        const std::vector<gr_complex>& samples,
        float threshold,
        int gapLen
        );
      ~pwr_rx_state_ctrl_cc_impl();
      void set_threshold(float thres);
      float get_threshold() const;
      void set_gap(int gapLen);
      int get_gap() const;
      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_PWR_RX_STATE_CTRL_CC_IMPL_H */

