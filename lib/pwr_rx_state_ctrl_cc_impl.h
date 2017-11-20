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
      const pmt::pmt_t d_fb_port;
      gr::thread::mutex d_mutex;
      
      const int d_ed_valid;
      int d_ed_cnt;
      int d_state;
      int d_cd_cnt;
      int d_ncolli;
      
      //boost::shared_ptr<gr::thread::thread> d_thread;
      //bool d_finished;
      //gr::thread::condition_variable d_pub_state;

      float d_thres_low;
      float d_thres_high;
      bool d_update;

      void enter_high();
      void enter_trans();
      void enter_low();
      void reset_state(bool reset);

     public:
      enum RXSTATE{
        HIGH,
        TRANS,
        LOW
      };
      
      void set_threshold(float high,float low);
      pwr_rx_state_ctrl_cc_impl(float high_db,float low_db);
      ~pwr_rx_state_ctrl_cc_impl();
      
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_PWR_RX_STATE_CTRL_CC_IMPL_H */

