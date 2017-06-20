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

#ifndef INCLUDED_LSA_STOP_N_WAIT_RX_CTRL_CC_IMPL_H
#define INCLUDED_LSA_STOP_N_WAIT_RX_CTRL_CC_IMPL_H

#include <lsa/stop_n_wait_rx_ctrl_cc.h>
#include <ctime>

namespace gr {
  namespace lsa {
    #define time_t long int

    class stop_n_wait_rx_ctrl_cc_impl : public stop_n_wait_rx_ctrl_cc
    {
     private:
      const pmt::pmt_t d_out_port;

      int d_state;
      float d_high_thres;
      float d_low_thres;
      float d_ed_thres;
      int d_voe_cnt;
      int d_voe_duration;
      bool d_burst_lock;
      int d_burst_voe_cnt;
      int d_target_burst_cnt;
      time_t d_clock_duration;
      int d_intf_cnt;

      void enter_search_collision();
      void enter_search_stop();
      void enter_wait_burst();
      void enter_wait_resume();

     public:
      stop_n_wait_rx_ctrl_cc_impl(float high_thres, float low_thres, float ed_thres);
      ~stop_n_wait_rx_ctrl_cc_impl();

      void set_high_threshold(float thres);
      float high_threshold()const;
      void set_low_threshold(float thres);
      float low_threshold()const;
      void set_ed_threshold(float thres);
      float ed_threshold()const;

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_STOP_N_WAIT_RX_CTRL_CC_IMPL_H */

