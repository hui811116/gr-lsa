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
#include "pwr_rx_state_ctrl_cc_impl.h"
#include <volk/volk.h>
#include <cstring>

namespace gr {
  namespace lsa {
    #define d_debug false
    #define dout d_debug && std::cout
    #define EVENT_COLLISION 2
    #define EVENT_CLEAR 3
    pwr_rx_state_ctrl_cc::sptr
    pwr_rx_state_ctrl_cc::make(float high_db,float low_db)
    {
      return gnuradio::get_initial_sptr
        (new pwr_rx_state_ctrl_cc_impl(high_db,low_db));
    }

    /*
     * The private constructor
     */
    pwr_rx_state_ctrl_cc_impl::pwr_rx_state_ctrl_cc_impl(float high_db,float low_db)
      : gr::block("pwr_rx_state_ctrl_cc",
              gr::io_signature::make(1, 1, sizeof(float)),
              gr::io_signature::make(0, 0, 0)),
              d_fb_port(pmt::mp("fb_out")),
              d_ed_valid(64)
    {
      enter_high();
      message_port_register_out(d_fb_port);
      set_threshold(high_db,low_db);
      d_update = false;
    }

    /*
     * Our virtual destructor.
     */
    pwr_rx_state_ctrl_cc_impl::~pwr_rx_state_ctrl_cc_impl()
    {
    }

    void
    pwr_rx_state_ctrl_cc_impl::enter_high()
    {
      d_state = HIGH;
      d_ed_cnt =0;
      d_cd_cnt=0;
      d_ncolli=0;
    }
    void
    pwr_rx_state_ctrl_cc_impl::enter_trans()
    {
      d_state = TRANS;
      d_ed_cnt =0;
      d_cd_cnt =0;
      d_ncolli = 1;
    }
    void
    pwr_rx_state_ctrl_cc_impl::enter_low()
    {
      d_state = LOW;
      d_ed_cnt =0;
    }
    void
    pwr_rx_state_ctrl_cc_impl::set_threshold(float high,float low)
    {
      gr::thread::scoped_lock guard(d_mutex);
      if(low>high){
        d_thres_high = low;
        d_thres_low = high;
      }else{
        d_thres_low = low;
        d_thres_high = high;
      }
    }
    void
    pwr_rx_state_ctrl_cc_impl::reset_state(bool reset)
    {
      gr::thread::scoped_lock guard(d_mutex);
      if(d_state != HIGH){
        d_update = true;
      }
    }
    void
    pwr_rx_state_ctrl_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = noutput_items;
    }

    int
    pwr_rx_state_ctrl_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const float *in = (const float *) input_items[0];
      int nin = ninput_items[0];
      int count =0;
      if(d_update){
        d_update = false;
        enter_high();
      }
      while(count<nin){
        switch(d_state){
          case HIGH:
            if(in[count++]>d_thres_high){
              d_ed_cnt++;
              if(d_ed_cnt==d_ed_valid){
                enter_trans();
                message_port_pub(d_fb_port,pmt::cons(pmt::PMT_NIL,pmt::from_long(EVENT_COLLISION)));
              }
            }else{
              d_ed_cnt=0;
            }
          break;
          case TRANS:
            if(in[count]>d_thres_high){
              d_cd_cnt++;
              if(d_cd_cnt == d_ed_valid){
                d_cd_cnt=0;
                d_ncolli++;
              }
            }else{
              d_cd_cnt=0;
            }
            if(in[count++]<d_thres_low){
              d_ed_cnt++;
              if(d_ed_cnt==d_ed_valid){
                enter_low();
              }
            }else{
              d_ed_cnt=0;
            }
          break;
          case LOW:
            if(in[count++]>d_thres_low){
              d_ed_cnt++;
              if(d_ed_cnt==d_ed_valid){
                d_ed_cnt=0;
                d_ncolli--;
                if(d_ncolli==0){
                  enter_high();
                  message_port_pub(d_fb_port,pmt::cons(pmt::PMT_NIL,pmt::from_long(EVENT_CLEAR)));
                }
              }
            }else{
              d_ed_cnt=0;
            }
          break;
          default:
            throw std::runtime_error("Undefined state");
          break;
        }
      }
      consume_each (count);
      return 0;
    }

  } /* namespace lsa */
} /* namespace gr */

