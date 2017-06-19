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
#include "stop_n_wait_rx_ctrl_cc_impl.h"

namespace gr {
  namespace lsa {

    #define d_debug true
    #define DEBUG d_debug && std::cout
    #define SNS_COLLISION 2
    #define SNS_CLEAR 3
    #define MAXLEN 125*8*8/2*4
    
    static int d_voe_valid = 128;
    static int d_min_gap = 8*8*8/2*4;
    static int d_burst_max_diff = 256;
    //static int d_max_waiting = 256*MAXLEN;
    static int d_max_waiting = CLOCKS_PER_SEC * 10;
    static unsigned char d_sns_collision[] = {0x00,0xff};
    static unsigned char d_sns_clear[] = {0x00,0xff,0x0f};

    enum SNSRXSTATE{
      SEARCH_COLLISION,
      SEARCH_STOP,
      WAIT_BURST,
      WAIT_RESUME
    };

    stop_n_wait_rx_ctrl_cc::sptr
    stop_n_wait_rx_ctrl_cc::make(float high_thres, float low_thres, float ed_thres)
    {
      return gnuradio::get_initial_sptr
        (new stop_n_wait_rx_ctrl_cc_impl(high_thres,low_thres,ed_thres));
    }

    /*
     * The private constructor
     */
    stop_n_wait_rx_ctrl_cc_impl::stop_n_wait_rx_ctrl_cc_impl(float high_thres, float low_thres, float ed_thres)
      : gr::block("stop_n_wait_rx_ctrl_cc",
              gr::io_signature::make3(3, 3, sizeof(gr_complex),sizeof(float),sizeof(float)),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
              d_out_port(pmt::mp("ctrl_out"))
    {
      d_state = SEARCH_COLLISION;
      set_high_threshold(high_thres);
      set_low_threshold(low_thres);
      set_ed_threshold(ed_thres);
      enter_search_collision();
      message_port_register_out(d_out_port);
    }

    /*
     * Our virtual destructor.
     */
    stop_n_wait_rx_ctrl_cc_impl::~stop_n_wait_rx_ctrl_cc_impl()
    {
    }

    void
    stop_n_wait_rx_ctrl_cc_impl::enter_search_collision()
    {
      d_state = SEARCH_COLLISION;
      d_voe_cnt = 0;
      d_voe_duration =0;
    }
    void
    stop_n_wait_rx_ctrl_cc_impl::enter_search_stop()
    {
      d_state = SEARCH_STOP;
      d_voe_duration =0;
      d_voe_cnt = 0;
      d_burst_lock = false;
      d_burst_voe_cnt =0;
      d_target_burst_cnt = 0;
    }
    void
    stop_n_wait_rx_ctrl_cc_impl::enter_wait_burst()
    {
      d_state = WAIT_BURST;
      d_voe_cnt =0;
      d_burst_voe_cnt=0;
      d_burst_lock = false;
      d_voe_duration =0;
      d_clock_duration = std::clock();
    }
    void
    stop_n_wait_rx_ctrl_cc_impl::enter_wait_resume()
    {
      d_voe_duration =0;
      d_state = WAIT_RESUME;
      d_voe_cnt=0;
      d_clock_duration = std::clock();
    }

    void
    stop_n_wait_rx_ctrl_cc_impl::set_high_threshold(float thres)
    {
      d_high_thres = thres;
    }

    float
    stop_n_wait_rx_ctrl_cc_impl::high_threshold() const
    {
      return d_high_thres;
    }

    void
    stop_n_wait_rx_ctrl_cc_impl::set_low_threshold(float thres)
    {
      d_low_thres = thres;
    }

    float
    stop_n_wait_rx_ctrl_cc_impl::low_threshold() const
    {
      return d_low_thres;
    }

    void
    stop_n_wait_rx_ctrl_cc_impl::set_ed_threshold(float thres)
    {
      d_ed_thres = thres;
    }
    
    float
    stop_n_wait_rx_ctrl_cc_impl::ed_threshold()const
    {
      return d_ed_thres;
    }

    void
    stop_n_wait_rx_ctrl_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      for(int i=0;i<ninput_items_required.size();++i){
        ninput_items_required[i] = noutput_items;
      }
    }

    int
    stop_n_wait_rx_ctrl_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      const float *voe = (const float*) input_items[1];
      const float *ed = (const float*) input_items[2];
      gr_complex *out = (gr_complex *) output_items[0];
      int nin = std::min(std::min(ninput_items[0],ninput_items[1]),ninput_items[2]);
      int nout =0;
      int count =0;
      
      switch(d_state){
        case SEARCH_COLLISION:
          while(nout<noutput_items && count<nin){
            if(voe[count]>d_high_thres){
              d_voe_cnt++;
              if(d_voe_cnt>=d_voe_valid){
                // consecutive higher than threshold
                pmt::pmt_t msg_out = pmt::cons(
                  pmt::intern("SNS_hdr"),
                  pmt::make_blob(d_sns_collision,SNS_COLLISION)
                );
                message_port_pub(d_out_port,msg_out);
                enter_search_stop();
                //DEBUG<<"\033[31;1m"<<"<SNS RX CTRL>Detect a collision event..."<<"\033[0m"<<std::endl;
                break;
              }
            }else{
              d_voe_cnt=0;
            }
            out[nout++] = in[count++];
          }
        break;
        case SEARCH_STOP:
          while(nout<noutput_items && count<nin){
            if(!d_burst_lock){
              if(voe[count]>d_high_thres){
                d_voe_duration++;
              }
              if(voe[count]<d_high_thres && voe[count]>d_low_thres){
                d_burst_voe_cnt++;
                if(d_burst_voe_cnt>=d_voe_valid){
                  d_burst_lock = true;
                  d_target_burst_cnt = (d_voe_duration>=d_min_gap)? d_voe_duration:d_min_gap;
                  DEBUG<<"\033[33;1m"<<"<SNS RX CTRL>VoE dropping down of high threshold...burst size:"
                  <<d_voe_duration<<"\033[0m"<<std::endl;
                  // debug for d_voe_duration;
                }
              }else{
                d_burst_voe_cnt=0;
              }
            }
            if(voe[count]<d_low_thres){
              d_voe_cnt++;
              if(d_voe_cnt>=d_voe_valid){
                DEBUG<<"\033[31;1m"<<"<SNS RX CTRL>VoE level drop out of low threshold, TX stop event detected"
                <<"\033[0m"<<std::endl;
                enter_wait_burst();
                break;
              }
            }else{
              d_voe_cnt=0;
            }
            out[nout++] = in[count++];
          }
        break;
        case WAIT_BURST:
          while(nout<noutput_items && count<nin){
            if(!d_burst_lock){
              if(ed[count]>d_ed_thres){
                d_voe_cnt++;
                if(d_voe_cnt>=d_voe_valid){
                  d_burst_lock = true;
                  d_burst_voe_cnt =0;
                  d_voe_cnt=0;
                  d_voe_duration =0; //
                  //DEBUG<<"\033[36;1m"<<"<SNS RX CTRL>detect a trigger signal, start track length"<<"\033[0m"<<std::endl;
                }
              }else{
                d_voe_cnt=0;
              }
            }else{
              d_burst_voe_cnt++;
              if(ed[count]<d_ed_thres){
                d_voe_cnt++;
                if(d_voe_cnt>=d_voe_valid){
                  if(abs(d_burst_voe_cnt-d_target_burst_cnt)<= d_burst_max_diff){
                    pmt::pmt_t msg_out = pmt::cons(
                      pmt::intern("SNS_hdr"),
                      pmt::make_blob(d_sns_clear,SNS_CLEAR));
                    message_port_pub(d_out_port,msg_out);
                    enter_wait_resume();
                    DEBUG<<"\033[31;1m"<<"<SNS RX CTRL>Detect a burst with matched length:"<<"\033[0m"
                    <<" ,target:"<<d_target_burst_cnt<<" ,matched:"<<d_burst_voe_cnt<<std::endl;
                    break;
                  }
                  DEBUG<<"\033[36;1m"<<"<SNS RX CTRL>length not matched, reset tracking registers"<<"\033[0m"
                  <<" ,expected:"<<d_target_burst_cnt<<" ,current:"<<d_burst_voe_cnt<<std::endl;
                  d_burst_voe_cnt = 0;
                  d_burst_lock = false;
                  d_voe_cnt=0;
                }
              }else{
                d_voe_cnt=0;
              }
            }
            out[nout++] = in[count++];
            //d_voe_duration++;
            if(std::clock()-d_clock_duration >= d_max_waiting){
              // waiting too long, maybe ProU is truned off...
              pmt::pmt_t msg_out = pmt::cons(
                pmt::intern("SNS_hdr"),
                pmt::make_blob(d_sns_clear,SNS_CLEAR)
              );
              message_port_pub(d_out_port,msg_out);
              enter_wait_resume();
              DEBUG<<"\033[33;1m"<<"<SNS RX CTRL>Waiting time exceed limit, notify tx to resume..."<<"\033[0m"<<std::endl;
              break;
            }
          }
        break;
        case WAIT_RESUME:
          while(nout<noutput_items && count<nin){
            if(voe[count]>d_low_thres){
              d_voe_cnt++;
              if(d_voe_cnt>=d_min_gap){
                //DEBUG<<"\033[31;1m"<<"<SNS RX CTRL>Signal level exceed low threshold...tx resume"<<"\033[0m"<<std::endl;
                enter_search_collision();
                break;
              }
            }else{
              d_voe_cnt=0;
            }
            out[nout++] = in[count++];
            // can add warning if not resume for a long time
          }
        break;
        default:
          throw std::runtime_error("Undefined state");
        break;
      }

      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (count);

      // Tell runtime system how many output items we produced.
      return nout;
    }

  } /* namespace lsa */
} /* namespace gr */

