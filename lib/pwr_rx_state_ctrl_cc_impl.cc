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
    static const pmt::pmt_t d_voe_tag = pmt::intern("voe_tag");
    static const unsigned char d_collision_bytes[] = {0xff,0x00};
    static const unsigned char d_clear_bytes[] = {0x00,0xff,0x0f};
    pwr_rx_state_ctrl_cc::sptr
    pwr_rx_state_ctrl_cc::make()
    {
      return gnuradio::get_initial_sptr
        (new pwr_rx_state_ctrl_cc_impl());
    }

    /*
     * The private constructor
     */
    pwr_rx_state_ctrl_cc_impl::pwr_rx_state_ctrl_cc_impl()
      : gr::block("pwr_rx_state_ctrl_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
              d_fb_port(pmt::mp("fb_out")),
              d_in_port(pmt::mp("pu_in")),
              d_colli_blob(pmt::cons(pmt::PMT_NIL,pmt::make_blob(d_collision_bytes,EVENT_COLLISION))),
              d_clear_blob(pmt::cons(pmt::PMT_NIL,pmt::make_blob(d_clear_bytes,EVENT_CLEAR)))
    {
      enter_idle();
      message_port_register_out(d_fb_port);
      message_port_register_in(d_in_port);
      set_msg_handler(d_in_port,boost::bind(&pwr_rx_state_ctrl_cc_impl::pu_msg_in,this,_1));
      set_tag_propagation_policy(TPP_DONT);
      d_report_event = IDLE;
    }

    /*
     * Our virtual destructor.
     */
    pwr_rx_state_ctrl_cc_impl::~pwr_rx_state_ctrl_cc_impl()
    {
    }

    void
    pwr_rx_state_ctrl_cc_impl::enter_idle()
    {
      d_state = IDLE;
      d_collision_cnt =0;
      pub_msg(EVENT_CLEAR);
    }
    void
    pwr_rx_state_ctrl_cc_impl::enter_collision()
    {
      d_state = COLLISION;
      d_collision_cnt++;
      pub_msg(EVENT_COLLISION);
    }
    void
    pwr_rx_state_ctrl_cc_impl::pub_msg(int event)
    {
      d_report_event = event;
      d_pub_state.notify_one();
    }
    void
    pwr_rx_state_ctrl_cc_impl::run()
    {
      while(true){
        if(d_report_event == EVENT_CLEAR){
          message_port_pub(d_fb_port,d_clear_blob);
        }else if(d_report_event == EVENT_COLLISION){
          dout<<"publish clear state"<<std::endl;
          message_port_pub(d_fb_port,d_colli_blob);
        }else{
          // undefined
        }
        gr::thread::scoped_lock lock(d_mutex);
        d_pub_state.wait(lock);
        lock.unlock();
        if(d_finished){
          return;
        }
      }
    }
    void
    pwr_rx_state_ctrl_cc_impl::pu_msg_in(pmt::pmt_t msg)
    {
      pmt::pmt_t k = pmt::car(msg);
      pmt::pmt_t v = pmt::cdr(msg);
      if(pmt::is_blob(v)){
        size_t io(0);
        const uint8_t* uvec = pmt::u8vector_elements(v,io);
        if(io>0){
          dout<<"received a valid pu packet of size="<<io<<", collision cnt="<<d_collision_cnt<<std::endl;
          // valid pu packet
          if(d_state == COLLISION && d_collision_cnt>0){
            // if detect a pu, subtract one
            d_collision_cnt--;
            dout<<"resolved one collision, "<<d_collision_cnt<<" left"<<std::endl; 
          }
          if(d_collision_cnt==0){
            enter_idle();
          }
        }
      }
    }
    bool
    pwr_rx_state_ctrl_cc_impl::start()
    {
      d_finished = false;
      d_thread = boost::shared_ptr<gr::thread::thread>
        (new gr::thread::thread(boost::bind(&pwr_rx_state_ctrl_cc_impl::run,this)));
      return block::start();
    }
    bool
    pwr_rx_state_ctrl_cc_impl::stop()
    {
      d_finished = true;
      d_pub_state.notify_one();
      d_thread->interrupt();
      d_thread->join();
      return block::stop();
    }
    void
    pwr_rx_state_ctrl_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      for(int i=0;i<ninput_items_required.size();++i)
        ninput_items_required[i] = noutput_items;
    }

    int
    pwr_rx_state_ctrl_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
      int nin = std::min(ninput_items[0],noutput_items);
      int count =0;
      std::vector<tag_t> tags;
      if(nin==0){
        consume_each(0);
        return 0;
      }
      memcpy(out,in,sizeof(gr_complex)*nin);
      get_tags_in_window(tags,0,0,nin,d_voe_tag);
      for(count = 0; count<nin;++count){
        if(!tags.empty()){
          int offset = tags[0].offset - nitems_read(0);
          if(offset == count){
            if(pmt::to_bool(tags[0].value)){
              enter_collision();
            }
            d_voe_state = pmt::to_bool(tags[0].value);
            tags.erase(tags.begin());
          }
        }
        //volk_32fc_x2_conjugate_dot_prod_32fc(&corr_val,in+count,d_samples.data(),d_samples.size());
        //volk_32fc_x2_conjugate_dot_prod_32fc(&eng,in+count,in+count,d_samples.size());
        //corr_norm = corr_val / (std::sqrt(eng*d_sEng)+gr_complex(1e-6,0)); 
        //if(outCorr){
        //  corr[count] = corr_norm;
        //}
        /*if(std::abs(corr_norm)>=d_threshold){
          if(d_state == COLLISION && !d_voe_state && d_gap_cnt>d_gapLen){
            d_collision_cnt--;
            if(d_collision_cnt==0){
              enter_idle();
            }
          }
        }*/
        //d_gap_cnt++;
      }
      consume_each (count);
      return nin;
    }

  } /* namespace lsa */
} /* namespace gr */

