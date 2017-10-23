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
    #define EVENT_COLLISION 2
    #define EVENT_CLEAR 3
    static const pmt::pmt_t d_voe_tag = pmt::intern("voe_tag");
    static const unsigned char d_collision_bytes[] = {0xff,0x00};
    static const unsigned char d_clear_bytes[] = {0x00,0xff,0x0f};
    pwr_rx_state_ctrl_cc::sptr
    pwr_rx_state_ctrl_cc::make(
      const std::vector<gr_complex>& samples,
      float threshold,
      int gapLen)
    {
      return gnuradio::get_initial_sptr
        (new pwr_rx_state_ctrl_cc_impl(samples,threshold,gapLen));
    }

    /*
     * The private constructor
     */
    pwr_rx_state_ctrl_cc_impl::pwr_rx_state_ctrl_cc_impl(
      const std::vector<gr_complex>& samples,
      float threshold,
      int gapLen)
      : gr::block("pwr_rx_state_ctrl_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make2(1, 2, sizeof(gr_complex),sizeof(gr_complex))),
              d_max_out(8192*2),
              d_fb_port(pmt::mp("fb_out")),
              d_colli_blob(pmt::cons(pmt::PMT_NIL,pmt::make_blob(d_collision_bytes,EVENT_COLLISION))),
              d_clear_blob(pmt::cons(pmt::PMT_NIL,pmt::make_blob(d_clear_bytes,EVENT_CLEAR)))
    {
      const int outMax = 8192*2;
      if(samples.empty()){
        throw std::invalid_argument("Empty sync words");
      }
      d_samples = samples;
      volk_32fc_x2_conjugate_dot_prod_32fc(&d_sEng, samples.data(),samples.data(),samples.size());
      enter_idle();
      message_port_register_out(d_fb_port);
      set_tag_propagation_policy(TPP_DONT);
      set_history(samples.size());
      set_max_noutput_items(outMax);
      d_corr_buf= (gr_complex*) volk_malloc(sizeof(gr_complex)*(outMax),volk_get_alignment());
      set_threshold(threshold);
      set_gap(gapLen);
      d_report_event = IDLE;
    }

    /*
     * Our virtual destructor.
     */
    pwr_rx_state_ctrl_cc_impl::~pwr_rx_state_ctrl_cc_impl()
    {
      volk_free(d_corr_buf);
    }

    void
    pwr_rx_state_ctrl_cc_impl::set_threshold(float thres)
    {
      if(thres>1 || thres<0)
        throw std::runtime_error("threshold should be within 1");
      d_threshold = thres;
    }
    float
    pwr_rx_state_ctrl_cc_impl::get_threshold() const
    {
      return d_threshold;
    }
    void
    pwr_rx_state_ctrl_cc_impl::set_gap(int gapLen)
    {
      if(gapLen<=0){
        throw std::invalid_argument("Gap size should be positive");
      }
      d_gapLen = gapLen;
      d_gap_cnt =0;
    }
    int
    pwr_rx_state_ctrl_cc_impl::get_gap()const
    {
      return d_gapLen;
    }

    void
    pwr_rx_state_ctrl_cc_impl::enter_idle()
    {
      d_state = IDLE;
      d_collision_cnt =0;
      d_gap_cnt =0;
      pub_msg(EVENT_CLEAR);
    }
    void
    pwr_rx_state_ctrl_cc_impl::enter_collision()
    {
      d_state = COLLISION;
      d_collision_cnt++;
      d_gap_cnt =0;
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
        ninput_items_required[i] = noutput_items + history();
    }

    int
    pwr_rx_state_ctrl_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
      gr_complex *corr= d_corr_buf;
      bool outCorr = (output_items.size()==2);
      if(outCorr){
        corr = (gr_complex *) output_items[1];
      }
      int nin = std::min(std::max((int)ninput_items[0]-(int)history(),0),noutput_items);
      int count =0;
      bool thresCheck = false;
      std::vector<tag_t> tags;
      gr_complex corr_val, eng;
      if(nin==0){
        consume_each(0);
        return 0;
      }
      memcpy(out,in,sizeof(gr_complex)*nin);
      get_tags_in_window(tags,0,0,nin,d_voe_tag);
      for(int count = 0; count<nin;++count){
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
        volk_32fc_x2_conjugate_dot_prod_32fc(&corr_val,in+count,d_samples.data(),d_samples.size());
        volk_32fc_x2_conjugate_dot_prod_32fc(&eng,in+count,in+count,d_samples.size());
        corr[count] = corr_val/(std::sqrt(eng*d_sEng)+gr_complex(1e-6,0));
        thresCheck = abs(corr[count])>=d_threshold;
        if(d_state == IDLE){
          if(thresCheck){
              // debugging, not in collision state, but detect the sync word of PU
          }
        }else if(d_state == COLLISION){
          if(thresCheck && !d_voe_state && d_gap_cnt>d_gapLen){
            d_collision_cnt--;
            if(d_collision_cnt==0){
              enter_idle();
            }
          }
        }else{
          throw std::runtime_error("invalid state");
        }
        count++;
        d_gap_cnt++;
      }
      consume_each (count);
      return nin;
    }

  } /* namespace lsa */
} /* namespace gr */

