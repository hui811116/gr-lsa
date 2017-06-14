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
#include <lsa/sinr_helper.h>
#include <gnuradio/block_detail.h>

namespace gr {
  namespace lsa {

    class sinr_helper_impl : public sinr_helper
    {
      public:
       sinr_helper_impl(float period, int hislen): block("sinr_helper",
                    gr::io_signature::make(0,0,0),
                    gr::io_signature::make(0,0,0)),
                    d_in_port(pmt::mp("SINR_in")),
                    d_osinr_port(pmt::mp("oSINR_out")),
                    d_csinr_port(pmt::mp("cSINR_out"))
       {
         message_port_register_in(d_in_port);
         message_port_register_out(d_osinr_port);
         message_port_register_out(d_csinr_port);
         set_msg_handler(d_in_port,boost::bind(&sinr_helper_impl::msg_in,this,_1));
         if(period<=0.0){
           throw std::invalid_argument("Invalid period");
         }
         d_period = period;
         if(hislen<=0){
          throw std::invalid_argument("Invalid history length");
         }
         d_hislen = hislen;
         d_osinr_stack.resize(d_hislen,1);
         d_csinr_stack.resize(d_hislen,1);
       }
       ~sinr_helper_impl(){}

       bool start()
       {
        d_finished = false;
        d_thread = boost::shared_ptr<gr::thread::thread>
          (new gr::thread::thread(boost::bind(&sinr_helper_impl::run,this)));
        return block::start();
       }
       bool stop()
       {
        d_finished = true;
        d_thread->interrupt();
        d_thread->join();
        return block::stop();
       }

      private:
       void msg_in(pmt::pmt_t msg)
       {
        assert(pmt::is_dict(msg));
        gr::thread::scoped_lock guard(d_mutex);
        double oSINR,cSINR;
        if(pmt::dict_has_key(msg,pmt::intern("original_SINR"))){
          oSINR = pmt::to_double(pmt::dict_ref(msg,pmt::intern("original_SINR"),pmt::from_double(1)));
          oSINR = (oSINR<=0)? 1 : oSINR;
          d_osinr_stack.push_back((float)oSINR);
          d_osinr_stack.erase(d_osinr_stack.begin());
        }
        if(pmt::dict_has_key(msg,pmt::intern("canceled_SINR"))){
          cSINR = pmt::to_double(pmt::dict_ref(msg,pmt::intern("canceled_SINR"),pmt::from_double(1)));
          cSINR = (cSINR<=0)? 1 : cSINR;
          d_csinr_stack.push_back((float)cSINR);
          d_csinr_stack.erase(d_csinr_stack.begin());
        }
        d_cancel_cnt++;
       }
       void run()
       {
        while(!d_finished){
          boost::this_thread::sleep(boost::posix_time::milliseconds(d_period));
          if(d_finished){
            return;
          }
          gen_sinr();
        }
       }
       void gen_sinr()
       {
         pmt::pmt_t osinr_out = pmt::cons(pmt::PMT_NIL,pmt::init_f32vector(d_hislen,d_osinr_stack.data()));
         pmt::pmt_t csinr_out = pmt::cons(pmt::PMT_NIL,pmt::init_f32vector(d_hislen,d_csinr_stack.data()));
         message_port_pub(d_osinr_port,osinr_out);
         message_port_pub(d_csinr_port,csinr_out);
       }
       const pmt::pmt_t d_in_port;
       const pmt::pmt_t d_osinr_port;
       const pmt::pmt_t d_csinr_port;
       std::vector<float> d_osinr_stack;
       std::vector<float> d_csinr_stack;
       uint32_t d_cancel_cnt;
       int d_hislen;
       float d_period;
       bool d_finished;
       boost::shared_ptr<gr::thread::thread> d_thread;
       gr::thread::mutex d_mutex;
    };

    sinr_helper::sptr 
    sinr_helper::make(float period, int hislen){
      return gnuradio::get_initial_sptr(new sinr_helper_impl(period, hislen));
    }

  } /* namespace lsa */
} /* namespace gr */

