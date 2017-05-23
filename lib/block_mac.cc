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
#include <lsa/block_mac.h>
#include <gnuradio/block_detail.h>
//#include <thread>
#include <queue>
//#include <atomic>

namespace gr {
  namespace lsa {

    class block_mac_impl:public block_mac{
      public: 
        block_mac_impl(int block_len, float timeout,int retx_lim, bool debug,bool verbose):block("block_mac",
                  gr::io_signature::make(0,0,0),
                  gr::io_signature::make(0,0,0)),
                  d_block_size(block_len),
                  d_verbose(verbose),
                  d_in_port(pmt::mp("msg_in")),
                  d_ack_port(pmt::mp("ack_in")),
                  d_out_port(pmt::mp("msg_out"))
        {
          if(block_len <0){
            throw std::invalid_argument("block length cannot be negative");
          }
          if(timeout<=0){
            throw std::invalid_argument("Timeout setting failed");
          }
          d_timeout = timeout;
          if(retx_lim<0){
            throw std::invalid_argument("Retransmission count cannot be negiative");
          }
          d_retx_lim = retx_lim;
          d_base = 0;
          d_debug = debug;
          message_port_register_in(d_in_port);
          message_port_register_in(d_ack_port);
          message_port_register_out(d_out_port);
          set_msg_handler(d_in_port,boost::bind(&block_mac_impl::msg_in,this,_1));
          set_msg_handler(d_ack_port,boost::bind(&block_mac_impl::ack_in,this,_1));
        }

        ~block_mac_impl()
        {
        }

        bool
        start()
        {
          d_thread = boost::shared_ptr<gr::thread::thread>
            (new gr::thread::thread(boost::bind(&block_mac_impl::thread_run,this)));
          return block::start();
        } 
        
        bool
        stop()
        {
          d_stop = true;
          //d_stop.store(true);
          d_ack_received.notify_one();
          d_thread->interrupt();
          d_thread->join();
          //std::cerr<<"<BLOCK MAC debug>calling stop"<<std::endl;
          return block::stop();
        }

        void
        msg_in(pmt::pmt_t msg)
        {
          pmt::pmt_t k = pmt::car(msg);
          pmt::pmt_t v = pmt::cdr(msg);
          enqueue(v);
        }

        void
        ack_in(pmt::pmt_t msg)
        {
          //gr::thread::scoped_lock lock(d_mutex);
          assert(pmt::is_pair(msg));
          pmt::pmt_t k = pmt::car(msg);
          pmt::pmt_t v = pmt::cdr(msg);
          if(pmt::to_long(k) == d_base){
            std::cerr<<"<BLOCK MAC> acked base point:"<<k<<std::endl;
            d_base++;
            //d_inc.store(true);
            d_inc=true;
          }
        }

        pmt::pmt_t 
        queue_pop()
        {
          gr::thread::scoped_lock lock(d_mutex);
          while(d_msg_queue.empty()){
            d_queue_filled.wait(lock);
          }
          pmt::pmt_t msg = d_msg_queue.front();
          d_msg_queue.pop();
          //lock.unlock();
          return msg;
        }
      private:
        void
        thread_run()
        {
          while(true){
            d_stop = false;
            d_inc = false;
            pmt::pmt_t to_send = queue_pop();
            int cur_base = d_base;
            int i=0;
            do{
              message_port_pub(d_out_port,pmt::cons(pmt::from_long(cur_base),to_send) );
              gr::thread::scoped_lock lock(d_mutex);
              // this is set for timeout;
              d_ack_received.timed_wait(lock, boost::posix_time::milliseconds(d_timeout));
              lock.unlock();
            }while( (i++ < d_retx_lim) && (!d_stop) && (!d_inc) );

            if(i>=d_retx_lim){
              // timeout
              // delare failed
              std::cerr<<"<Block Mac>Timeout, failed in transmission"<<std::endl;
            } else if(d_inc){
              // base point shifted
              std::cerr<<"<Block Mac>Base point received:"<<d_base<<std::endl;
            } else{
              // stopped;
            }
          }
        }
        void
        enqueue(pmt::pmt_t msg)
        {
          gr::thread::scoped_lock lock(d_mutex);
          if(d_msg_queue.size()<1000){
            d_msg_queue.push(msg);
          }
          else{
            std::cerr<<"enqueue failed: buffer full"<<std::endl;
          }
          d_queue_filled.notify_one();
          lock.unlock();
        }

        bool d_debug;
        bool d_verbose;
        int d_block_size;
        int d_retx_lim;
        int d_base;
        float d_timeout;

        gr::thread::mutex d_mutex;
        gr::thread::condition_variable d_ack_received;
        gr::thread::condition_variable d_queue_filled;
        boost::shared_ptr<gr::thread::thread> d_thread;
        bool d_stop;
        bool d_inc;
        //std::atomic_bool d_stop;
        //std::atomic_bool d_inc;
			  std::queue<pmt::pmt_t> d_msg_queue;
        const pmt::pmt_t d_out_port;
        const pmt::pmt_t d_in_port;
        const pmt::pmt_t d_ack_port;
    };

    block_mac::sptr
    block_mac::make(int block_len,float timeout, int retx_lim,bool debug,bool verbose){
      return gnuradio::get_initial_sptr(new block_mac_impl(block_len,timeout,retx_lim,debug,verbose));
    }

  } /* namespace lsa */
} /* namespace gr */

