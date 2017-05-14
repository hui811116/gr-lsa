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
#include <lsa/stat_report.h>
#include <gnuradio/block_detail.h>

namespace gr {
  namespace lsa {


class stat_report_impl: public stat_report
{
  public:
  stat_report_impl(float ms): block("stat_report",
                gr::io_signature::make(0,0,0),
                gr::io_signature::make(0,0,0)),
                d_msg_portname(pmt::mp("msg_in")),
                d_finished(false)
  {
    //message_port_register_out(pmt::mp("stat"));
    message_port_register_in(d_msg_portname);
    set_msg_handler(d_msg_portname,boost::bind(&stat_report_impl::msg_handler,this,_1));

    set_period(ms);
    // initialize statistics
  }
  ~stat_report_impl()
  {

  }
  void
  set_period(const float& ms)
  {
    if(ms<0){
      throw std::runtime_error("period cannot be negative");
    }
    d_period_ms = ms;
  }
  float
  period()const
  {
    return d_period_ms;
  }

  void
  msg_handler(pmt::pmt_t msg)
  {

  }

  bool
  start()
  {
    d_finished = false;
    d_thread = boost::shared_ptr<gr::thread::thread>
      (new gr::thread::thread(boost::bind(&stat_report_impl::run,this)));
    return block::start();
  }

  bool
  stop()
  {
    d_finished = true;
    d_thread->interrupt();
    d_thread->join();

    return block::stop();
  }

  private:
  
  void
  run()
  {
    while(!d_finished){
      boost::this_thread::sleep(boost::posix_time::milliseconds(d_period_ms));
      if(d_finished){
        return;
      }
      gen_stat();
      //message_port_pub(d_msg_portname,d_stat);
    }
  }
  void
  gen_stat()
  {
    // show the statistics 
    std::cout<<""<<std::endl;
  }

    boost::shared_ptr<gr::thread::thread> d_thread;
    bool d_finished;
    float d_period_ms;
    const pmt::pmt_t d_msg_portname;
    //pmt::pmt_t d_stat;

};

stat_report::sptr
    stat_report::make(float ms){
      return gnuradio::get_initial_sptr(new stat_report_impl(ms));
    }

  } /* namespace lsa */
} /* namespace gr */

