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
                //d_strobe_port(pmt::mp("strobe_in")),
                d_finished(false)
  {
    //message_port_register_in(d_strobe_port);
    message_port_register_in(d_msg_portname);
    set_msg_handler(d_msg_portname,boost::bind(&stat_report_impl::msg_handler,this,_1));
    //set_msg_handler(d_strobe_port,boost::bind(&stat_report_impl::strobe_in,this,_1));
    // 
    set_period(ms);
    // initialize statistics
    reset_cnt();
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
    // pair and dict will both enter this state
    if(pmt::is_dict(msg)){
      if(pmt::dict_has_key(msg,pmt::intern("LSA_hdr"))){
        assert(pmt::dict_has_key(msg,pmt::intern("Type")));
        pmt::pmt_t type = pmt::dict_ref(msg,pmt::intern("Type"),pmt::intern("wrong"));
        if(pmt::eqv(type,pmt::intern("Fresh_data"))){
          d_data_pkt_cnt++;
          d_suc_pkt_cnt++;
          if(pmt::dict_has_key(msg,pmt::intern("payload"))){
            d_acc_byte += pmt::to_long(pmt::dict_ref(msg,pmt::intern("payload"),pmt::from_long(0)));
          }
        }
        else if(pmt::eqv(type,pmt::intern("Retransmission"))){
          d_retx_pkt_cnt++;
          d_suc_pkt_cnt++;
        }
        else{
          return;
        }
      }
      else{
        pmt::pmt_t k = pmt::car(msg);
        pmt::pmt_t v = pmt::cdr(msg);
        // add bytes
        if(pmt::is_blob(v)){
          d_acc_byte+= pmt::blob_length(v);
        }
        // add pkt type
        if(pmt::eqv(k,pmt::intern("LSA_hdr"))){
          d_suc_pkt_cnt++;
        }
      }
    }
    
  }

  void
  strobe_in(pmt::pmt_t strobe)
  {
    gen_stat();
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

  int d_suc_pkt_cnt;
  int d_retx_pkt_cnt;
  int d_data_pkt_cnt;
  uint64_t d_byte_cnt;           // the overall byte counter
  uint64_t d_acc_byte;           // instanious byte count between periods
  unsigned int d_iter_acc;       // iteration counter for periods
  //int d_inst_acc_pkt;
  //int d_inst_iter_cnt;
  pmt::pmt_t d_msg;
  boost::shared_ptr<gr::thread::thread> d_thread;
  bool d_finished;
  float d_period_ms;
  const pmt::pmt_t d_msg_portname;
  //const pmt::pmt_t d_strobe_port;
  //pmt::pmt_t d_stat;

  void
  reset_cnt()
  {
    d_suc_pkt_cnt =0;
    d_retx_pkt_cnt =0;
    d_data_pkt_cnt =0;
    //d_inst_acc_pkt = 0;
    //d_inst_iter_cnt =0;
    d_byte_cnt = 0;              
    d_acc_byte = 0;              
    d_iter_acc =0;               
  }

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
    d_iter_acc++;
    d_byte_cnt+= d_acc_byte;

    // show the statistics 
    std::cout<<"\n===Statistics==="<<std::endl;
    std::cout<<"<Throughput(Bytes/s)>"<<d_acc_byte/d_period_ms*1000<<std::endl;
    std::cout<<"<Runtime>"<<d_iter_acc*d_period_ms/1000<<" s"<<std::endl;
    //std::cout<<"--------------------------------------------------"<<std::endl;
    std::cout<<"<Success packets>"<<d_suc_pkt_cnt<<std::endl;
    std::cout<<"<Data packets>"<<d_data_pkt_cnt<<std::endl;
    std::cout<<"<Retransmission>"<<d_retx_pkt_cnt<<std::endl;
    std::cout<<"===end==="<<std::endl;
    // update counters and registers
    d_acc_byte =0;
  }
};

stat_report::sptr
    stat_report::make(const float& ms){
      return gnuradio::get_initial_sptr(new stat_report_impl(ms));
    }

  } /* namespace lsa */
} /* namespace gr */

