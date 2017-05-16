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
#include <iomanip>

namespace gr {
  namespace lsa {

enum STATMODE{
  PROUTX=0,
  PROURX=1,
  SURX=2,
  SUTX=3
};

class stat_report_impl: public stat_report
{
  public:
  stat_report_impl(float ms,int mode): block("stat_report",
                gr::io_signature::make(0,0,0),
                gr::io_signature::make(0,0,0)),
                d_msg_portname(pmt::mp("msg_in")),
                d_finished(false)
  {
    message_port_register_in(d_msg_portname);
    set_msg_handler(d_msg_portname,boost::bind(&stat_report_impl::msg_handler,this,_1));
    set_period(ms);
    // initialize statistics
    reset_cnt();
    switch(mode)
    {
      case PROUTX:
        d_mode = PROUTX;
        d_mode_type = "ProU TX";
      break;
      case PROURX:
        d_mode = PROURX;
        d_mode_type = "ProU RX";
      break;
      case SURX:
        d_mode = SURX;
        d_mode_type = "SU RX";
      break;
      case SUTX:
        d_mode = SUTX;
        d_mode_type = "SU TX";
      break;
      default:
        throw std::runtime_error("Undefined mode, forced stop");
      break;
    }
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
        if(pmt::eqv(k,pmt::intern("LSA_hdr"))||
           pmt::eqv(k,pmt::intern("ProU"))){
          d_suc_pkt_cnt++;
        }
      }
    }
    else{
      //not dictionary, possible for transmitter messages
      if(pmt::eqv(msg,pmt::intern("TX"))){
        d_tx_cnt++;
      }
      else if(pmt::eqv(msg,pmt::intern("LSA_data"))){
        d_tx_cnt++;
        d_data_pkt_cnt++;
      }
      else if(pmt::eqv(msg,pmt::intern("LSA_RE"))){
        d_tx_cnt++;
        d_retx_pkt_cnt++;
      }
    }
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
  int d_suc_pkt_cnt;             // record success packets
  int d_retx_pkt_cnt;            // record retransmitted packets
  int d_data_pkt_cnt;            // record data packets
  uint64_t d_byte_cnt;           // the overall byte counter
  uint64_t d_acc_byte;           // instanious byte count between periods
  unsigned int d_iter_acc;       // iteration counter for periods
  
  int d_tx_cnt;                  // transmitted packets
  
  pmt::pmt_t d_msg;
  boost::shared_ptr<gr::thread::thread> d_thread;
  bool d_finished;
  float d_period_ms;
  const pmt::pmt_t d_msg_portname;
  
  int d_mode;                    // for different statistic reports
  std::string d_mode_type;       // for easy output
  

  void
  reset_cnt()
  {
    d_suc_pkt_cnt =0;
    d_retx_pkt_cnt =0;
    d_data_pkt_cnt =0;
    d_tx_cnt =0;
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
    }
  }
  void
  gen_stat()
  {
    
    d_iter_acc++;  // add for every statistic reporter
    std::cout<<"\n===============Statistics==============="<<std::endl;
    std::cout<<std::left<<std::setw(30)<<"<System Type>"<<std::right<<std::setw(10)<<d_mode_type<<std::endl;
    std::cout<<std::left<<std::setw(30)<<"<Runtime (s)>"<<std::right<<std::setw(10)<<d_iter_acc*d_period_ms/1000<<std::endl;
    switch(d_mode)
    {
      case PROUTX:
        std::cout<<std::left<<std::setw(30)<<"<Transmit packets>"<<std::right<<std::setw(10)<<d_tx_cnt<<std::endl;
      break;
      case PROURX:
        d_byte_cnt+= d_acc_byte;
        // show the statistics 
        std::cout<<std::left<<std::setw(30)<<"<Throughput(Bytes/s)>"<<std::right<<std::setw(10)<<d_acc_byte/d_period_ms*1000<<std::endl;
        std::cout<<std::left<<std::setw(30)<<"<Success packets>"<<std::right<<std::setw(10)<<d_suc_pkt_cnt<<std::endl;
        std::cout<<std::left<<std::setw(30)<<"<Data packets>"<<std::right<<std::setw(10)<<d_data_pkt_cnt<<std::endl;
        // update counters and registers
        d_acc_byte =0;
      break;
      case SUTX:
        std::cout<<std::left<<std::setw(30)<<"<Transmit packets>"<<std::right<<std::setw(10)<<d_tx_cnt<<std::endl;
        std::cout<<std::left<<std::setw(30)<<"<Data packets>"<<std::right<<std::setw(10)<<d_data_pkt_cnt<<std::endl;
        std::cout<<std::left<<std::setw(30)<<"<Retransmission>"<<std::right<<std::setw(10)<<d_retx_pkt_cnt<<std::endl;
      break;
      case SURX:
        d_byte_cnt+= d_acc_byte;
        // show the statistics 
        std::cout<<std::left<<std::setw(30)<<"<Throughput(Bytes/s)>"<<std::right<<std::setw(10)<<d_acc_byte/d_period_ms*1000<<std::endl;
        std::cout<<std::left<<std::setw(30)<<"<Success packets>"<<std::right<<std::setw(10)<<d_suc_pkt_cnt<<std::endl;
        std::cout<<std::left<<std::setw(30)<<"<Data packets>"<<std::right<<std::setw(10)<<d_data_pkt_cnt<<std::endl;
        std::cout<<std::left<<std::setw(30)<<"<Retransmission>"<<std::right<<std::setw(10)<<d_retx_pkt_cnt<<std::endl;
        // update counters and registers
        d_acc_byte =0;
      break;
    }
    std::cout<<"===================End=================="<<std::endl;
  }
};

stat_report::sptr
    stat_report::make(const float& ms, int mode){
      return gnuradio::get_initial_sptr(new stat_report_impl(ms, mode));
    }

  } /* namespace lsa */
} /* namespace gr */

