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
#include "su_pwrCtrl_stream_tx_bb_impl.h"
#include <cstring>

namespace gr {
  namespace lsa {
    #define TIMEOUT_MS 5000
    #define PHYLEN 6
    #define HDRLEN 4
    #define RETRYLIMIT 10
    static unsigned char d_su_preamble[] = {0x00,0x00,0x00,0x00,0xE6,0x00};
    static int d_hdr_len = 4;
    su_pwrCtrl_stream_tx_bb::sptr
    su_pwrCtrl_stream_tx_bb::make(
      const std::string& tagname,
      const std::string& filename,
      bool useFile,
      float pwr1,
      float pwr2)
    {
      return gnuradio::get_initial_sptr
        (new su_pwrCtrl_stream_tx_bb_impl(tagname,filename, useFile, pwr1, pwr2));
    }

    /*
     * The private constructor
     */
    su_pwrCtrl_stream_tx_bb_impl::su_pwrCtrl_stream_tx_bb_impl(
      const std::string& tagname,
      const std::string& filename,
      bool useFile,
      float pwr1,
      float pwr2)
      : gr::tagged_stream_block("su_pwrCtrl_stream_tx_bb",
              gr::io_signature::make(1, 1, sizeof(unsigned char)),
              gr::io_signature::make(1, 1, sizeof(unsigned char)), tagname),
              d_fb_port(pmt::mp("fb_in")),
              d_pwr_out_port(pmt::mp("pwr_out"))
    {
      message_port_register_in(d_fb_port);
      message_port_register_out(d_pwr_out_port);
      set_msg_handler(d_fb_port,boost::bind(&su_pwrCtrl_stream_tx_bb_impl::fb_in,this,_1));
      if(useFile){
        if(!read_data(filename)){
          d_useFile = false;
        }
        d_fileSize = d_data_src.size();
        d_useFile = true;
      }
      d_seqno = 0;
      d_collision_state = false;
      d_success_pkt =0;
      d_failed_pkt = 0;
      memcpy(d_buf,d_su_preamble,sizeof(char)*PHYLEN);
      set_power(pwr1,pwr2);
    }

    /*
     * Our virtual destructor.
     */
    su_pwrCtrl_stream_tx_bb_impl::~su_pwrCtrl_stream_tx_bb_impl()
    {
    }
    bool
    su_pwrCtrl_stream_tx_bb_impl::start()
    {
      d_finished = false;
      d_thread = boost::shared_ptr<gr::thread::thread>
                  (new gr::thread::thread(boost::bind(&su_pwrCtrl_stream_tx_bb_impl::run,this)));
      return block::start();
    }
    bool
    su_pwrCtrl_stream_tx_bb_impl::stop()
    {
      d_finished = true;
      d_thread->interrupt();
      d_thread->join();
      return block::stop();
    }
    void
    su_pwrCtrl_stream_tx_bb_impl::run()
    {
      pmt::pmt_t pwr_tag;
      double next_pwr;
      while(true){
        next_pwr = (d_collision_state) ? d_pwr_low : d_pwr_high;
        pwr_tag = pmt::cons(pmt::PMT_NIL,pmt::from_double(next_pwr));
        message_port_pub(d_pwr_out_port,pwr_tag);
        gr::thread::scoped_lock lock(d_mutex);
        d_update_pwr.wait(lock);
        lock.unlock();
        if(d_finished){
          return;
        }
      }
    }
    void
    su_pwrCtrl_stream_tx_bb_impl::fb_in(pmt::pmt_t msg)
    {
      gr::thread::scoped_lock guard(d_mutex);
      bool sensing = pmt::to_bool(pmt::dict_ref(msg,pmt::intern("LSA_sensing"),pmt::PMT_F));
      bool pu_clear = pmt::to_bool(pmt::dict_ref(msg,pmt::intern("PWR_clear"),pmt::PMT_F));
      int seqno = pmt::to_long(pmt::dict_ref(msg,pmt::intern("base"),pmt::from_long(-1)));
      std::list<boost::tuples::tuple<uint16_t, boost::posix_time::ptime, int> >::iterator it;
      if(sensing){
        if(!d_collision_state){
          d_collision_state = true;
          // change power to low level
          message_port_pub(d_pwr_out_port,pmt::cons(pmt::PMT_NIL,pmt::from_double((double)d_pwr_low)));
          return;
        }
      }
      if(d_collision_state && pu_clear){
        d_collision_state = false;
        message_port_pub(d_pwr_out_port,pmt::cons(pmt::PMT_NIL,pmt::from_double((double)d_pwr_high)));
        return;
      }
      // acking
      if(seqno<0){
        return;
      }
      it = d_ack_pending.begin();
      while(it!=d_ack_pending.end()){
        uint16_t tmp_seq = boost::tuples::get<0>(*it);
        if(tmp_seq == seqno){
          it = d_ack_pending.erase(it);
          d_success_pkt++;
          break;
        }
        it++;
      }
    }
    int
    su_pwrCtrl_stream_tx_bb_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
    {
      int nex_in = (d_useFile)? d_data_src[d_seqno].size() : ninput_items[0];
      int noutput_items;
      // if timeout, retransmit
      if(checkTimeout()){
        uint16_t first_seq = boost::tuples::get<0>(d_ack_pending.front());
        noutput_items = d_data_src[first_seq].size() + PHYLEN + HDRLEN;
      }else{
        noutput_items = d_data_src[d_seqno].size() + PHYLEN + HDRLEN;
      }
      return noutput_items ;
    }

    void
    su_pwrCtrl_stream_tx_bb_impl::set_power(float pwr1,float pwr2)
    {
      if(pwr1>pwr2){
        d_pwr_low = pwr2;
        d_pwr_high = pwr1;
      }
      d_pwr_low = pwr1;
      d_pwr_high = pwr2;
    }
    float 
    su_pwrCtrl_stream_tx_bb_impl::power_low() const
    {
      return d_pwr_low;
    }
    float
    su_pwrCtrl_stream_tx_bb_impl::power_high() const
    {
      return d_pwr_high;
    }
    // helper functions
    bool
    su_pwrCtrl_stream_tx_bb_impl::checkTimeout()
    {
      if(d_ack_pending.empty()){
        return false;
      }
      boost::posix_time::ptime timeNow = boost::posix_time::microsec_clock::local_time();
      boost::posix_time::time_duration diff = timeNow - boost::tuples::get<1>(d_ack_pending.front());
      return diff.total_milliseconds()>TIMEOUT_MS; 
    }
    bool
    su_pwrCtrl_stream_tx_bb_impl::read_data(const std::string& filename)
    {
      gr::thread::scoped_lock guard(d_mutex);
      std::string str,line;
      if(!d_data_src.empty()){
        for(int i=0; i<d_data_src.size();++i)
          d_data_src[i].clear();
        d_data_src.clear();
      }
      if(d_file.is_open())
        d_file.close();
      d_file.open(filename.c_str(),std::fstream::in);
      if(d_file.is_open()){
        while(getline(d_file,line,'\n')){
          std::istringstream temp(line);
          std::vector<unsigned char> u8;
          while(getline(temp,str,',')){
            int tmp = std::atoi(str.c_str());
            u8.push_back((unsigned char)tmp);
          }
          d_data_src.push_back(u8);
        }
      }else{
        d_data_src.clear();
      }
      d_file.close();
      return !d_data_src.empty();
    }
    int
    su_pwrCtrl_stream_tx_bb_impl::work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const unsigned char *in = (const unsigned char *) input_items[0];
      unsigned char *out = (unsigned char *) output_items[0];
      uint16_t next_seq;
      uint8_t pkt_len;
      boost::posix_time::ptime nowTime = boost::posix_time::microsec_clock::local_time();
      int retry_cnt = boost::get<2>(d_ack_pending.front());
      if(checkTimeout() && retry_cnt< RETRYLIMIT){
        next_seq = boost::get<0>(d_ack_pending.front());
        // update ack list
        retry_cnt++;
        d_ack_pending.push_back(boost::tuples::make_tuple(next_seq,nowTime,retry_cnt));
        d_ack_pending.pop_front();
      }else{
        if(retry_cnt == RETRYLIMIT){
          d_failed_pkt++;
          d_ack_pending.pop_front();
        }
        next_seq = d_seqno;
        d_seqno = (d_seqno == 0xffff)? 0 : d_seqno+1;
        d_seqno %= d_fileSize;
        // insert into ack list
        d_ack_pending.push_back(boost::tuples::make_tuple(next_seq,nowTime,0));
      }
      pkt_len = (uint8_t) d_data_src[next_seq].size();
      d_buf[PHYLEN-1] = pkt_len;
      uint8_t* u8_seq = (uint8_t*) &next_seq;
      d_buf[PHYLEN] = u8_seq[1];
      d_buf[PHYLEN+1] = u8_seq[0];
      d_buf[PHYLEN+2] = u8_seq[1];
      d_buf[PHYLEN+3] = u8_seq[0];
      noutput_items = pkt_len + PHYLEN+HDRLEN;
      memcpy(out,d_buf,sizeof(char)*noutput_items);
      return noutput_items;
    }

  } /* namespace lsa */
} /* namespace gr */

