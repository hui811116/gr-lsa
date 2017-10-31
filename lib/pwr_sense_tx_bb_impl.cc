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
#include "pwr_sense_tx_bb_impl.h"
#include <cstring>

namespace gr {
  namespace lsa {
    //static const pmt::pmt_t d_start_tag = pmt::intern("sns_start");
    //static const pmt::pmt_t d_stop_tag = pmt::intern("sns_stop");
    static const unsigned char d_phy[6] = {0x00,0x00,0x00,0x00,0xE6,0x00};
    pwr_sense_tx_bb::sptr
    pwr_sense_tx_bb::make(const std::string& lentag, const std::string& filename,
      bool useFile, float highPower, float lowPower, float senseTime)
    {
      return gnuradio::get_initial_sptr
        (new pwr_sense_tx_bb_impl(lentag,filename,useFile,highPower,lowPower, senseTime));
    }

    /*
     * The private constructor
     */
    pwr_sense_tx_bb_impl::pwr_sense_tx_bb_impl(
      const std::string& lentag,
      const std::string& filename,
      bool useFile,
      float highPower,
      float lowPower,
      float senseTime
      )
      : gr::tagged_stream_block("pwr_sense_tx_bb",
              gr::io_signature::make(1, 1, sizeof(char)),
              gr::io_signature::make(1, 1, sizeof(char)), lentag),
              d_fb_port(pmt::mp("fb_in")),
              d_pwr_port(pmt::mp("pwr_out")),
              d_name(pmt::intern(alias()))
    {
      message_port_register_in(d_fb_port);
      message_port_register_out(d_pwr_port);
      set_msg_handler(d_fb_port,boost::bind(&pwr_sense_tx_bb_impl::fb_in,this,_1));
      d_state = SENSING;
      memcpy(d_buf,d_phy,sizeof(char)*PHYLEN);
      if(useFile){
        if(!read_data(filename)){
          throw std::invalid_argument("File cannot be opened");
        }
      }
      d_useFile = useFile;
      set_power(highPower,lowPower);
      set_sense_time(senseTime);
      d_seqno = 0;
      //d_last_burst = false;
      //d_first_burst = false;
      d_sysTime = boost::posix_time::microsec_clock::local_time();
    }

    /*
     * Our virtual destructor.
     */
    pwr_sense_tx_bb_impl::~pwr_sense_tx_bb_impl()
    {
    }
    bool
    pwr_sense_tx_bb_impl::start()
    {
      d_finished = false;
      d_thread = boost::shared_ptr<gr::thread::thread>
          (new gr::thread::thread(boost::bind(&pwr_sense_tx_bb_impl::run,this)));
      return block::start();
    }
    bool
    pwr_sense_tx_bb_impl::stop()
    {
      d_finished = true;
      d_sense_received.notify_one();
      d_burst_ctrl.notify_one();
      d_thread->interrupt();
      d_thread->join();
      return block::stop();
    }
    void 
    pwr_sense_tx_bb_impl::set_power(float high, float low)
    {
      if(high>low){
        d_high_power = (double)high;
        d_low_power = (double)low;
      }else{
        d_high_power= (double)low;
        d_low_power = (double)high;
      }
    }
    void
    pwr_sense_tx_bb_impl::set_sense_time(float senseTime){
      if(senseTime<=0){
        std::cerr<<"Warning: sensing time set to non-positive number"<<std::endl;
        d_sense_time = 50.0;
      }else{
        d_sense_time = senseTime;
      }
    }
    float
    pwr_sense_tx_bb_impl::get_high_power() const
    {
      return (float)d_high_power;
    }
    float
    pwr_sense_tx_bb_impl::get_low_power() const
    {
      return (float)d_low_power;
    }
    void
    pwr_sense_tx_bb_impl::run()
    {
      //boost::posix_time::ptime sysTime = boost::posix_time::microsec_clock::local_time();
      while(true){
        if(d_finished){
          return;
        }
        gr::thread::scoped_lock lock(d_mutex);
        switch(d_state){
          case SENSING:
            //dout<<"<DEBUG> At Sensing stage, wait "<< d_sense_time <<" ms and change state"<<std::endl;
            d_sense_received.timed_wait(lock,boost::posix_time::milliseconds(d_sense_time));
            lock.unlock();
            if(d_sense_collision){
              //dout<<"<DEBUG> detect collision, remain sensing"<<std::endl;
              d_sense_collision = false;
              d_state = SENSING;
            }else{
              boost::posix_time::time_duration diff= boost::posix_time::microsec_clock::local_time()-d_sysTime;
              dout<<"<DEBUG> clear to send, change state to CLEAR, "<<
              diff.total_milliseconds()<<std::endl;
              //d_first_burst = true;
              d_state = CLEAR;
              d_pwr_msg = pmt::cons(pmt::PMT_NIL, pmt::from_double(d_high_power));
              message_port_pub(d_pwr_port,d_pwr_msg);
            }
          break;
          case COLLISION:
            d_sense_received.wait(lock);
            lock.unlock();
            if(d_sense_clear){
              boost::posix_time::time_duration diff= boost::posix_time::microsec_clock::local_time()-d_sysTime;
              dout<<"<DEBUG> found a clear tag, change state to Sensing, "<<
              diff.total_milliseconds()<<std::endl;
              d_sense_clear = false;
              //d_last_burst = true;
              d_state = SENSING;
            }else{
              d_state = COLLISION;
            }
          break;
          case CLEAR:
            d_sense_received.wait(lock);
            lock.unlock();
            if(d_sense_collision){
              //dout<<"<DEBUG> at Clear, detect a collision, chage state to COllISION"<<std::endl;
              d_sense_collision = false;
              d_state = COLLISION;
              d_pwr_msg = pmt::cons(pmt::PMT_NIL, pmt::from_double(d_low_power));
              message_port_pub(d_pwr_port,d_pwr_msg);
            }else{
              d_state = CLEAR;
            }
          break;
          default:
            std::runtime_error("undefined state");
          break;
        }
      }
    }
    void
    pwr_sense_tx_bb_impl::fb_in(pmt::pmt_t msg)
    {
      // if sense request
      if(!pmt::dict_has_key(msg,pmt::intern("SNS_ctrl"))){
        return;
      }
      //dout<<"<DEBUG> received a feedback from rx"<<std::endl;
      int ctrl_type = pmt::to_long(pmt::dict_ref(msg,pmt::intern("SNS_ctrl"),pmt::from_long(-1)));
      bool sensing = (ctrl_type == EVENT_COLLISION);
      bool pu_clear = (ctrl_type == EVENT_CLEAR);
      int seqno = pmt::to_long(pmt::dict_ref(msg,pmt::intern("base"),pmt::from_long(-1)));
      std::list<boost::tuples::tuple<uint16_t, boost::posix_time::ptime, int> >::iterator it;
      switch(d_state){
        case SENSING:
          if(sensing){
            d_sense_collision = true;
          }
        break;
        case COLLISION:
          if(pu_clear){
            d_sense_clear = true;
            d_sense_received.notify_one();
          }
        break;
        case CLEAR:
          if(sensing){
            d_sense_collision = true;
            d_sense_received.notify_one();
          }
        break;
        default:
          throw std::runtime_error("undefined state");
        break;
      }
      if(seqno<0){
        return;
      }
      for(it = d_ack_pending.begin();it!=d_ack_pending.end();++it){
        uint16_t tmp_seq = boost::tuples::get<0>(*it);
        if(seqno==tmp_seq){
          it = d_ack_pending.erase(it);
          break;
        }
      }
    }
    bool
    pwr_sense_tx_bb_impl::check_timeout()
    {
      if(d_ack_pending.empty()){
        return false;
      }else{
        boost::posix_time::time_duration diff = 
          boost::posix_time::microsec_clock::local_time() - boost::tuples::get<1>(d_ack_pending.front());
        if(diff.total_milliseconds() > ACK_TIMEOUT){
          return true;
        }else{
          return false;
        }
      }
    }
    bool
    pwr_sense_tx_bb_impl::read_data(const std::string& filename)
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
    pwr_sense_tx_bb_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
    {
      int noutput_items = 0;
      switch(d_state){
        case SENSING:
          noutput_items = 0;
        break;
        case COLLISION:
        case CLEAR:
          if(d_useFile){
            if(check_timeout()){
              int seq_begin = boost::tuples::get<0>(d_ack_pending.front());
              noutput_items = d_data_src[seq_begin].size()+PHYLEN+HDRLEN;
            }else{
              noutput_items = d_data_src[d_seqno].size()+PHYLEN+HDRLEN;
            }
          }else{
            noutput_items = ninput_items[0]+PHYLEN+HDRLEN;
          }
        break;
        default:
          throw std::runtime_error("undefined state");
        break;
      }
      return noutput_items ;
    }

    int
    pwr_sense_tx_bb_impl::work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const unsigned char *in = (const unsigned char *) input_items[0];
      unsigned char *out = (unsigned char *) output_items[0];
      const uint64_t nwrite = nitems_written(0);
      int nout;
      uint16_t outSeq;
      uint8_t pktlen;
      boost::posix_time::ptime timeNow = boost::posix_time::microsec_clock::local_time();
      switch(d_state){
        case SENSING:
          nout = 0;
          return nout;
        break;
        case COLLISION:
        case CLEAR:
          if(check_timeout()){
            outSeq = boost::tuples::get<0>(d_ack_pending.front());
            nout = (d_useFile)? d_data_src[outSeq].size() : ninput_items[0];
            nout +=  (PHYLEN + HDRLEN );
            int retry = boost::tuples::get<2>(d_ack_pending.front())+1;
            d_ack_pending.push_back(boost::tuples::make_tuple(outSeq,timeNow,retry));
            d_ack_pending.pop_front();
          }else{
            outSeq = d_seqno;
            if(d_useFile){
              nout = d_data_src[d_seqno].size() + PHYLEN + HDRLEN; 
              d_seqno = (d_seqno==0xffff) ? 0x0000 : d_seqno+1;
              d_seqno %= d_data_src.size(); 
            }else{
              nout = ninput_items[0] + PHYLEN + HDRLEN;
            }
            d_ack_pending.push_back(boost::tuples::make_tuple(outSeq,timeNow,0));
          }
        break;
        default:
          throw std::runtime_error("undefined state");
        break;
      }
      if(d_useFile){
        pktlen = (uint8_t) d_data_src[outSeq].size();
        memcpy(&out[PHYLEN+HDRLEN],d_data_src[outSeq].data(),sizeof(char)*pktlen);
      }else{
        pktlen = (uint8_t) ninput_items[0];
        memcpy(&out[PHYLEN+HDRLEN],in,sizeof(char)*pktlen);
      }
      memcpy(out,d_buf,sizeof(char)*(PHYLEN));
      uint8_t* u8seq = (uint8_t*) &outSeq;
      out[PHYLEN-1] = pktlen+HDRLEN;
      out[PHYLEN] = u8seq[1];
      out[PHYLEN+1] = u8seq[0];
      out[PHYLEN+2] = u8seq[1];
      out[PHYLEN+3] = u8seq[0];
      
      /*if(d_first_burst){
        d_first_burst = false;
        add_item_tag(0,nwrite,d_start_tag,pmt::PMT_T,d_name);
        boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::local_time()-d_sysTime;
        dout<<"<DEBUG> first burst tag added"<<diff.total_milliseconds()<<std::endl;
      }
      if(d_last_burst){
        d_last_burst = false;
        add_item_tag(0,nwrite+nout-1,d_stop_tag,pmt::PMT_T,d_name);
        boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::local_time()-d_sysTime;
        dout<<"<DEBUG> last collision packet and change state to sensing,"<<diff.total_milliseconds()<<std::endl;
      }*/
      return nout;
    }

  } /* namespace lsa */
} /* namespace gr */

