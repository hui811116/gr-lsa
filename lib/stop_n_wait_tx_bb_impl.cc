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
#include "stop_n_wait_tx_bb_impl.h"

namespace gr {
  namespace lsa {

    stop_n_wait_tx_bb::sptr
    stop_n_wait_tx_bb::make(const std::string& tagname, 
                            const std::string& filename, 
                            bool usef,bool verb,int send,
                            float mean, float std)
    {
      return gnuradio::get_initial_sptr
        (new stop_n_wait_tx_bb_impl(tagname,filename,usef,verb,send, mean, std));
    }

    /*
     * The private constructor
     */
    stop_n_wait_tx_bb_impl::stop_n_wait_tx_bb_impl(const std::string& tagname,
                                                   const std::string& filename, 
                                                   bool usef,bool verb, int send,
                                                   float mean, float std)
      : gr::tagged_stream_block("stop_n_wait_tx_bb",
              gr::io_signature::make(1, 1, sizeof(char)),
              gr::io_signature::make(1, 1, sizeof(char)), tagname),
              d_in_port(pmt::mp("msg_in")),
              d_tagname(pmt::intern(tagname)),
              d_rng(),
              d_mean(mean),
              d_std(std)
    {
      boost::poisson_distribution<> pd(d_mean);
      d_variate_poisson = boost::shared_ptr< boost::variate_generator<boost::mt19937, boost::poisson_distribution<> > >(
        new boost::variate_generator <boost::mt19937, boost::poisson_distribution<> >(d_rng,pd) );
      d_sns_stop = false; // waiting for receiver approve transmission
      message_port_register_in(d_in_port);
      set_msg_handler(d_in_port, boost::bind(&stop_n_wait_tx_bb_impl::msg_handler,this, _1));
      memcpy(d_buf,d_phy_field,sizeof(char)*PHYLEN);
      d_seq = 0x0000;
      d_usef = usef;
      d_verb = verb;
      if(usef){
        if(!read_data(filename)){
          d_usef = false;
        }
      }
      d_pkt_success_cnt=0;
      d_pkt_total=0;
      if(send<=0){
        throw std::invalid_argument("Invalid Send size...");
      }
      set_send(send);
      d_send_cnt=0;
      d_collision = false;
    }

    /*
     * Our virtual destructor.
     */
    stop_n_wait_tx_bb_impl::~stop_n_wait_tx_bb_impl()
    {
    }

    bool
    stop_n_wait_tx_bb_impl::read_data(const std::string& filename)
    {
      gr::thread::scoped_lock guard(d_mutex);
      std::string str,line;
      if(d_file.is_open()){
        d_file.close();
      }
      if(!d_data_src.empty()){
        for(int i=0;i<d_data_src.size();++i)
          d_data_src[i].clear();
        d_data_src.clear();
      }
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

    void
    stop_n_wait_tx_bb_impl::msg_handler(pmt::pmt_t msg)
    {
      gr::thread::scoped_lock guard(d_mutex);
      if(!pmt::dict_has_key(msg,pmt::intern("SNS_ctrl"))){
        return;
      }
      size_t ctrl_type = pmt::to_long(pmt::dict_ref(msg,pmt::intern("SNS_ctrl"),pmt::from_long(-1)));
      int seqno = pmt::to_long(pmt::dict_ref(msg,pmt::intern("base"),pmt::from_long(-1)));
      if(ctrl_type == SNS_COLLISION){
        d_collision = true;
      }else if(ctrl_type == SNS_CLEAR){
        d_collision = false;
      }else if(seqno>=0){
        std::list<srArq_t>::iterator it;
        for(it = d_arq_list.begin();it!=d_arq_list.end();++it){
          if(seqno == it->seq()){
            it = d_arq_list.erase(it);
            d_pkt_success_cnt++;
            break;
          }
        }
      }else{
        return;
      }
    }
    void
    stop_n_wait_tx_bb_impl::generate_new_pkt(const unsigned char* in, int nin)
    {
      int pkt_len = nin+MACLEN;
      memcpy(d_buf+PHYLEN+MACLEN,in,sizeof(char)*nin);
      uint8_t* u8_seq = (uint8_t*)&d_seq;
      d_buf[PHYLEN-1] = (unsigned char) pkt_len;
      d_buf[PHYLEN] = u8_seq[1];
      d_buf[PHYLEN+1] = u8_seq[0];
      d_buf[PHYLEN+2] = u8_seq[1];
      d_buf[PHYLEN+3] = u8_seq[0];
      pmt::pmt_t blob = pmt::make_blob(d_buf,pkt_len+PHYLEN);
      d_arq_list.push_back(srArq_t(d_seq,blob));
      d_seq = (d_seq==0xffff)? 0 : d_seq+1;
    }
    void
    stop_n_wait_tx_bb_impl::set_send(int send)
    {
      if(send>0){
        d_send_size = send;
      }
    }
    float
    stop_n_wait_tx_bb_impl::next_delay()
    {
      return d_variate_poisson->operator()();
    }
    void
    stop_n_wait_tx_bb_impl::wait()
    {
      int wait_cnt=0;
      int wait_lim = 10000/d_mean;
      while(!d_finished){
        gr::thread::scoped_lock lock(d_mutex);
        d_wait_delay.wait(lock);
        lock.unlock();
        if(d_finished){
          return;
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(next_delay()));
        if(d_finished){
          return;
        }
        wait_cnt = (d_collision)? wait_cnt+1 : wait_cnt;
        if(!d_collision || wait_cnt>=wait_lim){
          wait_cnt=0;
          d_sns_stop=false;
        }
      }
    }
    int
    stop_n_wait_tx_bb_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
    {
      int noutput_items =0;
      if(!d_sns_stop){
        std::list<srArq_t>::iterator it = d_arq_list.begin();
        if(d_arq_list.empty()){
          noutput_items = ninput_items[0] + PHYLEN + MACLEN;
          if(d_usef){
            noutput_items = d_data_src[d_seq%d_data_src.size()].size() + PHYLEN+MACLEN;
          }
        }else{
          noutput_items = it->blob_length();
        }
      }else{
        if(!d_gate_tag){
          noutput_items =1;
        }
      }
      return noutput_items;
    }

    int
    stop_n_wait_tx_bb_impl::work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const unsigned char *in = (const unsigned char *) input_items[0];
      unsigned char *out = (unsigned char *) output_items[0];
      int nin = ninput_items[0];
      if(d_usef){
        in = d_data_src[d_seq%d_data_src.size()].data();
        nin = d_data_src[d_seq%d_data_src.size()].size();
      }
      std::list<srArq_t>::iterator it = d_arq_list.begin();
      pmt::pmt_t blob;
      int nout =0;
      if(d_sns_stop){
        if(!d_gate_tag){
          d_gate_tag = true;
          add_item_tag(0,nitems_written(0),pmt::intern("sns_stop"),pmt::PMT_T);
          out[0] = 0;
          return 1;
        }else{
          return 0;
        }
      }else{
        // check arq 
        while(it!=d_arq_list.end()){
          if(it->timeout()){
            it->inc_retry();
            blob = it->msg();
            d_arq_list.push_back(*it);
            d_arq_list.pop_front();
            break;
          }else{
            // found a pkt that is not timeout...
            it = d_arq_list.end();
            break;
          }
        }
        if(it==d_arq_list.end()){
          // add new pkt
          // filling PKT_LEN+MAC SEQ
          int pkt_len = nin + MACLEN;
          assert(pkt_len <= MAX_PAYLOAD);
          generate_new_pkt(in,nin);
          nout = pkt_len+PHYLEN;
          memcpy(out,d_buf,sizeof(char)*nout);
          d_pkt_total++;
        }else{
          size_t io(0);
          const uint8_t* uvec = pmt::u8vector_elements(blob,io);
          memcpy(out,uvec,sizeof(char)*io);
          nout = io;
        }
        if(d_send_cnt==0){
          add_item_tag(0,nitems_written(0),pmt::intern("sns_start"),pmt::PMT_T);
        }
        d_send_cnt++;
        if(d_send_cnt==d_send_size){
          d_send_cnt=0;
          d_sns_stop = true;
          d_gate_tag = false;
          d_wait_delay.notify_one();
        }
        return nout;
      }
    }
    bool 
    stop_n_wait_tx_bb_impl::start()
    {
      d_finished = false;
      d_start_time = boost::posix_time::second_clock::local_time();
      d_thread = boost::shared_ptr<gr::thread::thread>
        (new gr::thread::thread(boost::bind(&stop_n_wait_tx_bb_impl::run,this)));
      d_stop_thread = boost::shared_ptr<gr::thread::thread>
        (new gr::thread::thread(boost::bind(&stop_n_wait_tx_bb_impl::wait,this)));
      return block::start();
    }
    bool
    stop_n_wait_tx_bb_impl::stop()
    {
      d_finished = true;
      d_thread->interrupt();
      d_stop_thread->interrupt();
      d_stop_thread->join();
      d_thread->join();
      return block::stop();
    }
    void
    stop_n_wait_tx_bb_impl::run()
    {
      while(!d_finished){
        boost::this_thread::sleep(boost::posix_time::milliseconds(10000));
        if(d_finished){
          return;
        }
        boost::posix_time::time_duration diff = boost::posix_time::second_clock::local_time()-d_start_time;
        if(d_verb){
          std::cout<<"<SNS TX>Execution time:"<<diff.total_seconds();
          std::cout<<" total packets:"<<d_pkt_total<<" ,success packets:"<<d_pkt_success_cnt<<std::endl;
        }
      }
    }

  } /* namespace lsa */
} /* namespace gr */

