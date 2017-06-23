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
#include "su_sr_transmitter_bb_impl.h"

namespace gr {
  namespace lsa {

#define d_debug false
#define DEBUG d_debug && std::cout

    static int LSAPHYLEN = 6;
    static int LSAMACLEN = 8;
    static unsigned char LSAPHY[] = {0x00,0x00,0x00,0x00,0xE6,0x00};
    static unsigned char LSAMAC[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};  // 2,2,2,2
    static int d_retx_retry_limit = 20;

    su_sr_transmitter_bb::sptr
    su_sr_transmitter_bb::make(const std::string& tagname, const std::string& filename, bool usef)
    {
      return gnuradio::get_initial_sptr
        (new su_sr_transmitter_bb_impl(tagname, filename,usef));
    }

    /*
     * The private constructor
     */
    su_sr_transmitter_bb_impl::su_sr_transmitter_bb_impl(const std::string& tagname, const std::string& filename, bool usef)
      : gr::tagged_stream_block("su_sr_transmitter_bb",
              gr::io_signature::make(1, 1, sizeof(unsigned char)),
              gr::io_signature::make(1, 1, sizeof(unsigned char)), tagname),
              d_tagname(tagname),
              d_msg_in(pmt::mp("msg_in"))
    {
      d_usef = usef;
      if(usef){
        if(!read_data(filename)){
          d_usef = false;
        }
      }
      d_seq = 0;
      message_port_register_in(d_msg_in);
      set_msg_handler(d_msg_in,boost::bind(&su_sr_transmitter_bb_impl::msg_in,this,_1));
      d_prou_present = false;
      memcpy(d_buf,LSAPHY,sizeof(char)* LSAPHYLEN);
    }

    /*
     * Our virtual destructor.
     */
    su_sr_transmitter_bb_impl::~su_sr_transmitter_bb_impl()
    {
    }

    int
    su_sr_transmitter_bb_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
    {
      int noutput_items = ninput_items[0]+LSAPHYLEN+LSAMACLEN;
      if(d_usef){
        noutput_items = d_data_src[d_seq%d_data_src.size()].size()+LSAPHYLEN+LSAMACLEN;
      }
      std::list<srArq_t>::iterator it;
      if(d_prou_present){        
        if(!retx_peek_front(noutput_items)){
          throw std::runtime_error("<SU SR TX>\033[33;1mERROR:In retransmission state but found no pending packets\033[0m");
        }
      }else{
        bool has_pkt = peek_front(noutput_items);
      }
      return noutput_items ;
    }

    void
    su_sr_transmitter_bb_impl::msg_in(pmt::pmt_t msg)
    {
      // should be filtered to save complexity of this block
      // only input dict with valid messages
      gr::thread::scoped_lock guard(d_mutex);
      // if is sensing information, lock current queue and change state
      // crc should handle invalid packets
      bool sensing = pmt::to_bool(pmt::dict_ref(msg,pmt::intern("LSA_sensing"),pmt::PMT_F));
      int seqno = pmt::to_long(pmt::dict_ref(msg,pmt::intern("base"),pmt::from_long(-1)));
      int qidx = pmt::to_long(pmt::dict_ref(msg,pmt::intern("queue_index"),pmt::from_long(-1)));
      int qsize = pmt::to_long(pmt::dict_ref(msg,pmt::intern("queue_size"),pmt::from_long(-1)));
      std::list<srArq_t>::iterator it;
      if(sensing){
        // alert
        if(!d_prou_present){
          // build retransmission
          d_prou_present = create_retx_queue();
        }else if(d_retx_cnt!=0){
          // already passed sensing delay
          // reset retry count due to additional interference detection
          reset_retx_retry();
        }
        return;
      }
      if(d_prou_present){
        if( (qidx==0&& qsize==0) || qsize!=d_retx_size || qidx>=d_retx_size){
          return;
        }else if(d_retx_size ==0){
          return;
        }
        if(d_retx_table.empty()){
          throw std::runtime_error("WTF retx");
        }
        if(update_retx_table(qidx)){
          DEBUG<<"<SU SR TX>Received a retransmission, idx:"<<qidx<<" ,retx count:"<<d_retx_cnt<<"(expected:"<<d_retx_size<<")"<<std::endl;
          if(d_retx_cnt >= d_retx_size){
            d_prou_present = false;
            clear_queue();
            DEBUG<<"<SU SR TX>"<<"\033[31;1m"<<"Retransmission complete! resume to clear state"<<"\033[0m"<<std::endl;
          }
        }
      }else{
        // in clear state
        if(qidx!=0 || qsize!=0){
          return;
        }
        if(dequeue(seqno)){
          // success
          DEBUG<<"<SU SR TX> dequeue seqno:"<<seqno<<std::endl;
        }
      }
    }

    void
    su_sr_transmitter_bb_impl::reset_retx_retry()
    {
      for(int i=0;i<d_retx_queue.size();++i){
        d_retx_queue[i].set_retry(0);
      }
    }

    bool
    su_sr_transmitter_bb_impl::create_retx_queue()
    {
      std::list<srArq_t>::iterator it;
      if(d_arq_queue.empty()){
            return false;
          }else{
            d_retx_cnt= 0;
            d_retx_idx= 0;
            d_retx_table.clear();
            d_retx_queue.clear();
            d_retx_size=d_arq_queue.size();
            d_retx_table.resize(d_retx_size,false);
            for(it=d_arq_queue.begin();it!=d_arq_queue.end();++it){
              srArq_t tmp = *it;
              tmp.set_retry(0);
              d_retx_queue.push_back(tmp);
            }
          }
          return true;
    }

    pmt::pmt_t
    su_sr_transmitter_bb_impl::get_retx(int idx)
    {
      pmt::pmt_t msg = pmt::PMT_NIL;
      if(idx>=d_retx_queue.size()){
        return pmt::PMT_NIL;
      }
      return d_retx_queue[idx].msg();
    }
    bool
    su_sr_transmitter_bb_impl::retx_peek_front(int& len)
    {
      if(d_retx_size!=0){
        len = d_retx_queue[d_retx_idx].blob_length();
      }
      return !d_retx_queue.empty();
    }

    bool
    su_sr_transmitter_bb_impl::peek_front(int& len)
    {
      std::list<srArq_t>::iterator it;
      it = d_arq_queue.begin();
        if(it!=d_arq_queue.end()){
          if(it->timeout()){
            if(it->retry()<LSARETRYLIM){
              len = it->blob_length();
            }
          }
        }
      return !d_arq_queue.empty();
    }

    void
    su_sr_transmitter_bb_impl::clear_queue()
    {
      d_arq_queue.clear();
      d_retx_table.clear();
      d_retx_queue.clear();
      d_retx_cnt =0;
      d_retx_size =0;
    }
    void
    su_sr_transmitter_bb_impl::enqueue(const srArq_t& arq)
    {
      d_arq_queue.push_back(arq);
    }
    pmt::pmt_t
    su_sr_transmitter_bb_impl::check_timeout()
    {
      pmt::pmt_t nx_msg = pmt::PMT_NIL;
      std::list<srArq_t>::iterator it = d_arq_queue.begin();
      while(it!=d_arq_queue.end()){
        if(it->timeout()) {
            //check retry count
            if(it->inc_retry()){
              // reaching limit, should abort...
              DEBUG<<"<SU SR TX>"<<"\033[32;1m"<<"timeout..."<<*it<<"\033[0m"<<std::endl;
              d_arq_queue.pop_front();
              it = d_arq_queue.begin();
            }else{
              // add retry
              nx_msg = it->msg();
              it->update_time();
              d_arq_queue.push_back(*it);
              d_arq_queue.pop_front();
              break;
            }
          }else{
            break;
          }
      }
      return nx_msg;
    }

    bool
    su_sr_transmitter_bb_impl::dequeue(int seq)
    {
      std::list<srArq_t>::iterator it;
      gr::thread::scoped_lock guard(d_mutex);
      for(it = d_arq_queue.begin();it!=d_arq_queue.end();++it){
        if(it->seq()==seq){
          it = d_arq_queue.erase(it);          
          return true;
        }
      }
      return false;
    }

    bool
    su_sr_transmitter_bb_impl::read_data(const std::string& filename)
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
      DEBUG<<"reading dataset:"<<filename;
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
        DEBUG<<"...done"<<std::endl;
      }else{
        DEBUG<<"...fail"<<std::endl;
        d_data_src.clear();
      }
      d_file.close();
      return !d_data_src.empty();
    }

    bool
    su_sr_transmitter_bb_impl::check_retx_table(int idx)
    {
      if(idx>=d_retx_table.size()){
        throw std::runtime_error("WTF...");
      }
      return d_retx_table[idx];
    }
    bool
    su_sr_transmitter_bb_impl::update_retx_table(int idx){
      if(idx>=d_retx_table.size()){
        throw std::runtime_error("WTF......");
      }
      if(d_retx_table[idx]==false){
        d_retx_cnt++;
        d_retx_table[idx] = true;
        return true;
      }
      return false;
    }

    int
    su_sr_transmitter_bb_impl::work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const unsigned char *in = (const unsigned char *) input_items[0];
      unsigned char *out = (unsigned char *) output_items[0];
      int nin = ninput_items[0];
      int nout;
      pmt::pmt_t nx_msg =pmt::PMT_NIL;
      if(d_usef){
        DEBUG<<"<SR SU TX>Use file data: index="<<d_seq%d_data_src.size()<<std::endl;
        in = d_data_src[d_seq%d_data_src.size()].data();
        nin = d_data_src[d_seq%d_data_src.size()].size();
      }
      if(d_prou_present){
        // should do retransmission
        nx_msg = get_retx(d_retx_idx);
        if(pmt::eqv(nx_msg,pmt::PMT_NIL)){
          throw std::runtime_error("WTF");
        }
        if(d_retx_cnt!=0){
          // already passed sensing delay
          d_retx_queue[d_retx_idx].inc_retry();
          if(d_retx_queue[d_retx_idx].retry()>d_retx_retry_limit 
             && (check_retx_table(d_retx_idx)==false) ){
              if(update_retx_table(d_retx_idx)){
                DEBUG<<"<SU SR TX>"<<"\033[32;1m"<<"Retransmission achieve retry limit, forced true..."<<"\033[0m"<<std::endl;
              }
              if(d_retx_cnt>=d_retx_size){
                clear_queue();
                d_prou_present = false;
                DEBUG<<"<SU SR TX>"<<"\033[31;1m"<<"Failure: Retransmission exceed retry limit! resume to clear state"<<"\033[0m"<<std::endl;
                nout =0;
                return 0;
              }
          }
        }
        // consider a retry count to reduce retransmission time
        uint8_t* qidx = (uint8_t*) &d_retx_idx;
        uint8_t* qsize= (uint8_t*) &d_retx_size;
        d_retx_idx = (d_retx_idx+1)%d_retx_size;
        size_t io(0);
        const uint8_t* uvec = pmt::u8vector_elements(nx_msg,io);
        memcpy(out,uvec,sizeof(char)*io);
        // filling retransmission fields
        out[LSAPHYLEN]  = qidx[1];
        out[LSAPHYLEN+1]= qidx[0];
        out[LSAPHYLEN+2]= qsize[1];
        out[LSAPHYLEN+3]= qsize[0];
        nout = io;
      }else{
        // check timeout and retry count
        nx_msg = check_timeout();
        if(pmt::eqv(nx_msg,pmt::PMT_NIL)){
          // transmit new message
          uint8_t* u8_idx = (uint8_t*)&d_seq;
          // debugging purpose
          memcpy(d_buf+LSAPHYLEN,LSAMAC,sizeof(char)*LSAMACLEN);
          memcpy(d_buf+LSAPHYLEN+LSAMACLEN,in,sizeof(char)*nin);
          d_buf[5] = (unsigned char)(nin+LSAMACLEN);
          d_buf[LSAPHYLEN+4] = u8_idx[1];
          d_buf[LSAPHYLEN+5] = u8_idx[0];
          d_buf[LSAPHYLEN+6] = u8_idx[1];
          d_buf[LSAPHYLEN+7] = u8_idx[0];
          nout = nin + LSAPHYLEN + LSAMACLEN;
          nx_msg = pmt::make_blob(d_buf,nout);
          d_seq = (d_seq == 0xffff)? 0:d_seq; // wrap around
          srArq_t temp_arq(d_seq++,nx_msg);
          enqueue(temp_arq);
          memcpy(out,d_buf,sizeof(char)*(nout) );
        }else{
          // send existing message
          size_t io(0);
          const uint8_t* uvec = pmt::u8vector_elements(nx_msg,io);
          nout = io;
          memcpy(out,uvec,sizeof(char)*io);
        }
      }
      // Tell runtime system how many output items we produced.
      return nout;
    }

  } /* namespace lsa */
} /* namespace gr */

