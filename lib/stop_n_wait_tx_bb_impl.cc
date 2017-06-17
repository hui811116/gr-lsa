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

   #define d_debug true 
   #define DEBUG d_debug && std::cout
   #define MAX_PAYLOAD = 125
   
    static int PHYLEN = 6;
    static int MACLEN = 2;
    static unsigned char d_phy_field[] = {0x00,0x00,0x00,0x00,0xE6,0x00};
    static unsigned char d_mac_field[] = {0x00,0x00};
   
    stop_n_wait_tx_bb::sptr
    stop_n_wait_tx_bb::make(const std::string& tagname)
    {
      return gnuradio::get_initial_sptr
        (new stop_n_wait_tx_bb_impl(tagname));
    }

    /*
     * The private constructor
     */
    stop_n_wait_tx_bb_impl::stop_n_wait_tx_bb_impl(const std::string& tagname)
      : gr::tagged_stream_block("stop_n_wait_tx_bb",
              gr::io_signature::make(1, 1, sizeof(char)),
              gr::io_signature::make(1, 1, sizeof(char)), tagname),
              d_in_port(pmt::mp("msg_in")),
              d_tagname(pmt::intern(tagname))
    {
      d_sns_stop = false;
      message_port_register_in(d_in_port);
      set_msg_handler(d_in_port, boost::bind(&stop_n_wait_tx_bb_impl::msg_handler,this, _1));
      memcpy(d_buf,d_phy_field,sizeof(char)*PHYLEN);
      d_seq = 0x0000;
    }

    /*
     * Our virtual destructor.
     */
    stop_n_wait_tx_bb_impl::~stop_n_wait_tx_bb_impl()
    {
    }

    void
    stop_n_wait_tx_bb_impl::msg_handler(pmt::pmt_t msg)
    {
      gr::thread::scoped_lock guard(d_mutex);
      if(pmt::dict_has_key(msg,pmt::intern("LSA_sensing"))){
        // update stop and wait state
        bool result = pmt::to_bool(pmt::dict_ref(msg,pmt::intern("LSA_sensing"),pmt::PMT_T));
        if(d_sns_stop && !result){
          DEBUG<<"\033[31;1m"<<"<SNS TX> ProU detected from feedback... resume to transmission mode"<<"\033[0m"<<std::endl;
          d_sns_stop = false;
          return;
        }else if(!d_sns_stop && result){
          DEBUG<<"\033[31;1m"<<"<SNS TX> Causing interference to ProU, Stop transmission"<<"\033[0m"<<std::endl;
          d_sns_stop = true;
          return;
        }
      }
      // arq queue update
      // find matched seqno
      int seqno = pmt::to_long(pmt::dict_ref(msg,pmt::intern("base"),pmt::from_long(-1)));
      if(seqno<0){
        return;
      }
      std::list<srArq_t>::iterator it;
      for(it = d_arq_list.begin();it!=d_arq_list.end();++it){
        if(seqno == it->seq()){
          it = d_arq_list.erase(it);
          break;
        }
      }
    }
    void
    stop_n_wait_tx_bb_impl::generate_new_pkt(const unsigned char* in, int nin)
    {
      int pkt_len = nin+PHYLEN;
      memcpy(d_buf+PHYLEN+MACLEN,in,sizeof(char)*nin);
      uint8_t* u8_seq = (uint8_t*)&d_seq;
      d_buf[PHYLEN-1] = (unsigned char) pkt_len;
      d_buf[PHYLEN] = u8_seq[1];
      d_buf[PHYLEN+1] = u8_seq[0];
      pmt::pmt_t blob = pmt::make_blob(d_buf,pkt_len+PHYLEN);
      d_arq_list.push_back(srArq_t(d_seq,blob));
      d_seq = (d_seq==0xffff)? 0 : d_seq+1;
    }

    int
    stop_n_wait_tx_bb_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
    {
      int noutput_items =0;
      if(!d_sns_stop){
        std::list<srArq_t>::iterator it = d_arq_list.begin();
        if(d_arq_list.empty()){
          noutput_items = ninput_items[0] + PHYLEN + MACLEN;
        }else{
          if(it->timeout() && it->retry()>= LSARETRYLIM){
            noutput_items = it->blob_length();
          }else{
            noutput_items = ninput_items[0] + PHYLEN + MACLEN;
          }
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
      std::list<srArq_t>::iterator it = d_arq_list.begin();
      pmt::pmt_t blob;
      int nout =0;
      if(d_sns_stop){
        return 0;
      }else{
        // check arq 
        while(it!=d_arq_list.end()){
          if(it->timeout()){
            if(it->inc_retry()){
              d_arq_list.pop_front();
              it = d_arq_list.begin();
            }else{
              blob = it->msg();
              it->update_time();
              d_arq_list.push_back(*it);
              d_arq_list.pop_front();
              break;
            }
          }else{
            // found a pkt that is not timeout...
            it = d_arq_list.end();
            break;
          }
        }
        if(it==d_arq_list.end()){
          // add new pkt
          // filling PKT_LEN+MAC SEQ
          int pkt_len = ninput_items[0] + MACLEN;
          assert(pkt_len <= MAX_PAYLOAD);
          generate_new_pkt(in,ninput_items[0]);
          nout = pkt_len+PHYLEN;
          memcpy(out,d_buf,sizeof(char)*nout);
        }else{
          size_t io(0);
          const uint8_t* uvec = pmt::u8vector_elements(blob,io);
          memcpy(out,uvec,sizeof(char)*io);
          nout = io;
        }
      }
      return nout;
    }

  } /* namespace lsa */
} /* namespace gr */

