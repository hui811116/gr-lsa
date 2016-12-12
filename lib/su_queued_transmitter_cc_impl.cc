/* -*- c++ -*- */
/* 
 * Copyright 2016 <+YOU OR YOUR COMPANY+>.
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
#include "su_queued_transmitter_cc_impl.h"

namespace gr {
  namespace lsa {

    enum su_tx_state_t{
      CLEAR_TO_SEND,
      PROU_PRESENT
      //ERROR_AND_RESET
    };

    su_queued_transmitter_cc::sptr
    su_queued_transmitter_cc::make(
      int max_queue_size,
      const std::string &sensing_tag,
      const std::string &index_tag,
      const std::string &accesscode
      )
    {
      return gnuradio::get_initial_sptr
        (new su_queued_transmitter_cc_impl(
          max_queue_size,
          sensing_tag,
          index_tag,
          accesscode));
    }

    /*
     * The private constructor
     */
    su_queued_transmitter_cc_impl::su_queued_transmitter_cc_impl(
      int max_queue_size,
      const std::string &sensing_tag,
      const std::string &index_tag,
      const std::string &accesscode)
      : gr::block("su_queued_transmitter_cc",
              gr::io_signature::make(1, 1, sizeof(char)),
              gr::io_signature::make(2, 2, sizeof(char))),
      d_max_queue_size(max_queue_size),
      d_rx_sensing_tag(pmt::string_to_symbol(sensing_tag)),
      d_rx_index_tag(pmt::string_to_symbol(index_tag)),
      d_state(CLEAR_TO_SEND),
      d_pkt_counter(0)
    {
      if (d_max_queue_size < 1) {
        throw std::invalid_argument("Queue size must be greater than 0.");
      }
      d_buffer_ptr = new std::vector< std::vector<unsigned char> >;
      set_accesscode(accesscode);
      set_tag_propagation_policy(TPP_DONT);
      message_port_register_in(pmt::mp("rx_info"));
      set_msg_handler(pmt::mp("rx_info"), boost::bind(&su_queued_transmitter_cc_impl::receiver_msg_handler, this, _1));

    }

    /*
     * Our virtual destructor.
     */
    su_queued_transmitter_cc_impl::~su_queued_transmitter_cc_impl()
    {
      d_buffer_ptr->clear();
      delete d_buffer_ptr;
    }

    void
    su_queued_transmitter_cc_impl::receiver_msg_handler(pmt::pmt_t rx_msg)
    {
      d_rx_tag_keys.clear();
      d_rx_tag_values.clear();
      int tmp_sensing=-1,tmp_index=-1;


      pmt::pmt_t dict_items(pmt::dict_items(rx_msg));
      while(!pmt::is_null(dict_items)) {
        pmt::pmt_t this_item(pmt::car(dict_items));
        d_rx_tag_keys.push_back(pmt::car(this_item));
        d_rx_tag_values.push_back(pmt::cdr(this_item));
        if(pmt::equal(pmt::car(this_item), d_rx_sensing_tag)) {
          d_sensing = pmt::to_bool(pmt::cdr(this_item));
          tmp_sensing=1;
        }
        if(pmt::equal(pmt::car(this_item), d_rx_index_tag)) {
          d_index = pmt::to_long(pmt::cdr(this_item));
          tmp_index=1;
        }
        dict_items = pmt::cdr(dict_items);
      } // end while loop
      
      
      if((tmp_sensing <0) || (tmp_index <0)){
        return;
      }// information missing
      int index_in_queue=check_queue(tmp_index);
      if(d_sensing){
        if(d_state == CLEAR_TO_SEND){
          //lock queue;
          d_sensing_queue=d_index_buffer;
          d_sensing_count=0;
          d_sensing_iter=0;
          d_state = PROU_PRESENT;
        }
      }//sensing proU
      else{
        if(d_state == CLEAR_TO_SEND){
          if(index_in_queue>=0){
            d_index_buffer.erase(d_index_buffer.begin(),d_index_buffer.begin()+index_in_queue+1);
            d_buffer_ptr->erase(d_buffer_ptr->begin(),d_buffer_ptr->begin()+index_in_queue+1);
          }
        }
        else if(d_state == PROU_PRESENT){
            check_sensing_queue(index_in_queue);  
          if(d_sensing_count==d_sensing_queue.size()){
            d_sensing_queue.clear();
            d_index_buffer.clear();
            d_buffer_ptr->clear();
            d_sensing_count=0;
            //d_sensing_iter=0;
            d_state == CLEAR_TO_SEND;
          }//all clear
        }
      }//sensing no prou
      
    }//end of rx msg handler

    void
    su_queued_transmitter_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      int ninput_reqd=0;
      if(d_state == CLEAR_TO_SEND){
        ninput_reqd = noutput_items;
      }
      for(int i=0;i<ninput_items_required.size();++i){
        ninput_items_required[i]=ninput_reqd;
      }
    }

    int
    su_queued_transmitter_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const unsigned char *in = (const unsigned char *) input_items[0];
      unsigned char *out_pkt = (unsigned char *) output_items[0];
      unsigned char *out_hdr = (unsigned char *) output_items[1];
      int n_consume=1;
      int tmp_hold_size=0;

      // Do <+signal processing+>
      unsigned long pkt_size=(unsigned long)ninput_items[0];
      unsigned char hdr[d_accesscode.size()+6];
      generate_hdr(hdr,pkt_size);
      int nout_port0=0,nout_port1=0;
      switch (d_state) {
        case CLEAR_TO_SEND:
          n_consume=ninput_items[0];
          pkt_size=ninput_items[0]+d_accesscode.size()+4+2+2;
          d_buffer_ptr->push_back(copy_input_bytes(in,ninput_items[0]));
          d_index_buffer.push_back(d_pkt_counter);
          d_pkt_counter++;
          memcpy(out_pkt+8,in,sizeof(char)*ninput_items[0]);
        break;
        case PROU_PRESENT:
          tmp_hold_size=(d_buffer_ptr->at(d_sensing_iter)).size();
          pkt_size=tmp_hold_size+d_accesscode.size()+8;
          d_sensing_iter++;
          d_sensing_iter %= d_sensing_queue.size();
          memcpy(out_pkt+8,(d_buffer_ptr->at(d_sensing_iter)).data(),sizeof(char)*tmp_hold_size);
        break;
        default:
          throw std::runtime_error("invalid state");
        break;
      }
      memcpy(out_pkt,hdr,sizeof(char)*8);
      memcpy(out_hdr,hdr,sizeof(char)*8);
      produce(0,pkt_size);
      produce(1,d_accesscode.size()+8);
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (n_consume);

      // Tell runtime system how many output items we produced.
      return WORK_CALLED_PRODUCE;
    }

    void
    su_queued_transmitter_cc_impl::set_accesscode(const std::string& accesscode)
    {
      d_accesscode.clear();
      int s_len=accesscode.length();
      //automatic append to multiple of 8
      int new_size=((s_len%8) == 0)? s_len/8 : s_len/8+1;
      unsigned char tmp=0x00;
      for(int i=0;i<s_len;++i){
        tmp |= ((accesscode[i]=='0') ? 0x00 : 0x01) << 7-(i%8);
        if((i+1)%8==0){
          d_accesscode.push_back(tmp);
          tmp=0x00;
        }
      }
    }

    int
    su_queued_transmitter_cc_impl::check_queue(int check_index)
    {
      int offset;
      if(!d_index_buffer.empty()){
        offset=d_index_buffer[0];
        if(offset>check_index){
          return -1;
        }
        if((check_index-offset) > d_index_buffer.size()){
          return -1;
        }
        if(d_index_buffer[check_index-offset] == check_index){
          return check_index-offset;
        }
      }
      return -1;
    }
    bool
    su_queued_transmitter_cc_impl::check_sensing_queue(int check_index)
    {
      if(!d_sensing_queue.empty()){
        if(check_index<0){return -1;}
        if(check_index >= d_sensing_queue.size()){return -1;}
        if(d_sensing_queue[check_index]>=0){
          d_sensing_queue[check_index]=-1;
          d_sensing_count++;
        }
      }
      return false;
    }
    void
    su_queued_transmitter_cc_impl::generate_hdr(unsigned char* out, unsigned long size)
    {
      unsigned char* pkt_len= (unsigned char*)& size;
      unsigned char* pkt_counter= (unsigned char*)& d_pkt_counter;
      //if(d_state == CLEAR_TO_SEND){
      //  d_pkt_counter++;
      //}
      unsigned char retx_index=0x00,retx_size=0x00;
      int ac_len=d_accesscode.size();
      for(int i=0;i<d_accesscode.size();++i){
        out[i]=d_accesscode[i];
      }
        out[ac_len]=pkt_len[0];
        out[ac_len+1]=pkt_len[1];
        out[ac_len+2]=pkt_len[0];
        out[ac_len+3]=pkt_len[1];
        if(d_state == PROU_PRESENT){
          assert(d_sensing_queue.size()!=0);
          retx_index=d_sensing_iter;
          pkt_counter = (unsigned char*)& d_index_buffer[d_sensing_iter];
          retx_size=(unsigned char) d_sensing_queue.size();
          //d_sensing_iter++;
          //d_sensing_iter%=d_sensing_queue.size();
        }
        out[ac_len+4]=retx_size;
        out[ac_len+5]=retx_index;
        out[ac_len+6]=pkt_counter[0];
        out[ac_len+7]=pkt_counter[1];
      //memcpy(out+d_accesscode.size()+4+2,in,sizeof(unsigned char));      
    }
    std::vector<unsigned char>
    su_queued_transmitter_cc_impl::copy_input_bytes(const unsigned char* in, int size)
    {
      std::vector<unsigned char> tmp;
      for(int i=0;i<size;++i){
        tmp.push_back(in[i]);
      }
      return tmp;
    }

  } /* namespace lsa */
} /* namespace gr */

