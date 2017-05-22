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
#include "packet_sink_impl.h"
#include <gnuradio/blocks/count_bits.h>

namespace gr {
  namespace lsa {

static const unsigned int CHIPSET[16] = {
  3765939820,
      3456596710,
      1826650030,
      1724778362,
      778887287,
      2061946375,
      4155403488,
      2272978638,
      2676511123,
      2985854233,
      320833617,
      422705285,
      1368596360,
      85537272,
      2287047455,
      4169472305
                                  /*3653456430,
                                  3986437410,
                                  786023250,
                                  585997365,
                                  1378802115,
                                  891481500,
                                  3276943065,
                                  2620728045,
                                  2358642555,
                                  3100205175,
                                  2072811015,
                                  2008598880,
                                  125537430,
                                  1618458825,
                                  2517072780,
                                  3378542520*/
                                  };
static const int MAX_PLD = 127;
enum SYSTEMSTATE{
  SEARCH_ZERO,
  HAVE_SYNC,
  LOAD_PAYLOAD
};
inline unsigned char slice(const float& f)
    {
      return (f>0)? 0x01:0x00;
    }

    packet_sink::sptr
    packet_sink::make(int threshold)
    {
      return gnuradio::get_initial_sptr
        (new packet_sink_impl(threshold));
    }

    /*
     * The private constructor
     */
    packet_sink_impl::packet_sink_impl(int threshold)
      : gr::block("packet_sink",
              gr::io_signature::make(1, 1, sizeof(float)),
              gr::io_signature::make(0, 0, 0)),
              d_threshold(threshold)
    {
      if(threshold<0){
        throw std::invalid_argument("Threshold cannot be negative");
      }
      d_pld_out = pmt::mp("pkt_out");
      message_port_register_out(d_pld_out);
      d_mask = 0x7ffffffe;
      enter_search();
    }

    /*
     * Our virtual destructor.
     */
    packet_sink_impl::~packet_sink_impl()
    {
    }

    unsigned char
    packet_sink_impl::decode_chip(const unsigned int& reg)
    {
      int min_thres = 33;
      int thres;
      unsigned char min_idx;
      for(int i=0;i<16;++i){
        thres = gr::blocks::count_bits32( (reg & d_mask)^(CHIPSET[i]&d_mask));
        if(thres < min_thres){
          min_idx  = (unsigned char)i;
          min_thres = thres;
        }
      }
      if(min_thres < d_threshold){
        return min_idx & 0x0f;
      }
      return 0xff;
    }

    void
    packet_sink_impl::enter_search()
    {
      d_state = SEARCH_ZERO;
      d_pre_cnt = 0;
      d_pkt_byte =0;
      d_chip_cnt= 0;
      d_data_reg = 0x00000000;
    }

    void
    packet_sink_impl::enter_have_sync()
    {
      d_state = HAVE_SYNC;
      d_pre_cnt = 0;
      d_data_reg = 0x00000000;
      d_symbol_cnt =0;
      d_chip_cnt =0;
      d_pkt_byte = 0;
      d_pkt_pld=0;
    }

    void
    packet_sink_impl::enter_load_payload()
    {
      d_state = LOAD_PAYLOAD;
      //d_pky_byte contain the length of bytes to be collected.
      d_symbol_cnt=0;
      d_chip_cnt=0;
      d_data_reg = 0x00000000;
    }

    void
    packet_sink_impl::msg_out()
    {
      pmt::pmt_t dict = pmt::make_dict();
      dict= pmt::dict_add(dict,pmt::intern("LSA_hdr"),pmt::PMT_T);
      dict= pmt::dict_add(dict,pmt::intern("queue_index"),pmt::from_long(d_qidx));
      dict= pmt::dict_add(dict,pmt::intern("queue_size"),pmt::from_long(d_qsize));
      dict= pmt::dict_add(dict,pmt::intern("payload"),pmt::from_long(d_pkt_pld));
      message_port_pub(d_pld_out,dict);
      //pmt::pmt_t blob = pmt::make_blob(d_buf,d_pkt_byte);
      //message_port_pub(d_pld_out,pmt::cons(pmt::PMT_NIL,blob));
    }

    void
    packet_sink_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = noutput_items;
    }

    int
    packet_sink_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const float *in = (const float *) input_items[0];
      int nin = ninput_items[0];
      int count = 0;
      while(count < nin){
        switch(d_state){
          case SEARCH_ZERO:
            while(count < nin){
              d_data_reg = (d_data_reg << 1) | (0x01 & slice(in[count++]));
              if(d_pre_cnt >0){
                d_chip_cnt++;
              }
              if(d_pre_cnt ==0 ){
                int thres = gr::blocks::count_bits32( (d_data_reg&d_mask)^ (d_mask&CHIPSET[0]) );
                if(thres < d_threshold){
                  // found a zero;
                  d_pre_cnt++;
                }
              }
              else{
                if(d_chip_cnt==32){
                  d_chip_cnt=0;
                    if(d_pkt_byte == 0){
                      //find sfd (LSB)0x7a(MSB)
                      // zigbee standard process LSB first
                      if(gr::blocks::count_bits32( (d_data_reg&d_mask) ^ (CHIPSET[0]&d_mask))<=d_threshold){
                        d_pre_cnt++;
                        d_pkt_byte = 0;
                      }
                      else if(gr::blocks::count_bits32( (d_data_reg&d_mask) ^ (CHIPSET[14]&d_mask))<=d_threshold){
                        //0xe
                        d_pkt_byte = 0xE0;
                      }
                      else{
                        enter_search();
                        break;
                      }
                    }
                    else{
                      if(gr::blocks::count_bits32( (d_data_reg&d_mask) ^ (CHIPSET[6]&d_mask))<=d_threshold){
                        //0xa
                        d_pkt_byte = d_pkt_byte | 0x6;
                        enter_have_sync();
                        break;
                      }
                      else{
                        //not correct sfd
                        enter_search();
                        break;
                      }
                    }
                }
              }
            }
          break;
          case HAVE_SYNC:
            while(count <nin){
              d_data_reg = (d_data_reg<<1) | (0x01&slice(in[count++]) );
              d_chip_cnt++;
              if(d_chip_cnt==32){
                d_chip_cnt=0;
                unsigned char c = decode_chip(d_data_reg);
              if(c==0xff){
                enter_search();
                break;
              }
              else{
                //check header
                  if(d_symbol_cnt==0){
                    // first symbol of header
                    d_pkt_byte = (c<<4);
                    d_symbol_cnt++;
                  }
                  else{
                    d_pkt_byte |= c;
                    if(d_pkt_byte == 0){
                      //NACK
                      pmt::pmt_t dict = pmt::make_dict();
                      dict = pmt::dict_add(dict,pmt::intern("LSA_hdr"),pmt::PMT_T);
                      //dict = pmt::dict_add(dict,pmt::intern("queue_index"),pmt::from_long(0));
                      //dict = pmt::dict_add(dict,pmt::intern("queue_size"),pmt::from_long(0));
                      dict = pmt::dict_add(dict,pmt::intern("payload"),pmt::from_long(0));
                      message_port_pub(d_pld_out,dict);
                      enter_search();
                      break;
                    }
                    else if(d_pkt_byte <= MAX_PLD){
                      //LSA special flags
                      //if(d_pkt_byte==1){
                        //ACK
                        //d_pkt_pld = 1;
                        //d_pkt_byte =2;
                        //enter_load_payload();
                      //}
                      //else{
                        d_pkt_pld = d_pkt_byte;
                        enter_load_payload();
                      //}
                      break;
                    }
                    else{
                      enter_search();
                      break;
                    }
                  }
                }
              }
            }
          break;
          case LOAD_PAYLOAD:
            while(count < nin){
              d_data_reg = (d_data_reg<<1) | (slice(in[count++]) &0x01);
              d_chip_cnt++;
              if(d_chip_cnt==32){
                d_chip_cnt=0;
                unsigned char c = decode_chip(d_data_reg);
                if(c==0xff){
                  enter_search();
                  break;
                }
                else{
                  if(d_symbol_cnt%2==0){
                    // first symbol of a byte
                    d_buf[d_symbol_cnt/2] = (c<<4);
                  }
                  else{
                    // second symbol of a byte
                    d_buf[d_symbol_cnt/2] |= c;
                  }
                  d_symbol_cnt++;
                  if(d_symbol_cnt/2 >= d_pkt_byte){
                    // payload length collected
                    // special cases output from here,
                    //if(d_pkt_pld == 1){
                      // ACK: queue size contain RETX or Fresh info
                      //d_qidx = d_buf[0];
                      //d_qsize = d_buf[1];
                    //}
                    //else if(d_pkt_pld==2){
                      // sensing:
                      //d_qidx = 0xff;
                      //d_qsize = 0x00;
                    //}
                    //else{
                      //d_qidx = d_buf[0];
                      //d_qsize = d_buf[1];
                    //}
                    //msg_out();
                    // current version do not count the BER of SU
                    pmt::pmt_t blob = pmt::make_blob(d_buf,d_pkt_byte);
                    message_port_pub(d_pld_out,pmt::cons(pmt::intern("LSA_hdr"),blob));
                    enter_search();
                    break;
                  }
                }
              }
            }
          break;
          default:
            throw std::runtime_error("Entering undefined state");
          break;
        }
      }
      
      consume_each (nin);
      return 0;
    }

  } /* namespace lsa */
} /* namespace gr */

