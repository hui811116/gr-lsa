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
#include "su_packet_sink_c_impl.h"
#include <gnuradio/blocks/count_bits.h>

namespace gr {
  namespace lsa {
static const unsigned int CHIPSET[16] = {
                                  3653456430,
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
                                  3378542520};
static const int MAX_PLD = 127;
static const int CODE_RATE_INV= 8;
static const unsigned int d_mask = 0x7ffffffe;

enum SYSTEMSTATE{
  SEARCH_ZERO,
  HAVE_SYNC,
  LOAD_PAYLOAD
};

    su_packet_sink_c::sptr
    su_packet_sink_c::make(
      const gr::digital::constellation_sptr& hdr_const,
      int threshold)
    {
      return gnuradio::get_initial_sptr
        (new su_packet_sink_c_impl(
          hdr_const,
          threshold));
    }

    /*
     * The private constructor
     */
    su_packet_sink_c_impl::su_packet_sink_c_impl(
      const gr::digital::constellation_sptr& hdr_const,
      int threshold)
      : gr::block("su_packet_sink_c",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(0, 0, 0)),
              d_msg_port(pmt::mp("msg")),
              d_cap(8192*2)
    {
      d_hdr_const = hdr_const->base();
      d_hdr_bps = hdr_const->bits_per_symbol();
      d_state = SEARCH_ZERO;
      enter_search();
      set_tag_propagation_policy(TPP_DONT);
      message_port_register_out(d_msg_port);
      // coded 
      d_threshold = (threshold<0)? 0: threshold;
      d_const_buf = new unsigned char[d_cap];
    }

    /*
     * Our virtual destructor.
     */
    su_packet_sink_c_impl::~su_packet_sink_c_impl()
    {
      delete [] d_const_buf;
    }

    void
    su_packet_sink_c_impl::enter_search()
    {
      d_state = SEARCH_ZERO;
      d_pre_cnt = 0;
      d_pkt_byte =0;
      d_chip_cnt= 0;
      d_data_reg = 0x00000000;
    }
    void
    su_packet_sink_c_impl::enter_have_sync()
    {
      d_state = HAVE_SYNC;
      d_pre_cnt = 0;
      d_data_reg = 0x00000000;
      d_symbol_cnt =0;
      d_chip_cnt =0;
      d_pkt_byte = 0;
    }
    void
    su_packet_sink_c_impl::enter_load_payload()
    {
      d_state = LOAD_PAYLOAD;
      //d_pky_byte contain the length of bytes to be collected.
      d_symbol_cnt=0;
      d_chip_cnt=0;
      d_data_reg = 0x00000000;
    }

    void
    su_packet_sink_c_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = noutput_items;
    }

    unsigned char
    su_packet_sink_c_impl::decode_chip(const unsigned int& reg)
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


    int
    su_packet_sink_c_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      int nin = std::min(ninput_items[0],d_cap/d_hdr_bps);
      for(int i=0;i<nin;++i){
        unsigned char temp =d_hdr_const->decision_maker(&in[i]);
          for(int j =0;j<d_hdr_bps;++j){
            // MSB endianess
            d_const_buf[i*d_hdr_bps+j] = (temp>> (d_hdr_bps-1-j)) & 0x01;
          }
      }
      int nbits = nin * d_hdr_bps;
      int count =0;
      while(count<nbits){
        switch(d_state)
        {
          case SEARCH_ZERO:
            while(count < nbits)
            {
              d_data_reg = (d_data_reg<<1) | (0x01 & d_const_buf[count++]);
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
                        //0xE
                        d_pkt_byte = 0xE0;
                      }
                      else{
                        enter_search();
                        break;
                      }
                    }
                    else{
                      if(gr::blocks::count_bits32( (d_data_reg&d_mask) ^ (CHIPSET[6]&d_mask))<=d_threshold){
                        //0x6
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
          while(count <nbits){
              d_data_reg = (d_data_reg<<1) | (0x01&d_const_buf[count++]);
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
                    // for special length settings
                    if(d_pkt_byte == 0){
                      // length 0 means NACK
                      enter_search();
                      break;
                    }
                    else if(d_pkt_byte <= MAX_PLD){
                      enter_load_payload();
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
          while(count < nbits){
              d_data_reg = (d_data_reg<<1) | (d_const_buf[count++]&0x01);
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
                    d_buf[d_symbol_cnt/2] = (c<<4);
                  }else{
                    d_buf[d_symbol_cnt/2] |= c;
                  }
                  d_symbol_cnt++;
                  if(d_symbol_cnt/2 >= d_pkt_byte){
                    pmt::pmt_t msg = pmt::cons(pmt::intern("LSA_hdr"),pmt::make_blob(d_buf,d_pkt_byte));
                    message_port_pub(d_msg_port,msg);
                    // reason: header may be intact
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
      consume_each (count);

      // Tell runtime system how many output items we produced.
      return 0;
    }

  } /* namespace lsa */
} /* namespace gr */

