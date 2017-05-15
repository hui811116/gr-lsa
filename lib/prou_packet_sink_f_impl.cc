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
#include "prou_packet_sink_f_impl.h"
#include <gnuradio/blocks/count_bits.h>

namespace gr {
  namespace lsa {

    static const unsigned int d_mask = 0x7ffffffe;
    static const int MAXPLD = 128-1;
    static const unsigned int CHIPSET[16] = {
      913698311,
      1734271091,
      1978533686,
      1591767910,
      3993450100,
      3765659486,
      121009647,
      1936154337,
      3381268985,
      2560696205,
      2316433608,
      2703199384,
      301517194,
      529307808,
      4173957649,
      2358812959
    };
    enum PKTSTATE{
      SEARCH,
      SYNC,
      PAYLOAD
    };

    inline unsigned char slice(const float& f)
    {
      return (f>0)? 0x01:0x00;
    }
    
    prou_packet_sink_f::sptr
    prou_packet_sink_f::make(int thres)
    {
      return gnuradio::get_initial_sptr
        (new prou_packet_sink_f_impl(thres));
    }

    /*
     * The private constructor
     */
    prou_packet_sink_f_impl::prou_packet_sink_f_impl(int thres)
      : gr::block("prou_packet_sink_f",
              gr::io_signature::make(1, 1, sizeof(float)),
              gr::io_signature::make(0, 0, 0))
    {
      set_threshold(thres);
      enter_search();
      d_pkt_out = pmt::mp("pkt_out");
      message_port_register_out(d_pkt_out);
    }

    void
    prou_packet_sink_f_impl::enter_search()
    {
      d_state = SEARCH;
      d_pre_cnt = 0;
      d_data_reg = 0x00000000;
      d_chip_cnt =0;
      d_byte_reg = 0x00;
    }
    void
    prou_packet_sink_f_impl::enter_sync()
    {
      d_state = SYNC;
      d_pre_cnt =0;
      d_chip_cnt =0;
      d_data_reg = 0x00000000;
      d_symbol_cnt = 0;
      d_pld_len =0;
      d_byte_reg = 0x00;
    }
    void
    prou_packet_sink_f_impl::enter_payload(const unsigned char& pld_len)
    {
      d_pld_len = pld_len;
      d_state = PAYLOAD;
      d_data_reg = 0x00000000;
      d_chip_cnt =0;
      d_symbol_cnt = 0;
      d_byte_reg = 0x00;
    }

    unsigned char
    prou_packet_sink_f_impl::chip_decoder(const unsigned int& c)
    {
      unsigned char d;
      int min_thres = 33;
      int thres;
      for(int i=0;i<16;++i){
        thres = gr::blocks::count_bits32( (c & d_mask)^(CHIPSET[i] & d_mask) );
        if(thres < min_thres){
        min_thres = thres;
        d = (unsigned char)i;
        }
      }
      if(min_thres < d_threshold){
        return d & 0x0f;
      }
      return 0xff;
    }
    /*
     * Our virtual destructor.
     */
    prou_packet_sink_f_impl::~prou_packet_sink_f_impl()
    {
    }

    void
    prou_packet_sink_f_impl::set_threshold(int thres)
    {
      if(thres<0){
        throw std::runtime_error("Threshold cannot be negative");
      }
      d_threshold = thres;
    }

    int
    prou_packet_sink_f_impl::threshold() const
    {
      return d_threshold;
    }

    void
    prou_packet_sink_f_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = noutput_items;
    }

    int
    prou_packet_sink_f_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const float *in = (const float *) input_items[0];
      int nin = ninput_items[0];
      int count = 0;
      while(count<nin){
        switch(d_state)
        {
          case SEARCH:
            while(count<nin){
              d_data_reg = (d_data_reg << 1) | (slice(in[count++])&0x01);
              if(d_pre_cnt>0){
                d_chip_cnt++;
              }
              if(d_pre_cnt ==0){
                int thres = gr::blocks::count_bits32( (d_data_reg & d_mask) ^ (CHIPSET[0] & d_mask) );
                if(thres < d_threshold){
                  d_pre_cnt++;
                }
              }
              else{
                if(d_chip_cnt == 32){
                  d_chip_cnt = 0;
                  if(d_byte_reg == 0)
                  {
                    if(gr::blocks::count_bits32((d_data_reg&d_mask)^(CHIPSET[0]&d_mask))<=d_threshold){
                      d_pre_cnt++;
                      d_byte_reg = 0x00;
                    }
                    else if(gr::blocks::count_bits32((d_data_reg&d_mask)^(CHIPSET[7]&d_mask)<=d_threshold )){
                      d_byte_reg = 0x70;
                    }
                    else{
                      enter_search();
                      break;
                    }
                  }
                  else{
                    if(gr::blocks::count_bits32((d_data_reg&d_mask)^(CHIPSET[10]&d_mask))<=d_threshold){
                      d_byte_reg |= 0x0A;
                      enter_sync();
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
          case SYNC:
            while(count<nin){
              d_data_reg = (d_data_reg<<1) | (slice(in[count++])&0x01 );
              d_chip_cnt++;
              if(d_chip_cnt==32){
                d_chip_cnt=0;
                unsigned char c = chip_decoder(d_data_reg);
                if(c == 0xff){
                  enter_search();
                  break;
                }
                else{
                  if(d_symbol_cnt == 0){
                    d_byte_reg = c<<4;
                    d_symbol_cnt++;
                  }
                  else{
                    d_byte_reg |= c;
                    if(d_byte_reg <= MAXPLD){
                      enter_payload(d_byte_reg);
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
          case PAYLOAD:
            while(count<nin){
              d_data_reg = (d_data_reg<<1 ) | (slice(in[count++])&0x01);
              d_chip_cnt++;
              if(d_chip_cnt==32){
                d_chip_cnt=0;
                unsigned char c = chip_decoder(d_data_reg);
                if(c==0xff){
                  enter_search();
                  break;
                }
                else{
                  if(d_symbol_cnt%2==0){
                    d_buf[d_symbol_cnt/2] = c<<4;
                  }
                  else{
                    d_buf[d_symbol_cnt/2] |= c;
                  }
                  d_symbol_cnt++;
                  if(d_symbol_cnt/2>=d_pld_len){
                    pmt::pmt_t blob = pmt::make_blob(d_buf,d_pld_len);
                    message_port_pub(d_pkt_out,pmt::cons(pmt::intern("ProU"),blob));
                    enter_search();
                    break;
                  }
                }
              }
            }
          break;
        }
      }


      consume_each (nin);
      return 0;
    }

  } /* namespace lsa */
} /* namespace gr */

