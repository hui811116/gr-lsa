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
#include "symbol_sync_receiver_cf_impl.h"
#include <gnuradio/math.h>
#include <gnuradio/expj.h>
#include <ctime>
#include <pmt/pmt.h>
#include <gnuradio/blocks/count_bits.h>

namespace gr {
  namespace lsa {

#define d_debug true
#define DEBUG d_debug && std::cout
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
static const pmt::pmt_t d_voe_tag = pmt::intern("voe_tag");

enum SYSTEMSTATE{
  SEARCH_ZERO,
  HAVE_SYNC,
  LOAD_PAYLOAD
};

    symbol_sync_receiver_cf::sptr
    symbol_sync_receiver_cf::make(
      const gr::digital::constellation_sptr& hdr_const,
      int threshold)
    {
      return gnuradio::get_initial_sptr
        (new symbol_sync_receiver_cf_impl(
          hdr_const,
          threshold));
    }

    /*
     * The private constructor
     */
    symbol_sync_receiver_cf_impl::symbol_sync_receiver_cf_impl(
      const gr::digital::constellation_sptr& hdr_const,
      int threshold)
      : gr::block("symbol_sync_receiver_cf",
              gr::io_signature::make3(1, 3, sizeof(gr_complex),sizeof(float),sizeof(float)),
              gr::io_signature::make2(0, 2, sizeof(float),sizeof(float))),
              d_msg_port(pmt::mp("msg")),
              d_cap(8192*2)
    {
      d_hdr_const = hdr_const->base();
      d_hdr_bps = hdr_const->bits_per_symbol();
      d_bytes_buf = new unsigned char[d_cap];
      d_state = SEARCH_ZERO;
      enter_search();
      set_tag_propagation_policy(TPP_DONT);
      message_port_register_out(d_msg_port);
      // coded 
      d_threshold = (threshold<0)? 0: threshold;
      d_current_time =0;
      d_voe_state = false;
      d_voe_do_not_pub =false;
    }

    /*
     * Our virtual destructor.
     */
    symbol_sync_receiver_cf_impl::~symbol_sync_receiver_cf_impl()
    {
      delete[] d_bytes_buf;
    }

    // coded modules
    void 
    symbol_sync_receiver_cf_impl::enter_search()
    {
      d_state = SEARCH_ZERO;
      d_pre_cnt = 0;
      d_pkt_byte =0;
      d_chip_cnt= 0;
      d_data_reg = 0x00000000;
    }

    void
    symbol_sync_receiver_cf_impl::enter_have_sync()
    {
      d_state = HAVE_SYNC;
      d_pre_cnt = 0;
      d_data_reg = 0x00000000;
      d_symbol_cnt =0;
      d_chip_cnt =0;
      d_pkt_byte = 0;
    }

    void
    symbol_sync_receiver_cf_impl::enter_load_payload()
    {
      d_state = LOAD_PAYLOAD;
      //d_pky_byte contain the length of bytes to be collected.
      d_symbol_cnt=0;
      d_chip_cnt=0;
      d_data_reg = 0x00000000;

    }

    unsigned char
    symbol_sync_receiver_cf_impl::decode_chip(const unsigned int& reg)
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
    symbol_sync_receiver_cf_impl::msg_out()
    {
      pmt::pmt_t msg=pmt::PMT_NIL; 
      pmt::pmt_t key = pmt::PMT_F;
      if(d_base>=0 && !d_voe_do_not_pub){
        key = pmt::PMT_T;
      }
      msg = pmt::cons(key,pmt::make_blob(d_out_buf,d_pkt_byte));
      // postfixing header information
      uint16_t base1,base2;
      d_qidx = d_out_buf[0]<<8;
      d_qidx|= d_out_buf[1];
      d_qsize= d_out_buf[2]<<8;
      d_qsize|=d_out_buf[3];
      base1 = d_out_buf[4]<<8;
      base1|= d_out_buf[5];
      base2 = d_out_buf[6]<<8;
      base2|= d_out_buf[7];
      if(base1==base2){
        d_base = base1;
      }else{
        d_base = -1;
      }
      message_port_pub(d_msg_port, msg);
    }
    void
    symbol_sync_receiver_cf_impl::update_voe_state(int idx)
    {
      if(!d_tags.empty()){
        if(idx==d_tags[0].offset){
          d_voe_state = pmt::to_bool(d_tags[0].value);
          if(d_state == SEARCH_ZERO){
            d_voe_do_not_pub = false;
          }else{
            d_voe_do_not_pub = true;
          }
          d_tags.erase(d_tags.begin());
        }
      }
    }

    void
    symbol_sync_receiver_cf_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      for(int i=0;i<ninput_items_required.size();++i){
        ninput_items_required[i] = noutput_items;
      }
    }

    int
    symbol_sync_receiver_cf_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      const float* phase = NULL;
      const float* freq = NULL;
      float *out_phase = NULL;
      float *out_freq = NULL;
      int nfix = (noutput_items<ninput_items[0])? noutput_items : ninput_items[0];
      nfix = std::min(nfix,(int)d_cap/d_hdr_bps);
      d_tags.clear();
      get_tags_in_window(d_tags,0,0,nfix,d_voe_tag);
      bool have_sync = (input_items.size()>=3);
      bool out_sync = (output_items.size()>=2);
      if(have_sync){
        phase = (const float*) input_items[1];
        freq = (const float*) input_items[2];
        nfix = std::min(nfix,ninput_items[1]);
        nfix = std::min(nfix,ninput_items[2]);
        if(out_sync){
          out_phase = (float *) output_items[0];
          out_freq = (float *) output_items[1];
          memcpy(out_phase,phase, sizeof(float)*nfix);
          memcpy(out_freq,freq,sizeof(float)*nfix);  
          // move to last line so that tagging act correctly.
        }
        //consume should also be moved to end
      }
      // convert indexes to bits based
      for(int i=0;i<d_tags.size();++i){
        d_tags[i].offset = (d_tags[i].offset-nitems_read(0))*d_hdr_bps;
      }
      for(int i=0;i<nfix;++i){
        unsigned char temp =d_hdr_const->decision_maker(&in[i]);
        for(int j =0;j<d_hdr_bps;++j){
          // MSB endianess
          d_bytes_buf[i*d_hdr_bps+j] = (temp>> (d_hdr_bps-1-j)) & 0x01;
        }
      }
      int nin = nfix * d_hdr_bps;
      int count=0;
      while(count < nin)
      {
        switch(d_state)
        {
          case SEARCH_ZERO:
            while(count < nin){
              update_voe_state(count);
              d_data_reg = (d_data_reg<<1) | (0x01 & d_bytes_buf[count++]);
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
          while(count <nin){
            update_voe_state(count);
            d_data_reg = (d_data_reg<<1) | (0x01&d_bytes_buf[count++]);
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
                    d_pkt_pld = 0;
                    enter_search();
                    break;
                  }else if(d_pkt_byte <= MAX_PLD){
                    d_pkt_pld = d_pkt_byte;
                    enter_load_payload();
                    break;
                  }else{
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
            update_voe_state(count);
            d_data_reg = (d_data_reg<<1) | (d_bytes_buf[count++]&0x01);
            d_chip_cnt++;
              if(d_chip_cnt==32){
                d_chip_cnt=0;
                unsigned char c = decode_chip(d_data_reg);
                if(c==0xff){
                  enter_search();
                  break;
                }else{
                  if(d_symbol_cnt%2==0){
                    d_out_buf[d_symbol_cnt/2] = (c<<4);
                  }else{
                    d_out_buf[d_symbol_cnt/2] |= c;
                  }
                  d_symbol_cnt++;
                  if(d_symbol_cnt/2 >= d_pkt_byte){
                    msg_out();
                    if(out_sync){
                      int index= (count+1)/d_hdr_bps;
                      if(d_base>=0 && !d_voe_do_not_pub){
                        add_item_tag(0,nitems_written(0)+index,pmt::intern("queue_index"),pmt::from_long(d_qidx));
                        add_item_tag(0,nitems_written(0)+index,pmt::intern("queue_size"),pmt::from_long(d_qsize));
                        add_item_tag(0,nitems_written(0)+index,pmt::intern("base"),pmt::from_long(d_base));
                        add_item_tag(0,nitems_written(0)+index,pmt::intern("pld_bytes"),pmt::from_long(d_pkt_byte));
                        add_item_tag(0,nitems_written(0)+index,pmt::intern("payload"),pmt::from_long(d_pkt_byte*8*CODE_RATE_INV/d_hdr_bps));  
                        add_item_tag(0,nitems_written(0)+index,pmt::intern("LSA_hdr"),pmt::PMT_T);
                      }
                    }
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
      std::vector<tag_t> tags;
      get_tags_in_window(tags,0,0,nfix);
      if(have_sync){
        if(out_sync){
          for(int i=0;i<tags.size();++i){
            long int offset = tags[i].offset - nitems_read(0);
            add_item_tag(0,nitems_written(0)+offset,tags[i].key,tags[i].value);
          }
          produce(0,nfix);
          produce(1,nfix);
        }
        consume(1,nfix);
        consume(2,nfix);
      }else{
        produce(0,0);
        produce(1,0);
        consume(1,0);
        consume(2,0);
      }
      consume(0,nfix);
      // Tell runtime system how many output items we produced.
      return WORK_CALLED_PRODUCE;
    }

  } /* namespace lsa */
} /* namespace gr */

