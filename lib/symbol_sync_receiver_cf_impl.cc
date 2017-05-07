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
enum SYSTEMSTATE{
  SEARCH_ZERO,
  HAVE_SYNC,
  LOAD_PAYLOAD
};

    symbol_sync_receiver_cf::sptr
    symbol_sync_receiver_cf::make(
      const gr::digital::constellation_sptr& hdr_const,
      int threshold,
      bool diffcode,
      bool buf_verbose,
      bool debug)
    {
      return gnuradio::get_initial_sptr
        (new symbol_sync_receiver_cf_impl(
          hdr_const,
          threshold,
          diffcode,
          buf_verbose,
          debug));
    }

    /*
     * The private constructor
     */
    symbol_sync_receiver_cf_impl::symbol_sync_receiver_cf_impl(
      const gr::digital::constellation_sptr& hdr_const,
      int threshold,
      bool diffcode,
      bool buf_verbose,
      bool debug)
      : gr::block("symbol_sync_receiver_cf",
              gr::io_signature::make3(1, 3, sizeof(gr_complex),sizeof(float),sizeof(float)),
              gr::io_signature::make2(0, 2, sizeof(float),sizeof(float)))
    {

      d_hdr_const = hdr_const->base();
      d_hdr_map = hdr_const->pre_diff_code();
      d_hdr_bps = hdr_const->bits_per_symbol();

      d_time_offset_count=0;
      d_timetag = pmt::string_to_symbol("ctime");
      d_msg_port = pmt::mp("msg");

      d_debug = debug;
      d_buf_verbose = buf_verbose;
      d_diffcode = diffcode;
      //d_state = SEARCH_ZERO;
      d_mask = 0x7ffffffe;
      enter_search();

      set_tag_propagation_policy(TPP_DONT);
      message_port_register_out(d_msg_port);

      // coded 
      d_threshold = (threshold<0)? 0: threshold;

      set_max_noutput_items(8192/d_hdr_bps);
      d_current_time = std::clock();
    }

    /*
     * Our virtual destructor.
     */
    symbol_sync_receiver_cf_impl::~symbol_sync_receiver_cf_impl()
    {
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
    symbol_sync_receiver_cf_impl::msg_out(int sym_idx)
    {
      pmt::pmt_t msg = pmt::make_dict();
      if(d_buf_verbose){
        long int time=0;
        int offset=0;
        int tmp_idx=0;
        assert(!d_time_table.empty());
        //if(d_time_table.empty()){
          //throw std::runtime_error("no timing info in table!");
        //}
        for(int i=0;i<d_time_table.size();++i){
          if(d_time_table[i].first>sym_idx){
            break;
          }
          tmp_idx = d_time_table[i].first;
          time = d_time_table[i].second;
        }
        offset = d_time_offset_count+(sym_idx-tmp_idx);    
          // for debug checking
          //if(time == 0){
            //std::cout<<"symbol index:"<<sym_idx<<std::endl;
            //std::cout<<"info in table:"<<std::endl;
            //for(int i=0;i<d_time_table.size();++i){
              //std::cout<<"index:"<<d_time_table[i].first<<" ,time:"<<d_time_table[i].second<<std::endl;
            //}
            //throw std::runtime_error("timing found zero, (2)");
          //}

        msg = pmt::dict_add(msg, pmt::intern("ctime"),pmt::from_long(time));
        msg = pmt::dict_add(msg, pmt::intern("buffer_offset"),pmt::from_long(offset));
      }
        // for header info
        msg = pmt::dict_add(msg, pmt::intern("LSA_hdr"),pmt::PMT_T);
        msg = pmt::dict_add(msg, pmt::intern("queue_index"),pmt::from_long(d_qidx));
        msg = pmt::dict_add(msg, pmt::intern("queue_size"),pmt::from_long(d_qsize));
        msg = pmt::dict_add(msg, pmt::intern("payload"),pmt::from_long(d_pkt_byte));
        // for debug
        // differentiation for retransmission and fresh data
        if(d_qsize!=0){
          msg = pmt::dict_add(msg, pmt::intern("Type"),pmt::intern("Retransmission"));
        }
        else{
          msg = pmt::dict_add(msg, pmt::intern("Type"),pmt::intern("Fresh_data"));
        }
        msg = pmt::dict_add(msg, pmt::intern("counter"),pmt::from_long(d_qidx));
        //
      
      message_port_pub(d_msg_port, msg);
    }

    void
    symbol_sync_receiver_cf_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
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
      //std::cerr<<"in:"<<ninput_items[0] << " ,out:"<<noutput_items<<std::endl;
      const float* phase = NULL;
      const float* time = NULL;
      float *out_phase = NULL;
      float *out_time = NULL;
      
      int nfix = (noutput_items<ninput_items[0])? noutput_items : ninput_items[0];

      bool have_sync = (input_items.size()>=3);
      bool out_sync = (output_items.size()>=2);
      if(have_sync){
        phase = (const float*) input_items[1];
        time = (const float*) input_items[2];
        nfix = std::min(nfix,ninput_items[1]);
        nfix = std::min(nfix,ninput_items[2]);
        if(out_sync){
          out_phase = (float *) output_items[0];
          out_time = (float *) output_items[1];
          memcpy(out_phase,phase, sizeof(float)*nfix);
          memcpy(out_time,time,sizeof(float)*nfix);  
          // move to last line so that tagging act correctly.
          //produce(0,nfix);
          //produce(1,nfix);
        }
        consume(1,nfix);
        consume(2,nfix);
      }

      // Do <+signal processing+>
      d_time_table.clear();
      std::vector<tag_t> time_tag;
      std::vector<int> tag_offsets;
      get_tags_in_range(time_tag, 0, nitems_read(0),nitems_read(0)+nfix, d_timetag);
      int tmp_offset=0;
      d_time_table.push_back( std::make_pair(0,d_current_time));
      for(int i=0;i<time_tag.size();++i){
        tmp_offset = time_tag[i].offset-nitems_read(0);
        tag_offsets.push_back(tmp_offset);
        d_current_time = pmt::to_long(time_tag[i].value);
        d_time_offset_count = 0;
        //std::cout<<"current time:"<<d_current_time<<std::endl;
        if(tmp_offset==0){
          d_time_table[0] = std::make_pair(0,d_current_time);
        }
        else{
          d_time_table.push_back( std::make_pair(tmp_offset,d_current_time));
        }
      }
      

      //should add sensing tags
      if(d_diffcode)
      {
        for(int i=0;i<nfix;++i){
        unsigned char temp =d_hdr_map[ d_hdr_const->decision_maker(&in[i])];
        for(int j =0;j<d_hdr_bps;++j){
          // MSB endianess
          d_bytes_buf[i*d_hdr_bps+j] = (temp>> (d_hdr_bps-1-j)) & 0x01;
        }
      }
      }
      else{
        for(int i=0;i<nfix;++i){
        unsigned char temp =d_hdr_const->decision_maker(&in[i]);
        for(int j =0;j<d_hdr_bps;++j){
          // MSB endianess
          d_bytes_buf[i*d_hdr_bps+j] = (temp>> (d_hdr_bps-1-j)) & 0x01;
        }
      }
      }
      
      int nin = nfix * d_hdr_bps;
      int count=0;
      while(count < nin)
      {
        switch(d_state)
        {
          case SEARCH_ZERO:
            while(count < nin)
            {
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
                      else if(gr::blocks::count_bits32( (d_data_reg&d_mask) ^ (CHIPSET[7]&d_mask))<=d_threshold){
                        //0x7
                        d_pkt_byte = 7<<4;
                      }
                      else{
                        enter_search();
                        break;
                      }
                    }
                    else{
                      if(gr::blocks::count_bits32( (d_data_reg&d_mask) ^ (CHIPSET[10]&d_mask))<=d_threshold){
                        //0xa
                        d_pkt_byte = d_pkt_byte | 0xa;
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
                      // NOTE: in data channel, this case should not happen!
                      msg_out( (count-1)/d_hdr_bps );
                      enter_search();
                      break;
                    }
                    else if(d_pkt_byte ==1){
                      // length 1 means ACK
                      // NOTE: in data channel, this case should not happen!
                      // NOTE: in control channel, there still have queue information to be recieved!
                      d_pkt_byte =2;
                      enter_load_payload();
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
          while(count < nin){
              d_data_reg = (d_data_reg<<1) | (d_bytes_buf[count++]&0x01);
              d_chip_cnt++;
              if(d_chip_cnt==32){
                d_chip_cnt=0;
                unsigned char c = decode_chip(d_data_reg);
                if(c==0xff){
                  enter_search();
                  break;
                }
                else{
                  if(d_symbol_cnt<4){
                    if(d_symbol_cnt==0){
                      d_qidx = c<<4;
                    }
                    else if(d_symbol_cnt==1){
                      d_qidx |= c;
                    }
                    else if(d_symbol_cnt==2){
                      d_qsize = c<<4;
                    }
                    else{
                      d_qsize |= c;
                      // all info are ready
                      // feedback to front end;
                      // first try:
                      //msg_out(noutput_items,true);
                    }
                  }
                  
                  d_symbol_cnt++;
                  if(d_symbol_cnt/2 >= d_pkt_byte){
                    // output header
                    // second try
                    msg_out( (count-1) /d_hdr_bps);
                    if(out_sync){
                      int index= (count-1)/d_hdr_bps;
                      add_item_tag(0,nitems_written(0)+index,pmt::intern("LSA_hdr"),pmt::PMT_T);
                      add_item_tag(0,nitems_written(0)+index,pmt::intern("queue_index"),pmt::from_long(d_qidx));
                      add_item_tag(0,nitems_written(0)+index,pmt::intern("queue_size"),pmt::from_long(d_qsize));
                      //add_item_tag(0,nitems_written(0)+i,pmt::intern("counter"),pmt::from_long(d_counter));
                      add_item_tag(0,nitems_written(0)+index,pmt::intern("pld_bytes"),pmt::from_long(d_pkt_byte));
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

      for(int i=0;i<time_tag.size();++i){
        pmt::pmt_t dict=pmt::make_dict();
        dict = pmt::dict_add(dict,pmt::intern("ctime"),time_tag[i].value);
        dict = pmt::dict_add(dict,pmt::intern("buffer_offset"),pmt::from_long(0));
        message_port_pub(d_msg_port,dict);
        // IMPORTANT: add timing tag in sync mode
        if(d_buf_verbose && have_sync && out_sync){
          add_item_tag(0,nitems_written(0)+tag_offsets[i],pmt::intern("ctime"),time_tag[i].value);
        }
      }
      d_time_offset_count+=(nfix-tmp_offset);  
      if(have_sync && out_sync){
        produce(0,nfix);
        produce(1,nfix);
      }
      
      consume(0,nfix);
      // Tell runtime system how many output items we produced.
      return WORK_CALLED_PRODUCE;
    }

  } /* namespace lsa */
} /* namespace gr */

