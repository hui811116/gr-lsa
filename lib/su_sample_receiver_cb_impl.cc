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
#include "su_sample_receiver_cb_impl.h"
#include <gnuradio/blocks/pdu.h>
#include <sstream>

namespace gr {
  namespace lsa {

    su_sample_receiver_cb::sptr
    su_sample_receiver_cb::make(
      const std::string& sensing_tag_id,
      const std::string& accesscode,
      const gr::digital::constellation_sptr& hdr_const,
      const gr::digital::constellation_sptr& pld_const,
      bool debug)
    {
      return gnuradio::get_initial_sptr
        (new su_sample_receiver_cb_impl(sensing_tag_id,accesscode,hdr_const,pld_const,debug));
    }

    enum SuRxState{
      SU_ONLY,
      INTERFERING
    };

    enum SyncState{
      SEARCH_SYNC_CODE,
      SYNC_WAIT_HEADER,
      WAIT_PAYLOAD
    };
    /*
     * The private constructor
     */
    su_sample_receiver_cb_impl::su_sample_receiver_cb_impl(
      const std::string& sensing_tag_id,
      const std::string& accesscode, 
      const gr::digital::constellation_sptr& hdr_const,
      const gr::digital::constellation_sptr& pld_const,
      bool debug)
      : gr::block("su_sample_receiver_cb",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(0, 0, 0)),
        d_src_id(pmt::intern(alias()))
    {
      d_hdr_sptr = hdr_const->base();
      d_pld_sptr = pld_const->base();
      d_hdr_bps = hdr_const->bits_per_symbol();
      d_pld_bps = pld_const->bits_per_symbol();

      d_state = SU_ONLY;
      d_bit_state = SEARCH_SYNC_CODE;

      d_msg_port = pmt::mp("info");
      d_pkt_port = pmt::mp("packet");
      d_debug_port = pmt::mp("debug");  //debug
      d_sensing_tag_id = pmt::string_to_symbol(sensing_tag_id);

      d_cap = 1024*64;
      d_byte_count = 0;
  
      set_max_noutput_items(d_cap);
      if(!set_accesscode(accesscode)){
        throw std::runtime_error("SU Receiver: Setting access code failed");
      }
      d_byte_reg = new unsigned char[d_cap];
      d_symbol_to_bytes = new unsigned char[d_cap];
      d_debug = debug;

      message_port_register_out(d_msg_port);
      message_port_register_out(d_pkt_port);
      message_port_register_out(d_debug_port); //debug
      set_tag_propagation_policy(TPP_DONT);
    }

    /*
     * Our virtual destructor.
     */
    su_sample_receiver_cb_impl::~su_sample_receiver_cb_impl()
    {
      delete [] d_byte_reg;
      delete [] d_symbol_to_bytes;
    }

    void
    su_sample_receiver_cb_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0]= noutput_items; 
    }

    void
    su_sample_receiver_cb_impl::pub_byte_pkt()
    {  
        int bits_count = d_pld_bps * d_byte_count;
        int byte_count = bits_count/8;
        unsigned char tmp;   
        
        for(int i=0;i<bits_count;++i){
          if(i%8==0){
            d_symbol_to_bytes[i/8]=0x00;
          }
          tmp = (d_byte_reg[i/d_pld_bps] >> (d_pld_bps-1-i%d_pld_bps)) & 0x01;
          d_symbol_to_bytes[i/8] =  ((d_symbol_to_bytes[i/8] >> (7-i%8)) | tmp ) << 7-i%8;
        }

        pmt::pmt_t pdu_meta = pmt::make_dict();
        pdu_meta = pmt::dict_add(pdu_meta, pmt::intern("payload_len"), pmt::from_long(d_payload_len));
        pdu_meta = pmt::dict_add(pdu_meta, pmt::intern("counter"), pmt::from_long(d_counter));
        
        d_pdu_vector = gr::blocks::pdu::make_pdu_vector(gr::blocks::pdu::byte_t,d_symbol_to_bytes,byte_count);
        pmt::pmt_t msg = pmt::cons(pdu_meta, d_pdu_vector);
        message_port_pub(d_pkt_port, msg);
    }


    void
    su_sample_receiver_cb_impl::feedback_info(bool type)
    {
      pmt::pmt_t sen_back = pmt::make_dict();
      //interfering case
      sen_back = pmt::dict_add(sen_back,d_sensing_tag_id,pmt::from_bool(type));
      if(!type)
      {
        //su pkt received
        sen_back = pmt::dict_add(sen_back,pmt::intern("payload"),pmt::from_long(d_payload_len));
        sen_back = pmt::dict_add(sen_back,pmt::intern("queue_index"),pmt::from_long(d_qidx));
        sen_back = pmt::dict_add(sen_back,pmt::intern("queue_size"),pmt::from_long(d_qsize));
        sen_back = pmt::dict_add(sen_back,pmt::intern("counter"),pmt::from_long(d_counter));
      }
      message_port_pub(d_msg_port,sen_back);
    }

    void
    su_sample_receiver_cb_impl::data_reg_reset()
    {
      d_byte_count=0;
    }

    size_t
    su_sample_receiver_cb_impl::header_nbits() const
    {
      return d_accesscode_len + 32+16+8+8;
    }

    bool
    su_sample_receiver_cb_impl::set_accesscode(const std::string& accesscode)
    {
      d_accesscode_len = accesscode.length();
      if(d_accesscode_len >64){
        return false;
      }
      d_mask = ((~0ULL) >> (64- d_accesscode_len) );
      d_accesscode=0;
      for(unsigned i=0;i<d_accesscode_len;++i){
        d_accesscode = (d_accesscode << 1) | (accesscode[i] & 1);
      }
      return true;
    }

    uint64_t
    su_sample_receiver_cb_impl::accesscode() const
    {
      return d_accesscode;
    }
    //helper function for header parsing
    uint16_t
    su_sample_receiver_cb_impl::_get_bit16(int begin_idx)
    {
      unsigned long tmp=0UL;
      for(int i=0;i<16;++i){
        tmp |= ((d_input[begin_idx+i])? 1 : 0) << (15-i);  // NOTE: this is how bits arrangement should be like
      }
      return tmp;
    }
    uint8_t
    su_sample_receiver_cb_impl::_get_bit8(int begin_idx)
    {
      unsigned char tmp=0;
      for(int i=0;i<8;++i){
        tmp |= ((d_input[begin_idx+i])? 1:0 ) << (7-i);
      }
      return tmp;
    }

    bool
    su_sample_receiver_cb_impl::parse_header()
    {
      uint16_t len0,len1;    
      len0 = _get_bit16(0);
      len1 = _get_bit16(16);
      if(len0 == len1){
        d_payload_len=len0;
        d_counter = _get_bit16(32);
        d_qidx = _get_bit8(48);
        d_qsize = _get_bit8(56);
        return true;
      }
      return false;
    }

    bool
    su_sample_receiver_cb_impl::insert_parse_byte(const gr_complex& sample)
    {
      unsigned char byte_hold; 
      if(d_byte_count == d_cap){
        d_byte_count =0;
        GR_LOG_CRIT(d_logger, "SU Receiver: Reaching maximum capacity, reset to initial.");
      }
      switch(d_bit_state)
      {
        case SEARCH_SYNC_CODE:
          byte_hold = d_hdr_sptr->decision_maker(&sample);
          for(int i=0;i<d_hdr_bps;++i)
          {
            uint64_t check_bits = (~0ULL);
            d_data_reg = (d_data_reg << 1) | ((byte_hold >> (d_hdr_bps-1-i) )& 0x01 );
            check_bits = (d_data_reg ^ d_accesscode) & d_mask;
            if(check_bits == 0){
              d_bit_state = SYNC_WAIT_HEADER;
              d_input.clear();
            }
          } 
        break;
        case SYNC_WAIT_HEADER:
          byte_hold = d_hdr_sptr->decision_maker(&sample);
          for(int i=0;i<d_hdr_bps;++i){
            d_input.push_back( (((byte_hold >> (d_hdr_bps-1-i)) & 0x01)==0x00 )? false : true);
          }
          if(d_input.size() == (header_nbits()-d_accesscode_len) )
          {
            if(parse_header()){
              d_bit_state = WAIT_PAYLOAD;
            }
            else{
              d_bit_state = SEARCH_SYNC_CODE;
            }
            d_byte_count=0;
            if(d_debug){
              std::stringstream ss;
              ss<<"SU_RX_header= "<<d_payload_len;
              GR_LOG_DEBUG(d_logger, ss.str());
            }
          }
        break;
        case WAIT_PAYLOAD:
          d_byte_reg[d_byte_count++] = d_pld_sptr->decision_maker(&sample);
          if(d_payload_len*8 == (d_byte_count * d_pld_sptr->bits_per_symbol()))
          {
            pub_byte_pkt();
            d_bit_state = SEARCH_SYNC_CODE;
            return true;
          }
        break;
        default:
          std::runtime_error("SU Receiver: Entering wrong bit processing state");
        break;
      }
      //MSB parsing
      return false;
    }

    void
    su_sample_receiver_cb_impl::check_tags(std::vector<tag_t>& out_tags, const std::vector<tag_t>& in_tags)
    {
      bool sensing;
      for(int i=0;i<in_tags.size();++i){
        if(in_tags[i].key == d_sensing_tag_id){
          sensing = pmt::to_bool(in_tags[i].value);
            out_tags.push_back(in_tags[i]);            
        }
      }
    }
    int
    su_sample_receiver_cb_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      noutput_items = ninput_items[0];
      const gr_complex *in = (const gr_complex *) input_items[0];
      std::vector<tag_t> intf_idx;
      std::vector<tag_t> tags;
      get_tags_in_range(tags,0,nitems_read(0),nitems_read(0)+noutput_items);
      check_tags(intf_idx,tags);
      int sense_offset;
      bool sense_state;
      int count =0;
      tag_t check_tag;
      pmt::pmt_t debug = pmt::make_dict();

      while(count < noutput_items)
      {
        if(!intf_idx.empty()){
          check_tag = intf_idx.front();
          sense_offset = check_tag.offset-nitems_read(0);
          sense_state = to_bool(check_tag.value);
          intf_idx.erase(intf_idx.begin());
        }
        if(count == sense_offset){
          d_state = (sense_state)? INTERFERING: SU_ONLY;
          if(sense_state){
            feedback_info(true);
            if(d_debug){
              GR_LOG_DEBUG(d_logger,"Interfering signal detected");
            }
            data_reg_reset();
          }
        }
        switch(d_state)
        {
          case SU_ONLY:
            if(insert_parse_byte(in[count])){
              feedback_info(false);
              data_reg_reset();
            }
          break;
          case INTERFERING:
            //do nothing?
          break;
          default:
            std::runtime_error("SU RX:entering wrong state");
          break;
        }
        count++;
      }
      
      consume_each (noutput_items);
      return noutput_items;
    }

  } /* namespace lsa */
} /* namespace gr */

