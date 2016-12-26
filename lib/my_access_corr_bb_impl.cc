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
#include "my_access_corr_bb_impl.h"


namespace gr {
  namespace lsa {

    my_access_corr_bb::sptr
    my_access_corr_bb::make(const std::string& access_code, int threshold)
    {
      return gnuradio::get_initial_sptr
        (new my_access_corr_bb_impl(access_code, threshold));
    }

    /*
     * The private constructor
     */

    enum SyncType {
      CODE_SYNC,
      CODE_SEARCH
    };
    my_access_corr_bb_impl::my_access_corr_bb_impl(const std::string& access_code, unsigned int threshold)
      : gr::block("my_access_corr_bb",
              gr::io_signature::make(1, 1, sizeof(char)),
              gr::io_signature::make(0, 0, 0))
    {
      if(!set_access_code(access_code)) {
        throw std::runtime_error("my_access_corr: Setting access code failed");
      }
      
      set_threshold(threshold);
      set_tag_propagation_policy(TPP_DONT);

      d_out_port = pmt::mp("info");
      d_state = CODE_SEARCH;
      message_port_register_out(d_out_port);
    }

    /*
     * Our virtual destructor.
     */
    my_access_corr_bb_impl::~my_access_corr_bb_impl()
    {
    }

    bool
    my_access_corr_bb_impl::set_access_code(const std::string& access_code)
    {
      d_accesscode_len=access_code.length();
      if(d_accesscode_len >64){
        return false;
      }
      d_mask = ((~0ULL) >> (64- d_accesscode_len) );
      d_accesscode=0;
      for(unsigned i=0;i<d_accesscode_len;++i){
        d_accesscode = (d_accesscode << 1) | (access_code[i] & 1);
      }
      return true;
    }

    unsigned long long
    my_access_corr_bb_impl::access_code() const
    {
      return d_accesscode;
    }

    void
    my_access_corr_bb_impl::set_threshold(unsigned int threshold)
    {
      d_threshold=threshold;
    }

    unsigned int
    my_access_corr_bb_impl::get_threshold()const
    {
      return d_threshold;
    }


    void
    my_access_corr_bb_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      for(int i=0;i<ninput_items_required.size();++i){
          ninput_items_required[i]=noutput_items;
      }
    }

    size_t my_access_corr_bb_impl::header_nbits() const
    {
      return d_accesscode_len + 32 + 2*8 + 16;
    }

    // bits field :  |16 bits payload| 16bits payload| 8bits queue index| 8bits queue size| 16 bits counter|

    bool my_access_corr_bb_impl::payload_matched(int& payload_len)
    {
      uint16_t len0=0; 
      uint16_t len1=0;
      // order in d_input: MSB 
      for(int i=0;i<16;++i){
        len0 |= ( (d_input[i])? 0x0001 : 0x0000 )<< (15-i);
        len1 |= ( (d_input[i+16])? 0x0001 : 0x0000 ) << (15-i);
      }
      if(len0==len1){
        payload_len = static_cast<int>(len0);
        return true;
      }
      return false;
    }

    void my_access_corr_bb_impl::extract_header(std::vector<pmt::pmt_t> & info)
    {
      uint8_t q_index=0x00;
      uint8_t q_size=0x00;
      uint16_t counter=0x0000;
      for(int i=0;i<8;++i){
        q_index |= ((d_input[48+i])? 0x01 : 0x00) << 7-i;
        q_size  |= ((d_input[56+i])? 0x01 : 0x00) << 7-i;
      }
      for(int j=0;j<16;++j){
        counter |= ((d_input[32+j])? 0x0001 : 0x00) << 15-j;
      }
      d_info = pmt::dict_add(d_info, pmt::intern("queue_index"), pmt::from_long(q_index));
      
      d_info = pmt::dict_add(d_info, pmt::intern("queue_size"), pmt::from_long(q_size));
      
      d_info = pmt::dict_add(d_info, pmt::intern("packet_counter"), pmt::from_long(counter));
      info.push_back(d_info);
      
    }

    void
    my_access_corr_bb_impl::insert_bits (unsigned char in_b)
    {
      d_input.push_back( ((in_b & 0x01)!= 0)? true:false );
    }

    bool
    my_access_corr_bb_impl::parse_bits (int n_bits_in, const unsigned char* in, std::vector<pmt::pmt_t>& info, int& count)
    {
      count=0;
      while(count < n_bits_in){
        switch(d_state){
          case CODE_SEARCH:
            while(count < n_bits_in){
              d_data_reg = (d_data_reg << 1) | ((in[count++]) & 0x1);
              uint64_t check_bits =(~0ULL);
              check_bits = (d_data_reg ^ d_accesscode) & d_mask;
              if(check_bits == 0){
                d_state = CODE_SYNC;
                d_data_reg = 0;
                d_input.clear();
                break;
              }
            }
          break;
          case CODE_SYNC:
            while(count <= n_bits_in){
              insert_bits(in[count++]);
              if(d_input.size() == (header_nbits()- d_accesscode_len)){
                int payload_len=0; 
                if(payload_matched(payload_len)){
                  d_info = pmt::make_dict();
                  d_info = pmt::dict_add(d_info, pmt::intern("payload"), pmt::from_long(payload_len));
                  //info.push_back(d_info);
                  extract_header(info);
                  return true;
                }
                else{
                  return false;
                }
                d_state=CODE_SEARCH;
                break;
              }
            }
          break;
          default:
            throw std::runtime_error("my_access_corr: Parse bits enter wrong state!");
          break;
        }
      }
      return false;
    }

    int
    my_access_corr_bb_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      int count=0;
      const unsigned char *in = (const unsigned char *) input_items[0];
      //unsigned char* out = (unsigned char*) output_items[0];
      std::vector<pmt::pmt_t> info;
      if(parse_bits(noutput_items,in,info,count)){
        for(int i=0;i<info.size();++i){
          message_port_pub(d_out_port, info[i]);
        }
      }
      else{
        // Comment off the following line to avoid unwanted info
        //message_port_pub(d_out_port, pmt::PMT_F);
      }
      
      consume_each(noutput_items);
      return count;
    }

  } /* namespace lsa */
} /* namespace gr */

