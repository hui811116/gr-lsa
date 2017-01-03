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
#include "prou_sample_receiver_cb_impl.h"

namespace gr {
  namespace lsa {

    prou_sample_receiver_cb::sptr
    prou_sample_receiver_cb::make(
      const gr::digital::constellation_sptr& su_hdr_const,
      int su_pld_bps,
      int pu_nfilts,
      int su_nfilts,
      bool mode)
    {
      return gnuradio::get_initial_sptr
        (new prou_sample_receiver_cb_impl(
          su_hdr_const,
          su_pld_bps,
          pu_nfilts,
          su_nfilts,
          mode));
    }

    enum proURxMode{
      STANDARD,
      INTERFERENCE_CANCELLATION
    };
    enum suSyncState{
      SEARCH_ACCESSCODE,
      HEADER_WAIT,
      PAYLOAD_WAIT
    };
    enum intfState{
      CLEAR,
      RETRANSMISSION
    };
    enum intfSyncState{
      INTF_SEARCH,
      INTF_HEADER,
      INTF_PAYLOAD_WAIT
    };
    /*
     * The private constructor
     */
    prou_sample_receiver_cb_impl::prou_sample_receiver_cb_impl(
      const gr::digital::constellation_sptr& su_hdr_const,
      int su_pld_bps,
      int pu_nfilts,
      int su_nfilts,
      bool mode)
      : gr::block("prou_sample_receiver_cb",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(gr_complex)))
    {
      //TODO: testing configuration, reivise for actual implementation
      d_pu_filters = std::vector<gr::filter::kernel::fir_filter_ccf*>(32);
      std::vector<float> vtaps(1.0);
      for(int i=0;i<32;++i){
        d_pu_filters[i] = new gr::filter::kernel::fir_filter_ccf(1, vtaps);
      }

      d_sig_buffer = new std::vector< std::vector<gr_complex> >;
      d_process_cap = 1024*1024; //memory preallocated
      d_cap_init = 1024*1024;
      d_process_buffer = new gr_complex[d_process_cap];
      d_process_size = 0;
      d_process_idx = 0;

      // NOTE: current version only support STANDARD mode
      // d_mode = (mode) INTERFERENCE_CANCELLATION : STANDARD;
      d_mode = STANDARD;
      d_state = SEARCH_ACCESSCODE;
      d_intf_state = CLEAR;

      d_su_hdr_const = su_hdr_const->base();
      d_su_bps = su_hdr_const->bits_per_symbol();
      d_su_pld_bps = su_pld_bps;
      d_su_hdr_bits_len = (4+2+2)*8;

    }

    /*
     * Our virtual destructor.
     */
    prou_sample_receiver_cb_impl::~prou_sample_receiver_cb_impl()
    {
      for(int i=0;i<32;++i)
      {
        delete d_pu_filters[i];
      }
      d_sig_buffer->clear();
      delete d_sig_buffer;
      delete [] d_process_buffer;
    }

    bool
    prou_sample_receiver_cb_impl::append_samples(const gr_complex* in, int size)
    {
      // forecast should handle the input sample length carefully
      switch(d_intf_state)
      {
        case CLEAR:
          if(d_process_size > 0.5* d_process_cap){
            //still clear but sample length greater than half of capacity
            reduce_sample(d_process_cap/3);
          }
          memcpy(d_process_buffer + d_process_size, in, sizeof(gr_complex)*size);
          d_process_size += size;
        break;
        case RETRANSMISSION:
          if(d_process_size > 2*d_process_cap/3){
            double_cap();
            if(d_process_cap > 16*d_cap_init){
              //failed force reset
              return false;
            }
          }
          memcpy(d_process_buffer + d_process_size, in, sizeof(gr_complex)*size);
          d_process_size += size;
        break;
        default:
          std::runtime_error("ProU RX: Func<append_samples>: Entering wrong state");
        break;
      }
      return true;
    }

    bool
    prou_sample_receiver_cb_impl::process_symbols()
    {
      int count = 0;
      unsigned char symbol;
      uint64_t check_bits;
      while(d_process_idx + count < d_process_size){
        symbol = d_su_hdr_const->decision_maker(&d_process_buffer[ d_process_idx + count]);
        switch(d_state)
        {
          case SEARCH_ACCESSCODE:
            for(int i=0;i<d_su_bps;++i){
              d_su_sync_reg = (d_su_sync_reg << 1) | ((symbol >> (d_su_bps-1-i) ) & 0x01);
            }
            check_bits = (d_su_sync_reg ^ d_su_accesscode) & d_su_code_mask;
            if(check_bits == 0){
              d_state = HEADER_WAIT;
              d_su_bit_input.clear();
              d_su_pkt_begin = (d_process_idx + count) - (d_su_code_len)/d_su_bps;
            }
          break;
          case HEADER_WAIT:
            for(int i=0;i<d_su_bps;++i){
              d_su_bit_input.push_back( ((symbol >> (d_su_bps-1-i)) & 0x01) );
            }
            if(d_su_bit_input.size() == d_su_hdr_bits_len){
              //uint16_t pld_len;
              if(parse_su_header(d_qidx,d_qsize,d_pld_len, d_su_bit_input)){
                d_state = PAYLOAD_WAIT;
                d_su_pld_counter = ( d_pld_len*8)/d_su_pld_bps + d_su_hdr_bits_len/d_su_bps;
              }
            }
          break;
          case PAYLOAD_WAIT:
          assert(d_su_pld_counter>=0);
            if(d_su_pld_counter==0){
              d_state = SEARCH_ACCESSCODE;
              if(intf_decision_maker()){

              }
              
            }
            d_su_pld_counter--;
          break;
          default:
          break;
        }
        count++;
      }
      //update
      d_process_idx = d_process_size;
    }

    bool
    prou_sample_receiver_cb_impl::parse_su_header(uint8_t& qidx, uint8_t& qsize, uint16_t& pld_len, const std::vector<unsigned char>& input)
    {
      uint16_t len0,len1;
      //int pld_len;
      len0 = _get_bit16(0,input);
      len1 = _get_bit16(16,input);
      if(len0 == len1)
      {
        pld_len = len0;
        qidx = _get_bit8(48,input);
        qsize = _get_bit8(56,input);
        //d_su_pld_counter = len0 * 8 /d_su_pld_bps;
        return true;
      }
      return false;
    }

    //INTERFERENCE CANCELLATION HELPER FUNCTIONS
    uint16_t
    prou_sample_receiver_cb_impl::_get_bit16(int begin_idx, const std::vector<unsigned char>& input)
    {
      unsigned long tmp=0UL;
      for(int i=0;i<16;++i){
        tmp |= (input[begin_idx+i] & 0x0001) << (15-i);  // NOTE: this is how bits arrangement should be like
      }
      return tmp;
    }

    uint8_t
    prou_sample_receiver_cb_impl::_get_bit8(int begin_idx, const std::vector<unsigned char>& input)
    {
      unsigned char tmp =0;
      for(int i=0;i<8;++i){
        tmp |= (input[begin_idx+i] & 0x01) << (7-i);
      }
      return tmp;
    }

    //SET FUNCTIONS
    bool
    prou_sample_receiver_cb_impl::set_su_accesscode(const std::string& su_accesscode)
    {
      d_su_code_len = su_accesscode.length();
      if(d_su_code_len > 64){
        return false;
      }
     
      d_su_code_mask = ((~0ULL) >> (64-d_su_code_len) );
      for(unsigned i=0;i<d_su_code_len;++i){
        d_su_accesscode = (d_su_accesscode << 1) | (su_accesscode[i] & 1);
      }
      return true;
    }

    //GET FUNCTIONS
    uint64_t
    prou_sample_receiver_cb_impl::su_accesscode() const
    {
      return d_su_accesscode;
    }

    //BUFFER HELPER FUNCTION
    void
    prou_sample_receiver_cb_impl::reduce_sample(int nleft)
    {
      assert(nleft<d_process_size);
      gr_complex tmp[nleft];
      memcpy(tmp, d_process_buffer + ( d_process_size - nleft), sizeof(gr_complex) * nleft);
      memcpy(d_process_buffer, tmp, sizeof(gr_complex) * nleft);
      d_process_size = nleft;
      d_process_idx = nleft;
    }

    void
    prou_sample_receiver_cb_impl::double_cap()
    {
      gr_complex tmp[d_process_size];
      memcpy(tmp, d_process_buffer, sizeof(gr_complex)*d_process_size);
      delete [] d_process_buffer;
      d_process_buffer = new gr_complex[d_process_cap*2];
      memcpy(d_process_buffer, tmp, sizeof(gr_complex)*d_process_size);
      d_process_cap*=2;
      //size remain the same, index also
    }

    void
    prou_sample_receiver_cb_impl::reset_buffer()
    {
      delete [] d_process_buffer;
      d_process_buffer = new gr_complex[d_cap_init];
      d_process_cap = d_cap_init;
      d_process_size = 0;
      d_process_idx = 0;
    }

    //INTERFERENCE CANCELLER FUNCTIONS
    bool
    prou_sample_receiver_cb_impl::intf_decision_maker()
    {
      //cei will be kept until this function ends
      int pkt_symbol_len = d_pld_len*8/d_su_pld_bps + d_su_hdr_bits_len/d_su_bps;
      bool test_voe = calc_var_energy(d_process_buffer + d_su_pkt_begin, pkt_symbol_len,-40,5);
      switch(d_intf_state)
      {
        case CLEAR:
          if(d_qsize!=0x00){
            d_intf_state = RETRANSMISSION;
            reset_intf_reg();
            d_retx_buf_idx.resize(d_qsize);
            d_retx_pkt_len.resize(d_qsize,0);
            d_retx_count = 0;
            d_cei_pkt_len.clear();
            d_cei_buf_idx.clear();
            //calc all cancellation enabling information
            calc_cei_all();            
          }
        break;
        case RETRANSMISSION:
          if( (d_qsize == 0x00)|| (d_qsize != d_retx_buf_idx.size()) ){
            //new pkt transmitting, retransmission ends
            if(d_retx_count != d_retx_buf_idx.size()){
              //fail to receive all retransmission, but still have chances to cancel interference
            }
            d_intf_state = CLEAR;
          }

        break;
        default:
          std::runtime_error("ProU RX: function<intf_decision_maker> Entering wrong state");
        break;
      }
      update_retx_info(test_voe);
      if((d_retx_count!=0) && (d_retx_count == d_retx_pkt_len.size()) ){
        do_interference_cancellation();
        d_retx_count=0;
        return true;
      }
      return false;
    }

    void
    prou_sample_receiver_cb_impl::update_retx_info(bool test_voe)
    {
      switch(d_intf_state)
      {
        case RETRANSMISSION:
          if( !test_voe){
            if(d_retx_pkt_len[d_qidx] == 0){
              d_retx_count++;
            }
            d_retx_buf_idx[d_qidx] = d_su_pkt_begin;
            d_retx_pkt_len[d_qidx] = d_pld_len*8/d_su_pld_bps + d_su_hdr_bits_len/d_su_bps;
          }
        break;
        case CLEAR:
        default:
          return;
        break;
      }
    }

    void
    prou_sample_receiver_cb_impl::reset_intf_reg()
    {
      //reset the registers required to decide whether retransmission is done or not.
      d_retx_count=0;
      d_retx_pkt_len.clear();
      d_retx_buf_idx.clear();
    }

    bool
    prou_sample_receiver_cb_impl::calc_var_energy(const gr_complex* in, size_t length, double threshold_db, int bin)
    {
      //TODO
      double voe = -1000;
      threshold_db = -50;
      return voe > threshold_db;
    }

    void
    prou_sample_receiver_cb_impl::calc_cei_all()
    {
      unsigned symbol;
      int state = INTF_SEARCH;
      uint64_t bit_reg = 0ULL;
      uint8_t qidx,qsize;
      uint16_t pld_len;
      int pld_count;
      int pkt_len;
      std::vector<unsigned char> input;
      for(int i=0;i<d_process_size;++i){
        symbol = d_su_hdr_const->decision_maker(&d_process_buffer[i]);
        switch(state)
        {
          case INTF_SEARCH:
            for(int j=0;j<d_su_bps;++j){
              bit_reg = (bit_reg << 1) | ((symbol >> (d_su_bps-1-j)) & 0x01);
            }
            if( ((bit_reg ^ d_su_accesscode)& d_su_code_mask) == 0 ){
              input.clear();
              state = INTF_HEADER;
            }
          break;
          case INTF_HEADER:
            for(int j=0;j<d_su_bps;++j){
              input.push_back( (symbol >> (d_su_bps-1-j)) & 0x01);
            }
            if(input.size() == d_su_hdr_bits_len){
              if(parse_su_header(qidx,qsize,pld_len,input)){
                pld_count = pld_len*8/d_su_pld_bps;
                pkt_len = pld_count + (d_su_hdr_bits_len + d_su_code_len)/d_su_bps;
                state = INTF_PAYLOAD_WAIT;
              }
            }
          break;
          case INTF_PAYLOAD_WAIT:
            if(pld_count==0){
              //TODO
              d_cei_buf_idx.push_back(i-pkt_len+1);
              d_cei_pkt_len.push_back(pkt_len);
            }
            pld_count--;
          break;
          default:
            std::runtime_error("PROU RX: calc_cei_all--Entering wrong state");
          break;
        }

      }
    }

    void
    prou_sample_receiver_cb_impl::do_interference_cancellation()
    {
      //TODO
      //input:
      // d_retx_buf_idx, d_retx_pkt_len
      // d_cei_buf_idx, d_cei_pkt_len
    }

    //FLOW CONTROL FUNCTIONS

    void
    prou_sample_receiver_cb_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
    }

    int
    prou_sample_receiver_cb_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];

      // Do <+signal processing+>
      std::vector<tag_t> tags;
      get_tags_in_range(tags,0,nitems_read(0),nitems_read(0)+noutput_items,pmt::intern("var_energy"));

      switch(d_mode)
      {
        case STANDARD:
          if(!tags.empty()){
            return 0;
          }
          memcpy(out, in, sizeof(gr_complex)*noutput_items);
        break;

        case INTERFERENCE_CANCELLATION:
          if(!append_samples(in, noutput_items)){
            //avoid memory overflow, force reset
            reset_buffer();
            reset_intf_reg();
            d_intf_state = CLEAR;
            d_state = SEARCH_ACCESSCODE;
          }
          if(process_symbols()){
            //TODO
            //return true if interference cancellation is conducted,
            // output the processed samples
            // the number of samples may be to long
            // how to do it efficiently
          }
        break;
        default:
          std::runtime_error("ProU RX: Wrong Receiver Mode!");
        break;
      }

      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (noutput_items);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace lsa */
} /* namespace gr */

