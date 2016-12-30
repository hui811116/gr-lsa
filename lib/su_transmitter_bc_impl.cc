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
#include "su_transmitter_bc_impl.h"

namespace gr {
  namespace lsa {


    enum SuTxState{
      CLEAR_TO_SEND,
      RETRANSMISSION
    };

    su_transmitter_bc::sptr
    su_transmitter_bc::make(
      const std::string& lengthtagname,
      const std::string& sensing_tag,
      const std::string& accesscode,
      const std::vector<gr_complex>& hdr_points,
      const std::vector<gr_complex>& pld_points,
      int qmax,
      bool debug)
    {
      return gnuradio::get_initial_sptr
        (new su_transmitter_bc_impl(
          lengthtagname,
          sensing_tag,
          accesscode,
          hdr_points,
          pld_points,
          qmax,
          debug));
    }

    /*
     * The private constructor
     */
    su_transmitter_bc_impl::su_transmitter_bc_impl(
      const std::string& lengthtagname,
      const std::string& sensing_tag,
      const std::string& accesscode,
      const std::vector<gr_complex>& hdr_points,
      const std::vector<gr_complex>& pld_points,
      int qmax,
      bool debug)
      : gr::tagged_stream_block("su_transmitter_bc",
              gr::io_signature::make(1, 1, sizeof(char)),
              gr::io_signature::make(1, 1, sizeof(gr_complex)), lengthtagname),
      d_lengthtagname(pmt::string_to_symbol(lengthtagname)),
      d_sensingtagname(pmt::string_to_symbol(sensing_tag)),
      d_src_id(pmt::intern(alias()))
    {
      if (!set_accesscode(accesscode)){
        throw std::invalid_argument("SU TX: Invalid accesscode");
      }
      if (qmax<1) {
        throw std::invalid_argument ("SU TX: Invalid queue size");
      }
      d_qmax = qmax;
      d_debug = debug;

      d_hdr_points = hdr_points;
      d_pld_points = pld_points;

      // NOTE: make sure the length is divisible by any kind of modulation
      d_hdr_samp_len = (accesscode.length()+8*8) / (int) log2(d_hdr_points.size());

      d_buffer_ptr = new std::vector< std::vector<gr_complex> >;
      d_hdr_buffer = new unsigned char[d_accesscode.size()+8];
      d_hdr_symbol_buffer = new unsigned char[d_hdr_samp_len];

      size_t max_pld_symbol_size=65536;
      d_pld_symbol_buffer = new unsigned char[max_pld_symbol_size];

      d_sensing_info_port = pmt::mp("sensing_info");
      d_header_info_port = pmt::mp("header");
      d_debug_port = pmt::mp("debug");
      message_port_register_in(d_sensing_info_port);
      message_port_register_out(d_header_info_port);
      message_port_register_out(d_debug_port);

      set_tag_propagation_policy(TPP_DONT);
      set_msg_handler(d_sensing_info_port, boost::bind(&su_transmitter_bc_impl::receiver_msg_handler, this , _1));
      
      //initialization
      d_qiter = 0;
      d_qsize = 0;
      d_pkt_counter = 0;
      d_state = CLEAR_TO_SEND;

    }

    /*
     * Our virtual destructor.
     */
    su_transmitter_bc_impl::~su_transmitter_bc_impl()
    {
      d_buffer_ptr->clear();
      delete d_buffer_ptr;
      delete [] d_hdr_buffer;
      delete [] d_hdr_symbol_buffer;
      delete [] d_pld_symbol_buffer;
    }

    int
    su_transmitter_bc_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
    {
      //TODO
      int noutput_items =  ninput_items[0];
      //return noutput_items ;
      switch(d_state)
      {
        case CLEAR_TO_SEND:
          noutput_items = (ninput_items[0]*8)/(int)log2(d_hdr_points.size())+d_hdr_samp_len;
        break;
        case RETRANSMISSION:
          assert(!d_pld_len_buffer.empty());
          noutput_items = (d_buffer_ptr->at(d_qiter)).size() + d_hdr_samp_len;
        break;
        default:
        break;

      }
      return noutput_items;
    }


    void
    su_transmitter_bc_impl::receiver_msg_handler(pmt::pmt_t msg)
    {
      std::vector<pmt::pmt_t> tag_keys;
      std::vector<pmt::pmt_t> tag_vals;
      pmt::pmt_t dict_items(pmt::dict_items(msg));
      pmt::pmt_t debug_tag_found = pmt::make_dict();
      while(!pmt::is_null(dict_items)) {
        pmt::pmt_t this_item(pmt::car(dict_items));
        tag_keys.push_back(pmt::car(this_item));
        tag_vals.push_back(pmt::cdr(this_item));
        dict_items = pmt::cdr(dict_items);
      }
      for(int i=0;i<tag_keys.size();++i){
        if(pmt::equal(tag_keys[i],d_sensingtagname)){
          debug_tag_found = pmt::dict_add(debug_tag_found, pmt::intern("sutx_debug"), pmt::string_to_symbol("found sensing tag"));
          message_port_pub(d_debug_port,debug_tag_found);
        }
      }
    }

    bool
    su_transmitter_bc_impl::set_accesscode(const std::string& accesscode)
    {
      if(accesscode.length()<1){return false;}
      int code_size = accesscode.length()/8;
      unsigned char tmp;
      for(int i=0;i<code_size;++i)
      {
        tmp=0x00;
        for(int j=0;j<8;++j){
          tmp = tmp | (((accesscode[i*8+j]=='0')? 0x00 : 0x01) << (7-j));
        }
        d_accesscode.push_back(tmp);
      }
      return true;
    }

    size_t 
    su_transmitter_bc_impl::header_nbits()const
    {
      return (d_accesscode.size()+ 8)*8;
    }

    void
    su_transmitter_bc_impl::generate_hdr(int pld_len, bool type)
    {
      uint16_t pld_len16 = (uint16_t)pld_len;
      unsigned char* pkt_len= (unsigned char*)& pld_len16;
      unsigned char* pkt_counter= (unsigned char*)& d_pkt_counter;
      
      //  HEADER FORMAT
      //  | n bits accesscode ||  16 bits length || 16 bits length ||16 bits counter || 8 bits queue idx || 8 bits queue size
      int ac_len=d_accesscode.size();
      for(int i=0;i<d_accesscode.size();++i){
        d_hdr_buffer[i]=d_accesscode[i];
      }
        d_hdr_buffer[ac_len]=pkt_len[1];
        d_hdr_buffer[ac_len+1]=pkt_len[0];
        d_hdr_buffer[ac_len+2]=pkt_len[1];
        d_hdr_buffer[ac_len+3]=pkt_len[0];
        if(type){
          assert(d_pld_len_buffer.size()!=0);
          //d_qidx=d_qiter;
          pkt_counter = (unsigned char*)& d_counter_buffer[d_qiter];
          //d_qsize=(unsigned char) d_pld_len_buffer.size();
        }
        d_hdr_buffer[ac_len+7] = d_qsize;
        d_hdr_buffer[ac_len+6] = d_qiter;
        d_hdr_buffer[ac_len+4] = pkt_counter[1];
        d_hdr_buffer[ac_len+5] = pkt_counter[0];
    }

    void
    su_transmitter_bc_impl::queue_size_adapt()
    {
      int erase_size = d_qmax/2;
        GR_LOG_CRIT(d_logger, "SU Queued Transmitter: Reaching maximum capacity, removing contents.");
        d_buffer_ptr->erase(d_buffer_ptr->begin(), d_buffer_ptr->begin()+erase_size);
        d_counter_buffer.erase(d_counter_buffer.begin(), d_counter_buffer.begin()+erase_size);
        d_pld_len_buffer.erase(d_pld_len_buffer.begin(), d_pld_len_buffer.begin()+erase_size);
    }

    void
    su_transmitter_bc_impl::_repack(unsigned char* out, const unsigned char* in, int size, int const_m)
    {
      unsigned char tmp;
      for(int i=0;i<size *8;++i)
      {
        tmp = tmp | (((in[i/8]>>(7-i%8)) & 0x01) << (const_m-1- (i%const_m)) );
        if( (i+1) % const_m == 0){
          out[i/const_m] = tmp;
          tmp = 0x00;
        }
      }
      if( (size*8 % const_m) !=0)
        out[size*8/const_m] = tmp;
    }

    void
    su_transmitter_bc_impl::_map_sample(gr_complex* out, const unsigned char* in, int size, const std::vector<gr_complex>& mapper)
    {
      for(int i=0;i<size;++i){
        out[i] = mapper[in[i]];
      }
    }

    void
    su_transmitter_bc_impl::store_to_queue(gr_complex* samp, int pld_samp_len, int pld_bytes_len)
    {
      std::vector<gr_complex> samp_vec;
      for(int i=0;i<pld_samp_len;++i){
        samp_vec.push_back(samp[i]);
      }
      d_buffer_ptr->push_back(samp_vec);
      d_counter_buffer.push_back(d_pkt_counter);
      d_pld_len_buffer.push_back(pld_bytes_len);
    }

    int
    su_transmitter_bc_impl::work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const unsigned char *in = (const unsigned char *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
/*
      std::vector<tag_t> tags;
      get_tags_in_range(tags, 0, nitems_read(0), nitems_read(0)+ninput_items[0]);
      for(int i=0;i<tags.size();++i){
        tags[i].offset -= nitems_read(0);
        add_item_tag(0,nitems_written(0)+tags[i].offset,tags[i].key,tags[i].value);
      }
      */

      int payload_len = ninput_items[0];
      int pld_symbol_samp_len;
      pmt::pmt_t hdr_info = pmt::make_dict();
      pmt::pmt_t debug_info = pmt::make_dict();

      //gr_complex hdr_symbol[d_hdr_samp_len];
      switch(d_state)
      {
        case CLEAR_TO_SEND:
          if(d_buffer_ptr->size() == d_qmax){
            queue_size_adapt();
          }
          generate_hdr(ninput_items[0],false);
          _repack(d_hdr_symbol_buffer, d_hdr_buffer, header_nbits()/8, (int)log2(d_hdr_points.size()));
          _map_sample(out, d_hdr_symbol_buffer, d_hdr_samp_len, d_hdr_points);
          _repack(d_pld_symbol_buffer, (const unsigned char*)input_items[0], ninput_items[0], (int)log2(d_pld_points.size()));
          _map_sample(out+d_hdr_samp_len, d_pld_symbol_buffer, pld_symbol_samp_len, d_pld_points);

          pld_symbol_samp_len = payload_len*8/log2(d_pld_points.size());
          store_to_queue(out+d_hdr_samp_len, pld_symbol_samp_len, ninput_items[0]);
          hdr_info = pmt::dict_add(hdr_info, pmt::intern("pkt_counter"), pmt::from_long(d_pkt_counter));
          hdr_info = pmt::dict_add(hdr_info, pmt::intern("queue_index"),pmt::from_long(d_qiter));
          hdr_info = pmt::dict_add(hdr_info, pmt::intern("queue_size"),pmt::from_long(d_qsize));
          d_pkt_counter++;
        break;

        case RETRANSMISSION:
          assert(d_qiter < d_pld_len_buffer.size());
          generate_hdr(d_pld_len_buffer[d_qiter],true);
          _repack(d_hdr_symbol_buffer, d_hdr_buffer,header_nbits()/8,(int)log2(d_hdr_points.size()));
          _map_sample(out, d_hdr_symbol_buffer, d_hdr_samp_len, d_hdr_points);

          pld_symbol_samp_len = (d_buffer_ptr->at(d_qiter)).size();
          payload_len = d_pld_len_buffer[d_qiter];
          hdr_info = pmt::dict_add(hdr_info, pmt::intern("pkt_counter"), pmt::from_long((uint16_t)d_counter_buffer[d_qiter]));
          hdr_info = pmt::dict_add(hdr_info, pmt::intern("queue_index"),pmt::from_long(0));
          hdr_info = pmt::dict_add(hdr_info, pmt::intern("queue_size"),pmt::from_long(0));
          memcpy(out+d_hdr_samp_len,&(d_buffer_ptr->at(d_qiter)),sizeof(gr_complex)*pld_symbol_samp_len);
          d_qiter++;
          d_qiter %= d_qsize;
        break;
        default:
        std::runtime_error("SU TX: Entering wrong state");
        break;
      }
      //number of output processing
      noutput_items = pld_symbol_samp_len + d_hdr_samp_len;

      add_item_tag(0, nitems_written(0), pmt::intern("sutx_pkt_count"), pmt::from_long(d_pkt_counter));
      hdr_info = pmt::dict_add(hdr_info, pmt::intern("payload_len"), pmt::from_long(payload_len));
      hdr_info = pmt::dict_add(hdr_info, pmt::intern("SUTX_state"), pmt::string_to_symbol(((d_state == CLEAR_TO_SEND)? "clear_to_send" : "proU_present")));
      message_port_pub(d_header_info_port,hdr_info);

//debug
      if(d_debug){
        debug_info = pmt::dict_add(debug_info, pmt::intern("header_bits"),pmt::from_long(header_nbits()));
        debug_info = pmt::dict_add(debug_info, pmt::intern("payload_len"),pmt::from_long(payload_len));
        debug_info = pmt::dict_add(debug_info, pmt::intern("noutput_items"),pmt::from_long(noutput_items));
        message_port_pub(d_debug_port, debug_info);
      }
//end debug

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace lsa */
} /* namespace gr */
