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

#ifndef INCLUDED_LSA_HEADER_PAYLOAD_PARSER_CB_IMPL_H
#define INCLUDED_LSA_HEADER_PAYLOAD_PARSER_CB_IMPL_H

#include <lsa/header_payload_parser_cb.h>
#include <gnuradio/digital/constellation.h>

namespace gr {
  namespace lsa {

    class header_payload_parser_cb_impl : public header_payload_parser_cb
    {
     private:
      // Nothing to declare in this block.
      // message port
      pmt::pmt_t d_msg_port;

      // private members
      float d_threshold;
      //std::vector<gr_complex> d_buffer;
      std::vector<gr_complex> d_symbols;
      float symbol_norm;
      std::vector<unsigned char> d_accessbits;
      // find out how to use it in GRC
      gr::digital::constellation_sptr d_hdr_const_ptr;
      gr::digital::constellation_sptr d_pld_const_ptr;
      //
      

      void cal_correlation(std::vector<gr_complex>& corr, const gr_complex * in, int i_size);
      bool corr_thres_locate_pkt(std::vector<int>& indices, const std::vector<gr_complex>& corr);
      void sync_accessbits(std::vector<int>& pos, const unsigned char* in, int i_size);
      //bool hdr_demux();
      //void parse_packet_length();
      void repack_bits_lsb(unsigned char* out, unsigned char* in,unsigned int len,unsigned int bps);


     public:
      
      header_payload_parser_cb_impl(gr::digital::constellation_sptr hdr_constellation,
                                    gr::digital::constellation_sptr pld_constellation,
                                    const std::vector<gr_complex>& symbols,
                                    const std::string& accessbits,
                                    float threshold);
                                    

      ~header_payload_parser_cb_impl();

      // Where all the action really happens
      //void forecast (int noutput_items, gr_vector_int &ninput_items_required);
      std::string accessbits() const;
      void set_accessbits(const std::string & accesscode);

      float threshold() const;
      void set_threshold(float threshold);

      std::vector<gr_complex> symbols() const;
      void set_symbols(const std::vector<gr_complex>& symbols);


      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_HEADER_PAYLOAD_PARSER_CB_IMPL_H */

