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

#ifndef INCLUDED_LSA_SU_TRANSMITTER_BC_IMPL_H
#define INCLUDED_LSA_SU_TRANSMITTER_BC_IMPL_H

#include <lsa/su_transmitter_bc.h>

namespace gr {
  namespace lsa {

    class su_transmitter_bc_impl : public su_transmitter_bc
    {
     private:
      // Nothing to declare in this block.
      bool d_debug;
      int d_state;
      int d_qmax;

      int d_qiter;
      int d_qsize;
      uint64_t d_pkt_counter;

      pmt::pmt_t d_lengthtagname;
      pmt::pmt_t d_sensingtagname;
      pmt::pmt_t d_rxindextagname;

      pmt::pmt_t d_src_id;
      pmt::pmt_t d_sensing_info_port;
      
      pmt::pmt_t d_debug_port;

      std::vector<gr_complex> d_hdr_points;
      std::vector<gr_complex> d_pld_points;
      std::vector<int> d_hdr_map;
      std::vector<int> d_pld_map;
      gr::digital::constellation_sptr d_hdr_const;
      gr::digital::constellation_sptr d_pld_const;

      size_t d_hdr_samp_len;
      std::vector< std::vector<gr_complex> >* d_buffer_ptr;
      std::vector<uint64_t> d_counter_buffer;
      std::vector<uint16_t> d_pld_len_buffer;

      std::vector<uint64_t> d_retx_counter_buffer;
      

      unsigned char* d_hdr_buffer;
      unsigned char* d_hdr_symbol_buffer;
      unsigned char* d_pld_symbol_buffer;


      std::vector<unsigned char> d_accesscode;

      void receiver_msg_handler(pmt::pmt_t msg);

      size_t header_nbits() const;

      void generate_hdr(int pld_len, bool type);

      void queue_size_adapt ();

      void _repack(
        unsigned char* out,
        const unsigned char* in,
        int size,
        int const_m
        );

      void _map_sample(
        gr_complex* out,
        const unsigned char* in,
        int size,
        const std::vector<gr_complex>& mapper,
        const std::vector<int>& idx_map
        );

      void store_to_queue (
        gr_complex* samp, 
        int pld_samp_len, 
        int pld_bytes_len
        );

     protected:
      int calculate_output_stream_length(const gr_vector_int &ninput_items);

     public:
      su_transmitter_bc_impl(
        const std::string& lengthtagname,
        const std::string& sensing_tag,
        const std::string& accesscode,
        const gr::digital::constellation_sptr& hdr_const,
        const gr::digital::constellation_sptr& pld_const,
        //const std::vector<gr_complex>& hdr_points,
        //const std::vector<gr_complex>& pld_points,
        int qmax,
        bool debug);
      ~su_transmitter_bc_impl();

      // Where all the action really happens
      int work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);


      bool set_accesscode(const std::string& accesscode);

    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_SU_TRANSMITTER_BC_IMPL_H */

