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

#ifndef INCLUDED_LSA_SU_QUEUED_TRANSMITTER_CC_IMPL_H
#define INCLUDED_LSA_SU_QUEUED_TRANSMITTER_CC_IMPL_H

#include <lsa/su_queued_transmitter_cc.h>

namespace gr {
  namespace lsa {

    class su_queued_transmitter_cc_impl : public su_queued_transmitter_cc
    {
     private:
      // Nothing to declare in this block.
      int d_state;
      int d_max_queue_size;

      int d_index;
      bool d_sensing;
            
      pmt::pmt_t d_rx_sensing_tag;
      pmt::pmt_t d_rx_index_tag;
      std::vector<pmt::pmt_t> d_rx_tag_keys;  //temporary buffer for multiple rx message
      std::vector<pmt::pmt_t> d_rx_tag_values;//temporary buffer for multiple rx message value


      // sensing information 
      std::vector< std::vector<unsigned char> >* d_buffer_ptr;
      std::vector<unsigned int> d_index_buffer;
      std::vector<unsigned int> d_sensing_queue;
      int d_sensing_count;


      std::vector<unsigned char> d_accesscode;

      int check_queue(int check_index);
      bool check_sensing_queue(int check_index);

      //gr::digital::constellation_sptr d_hdr_const;
      //gr::digital::constellation_sptr d_pld_const;

     public:
      su_queued_transmitter_cc_impl(
        const std::string & sensing_tag,
        const std::string & index_tag,
        const std::string & accesscode);

      ~su_queued_transmitter_cc_impl();

      // Where all the action really happens
      void receiver_msg_handler(pmt::pmt_t rx_msg);

      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

      void set_accesscode(const std::string & accesscode);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_SU_QUEUED_TRANSMITTER_CC_IMPL_H */

