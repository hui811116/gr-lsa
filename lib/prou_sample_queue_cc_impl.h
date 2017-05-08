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

#ifndef INCLUDED_LSA_PROU_SAMPLE_QUEUE_CC_IMPL_H
#define INCLUDED_LSA_PROU_SAMPLE_QUEUE_CC_IMPL_H

#include <lsa/prou_sample_queue_cc.h>

namespace gr {
  namespace lsa {

    class prou_sample_queue_cc_impl : public prou_sample_queue_cc
    {
     private:
      // Nothing to declare in this block.
      pmt::pmt_t d_info_port;
      pmt::pmt_t d_sensing_tagname;

      gr_complex* d_sample_buffer;
      //std::vector<pmt::pmt_t> d_pkt_info;
      std::vector<pmt::pmt_t> d_buffer_info;
      std::vector<pmt::pmt_t> d_sync_info;

      int d_sample_idx;
      const unsigned int d_sample_cap;
      int d_sample_size;
      //int d_port_2_idx;
      
      long int d_update_time;
      long int d_current_time;
      long int d_last_time;
      float d_timeout;

      int d_sps;
      int d_bps;
      int d_output_count;
      const int d_min_output_items;

      bool d_debug;
      //helper function

      bool check_port1(int nout, std::vector<pmt::pmt_t>& tags);
      void half_queue_cap();
      /*
      void out_items_handler(
        gr_complex* out, 
        gr_complex* sample_out, 
        const gr_complex* in, 
        int noutput_items, 
        int ninput_items
        );*/

      /*void append_samples(
        const gr_complex* in, 
        int ninput_items,
        int noutput_items, 
        int& consume_count, 
        long int time);*/

      //void update_sample_buffer();

     public:
      // message handler
      void info_msg_handler(pmt::pmt_t msg);
      
      prou_sample_queue_cc_impl(
        const std::string& sensing_tagname, 
        int bps,
        int sps,
        bool debug);
      ~prou_sample_queue_cc_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_PROU_SAMPLE_QUEUE_CC_IMPL_H */

