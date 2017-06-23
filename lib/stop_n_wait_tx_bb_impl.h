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

#ifndef INCLUDED_LSA_STOP_N_WAIT_TX_BB_IMPL_H
#define INCLUDED_LSA_STOP_N_WAIT_TX_BB_IMPL_H

#include <lsa/stop_n_wait_tx_bb.h>
#include "utils.h"
#include <fstream>

namespace gr {
  namespace lsa {

    class stop_n_wait_tx_bb_impl : public stop_n_wait_tx_bb
    {
     private:
      // Nothing to declare in this block.
      const pmt::pmt_t d_in_port;
      const pmt::pmt_t d_tagname;
      std::list<srArq_t> d_arq_list;
      gr::thread::mutex d_mutex;
      bool d_sns_stop;
      bool d_state_change;
      unsigned char d_buf[256];
      uint16_t d_seq;

      std::fstream d_file;
      std::vector< std::vector<unsigned char> > d_data_src;
      bool d_usef;

      bool read_data(const std::string& filename);
      void msg_handler(pmt::pmt_t msg);
      void generate_new_pkt(const unsigned char* in, int nin);

     protected:
      int calculate_output_stream_length(const gr_vector_int &ninput_items);

     public:
      stop_n_wait_tx_bb_impl(const std::string& tagname,const std::string& filename, bool usef);
      ~stop_n_wait_tx_bb_impl();

      // Where all the action really happens
      int work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_STOP_N_WAIT_TX_BB_IMPL_H */

