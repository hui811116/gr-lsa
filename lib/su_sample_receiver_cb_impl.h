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

#ifndef INCLUDED_LSA_SU_SAMPLE_RECEIVER_CB_IMPL_H
#define INCLUDED_LSA_SU_SAMPLE_RECEIVER_CB_IMPL_H

#include <lsa/su_sample_receiver_cb.h>
#include <gnuradio/filter/fir_filter.h>
#include <gnuradio/digital/contellation.h>
#include <pmt/pmt.h>

namespace gr {
  namespace lsa {

    class su_sample_receiver_cb_impl : public su_sample_receiver_cb
    {
     private:
      // Nothing to declare in this block.

      int d_nfilters;
      std::vector<gr::filter::kernel::fir_filter_ccf*> d_filters;
      //std::vector<gr::filter::kernel::fir_filter_ccf*> 
      std::vector< std::vector<float> > d_taps;
      //std::vector< std::vector<float> > d_dtaps;
      std::vector<float> d_updated_taps;

      pmt::pmt_t d_scr_id;
      pmt::pmt_t d_sensing_tag_id;
      pmt::pmt_t d_msg_port;
      pmt::pmt_t d_debug_port;

      gr::digital::constellation_sptr d_hdr_sptr;

      int d_state;

      unsigned char * d_bytes_reg;


      bool symbol_segment(std::vector<tag_t>& tags);

     public:
      su_sample_receiver_cb_impl(gr::digital::constellation_sptr hdr_sptr);
      ~su_sample_receiver_cb_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_SU_SAMPLE_RECEIVER_CB_IMPL_H */

