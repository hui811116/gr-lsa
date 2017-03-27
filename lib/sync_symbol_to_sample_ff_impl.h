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

#ifndef INCLUDED_LSA_SYNC_SYMBOL_TO_SAMPLE_FF_IMPL_H
#define INCLUDED_LSA_SYNC_SYMBOL_TO_SAMPLE_FF_IMPL_H

#include <lsa/sync_symbol_to_sample_ff.h>
#include <pmt/pmt.h>

namespace gr {
  namespace lsa {

    class sync_symbol_to_sample_ff_impl : public sync_symbol_to_sample_ff
    {
     private:
      // Nothing to declare in this block.
      pmt::pmt_t d_hdr_tagname;
      int d_sps;
      int d_nfilts;

     public:
      sync_symbol_to_sample_ff_impl(
        int sps,
        int nfilts,
        const std::string& hdr_tagname);
      ~sync_symbol_to_sample_ff_impl();

      // Where all the action really happens
      int work(int noutput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_SYNC_SYMBOL_TO_SAMPLE_FF_IMPL_H */

