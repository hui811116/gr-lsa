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

#ifndef INCLUDED_LSA_CORRELATE_EXTRACT_CC_IMPL_H
#define INCLUDED_LSA_CORRELATE_EXTRACT_CC_IMPL_H

#include <lsa/correlate_extract_cc.h>

namespace gr {
  namespace lsa {

    class correlate_extract_cc_impl : public correlate_extract_cc
    {
     private:
      // Nothing to declare in this block.
      std::vector<gr_complex> d_symbols;
      float d_threshold;

      void calc_corr(gr_complex* corr,const gr_complex* in, int in_size);


     public:
      correlate_extract_cc_impl(const std::vector<gr_complex>& symbols, float threshold);
      ~correlate_extract_cc_impl();

      // Where all the action really happens
      //void forecast (int noutput_items, gr_vector_int &ninput_items_required);
      std::vector<gr_complex> symbols() const;

      void set_symbols(const std::vector<gr_complex>& symbols);

      float get_threshold() const;

      void set_threshold(float threshold);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_CORRELATE_EXTRACT_CC_IMPL_H */

