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

#ifndef INCLUDED_LSA_SYMBOL_CORRECTOR_CC_IMPL_H
#define INCLUDED_LSA_SYMBOL_CORRECTOR_CC_IMPL_H

#include <lsa/symbol_corrector_cc.h>

namespace gr {
  namespace lsa {

    class symbol_corrector_cc_impl : public symbol_corrector_cc
    {
     private:
      float d_phase;
      float d_amp;

     public:
      symbol_corrector_cc_impl();
      ~symbol_corrector_cc_impl();

      // Where all the action really happens
      int work(int noutput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_SYMBOL_CORRECTOR_CC_IMPL_H */
