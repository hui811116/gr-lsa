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

#ifndef INCLUDED_LSA_INTERFERENCE_ENERGY_DETECTOR_CC_IMPL_H
#define INCLUDED_LSA_INTERFERENCE_ENERGY_DETECTOR_CC_IMPL_H

#include <lsa/interference_energy_detector_cc.h>

namespace gr {
  namespace lsa {

    class interference_energy_detector_cc_impl : public interference_energy_detector_cc
    {
     private:
      // Nothing to declare in this block.
      float* d_energy_reg;
      float* d_voe_reg;
      int d_blocklength;
      size_t d_buffer_length;
      float d_ed_threshold_db;
      float d_voe_threshold_db;

      pmt::pmt_t d_voe_tagname;
      pmt::pmt_t d_ed_tagname;
      //pmt::pmt_t d_debug;

     public:
      interference_energy_detector_cc_impl(
        const std::string& ed_tagname,
        const std::string& voe_tagname,
        float ed_threshold,
        float voe_threshold,
        size_t blocklength);
      ~interference_energy_detector_cc_impl();

      // Where all the action really happens
      int work(int noutput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);

      void set_ed_threshold(float threshold_db);
      float ed_threshold() const;

      void set_voe_threshold(float threshold_db);
      float voe_threshold() const;

      void set_blocklength(size_t blocklength);
      size_t blocklength() const;


    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_INTERFERENCE_ENERGY_DETECTOR_CC_IMPL_H */

