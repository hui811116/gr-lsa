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

#ifndef INCLUDED_LSA_MODIFIED_COSTAS_LOOP_CC_IMPL_H
#define INCLUDED_LSA_MODIFIED_COSTAS_LOOP_CC_IMPL_H

#include <lsa/modified_costas_loop_cc.h>
#include <pmt/pmt.h>

namespace gr {
  namespace lsa {

    class modified_costas_loop_cc_impl : public modified_costas_loop_cc
    {
     private:
      int d_order;
      float d_error;
      float d_noise;

      //float d_prev_freq;      
      bool d_intf_state;

      bool d_burst_mode;
      bool d_found_burst;

      pmt::pmt_t d_intf_tagname;

       /*! \brief the phase detector circuit for 8th-order PSK loops.
       *
       *  \param sample complex sample
       *  \return the phase error
       */
      float phase_detector_8(gr_complex sample) const;    // for 8PSK

      /*! \brief the phase detector circuit for fourth-order loops.
       *
       *  \param sample complex sample
       *  \return the phase error
       */
      float phase_detector_4(gr_complex sample) const;    // for QPSK

      /*! \brief the phase detector circuit for second-order loops.
       *
       *  \param sample a complex sample
       *  \return the phase error
       */
      float phase_detector_2(gr_complex sample) const;    // for BPSK


      /*! \brief the phase detector circuit for 8th-order PSK
       *  loops. Uses tanh instead of slicing and the noise estimate
       *  from the message port to estimated SNR of the samples.
       *
       *  \param sample complex sample
       *  \return the phase error
       */
      float phase_detector_snr_8(gr_complex sample) const;    // for 8PSK

      /*! \brief the phase detector circuit for fourth-order
       *  loops. Uses tanh instead of slicing and the noise estimate
       *  from the message port to estimated SNR of the samples.
       *
       *  \param sample complex sample
       *  \return the phase error
       */
      float phase_detector_snr_4(gr_complex sample) const;    // for QPSK

      /*! \brief the phase detector circuit for second-order
       *  loops. Uses tanh instead of slicing and the noise estimate
       *  from the message port to estimated SNR of the samples.
       *
       *  \param sample a complex sample
       *  \return the phase error
       */
      float phase_detector_snr_2(gr_complex sample) const;    // for BPSK


      float (modified_costas_loop_cc_impl::*d_phase_detector)(gr_complex sample) const;
     public:
      modified_costas_loop_cc_impl(float loop_bw, int order, bool use_snr, 
                                   const std::string& intf_tagname,
                                   bool burst_mode);
      ~modified_costas_loop_cc_impl();

      float error() const;

      void handle_set_noise(pmt::pmt_t msg);

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_MODIFIED_COSTAS_LOOP_CC_IMPL_H */

