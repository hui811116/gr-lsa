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


#ifndef INCLUDED_LSA_PROU_SAMPLE_RECEIVER_CB_H
#define INCLUDED_LSA_PROU_SAMPLE_RECEIVER_CB_H

#include <lsa/api.h>
#include <gnuradio/block.h>
#include <gnuradio/filter/fir_filter.h>
#include <gnuradio/digital/constellation.h>
#include <gnuradio/blocks/control_loop.h>

namespace gr {
  namespace lsa {

    /*!
     * \brief <+description of block+>
     * \ingroup lsa
     *
     */
    class LSA_API prou_sample_receiver_cb
    : virtual public gr::block,
      virtual public gr::blocks::control_loop
    {
     public:
      typedef boost::shared_ptr<prou_sample_receiver_cb> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of lsa::prou_sample_receiver_cb.
       *
       * To avoid accidental use of raw pointers, lsa::prou_sample_receiver_cb's
       * constructor is in a private implementation
       * class. lsa::prou_sample_receiver_cb::make is the public interface for
       * creating new instances.
       */
      static sptr make(
        const gr::digital::constellation_sptr& su_hdr_const,
        int su_pld_bps,
        const std::string& su_accesscode,
        double plf_sps,
        float plf_loop_bw,
        const std::vector<float>& plf_taps,
        int su_nfilts,
        float costas_loop_bw,
        int costas_order,
        bool mode,
        bool debug);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_PROU_SAMPLE_RECEIVER_CB_H */

