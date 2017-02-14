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


#ifndef INCLUDED_LSA_SU_SAMPLE_RECEIVER_CB_H
#define INCLUDED_LSA_SU_SAMPLE_RECEIVER_CB_H

#include <lsa/api.h>
#include <gnuradio/block.h>
#include <gnuradio/digital/constellation.h>
#include <pmt/pmt.h>

namespace gr {
  namespace lsa {

    /*!
     * \brief <+description of block+>
     * \ingroup lsa
     *
     */
    class LSA_API su_sample_receiver_cb : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<su_sample_receiver_cb> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of lsa::su_sample_receiver_cb.
       *
       * To avoid accidental use of raw pointers, lsa::su_sample_receiver_cb's
       * constructor is in a private implementation
       * class. lsa::su_sample_receiver_cb::make is the public interface for
       * creating new instances.
       */
      static sptr make(
        const std::string& sensing_tag_id,
        const std::string& accesscode,
        const gr::digital::constellation_sptr& hdr_const,
        const gr::digital::constellation_sptr& pld_const,
        double sps,
        float loop_bw,
        const std::vector<float> &taps,
        unsigned int filter_size,
        float init_phase,
        float max_rate_deviation,
        int osps,
        int cos_order,
        bool debug,
        bool sync);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_SU_SAMPLE_RECEIVER_CB_H */

