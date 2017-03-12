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


#ifndef INCLUDED_LSA_SU_TRANSMITTER_BC_H
#define INCLUDED_LSA_SU_TRANSMITTER_BC_H

#include <lsa/api.h>
#include <gnuradio/tagged_stream_block.h>
#include <gnuradio/digital/constellation.h>

namespace gr {
  namespace lsa {

    /*!
     * \brief <+description of block+>
     * \ingroup lsa
     *
     */
    class LSA_API su_transmitter_bc : virtual public gr::tagged_stream_block
    {
     public:
      typedef boost::shared_ptr<su_transmitter_bc> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of lsa::su_transmitter_bc.
       *
       * To avoid accidental use of raw pointers, lsa::su_transmitter_bc's
       * constructor is in a private implementation
       * class. lsa::su_transmitter_bc::make is the public interface for
       * creating new instances.
       */
      static sptr make(
        const std::string& lengthtagname,
        const std::string& sensing_tag,
        const std::string& accesscode,
        const gr::digital::constellation_sptr& hdr_const,
        const gr::digital::constellation_sptr& pld_const,
        //const std::vector<gr_complex>& hdr_points,
        //const std::vector<gr_complex>& pld_points,
        int qmax,
        bool debug);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_SU_TRANSMITTER_BC_H */

