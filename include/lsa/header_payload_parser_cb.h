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


#ifndef INCLUDED_LSA_HEADER_PAYLOAD_PARSER_CB_H
#define INCLUDED_LSA_HEADER_PAYLOAD_PARSER_CB_H

#include <lsa/api.h>
#include <gnuradio/block.h>
#include <gnuradio/digital/constellation.h>

namespace gr {
  namespace lsa {

    /*!
     * \brief <+description of block+>
     * \ingroup lsa
     *
     */
    class LSA_API header_payload_parser_cb : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<header_payload_parser_cb> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of lsa::header_payload_parser_cb.
       *
       * To avoid accidental use of raw pointers, lsa::header_payload_parser_cb's
       * constructor is in a private implementation
       * class. lsa::header_payload_parser_cb::make is the public interface for
       * creating new instances.
       */
      
      static sptr make(gr::digital::constellation_sptr hdr_constellation,
                       gr::digital::constellation_sptr pld_constellation,
                       const std::vector<gr_complex>& symbols,
                       const std::vector<unsigned char>& accessbits,
                       float threshold);
                       
      /*static sptr make(std::vector<gr_complex> hdr_constell,
        std::vector<int> hdr_pre_diff_code,
        unsigned int hdr_rotational_symmetry,
        unsigned int hdr_dimensionality,
        std::vector<gr_complex> pld_constell,
        std::vector<int> pld_pre_diff_code,
        unsigned int pld_rotational_symmetry,
        unsigned int pld_dimensionality,
        const std::vector<gr_complex>& symbols,
        const std::vector<unsigned char>& accessbits,
        double threshold);*/
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_HEADER_PAYLOAD_PARSER_CB_H */

