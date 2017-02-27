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


#ifndef INCLUDED_LSA_SYMBOL_RECEIVER_C_H
#define INCLUDED_LSA_SYMBOL_RECEIVER_C_H

#include <lsa/api.h>
#include <gnuradio/sync_block.h>
#include <gnuradio/digital/constellation.h>

namespace gr {
  namespace lsa {

    /*!
     * \brief <+description of block+>
     * \ingroup lsa
     *
     */
    class LSA_API symbol_receiver_c : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<symbol_receiver_c> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of lsa::symbol_receiver_c.
       *
       * To avoid accidental use of raw pointers, lsa::symbol_receiver_c's
       * constructor is in a private implementation
       * class. lsa::symbol_receiver_c::make is the public interface for
       * creating new instances.
       */
      static sptr make(const std::string& accesscode,
        const gr::digital::constellation_sptr& hdr_const,
        const gr::digital::constellation_sptr& pld_const,
        const std::string& sensing_tagname,
        int sps,
        bool debug);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_SYMBOL_RECEIVER_C_H */

