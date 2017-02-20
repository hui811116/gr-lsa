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


#ifndef INCLUDED_LSA_SU_HEADER_PREFIX_H
#define INCLUDED_LSA_SU_HEADER_PREFIX_H

#include <lsa/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr {
  namespace lsa {

    /*!
     * \brief <+description of block+>
     * \ingroup lsa
     *
     */
    class LSA_API su_header_prefix : virtual public gr::tagged_stream_block
    {
     public:
      typedef boost::shared_ptr<su_header_prefix> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of lsa::su_header_prefix.
       *
       * To avoid accidental use of raw pointers, lsa::su_header_prefix's
       * constructor is in a private implementation
       * class. lsa::su_header_prefix::make is the public interface for
       * creating new instances.
       */
      static sptr make(
        const std::string & accesscode="1110010010110011",
        const std::string& lengthtagname="packet_len", 
        int mode=0,
        int size=0);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_SU_HEADER_PREFIX_H */

