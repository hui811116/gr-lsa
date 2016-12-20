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


#ifndef INCLUDED_LSA_SU_QUEUED_TRANSMITTER_CC_H
#define INCLUDED_LSA_SU_QUEUED_TRANSMITTER_CC_H

#include <lsa/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace lsa {

    /*!
     * \brief <+description of block+>
     * \ingroup lsa
     *
     */
    class LSA_API su_queued_transmitter_cc : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<su_queued_transmitter_cc> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of lsa::su_queued_transmitter_cc.
       *
       * To avoid accidental use of raw pointers, lsa::su_queued_transmitter_cc's
       * constructor is in a private implementation
       * class. lsa::su_queued_transmitter_cc::make is the public interface for
       * creating new instances.
       */
      static sptr make(
        int max_queue_size=1024,
        const std::string & sensing_tag="sensing_info",
        const std::string & index_tag="packet_index",
        const std::string & accesscode="1101100011000111");
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_SU_QUEUED_TRANSMITTER_CC_H */

