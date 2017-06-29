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


#ifndef INCLUDED_LSA_SIGNAL_POWER_TAGGER_CC_H
#define INCLUDED_LSA_SIGNAL_POWER_TAGGER_CC_H

#include <lsa/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace lsa {

    /*!
     * \brief <+description of block+>
     * \ingroup lsa
     *
     */
    class LSA_API signal_power_tagger_cc : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<signal_power_tagger_cc> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of lsa::signal_power_tagger_cc.
       *
       * To avoid accidental use of raw pointers, lsa::signal_power_tagger_cc's
       * constructor is in a private implementation
       * class. lsa::signal_power_tagger_cc::make is the public interface for
       * creating new instances.
       */
      static sptr make(int calc_len,int tag);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_SIGNAL_POWER_TAGGER_CC_H */
