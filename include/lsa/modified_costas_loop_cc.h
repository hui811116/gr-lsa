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


#ifndef INCLUDED_LSA_MODIFIED_COSTAS_LOOP_CC_H
#define INCLUDED_LSA_MODIFIED_COSTAS_LOOP_CC_H

#include <lsa/api.h>
#include <gnuradio/blocks/control_loop.h>
#include <gnuradio/block.h>

namespace gr {
  namespace lsa {

    /*!
     * \brief <+description of block+>
     * \ingroup lsa
     *
     */
    class LSA_API modified_costas_loop_cc :
     virtual public gr::block,
     virtual public blocks::control_loop
    {
     public:
      typedef boost::shared_ptr<modified_costas_loop_cc> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of lsa::modified_costas_loop_cc.
       *
       * To avoid accidental use of raw pointers, lsa::modified_costas_loop_cc's
       * constructor is in a private implementation
       * class. lsa::modified_costas_loop_cc::make is the public interface for
       * creating new instances.
       */
      static sptr make(float loop_bw, int order, bool use_snr=false);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_MODIFIED_COSTAS_LOOP_CC_H */

