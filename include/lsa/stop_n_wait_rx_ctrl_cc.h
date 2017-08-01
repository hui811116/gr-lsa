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


#ifndef INCLUDED_LSA_STOP_N_WAIT_RX_CTRL_CC_H
#define INCLUDED_LSA_STOP_N_WAIT_RX_CTRL_CC_H

#include <lsa/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace lsa {

    /*!
     * \brief <+description of block+>
     * \ingroup lsa
     *
     */
    class LSA_API stop_n_wait_rx_ctrl_cc : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<stop_n_wait_rx_ctrl_cc> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of lsa::stop_n_wait_rx_ctrl_cc.
       *
       * To avoid accidental use of raw pointers, lsa::stop_n_wait_rx_ctrl_cc's
       * constructor is in a private implementation
       * class. lsa::stop_n_wait_rx_ctrl_cc::make is the public interface for
       * creating new instances.
       */
      static sptr make(float ed_thres,float period,const std::vector<gr_complex>& samples);
      virtual void set_ed_threshold(float thres)=0;
      virtual float ed_threshold()const=0;
      
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_STOP_N_WAIT_RX_CTRL_CC_H */

