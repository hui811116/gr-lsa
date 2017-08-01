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


#ifndef INCLUDED_LSA_DUMP_TX_H
#define INCLUDED_LSA_DUMP_TX_H

#include <lsa/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace lsa {

    /*!
     * \brief <+description+>
     *
     */
    class LSA_API dump_tx : virtual public block
    {
      public:
        typedef boost::shared_ptr<dump_tx> sptr;
        static sptr make(const std::string& filename,int avg_size, float timeout, bool verb);

        virtual void set_avg_size(int avg_size)=0;
        virtual int avg_size()const=0;
        virtual void set_timeout(float timeout)=0;
        virtual float timeout()const=0;
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_DUMP_TX_H */

