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


#ifndef INCLUDED_LSA_THROUGHPUT_FILE_SINK_H
#define INCLUDED_LSA_THROUGHPUT_FILE_SINK_H

#include <lsa/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace lsa {

    /*!
     * \brief <+description+>
     *
     */
    class LSA_API throughput_file_sink : virtual public block
    {
    public:
      typedef boost::shared_ptr<throughput_file_sink> sptr;
      static sptr make(const std::string& filename, int sys, bool verbose);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_THROUGHPUT_FILE_SINK_H */

