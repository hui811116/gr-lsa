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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "chip_mapper_bb_impl.h"

namespace gr {
  namespace lsa {

static const unsigned int CHIPSET[16] = {3653456430,
                                  3986437410,
                                  786023250,
                                  585997365,
                                  1378802115,
                                  891481500,
                                  3276943065,
                                  2620728045,
                                  2358642555,
                                  3100205175,
                                  2072811015,
                                  2008598880,
                                  125537430,
                                  1618458825,
                                  2517072780,
                                  3378542520};

    chip_mapper_bb::sptr
    chip_mapper_bb::make()
    {
      return gnuradio::get_initial_sptr
        (new chip_mapper_bb_impl());
    }

    /*
     * The private constructor
     */
    chip_mapper_bb_impl::chip_mapper_bb_impl()
      : gr::block("chip_mapper_bb",
              gr::io_signature::make(1, 1, sizeof(char)),
              gr::io_signature::make(1, 1, sizeof(char)))
    {
      set_relative_rate(8); // coded
    }

    /*
     * Our virtual destructor.
     */
    chip_mapper_bb_impl::~chip_mapper_bb_impl()
    {
    }

    void
    chip_mapper_bb_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      ninput_items_required[0] = noutput_items/relative_rate();
    }

    int
    chip_mapper_bb_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const unsigned char *in = (const unsigned char *) input_items[0];
      unsigned char *out = (unsigned char *) output_items[0];

      int nin = (ninput_items[0]*relative_rate()<=noutput_items)? nin : noutput_items/relative_rate();
      int nout= nin * relative_rate();
      //assert(nin<=noutput_items/8);
      //if(nin!=0)
      //std::cout<<"nin:"<<nin<<" ,nout:"<<noutput_items<<std::endl;
      for(int i=0;i<nin;++i){
        int s1= (in[i]>>4) & 0x0f;
        int s2= (in[i]) & 0x0f;
        const uint8_t* s1_u8 =(const uint8_t*) &CHIPSET[s1];
        const uint8_t* s2_u8 =(const uint8_t*) &CHIPSET[s2];
        out[8*i] = s1_u8[3];
        out[8*i+1]=s1_u8[2];
        out[8*i+2]=s1_u8[1];
        out[8*i+3]=s1_u8[0];

        out[8*i+4]=s2_u8[3];
        out[8*i+5]=s2_u8[2];
        out[8*i+6]=s2_u8[1];
        out[8*i+7]=s2_u8[0];
      }
      

      consume_each (nin);
      return nout;
    }

  } /* namespace lsa */
} /* namespace gr */

