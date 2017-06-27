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
#include "byte_to_symbol_bc_impl.h"

namespace gr {
  namespace lsa {

    
    #define CHIPRATE 8
    #define SYMBITS 4
    typedef std::complex<float> CPX;
    static const int d_rate = CHIPRATE*SYMBITS;
    static const int d_symbol_size = 16;
    static const CPX d_map[][16] = {
{CPX(1,1),CPX(-1,1),CPX(1,-1),CPX(-1,1),CPX(1,1),CPX(1,1),CPX(1,1),CPX(1,1),CPX(-1,1),CPX(-1,1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,-1)},
{CPX(1,1),CPX(1,-1),CPX(1,1),CPX(-1,1),CPX(1,-1),CPX(-1,1),CPX(1,1),CPX(1,1),CPX(1,1),CPX(1,1),CPX(-1,1),CPX(-1,1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,-1)},
{CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(-1,1),CPX(1,-1),CPX(-1,1),CPX(1,1),CPX(1,1),CPX(1,1),CPX(1,1),CPX(-1,1),CPX(-1,1),CPX(1,1),CPX(1,-1)},
{CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(-1,1),CPX(1,-1),CPX(-1,1),CPX(1,1),CPX(1,1),CPX(1,1),CPX(1,1),CPX(-1,1),CPX(-1,1)},
{CPX(-1,1),CPX(-1,1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(-1,1),CPX(1,-1),CPX(-1,1),CPX(1,1),CPX(1,1),CPX(1,1),CPX(1,1)},
{CPX(1,1),CPX(1,1),CPX(-1,1),CPX(-1,1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(-1,1),CPX(1,-1),CPX(-1,1),CPX(1,1),CPX(1,1)},
{CPX(1,1),CPX(1,1),CPX(1,1),CPX(1,1),CPX(-1,1),CPX(-1,1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(-1,1),CPX(1,-1),CPX(-1,1)},
{CPX(1,-1),CPX(-1,1),CPX(1,1),CPX(1,1),CPX(1,1),CPX(1,1),CPX(-1,1),CPX(-1,1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(-1,1)},
{CPX(1,-1),CPX(1,1),CPX(1,1),CPX(1,1),CPX(1,-1),CPX(-1,1),CPX(-1,1),CPX(1,-1),CPX(1,1),CPX(1,1),CPX(-1,1),CPX(1,1),CPX(-1,1),CPX(1,1),CPX(1,-1),CPX(1,1)},
{CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,1),CPX(1,1),CPX(1,-1),CPX(-1,1),CPX(-1,1),CPX(1,-1),CPX(1,1),CPX(1,1),CPX(-1,1),CPX(1,1),CPX(-1,1),CPX(1,1)},
{CPX(-1,1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,1),CPX(1,1),CPX(1,-1),CPX(-1,1),CPX(-1,1),CPX(1,-1),CPX(1,1),CPX(1,1),CPX(-1,1),CPX(1,1)},
{CPX(-1,1),CPX(1,1),CPX(-1,1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,1),CPX(1,1),CPX(1,-1),CPX(-1,1),CPX(-1,1),CPX(1,-1),CPX(1,1),CPX(1,1)},
{CPX(1,1),CPX(1,1),CPX(-1,1),CPX(1,1),CPX(-1,1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,1),CPX(1,1),CPX(1,-1),CPX(-1,1),CPX(-1,1),CPX(1,-1)},
{CPX(-1,1),CPX(1,-1),CPX(1,1),CPX(1,1),CPX(-1,1),CPX(1,1),CPX(-1,1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,1),CPX(1,1),CPX(1,-1),CPX(-1,1)},
{CPX(1,-1),CPX(-1,1),CPX(-1,1),CPX(1,-1),CPX(1,1),CPX(1,1),CPX(-1,1),CPX(1,1),CPX(-1,1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,1),CPX(1,1)},
{CPX(1,1),CPX(1,1),CPX(1,-1),CPX(-1,1),CPX(-1,1),CPX(1,-1),CPX(1,1),CPX(1,1),CPX(-1,1),CPX(1,1),CPX(-1,1),CPX(1,1),CPX(1,-1),CPX(1,1),CPX(1,-1),CPX(1,1)} 
    };

    byte_to_symbol_bc::sptr
    byte_to_symbol_bc::make()
    {
      return gnuradio::get_initial_sptr
        (new byte_to_symbol_bc_impl());
    }

    /*
     * The private constructor
     */
    byte_to_symbol_bc_impl::byte_to_symbol_bc_impl()
      : gr::block("byte_to_symbol_bc",
              gr::io_signature::make(1, 1, sizeof(unsigned char)),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
              d_src_id(pmt::intern(alias()))
    {
      set_min_noutput_items(d_rate);
      set_tag_propagation_policy(TPP_DONT);
    }

    /*
     * Our virtual destructor.
     */
    byte_to_symbol_bc_impl::~byte_to_symbol_bc_impl()
    {
    }

    void
    byte_to_symbol_bc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = noutput_items/(d_rate);
    }

    int
    byte_to_symbol_bc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const unsigned char *in = (const unsigned char *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];

      int nin = ninput_items[0];
      int nout = noutput_items/(d_rate);
      int count = std::min(nin,nout);
      std::vector<tag_t> tags;
      nout*= (d_rate);
      for(int i=0;i<count;++i){
        int s1 = in[i]>>4 & 0x0f;
        int s2 = in[i]& 0x0f;
        memcpy(out+i*d_rate,d_map[s1],sizeof(gr_complex)*d_symbol_size);
        memcpy(out+i*d_rate+d_symbol_size,d_map[s2],sizeof(gr_complex)*d_symbol_size);
      }
      if(count!=0){
        get_tags_in_window(tags,0,0,count);
      }
      for(int i=0;i<tags.size();++i)
      {
        int offset = tags[i].offset - nitems_read(0);
        add_item_tag(0,nitems_written(0)+offset*d_rate,tags[i].key,tags[i].value,d_src_id);
      }
      consume_each (count);
      return nout;
    }

  } /* namespace lsa */
} /* namespace gr */

