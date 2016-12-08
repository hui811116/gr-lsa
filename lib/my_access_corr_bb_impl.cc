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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "my_access_corr_bb_impl.h"

namespace gr {
  namespace lsa {

    my_access_corr_bb::sptr
    my_access_corr_bb::make(const std::string& access_code, int threshold)
    {
      return gnuradio::get_initial_sptr
        (new my_access_corr_bb_impl(access_code, threshold));
    }

    /*
     * The private constructor
     */
    my_access_corr_bb_impl::my_access_corr_bb_impl(const std::string& access_code, unsigned int threshold)
      : gr::block("my_access_corr_bb",
              gr::io_signature::make(1, 1, sizeof(char)),
              gr::io_signature::make(2, 2, sizeof(char)))
    {
      set_access_code(access_code);
      set_threshold(threshold);
      set_tag_propagation_policy(TPP_DONT);
    }

    /*
     * Our virtual destructor.
     */
    my_access_corr_bb_impl::~my_access_corr_bb_impl()
    {
    }

    void
    my_access_corr_bb_impl::set_access_code(const std::string& access_code)
    {
      d_access_code.clear();
      for(int i=0;i<access_code.length();++i){
          d_access_code.push_back((access_code[i]!='0')? 0x01:0x00);
      }
    }

    void
    my_access_corr_bb_impl::set_threshold(unsigned int threshold)
    {
      d_threshold=threshold;
    }

    unsigned int
    my_access_corr_bb_impl::get_threshold()const
    {
      return d_threshold;
    }

    std::string
    my_access_corr_bb_impl::get_access_code()const
    {
      std::string temp;
      for(int i=0;i<d_access_code.size();++i){
        temp+=d_access_code[i];
      }
    }

    /*void
    my_access_corr_bb_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      
    }*/

    int
    my_access_corr_bb_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      int nin = ninput_items[0];
      int ns= d_access_code.size();
      const unsigned char *in = (const unsigned char *) input_items[0];
      unsigned char *out = (unsigned char *) output_items[0];
      unsigned char *corr;
      unsigned char corr_mem[nin-ns+1];
      if(output_items.size()>1)
        corr=(unsigned char *) output_items[1];

      memcpy(out,in,sizeof(unsigned char)*nin);
      
      int count;
      for(int i=0;i<nin-ns+1;++i){
        count=0;
        for(int j=0;j<ns;++j){
          count+= (in[i+j]==d_access_code[j])? 1:0;
        }
        corr_mem[i]=count;
        if(output_items.size()>1)
          corr[i]=count;
        if(count>=d_threshold){
          add_item_tag(0, nitems_written(0) + i, pmt::intern("count"),
                       pmt::from_long(count), pmt::intern(alias()));
          if(output_items.size()>1)
            add_item_tag(1,nitems_written(1) + i, pmt::intern("count"),pmt::from_long(count),pmt::intern(alias()));
        }
      }
      // Do <+signal processing+>
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (nin);
      produce(0,ninput_items[0]);
      produce(1,nin-ns+1);
      // Tell runtime system how many output items we produced.
      return WORK_CALLED_PRODUCE;
    }

  } /* namespace lsa */
} /* namespace gr */

