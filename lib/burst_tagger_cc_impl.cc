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
#include "burst_tagger_cc_impl.h"

namespace gr {
  namespace lsa {

    burst_tagger_cc::sptr
    burst_tagger_cc::make(const std::string& tagname,int mult)
    {
      return gnuradio::get_initial_sptr
        (new burst_tagger_cc_impl(tagname,mult));
    }

    /*
     * The private constructor
     */
    burst_tagger_cc_impl::burst_tagger_cc_impl(const std::string& tagname,int mult)
      : gr::block("burst_tagger_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
              d_sob_tag(pmt::intern("tx_sob")),
              d_eob_tag(pmt::intern("tx_eob")),
              d_tagname(pmt::intern(tagname)),
              d_src_id(pmt::intern(alias()))
    {
      if(mult<=0){
        throw std::invalid_argument("Multiplier should be positive");
      }
      d_mult = mult;
      d_count=0;
      set_tag_propagation_policy(TPP_DONT);
    }

    /*
     * Our virtual destructor.
     */
    burst_tagger_cc_impl::~burst_tagger_cc_impl()
    {
    }

    void
    burst_tagger_cc_impl::add_sob(int offset)
    {
      add_item_tag(0,nitems_written(0)+offset,d_sob_tag,pmt::PMT_T,d_src_id);
    }
    void
    burst_tagger_cc_impl::add_eob(int offset)
    {
      if(offset<0){
        return;
      }
      add_item_tag(0,nitems_written(0)+offset,d_eob_tag,pmt::PMT_T,d_src_id);
    }
    void
    burst_tagger_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = noutput_items;
    }

    int
    burst_tagger_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
      std::vector<tag_t> tags;
      int nin = std::min(noutput_items,ninput_items[0]);
      if(nin==0){
        consume_each(0);
        return 0;
      }
      if(d_count==0){
        get_tags_in_window(tags,0,0,nin,d_tagname);
        if(!tags.empty()){
          int offset = tags[0].offset-nitems_read(0);
          if(offset==0){
            add_sob(0);
            d_count = pmt::to_long(tags[0].value)*d_mult;
            //DEBUG<<"update count to:"<<d_count<<" found tag at "<<nitems_read(0)<<std::endl;
          }else{
            consume_each(offset);
            return 0;
          }
        }
      }
      if(d_count){
        int nout = std::min(d_count,nin);
        memcpy(out,in,sizeof(gr_complex)*nout);
        d_count-=nout;
        if(d_count==0){
          //DEBUG<<"output complete, add end tag at:"<<nitems_written(0)+nout-1<<std::endl;
          add_eob(nout-1);
        }
        consume_each (nout);
        return nout;
      }else{
        consume_each(nin);
        return 0;
      }
    }

  } /* namespace lsa */
} /* namespace gr */

