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
    burst_tagger_cc::make(const std::string& tagname)
    {
      return gnuradio::get_initial_sptr
        (new burst_tagger_cc_impl(tagname));
    }

    /*
     * The private constructor
     */
    burst_tagger_cc_impl::burst_tagger_cc_impl(const std::string& tagname)
      : gr::sync_block("burst_tagger_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(gr_complex)))
    {
      d_tagname = pmt::string_to_symbol(tagname);
      d_count =0;
      //d_state = false;
      std::stringstream ss;
      ss<<name()<<unique_id();
      d_id = pmt::string_to_symbol(ss.str());
    }

    void
    burst_tagger_cc_impl::add_sob(const uint64_t& offset)
    {
      add_item_tag(0,offset,pmt::intern("tx_sob"),pmt::PMT_T,d_id);
    }

    void
    burst_tagger_cc_impl::add_eob(const uint64_t& offset)
    {
      add_item_tag(0,offset, pmt::intern("tx_eob"),pmt::PMT_T,d_id);
    }
    /*
     * Our virtual destructor.
     */
    burst_tagger_cc_impl::~burst_tagger_cc_impl()
    {
    }

    int
    burst_tagger_cc_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];

      std::vector<tag_t> tags;
      get_tags_in_range(tags,0,nitems_read(0),nitems_read(0)+noutput_items,d_tagname);
      memcpy(out,in,sizeof(gr_complex)*noutput_items);

      for(int i=0;i<noutput_items;++i){
        if(!tags.empty()){
          uint64_t offset = tags[0].offset - nitems_read(0);
          if(offset == i){
            add_sob(nitems_written(0)+i);
            d_count = pmt::to_uint64(tags[0].value);
            tags.erase(tags.begin());
          }
        }
        if(d_count>0){
          d_count--;
          if(d_count==0){
            add_eob(nitems_written(0)+i);
          }
        }
      }
      return noutput_items;
    }

  } /* namespace lsa */
} /* namespace gr */

