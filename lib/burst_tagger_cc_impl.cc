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

    #define d_debug true
    #define DEBUG d_debug && std::cout

    burst_tagger_cc::sptr
    burst_tagger_cc::make(const std::string& tagname, int mult)
    {
      return gnuradio::get_initial_sptr
        (new burst_tagger_cc_impl(tagname,mult));
    }

    /*
     * The private constructor
     */
    burst_tagger_cc_impl::burst_tagger_cc_impl(const std::string& tagname,int mult)
      : gr::sync_block("burst_tagger_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(gr_complex)))
    {
      d_tagname = pmt::string_to_symbol(tagname);
      d_count =0;
      d_mult = mult;
      std::stringstream ss;
      ss<<name()<<unique_id();
      d_id = pmt::string_to_symbol(ss.str());
      set_tag_propagation_policy(TPP_DONT);
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
      std::sort(tags.begin(),tags.end(),tag_t::offset_compare);
      
      if(!tags.empty()){
        int offset = tags[0].offset-nitems_read(0);
        //DEBUG<<"Found a tag at idx="<<next_tag_idx<<" ,d_count="<<d_count<<std::endl;
        if(d_count==offset){
          if(offset==0){
            add_sob(nitems_written(0));  
            d_count = pmt::to_long(tags[0].value)*d_mult;
          }else{
            memcpy(out,in,sizeof(gr_complex)*(offset+1));
            d_count+=(pmt::to_long(tags[0].value)*d_mult);
            d_count-=(offset+1);
            if(d_count<=0){
              add_eob(nitems_written(0)+offset);
              d_count=0;
            }
            return offset+1;
          }
        }else{
          if(offset>0){
            add_eob(offset-1);
          }
          d_count=0;
          memcpy(out,in,sizeof(gr_complex)*offset);
          return offset;
        }
        int nout = std::min((long int)noutput_items,d_count);
        if(nout!=0 && nout==d_count){
          add_eob(nitems_written(0)+nout-1);
          //DEBUG<<"add eob abs:"<<nitems_written(0)+nout-1<<std::endl;
        }
        d_count-=nout;
        memcpy(out,in,sizeof(gr_complex)*nout);
        return nout;
      }else{
        // empty tags
        if(d_count){
          noutput_items = std::min(d_count,(long int)noutput_items);
          d_count-=noutput_items;
          if(d_count==0){
            add_eob(nitems_written(0)+noutput_items-1);
          }
        }
        memcpy(out,in,sizeof(gr_complex)*noutput_items);
        return noutput_items;
      }
    }

  } /* namespace lsa */
} /* namespace gr */

