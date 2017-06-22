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
#include "sfd_tagger_cc_impl.h"

namespace gr {
  namespace lsa {

    #define d_debug false
    #define DEBUG d_debug && std::cout
    
    static const pmt::pmt_t d_cfo_tag = pmt::intern("cfo_est");
    static const pmt::pmt_t d_phase_tag=pmt::intern("phase_est");
    static const pmt::pmt_t d_corr_tag = pmt::intern("corr_val");
    static const pmt::pmt_t d_sfd_tag = pmt::intern("sfd_est");
    sfd_tagger_cc::sptr
    sfd_tagger_cc::make(int explen)
    {
      return gnuradio::get_initial_sptr
        (new sfd_tagger_cc_impl(explen));
    }

    /*
     * The private constructor
     */
    sfd_tagger_cc_impl::sfd_tagger_cc_impl(int explen)
      : gr::block("sfd_tagger_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(gr_complex)))
    {
      if(explen<=0){
        throw std::invalid_argument("Expected length should be greater than 0");
      }
      d_explen = explen;
      d_state = false;
      set_tag_propagation_policy(TPP_DONT);
    }

    /*
     * Our virtual destructor.
     */
    sfd_tagger_cc_impl::~sfd_tagger_cc_impl()
    {
    }

    void
    sfd_tagger_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      if(!d_state){
        ninput_items_required[0] = noutput_items;
      }else{
        ninput_items_required[0] = std::max(noutput_items,d_explen);
      }
    }

    int
    sfd_tagger_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
      gr_complex corrval, eng, corr_norm;
      int nin = std::min(noutput_items,ninput_items[0]);
      int count =0;
      int nout =0;
      std::vector<tag_t> tags, pub_tags, phase_tags, corr_tags;
      get_tags_in_window(tags,0,0,nin,d_cfo_tag);
      if(!d_state){
        if(!tags.empty()){
          int offset = tags[0].offset - nitems_read(0);
          count = offset;
          d_state = true;
          nout = offset;
          DEBUG<<"\033[33;1m"<<"<Phase Correct> found a preamble candidate, change state to find SFD"<<"\033[0m"<<std::endl;
        }else{
          count = nin;
          nout = nin;
        }
        memcpy(out,in,sizeof(gr_complex)*nout);
      }else{
        // phase tracking state
        // guarantee noutput_items is enough so that tagging will not cause problem
        if(noutput_items<d_explen || ninput_items[0]<d_explen){
          consume_each(0);
          return 0;
        }else{
          // valid
          // only consume expected length and find
          get_tags_in_window(phase_tags,0,0,d_explen,d_phase_tag);
          get_tags_in_window(corr_tags,0,0,d_explen,d_corr_tag);
          assert(phase_tags.size()==corr_tags.size());
          if(!corr_tags.empty()){
            float max_corr = 0;
            int max_idx = 0;
            for(int i=0;i<corr_tags.size();++i){
              float temp_corr = pmt::to_float(corr_tags[i].value);
              if(temp_corr>max_corr){
                max_corr = temp_corr;
                max_idx = i;
              }
            }
            // find the maximum corr_val
            int offset = corr_tags[max_idx].offset - nitems_read(0);
            add_item_tag(0,nitems_written(0)+offset,d_sfd_tag,corr_tags[max_idx].value);
            add_item_tag(0,nitems_written(0)+offset,phase_tags[max_idx].key,phase_tags[max_idx].value);
            DEBUG<<"\033[33;1m"<<"<SFD Tagger>found a phase tag by cross-correlation with SFD"<<"\033[0m"<<std::endl;
          }else{
            DEBUG<<"\033[33;1m"<<"<SFD Tagger>No phase tag found within expected length"<<"\033[0m"<<std::endl;
          }
          d_state = false;
          // rotate expected length
          memcpy(out,in,sizeof(gr_complex)*d_explen);
          nout = d_explen;
          count = d_explen;
        }
      }
      // tags handling
      if(count!=0){
        get_tags_in_window(pub_tags,0,0,count);
        for(int i=0;i<pub_tags.size();++i){
          int offset = pub_tags[i].offset - nitems_read(0);
          if(offset<nout){
            // FIXME: exclude cfo tags, phase_est, corr_val
            if(!pmt::eqv(d_cfo_tag,pub_tags[i].key) 
                && !pmt::eqv(d_phase_tag,pub_tags[i].key)
                && !pmt::eqv(d_corr_tag,pub_tags[i].key))
              add_item_tag(0,nitems_written(0)+offset,pub_tags[i].key,pub_tags[i].value);
          }else{
            break;
          }
        }
      }
      consume_each (count);
      return nout;
    }

  } /* namespace lsa */
} /* namespace gr */

