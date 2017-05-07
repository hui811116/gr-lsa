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
#include "symbol_corrector_cc_impl.h"
#include <gnuradio/expj.h>

namespace gr {
  namespace lsa {

    symbol_corrector_cc::sptr
    symbol_corrector_cc::make()
    {
      return gnuradio::get_initial_sptr
        (new symbol_corrector_cc_impl());
    }

    /*
     * The private constructor
     */
    symbol_corrector_cc_impl::symbol_corrector_cc_impl()
      : gr::sync_block("symbol_corrector_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
              d_update_len(256)
    {
      d_phase = 0.0;
      d_amp =1.0;

      d_len_count = 0;
      d_max_corr=0.0;

      d_trigger = false;

      set_tag_propagation_policy(TPP_DONT);
    }

    /*
     * Our virtual destructor.
     */
    symbol_corrector_cc_impl::~symbol_corrector_cc_impl()
    {
    }

    int
    symbol_corrector_cc_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];

      std::vector<tag_t> phase_tags, corr_tags;
      get_tags_in_range(phase_tags, 0,nitems_read(0),nitems_read(0)+noutput_items,pmt::intern("phase_est"));
      //get_tags_in_range(amp_tags,0,nitems_read(0),nitems_read(0)+noutput_items,pmt::intern("amp_est"));
      get_tags_in_range(corr_tags, 0,nitems_read(0),nitems_read(0)+noutput_items,pmt::intern("corr_est"));
      assert(phase_tags.size() == corr_tags.size());
      for(int i=0;i<noutput_items;++i){
        if(!phase_tags.empty()){
          int offset = phase_tags[0].offset - nitems_read(0);
          if(offset == i){
            if(!d_trigger){
              d_trigger = true;
              d_len_count = d_update_len;
              d_max_corr = pmt::to_float(corr_tags[0].value);
              d_phase = pmt::to_float(phase_tags[0].value);
            }
            else{
              float tmp_corr = pmt::to_float(corr_tags[0].value);
              if(tmp_corr>d_max_corr){
                d_phase = pmt::to_float(phase_tags[0].value);
                d_max_corr = pmt::to_float(corr_tags[0].value);
              }
            }
            //d_phase = pmt::to_float(phase_tags[0].value);
            phase_tags.erase(phase_tags.begin());
            corr_tags.erase(corr_tags.begin());
          }  
        }

        d_len_count = (d_len_count==0) ? 0 : d_len_count-1;    
        d_trigger = (d_len_count==0)?false : true;
        
        out[i] = in[i] * gr_expj(-d_phase);
      }

      return noutput_items;
    }

  } /* namespace lsa */
} /* namespace gr */

