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
#include "correlate_sync_cc_impl.h"
#include <volk/volk.h>
#include <gnuradio/math.h>

namespace gr {
  namespace lsa {

    static int MINGAP = (64+12*8*8)/2*4;

    correlate_sync_cc::sptr
    correlate_sync_cc::make(
      const std::vector<gr_complex>& samples,
      float threshold)
    {
      return gnuradio::get_initial_sptr
        (new correlate_sync_cc_impl(samples,
      threshold));
    }

    /*
     * The private constructor
     */
    correlate_sync_cc_impl::correlate_sync_cc_impl(
      const std::vector<gr_complex>& samples,
      float threshold)
      : gr::block("correlate_sync_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make2(1, 2, sizeof(gr_complex), sizeof(gr_complex))),
              d_cap(24*1024)
    {
      set_max_noutput_items(d_cap);
      // Create time-reversed conjugate of symbols
      if(samples.size()==0){
        throw std::runtime_error("Empty preamble, abort");
      }
      d_samples = samples;
      // calculate samples energy
      volk_32fc_x2_conjugate_dot_prod_32fc(&d_samples_eng, samples.data(), samples.data(), samples.size());
      set_threshold(threshold); 
      set_history(samples.size());
      set_tag_propagation_policy(TPP_DONT);
    }

    /*
     * Our virtual destructor.
     */
    correlate_sync_cc_impl::~correlate_sync_cc_impl()
    {
    }

    void
    correlate_sync_cc_impl::set_threshold(float thres)
    {
      if(thres>1 || thres <0){
        throw std::runtime_error("Threshold should be normalized to [0,1]");
      }
      d_threshold = thres;
    }
    float
    correlate_sync_cc_impl::threshold()const
    {
      return d_threshold;
    }


    void
    correlate_sync_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      for(int i=0;i<ninput_items_required.size();++i){
        ninput_items_required[i] = noutput_items + history();
      }
    }
    

    int
    correlate_sync_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
      gr_complex *corr = NULL;
      int nin = std::min(std::max(ninput_items[0]-(int)history(),0), noutput_items);
      int count =0;
      int nout =nin;
      bool have_corr = (output_items.size()>=2);
      if(have_corr){
        corr = (gr_complex*) output_items[1];
      }
      if(nin==0){
        consume_each(0);
        return 0;
      }
      memcpy(out,in,sizeof(gr_complex)* nin);
      // calculate cross correlation
      gr_complex corrval;
      gr_complex eng;
      gr_complex corr_norm;
      float phase;
      std::vector<tag_t> tags;
      get_tags_in_window(tags,0,0,nin);
      for(count=0;count<nin;++count){
        volk_32fc_x2_conjugate_dot_prod_32fc(&corrval, in+count, d_samples.data(), d_samples.size());
        volk_32fc_x2_conjugate_dot_prod_32fc(&eng, in+count, in+count, d_samples.size());
        // To prevent overflow
        corr_norm = corrval / (std::sqrt(eng*d_samples_eng)+gr_complex(1e-6,0));
        if(have_corr){
          corr[count] = corr_norm;
        }
        if(std::abs(corr_norm)>=d_threshold){
          // detect a possible preamble
          phase = fast_atan2f(corrval.imag(),corrval.real());
          add_item_tag(0,nitems_written(0)+count,pmt::intern("phase_est"),pmt::from_float(phase));
          add_item_tag(0,nitems_written(0)+count,pmt::intern("corr_val"),pmt::from_float(std::abs(corr_norm)));
          if(have_corr){
            add_item_tag(1,nitems_written(1)+count,pmt::intern("phase_est"),pmt::from_float(phase));
          }
        }
      }
      for(int i=0;i<tags.size();++i){
        int offset = tags[i].offset - nitems_read(0);
        add_item_tag(0,nitems_written(0)+offset,tags[i].key,tags[i].value);
      }
      consume_each (count);
      return nout;
    }

  } /* namespace lsa */
} /* namespace gr */

