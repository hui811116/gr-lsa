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
#include <cmath>
#include <numeric>

namespace gr {
  namespace lsa {

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
              gr::io_signature::make2(1, 2, sizeof(gr_complex), sizeof(float)))
    {
      const size_t nitems = 24*1024;
      set_max_noutput_items(nitems);
      d_corr = (gr_complex *)volk_malloc(sizeof(gr_complex)*nitems, volk_get_alignment());
      d_corr_mag = (float *)volk_malloc(sizeof(float)*nitems, volk_get_alignment());
      d_eng = (float*) volk_malloc(sizeof(float)*nitems,volk_get_alignment());
      d_norm_corr = new float[nitems];
      // Create time-reversed conjugate of symbols
      d_samples = samples;
      float sample_eng = 0;
      for(size_t i=0; i < d_samples.size(); i++) {
          d_samples[i] = conj(d_samples[i]);
          sample_eng += norm(d_samples[i]);
      }
      if(sample_eng < 1e-10){
        throw std::runtime_error("correlation samples energy cannot be zero");
      }
      d_samples_eng = sample_eng;
      d_eng_log = logf(sample_eng);

      std::reverse(d_samples.begin(), d_samples.end());

      //set_mark_delay(mark_delay);
      set_threshold(threshold);

      // Correlation filter
      d_filter = new kernel::fft_filter_ccc(1, d_samples);
      set_history(d_filter->ntaps());
    }

    /*
     * Our virtual destructor.
     */
    correlate_sync_cc_impl::~correlate_sync_cc_impl()
    {
      delete d_filter;
      delete d_norm_corr;
      volk_free(d_corr);
      volk_free(d_corr_mag);
      volk_free(d_eng);
    }

    void
    correlate_sync_cc_impl::set_threshold(float thres)
    {
      if(thres>1 || thres <0){
        throw std::runtime_error("Threshold should be normalized to [0,1]");
      }
      d_threshold = thres;
      d_thres_log = logf(d_threshold);
    }


    void
    correlate_sync_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      for(int i=0;i<ninput_items_required.size();++i)
      {
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
      gr_complex *corr = d_corr;
      float* corr_mag = d_norm_corr;
      bool have_corr = (output_items.size()>=2);
      if(have_corr){
        corr_mag = (float*) output_items[1];
      }
      memcpy(out,in, sizeof(gr_complex)* noutput_items);
      
      d_filter->filter(noutput_items, in, corr);
      volk_32fc_magnitude_32f(corr_mag, corr, noutput_items);
      volk_32fc_magnitude_squared_32f(d_eng,in,noutput_items + history());
      
      float eng_acc=0.0;
      float detection =0.0;
      float phase;
      for(int i=0;i<noutput_items;++i){
        eng_acc=std::accumulate(d_eng+i,d_eng+i+d_samples.size()-1,0.0);
        if(2*logf(corr_mag[i])>(logf(eng_acc)+d_eng_log) ){
          //only noise present
          continue;
        }
        detection = logf(corr_mag[i]) - 0.5*logf(eng_acc);

        if(detection >= d_thres_log + 0.5*d_eng_log){
          phase = fast_atan2f(corr[i].imag(), corr[i].real());
          if(corr[i].real() < 0.0)
            phase += M_PI;

          add_item_tag(0,nitems_written(0)+i, pmt::intern("corr_est"),pmt::from_bool(pmt::PMT_T));
          add_item_tag(0,nitems_written(0)+i, pmt::intern("phase_est"),pmt::from_float(phase));
          if(have_corr){
            add_item_tag(1,nitems_written(1)+i,pmt::intern("corr_est"),pmt::from_bool(pmt::PMT_T));
            add_item_tag(1,nitems_written(1)+i, pmt::intern("phase_est"),pmt::from_float(phase));
          }
        }
        /*
        if(eng_acc < 1e-6){
          //GR_LOG_WARN(d_logger,"accumulated energy is zero, terminated");
          cor_mag[i] = 0.0;
        }
        else{
          cor_mag[i] = d_corr_mag[i]/sqrt(eng_acc)/sqrt(d_samples_eng);  
          //if(cor_mag[i] >1){
           // std::stringstream ss;
           // ss<<"iteration:"<<i<<" ,eng_acc:"<<eng_acc<<" ,d_sample_eng:"<<d_samples_eng<<std::endl;
           // ss<<"noutput_items:"<<noutput_items<<" ,input_items:"<<ninput_items[0]<<std::endl;
           // GR_LOG_DEBUG(d_logger,ss.str());
          //}
        }*/
      }
      
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (noutput_items);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace lsa */
} /* namespace gr */

