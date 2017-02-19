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
#include "poly_costas_sync_cc_impl.h"

#include <gnuradio/math.h>
#include <gnuradio/expj.h>
#include <gnuradio/sincos.h>

#include <cstdio>
#include <cmath>
#include <algorithm>
#include <sstream>

namespace gr {
  namespace lsa {

    poly_costas_sync_cc::sptr
    poly_costas_sync_cc::make(
      double plf_sps,
      float plf_loop_bw,
      const std::vector<float> &plf_taps,
      int plf_nfilts,
      float costas_loop_bw,
      int costas_order,
      const std::string& sense_tagname
      )
    {
      return gnuradio::get_initial_sptr
        (new poly_costas_sync_cc_impl(
          plf_sps,
          plf_loop_bw,
          plf_taps,
          plf_nfilts,
          costas_loop_bw,
          costas_order,
          sense_tagname));
    }

    /*
     * The private constructor
     */
    poly_costas_sync_cc_impl::poly_costas_sync_cc_impl(
      double plf_sps,
      float plf_loop_bw,
      const std::vector<float>& plf_taps,
      int plf_nfilts,
      float costas_loop_bw,
      int costas_order,
      const std::string& sense_tagname)
      : gr::block("poly_costas_sync_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
      blocks::control_loop(costas_loop_bw,1.0,-1.0)
    {
      if(plf_taps.empty()){
        throw std::runtime_error("ProU RX: Polyphase constructor taps error");
      }
      set_tag_propagation_policy(TPP_DONT);

      d_error = new float[8192];
      d_time_sync_symbol = new gr_complex[8192];

      d_plf_nfilts = plf_nfilts;
      d_plf_sps = floor(plf_sps);
      d_plf_damping = 2*d_plf_nfilts;
      d_plf_loop_bw = plf_loop_bw;
      d_plf_max_dev = 1.5;
      d_plf_osps = 1;
      d_plf_k = plf_nfilts/2.0;
      d_plf_rate = (d_plf_sps-floor(d_plf_sps))*(double)d_plf_nfilts;
      d_plf_rate_i = (int) floor(d_plf_rate);
      d_plf_rate_f = d_plf_rate - (float)d_plf_rate_i;
      d_plf_filtnum = (int)floor(d_plf_k);

      d_plf_filters = std::vector<gr::filter::kernel::fir_filter_ccf*>(d_plf_nfilts);
      d_plf_diff_filters = std::vector<gr::filter::kernel::fir_filter_ccf*>(d_plf_nfilts);

      std::vector<float> vtaps(1,0);
      for(int i=0;i<d_plf_nfilts;++i){
        d_plf_filters[i] = new gr::filter::kernel::fir_filter_ccf(1,vtaps);
        d_plf_diff_filters[i] = new gr::filter::kernel::fir_filter_ccf(1,vtaps);
      }

      std::vector<float> dtaps;
      plf_create_diff_taps(plf_taps,dtaps);
      plf_set_taps(plf_taps, d_plf_taps, d_plf_filters);
      plf_set_taps(dtaps, d_plf_dtaps, d_plf_diff_filters);

      float denum = (1.0+2.0*d_plf_damping*d_plf_loop_bw+d_plf_loop_bw*d_plf_loop_bw);
      d_plf_alpha = (4*d_plf_damping*d_plf_loop_bw) / denum;
      d_plf_beta = (4*d_plf_loop_bw*d_plf_loop_bw)/denum;
      d_plf_out_idx = 0;
      d_plf_error =0;
      set_relative_rate((float)d_plf_osps/(float)d_plf_sps);
      //Costas loop setup
      d_costas_order = costas_order;
      d_costas_error = 0;
      d_costas_noise = 1.0;
      switch(d_costas_order)
      {
        case 2:
          d_costas_phase_detector = &poly_costas_sync_cc_impl::costas_phase_detector_2;
        break;
        case 4:
          d_costas_phase_detector = &poly_costas_sync_cc_impl::costas_phase_detector_4;
        break;
        case 8:
          d_costas_phase_detector = &poly_costas_sync_cc_impl::costas_phase_detector_8;
        break;
        default:
          std::runtime_error("SYNC:invalid order, should be 2,4 or 8");
        break;
      }
      d_sensing_tag = pmt::string_to_symbol(sense_tagname);
      d_sensing_state = false;
    }

    /*
     * Our virtual destructor.
     */
    poly_costas_sync_cc_impl::~poly_costas_sync_cc_impl()
    {
      delete [] d_error;
      delete [] d_time_sync_symbol;
      for(int i=0;i<d_plf_nfilts;++i){
        delete d_plf_filters[i];
        delete d_plf_diff_filters[i];
      }

    }

    //POLYPHASE CLOCK SYNCHRONIZATION

    void
    poly_costas_sync_cc_impl::plf_create_diff_taps(
      const std::vector<float>& newtaps,
      std::vector<float>& difftaps)
    {
      std::vector<float> diff_filter(3);
      diff_filter[0]=-1;
      diff_filter[1]=0;
      diff_filter[2]=1;
      float pwr=0;
      difftaps.clear();
      difftaps.push_back(0);
      for(unsigned int i=0;i<newtaps.size()-2;i++){
        float tap=0;
        for(unsigned int j=0;j<diff_filter.size();j++){
          tap += diff_filter[j]*newtaps[i+j];
        }
        difftaps.push_back(tap);
        pwr += fabsf(tap);
      }
      difftaps.push_back(0);
      for (unsigned int i=0;i<difftaps.size();++i){
        difftaps[i] *= d_plf_nfilts/pwr;
        if(difftaps[i]!= difftaps[i]){
          throw std::runtime_error("ProU RX::CreateDiffTaps Failed");
        }
      }
    }

    void
    poly_costas_sync_cc_impl::plf_set_taps(
      const std::vector<float>& newtaps,
      std::vector< std::vector<float> >& ourtaps,
      std::vector< gr::filter::kernel::fir_filter_ccf*> &ourfilter)
    {
      int i,j;
      unsigned int ntaps=newtaps.size();
      d_plf_taps_per_filter = (unsigned int) ceil((double)ntaps/(double)d_plf_nfilts);

      ourtaps.resize(d_plf_nfilts);

      std::vector<float> tmp_taps = newtaps;
      while((float)(tmp_taps.size()) < d_plf_nfilts*d_plf_taps_per_filter)
      {
        tmp_taps.push_back(0.0);
      }
      for(i=0;i<d_plf_nfilts;++i){
        ourtaps[i] = std::vector<float>(d_plf_taps_per_filter,0);
        for(j=0;j<d_plf_taps_per_filter;++j){
          ourtaps[i][j] = tmp_taps[i+j*d_plf_nfilts];
        }
        ourfilter[i]->set_taps(ourtaps[i]);
      }
      set_history(d_plf_taps_per_filter+d_plf_sps+d_plf_sps);
      set_output_multiple(d_plf_osps);
      //two flow output control lines not copied
    }

    int
    poly_costas_sync_cc_impl::plf_core(
      gr_complex* out,
      float* error,
      const gr_complex* in,
      int nsample,
      int& nconsume,
      std::vector<tag_t>& tags,
      std::vector<tag_t>& out_tags)
    {
      int i=0, count=0;
      float error_r,error_i;
      bool state = d_sensing_state;
      while(i<nsample){
        if(!tags.empty()){
          size_t offset = tags[0].offset - nitems_read(0);
          if( (offset >=(size_t)count) && (offset<(size_t)(count + d_plf_sps))){
            if(pmt::to_bool(tags[0].value) && (state==false)){
              d_prev_plf_k = d_plf_k;
              d_prev_plf_rate_f = d_plf_rate_f;
              tag_t tmp_tag;
              tmp_tag.offset = i;
              tmp_tag.value = pmt::from_bool(true);
              out_tags.push_back(tmp_tag);
              state = !state;
            }
            else if( !(pmt::to_bool(tags[0].value)) && (state==true)){
              d_plf_k = d_prev_plf_k;
              d_plf_rate_f = d_prev_plf_rate_f;
              tag_t tmp_tag;
              tmp_tag.offset = i;
              tmp_tag.value = pmt::from_bool(false);
              out_tags.push_back(tmp_tag);
              state = !state;
            }
            tags.erase(tags.begin());
          }
        }
        while(d_plf_out_idx<d_plf_osps){
          d_plf_filtnum = (int) floor(d_plf_k);
          while(d_plf_filtnum >= d_plf_nfilts){
            d_plf_k -= d_plf_nfilts;
            d_plf_filtnum -= d_plf_nfilts;
            count+=1;
          }
          while(d_plf_filtnum <0){
            d_plf_k+=d_plf_nfilts;
            d_plf_filtnum+=d_plf_nfilts;
            count-=1;
          }
          out[i+d_plf_out_idx] = d_plf_filters[d_plf_filtnum]->filter(&in[count+d_plf_out_idx]);
          d_plf_k = d_plf_k +d_plf_rate_i + d_plf_rate_f;
          d_plf_out_idx++;

          d_error[i] = d_plf_error;

          if(i+d_plf_out_idx>= nsample){
            nconsume = count;
            return i;
          }
        }
        d_plf_out_idx =0;
        gr_complex diff = d_plf_diff_filters[d_plf_filtnum]->filter(&in[count]);
        error_r = out[i].real() * diff.real();
        error_i = out[i].imag() * diff.imag();
        //d_plf_error = (error_i+error_r)/2.0;
        d_plf_error = (state? 0 : (error_i+error_r)/2.0);

        for(int s=0;s<d_plf_sps;s++){
          d_plf_rate_f = d_plf_rate_f + d_plf_beta* d_plf_error;//first order loop filter
          d_plf_k = d_plf_k + d_plf_rate_f + d_plf_alpha*d_plf_error; // second order loop filter
        }

        d_plf_rate_f = gr::branchless_clip(d_plf_rate_f, d_plf_max_dev);
        i+= d_plf_osps;
        count += (int)floor(d_plf_sps);
      }
      nconsume = count;
      return i;
    }

    float
    poly_costas_sync_cc_impl::costas_phase_detector_2(const gr_complex& sample) const
    {
      return (sample.real()*sample.imag());
    }
    float
    poly_costas_sync_cc_impl::costas_phase_detector_4(const gr_complex& sample) const
    {
      return ((sample.real()>0 ? 1.0 : -1.0) * sample.imag() -
        (sample.imag()>0 ? 1.0 : -1.0) * sample.real());
    }

    float
    poly_costas_sync_cc_impl::costas_phase_detector_8(const gr_complex& sample)const
    {
      float K = (sqrt(2.0) - 1);
      if(fabsf(sample.real()) >= fabsf(sample.imag())) {
  return ((sample.real()>0 ? 1.0 : -1.0) * sample.imag() -
    (sample.imag()>0 ? 1.0 : -1.0) * sample.real() * K);
      }
      else {
  return ((sample.real()>0 ? 1.0 : -1.0) * sample.imag() * K -
    (sample.imag()>0 ? 1.0 : -1.0) * sample.real());
      }
    }

    int
    poly_costas_sync_cc_impl::costas_core(
      gr_complex* out,
      float* error,
      const gr_complex* in,
      int nsample,
      std::vector<tag_t>& tags)
    {
      gr_complex nco_out;
      for(int i=0;i<nsample;++i)
      {
        if(!tags.empty()){
          if(tags[0].offset == (size_t)i){
            d_sensing_state = pmt::to_bool(tags[0].value);
            if(d_sensing_state){
              d_prev_cos_phase = d_phase;
              d_prev_cos_freq = d_freq;
            }
            else{
              d_phase = d_prev_cos_phase;
              d_freq = d_prev_cos_freq;
            }
            add_item_tag(0,nitems_written(0)+i,d_sensing_tag, pmt::from_bool(d_sensing_state));
            tags.erase(tags.begin());
          }
        }
        nco_out= gr_expj(-d_phase);
        out[i]=nco_out*in[i];
        d_costas_error = (*this.*d_costas_phase_detector)(out[i]);
        //d_costas_error = branchless_clip(d_costas_error,1.0);
        d_costas_error = (d_sensing_state? 0 : (branchless_clip(d_costas_error,1.0)));

        advance_loop(d_costas_error);
        phase_wrap();
        frequency_limit();

        error[i]=d_freq;
      }
      return nsample;
    }
    //main data flow 
    void
    poly_costas_sync_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      ninput_items_required[0] = (noutput_items + history()) * (d_plf_sps/d_plf_osps);
    }

    int
    poly_costas_sync_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];

      // Do <+signal processing+>
      int true_output=0;
      int true_consume=0;
      int costas_nout;
      std::vector<tag_t> plf_tags,tags;
      get_tags_in_window(plf_tags, 0,0,d_plf_sps*noutput_items, d_sensing_tag);
      
      true_output = plf_core(d_time_sync_symbol, d_error, in, noutput_items, true_consume, plf_tags, tags);
      costas_nout = costas_core(out,d_error, d_time_sync_symbol,true_output, tags);

      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (true_consume);

      // Tell runtime system how many output items we produced.
      return true_output;
    }

  } /* namespace lsa */
} /* namespace gr */

