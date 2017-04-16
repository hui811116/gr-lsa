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
#include "burst_synchronizer_cc_impl.h"
#include <volk/volk.h>
#include <gnuradio/expj.h>
#include <gnuradio/math.h>
#include <math.h>
#include <algorithm>
#include <iterator>

namespace gr {
  namespace lsa {

#define TWO_PI (M_PI * 2.0f);

    enum burstStatus{
      FIND_BURST,
      LOCK_BURST,
      OUTPUT_BURST
    };

    burst_synchronizer_cc::sptr
    burst_synchronizer_cc::make(int min_len, int sps, const std::vector<float>& window,
    int arity, const std::vector<float>& taps)
    {
      return gnuradio::get_initial_sptr
        (new burst_synchronizer_cc_impl(min_len,sps,window,arity,taps));
    }

    /*
     * The private constructor
     */
    static int fft_size = 256;
    burst_synchronizer_cc_impl::burst_synchronizer_cc_impl(
      int min_len, int sps, const std::vector<float>& window,
      int arity, const std::vector<float>& taps)
      : gr::block("burst_synchronizer_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
        d_cap(1024*24)
    {
      if(!window.empty() && window.size()!=fft_size){
        throw std::runtime_error("window size should be equal to fft size");
      }
      d_window = window;
      d_fft = new gr::fft::fft_complex(fft_size,true,1);
      d_fft_out = (gr_complex*)volk_malloc(sizeof(gr_complex)*2048,volk_get_alignment());
      d_in_pwr = (gr_complex*)volk_malloc(sizeof(gr_complex)*2048,volk_get_alignment());
      d_dec_out = (gr_complex*) volk_malloc(sizeof(gr_complex)*2048,volk_get_alignment());
      
      d_ntaps = (int)taps.size();
      d_taps = taps;
      std::reverse(d_taps.begin(),d_taps.end());
      d_volk_taps = (float*) volk_malloc(sizeof(float)*d_ntaps,volk_get_alignment());
      for(int i=0;i<d_ntaps;++i){
        d_volk_taps[i] = d_taps[i];
      }

      d_sps = sps;
      d_min_len = min_len;
      d_sample_buffer = (gr_complex*)volk_malloc(sizeof(gr_complex)*d_cap,volk_get_alignment());
      //d_sample_buffer = new gr_complex[d_cap];
      d_samp_size = 0;
      d_state = false;
      d_arity = arity;
      d_burst_status = FIND_BURST;
      d_out_counter = 0;
      set_tag_propagation_policy(TPP_DONT);
    }

    /*
     * Our virtual destructor.
     */
    burst_synchronizer_cc_impl::~burst_synchronizer_cc_impl()
    {
      delete d_fft;
      //delete [] d_sample_buffer;
      volk_free(d_sample_buffer);
      volk_free(d_fft_out);
      volk_free(d_in_pwr);
      volk_free(d_dec_out);
      volk_free(d_volk_taps);
    }
    
    void
    burst_synchronizer_cc_impl::decimation_filter()
    {
      int len = floor((d_samp_size-d_ntaps) /d_sps);
      if(len>2048){
        throw std::runtime_error("decimated output greater than max capacity");
      }
      const gr_complex* ar;
      for(int i=0;i<len;++i){
        ar = &d_sample_buffer[i*d_sps];
        volk_32fc_32f_dot_prod_32fc_a(&d_dec_out[i],ar,d_volk_taps,d_ntaps);
      }
    }

    void
    burst_synchronizer_cc_impl::squaring_core(const gr_complex* in, int size)
    {
      //initialization
      if(size>2048){
        throw std::runtime_error("squaring size larger than available buffer size");
      }
      volk_32fc_s32f_power_32fc(d_in_pwr,in,d_arity,size);
    }


    float
    burst_synchronizer_cc_impl::coarse_cfo_estimation(const gr_complex* in, int input_data_size)
    {
      gr_complex* dst = d_fft->get_inbuf();
      if(d_window.size()){
        volk_32fc_32f_multiply_32fc(&dst[0],in,&d_window[0],fft_size);
      }
      else{
        memcpy(d_fft->get_inbuf(), in, input_data_size);
      }
      d_fft->execute();
      unsigned int len = (unsigned int)(ceil(fft_size/2.0));
      unsigned short int max_idx;
      memcpy(&d_fft_out[0],&d_fft->get_outbuf()[len],sizeof(gr_complex)*(fft_size-len));
      memcpy(&d_fft_out[fft_size-len],&d_fft->get_outbuf()[0],sizeof(gr_complex)*len);
      volk_32fc_index_max_16u(&max_idx, d_fft_out,fft_size);
      float denom = (float)fft_size*d_arity;
      return max_idx/denom; //M-PSK arity;
    }

    void
    burst_synchronizer_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      switch(d_burst_status){
        case FIND_BURST:
        break;
        case LOCK_BURST:
          noutput_items = 0;
        break;  
        case OUTPUT_BURST:
          noutput_items = 0;
        break;
        default:
          throw std::runtime_error("Wrong state");
        break;
        
      }
      for(int i=0;i<ninput_items_required.size();++i){
        ninput_items_required[i] = noutput_items;
      }

    }

    int
    burst_synchronizer_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
      int nin = (noutput_items<ninput_items[0]) ? noutput_items: ninput_items[0];
      // copy samples out for this is still working project;
      std::vector<tag_t> tags;

      int tmp_bgn=0;
      int consume=0,nout = 0;
      get_tags_in_window(tags, 0,0, nin);

      switch(d_burst_status){
        case FIND_BURST:
        {
          for(int i=0;i<tags.size();++i){
            if((!d_state) && (pmt::eqv(pmt::intern("ed_begin"),tags[i].key))){
              d_state = true;
              tmp_bgn = tags[i].offset - nitems_read(0);
              consume = tmp_bgn;
            }
            if((d_state) && (pmt::eqv(pmt::intern("ed_end"),tags[i].key))){
              d_state = false;
              consume = tags[i].offset - nitems_read(0)+1;
              if( ((d_samp_size + (consume-tmp_bgn))>= d_min_len)
              && ((d_samp_size + (consume-tmp_bgn))<=d_cap) ){
                int con_len = tags[i].offset - nitems_read(0)-tmp_bgn+1;
                consume = tags[i].offset - nitems_read(0)+1;
                memcpy(d_sample_buffer+d_samp_size,in+tmp_bgn,sizeof(gr_complex)*con_len);
                d_samp_size += con_len;
                d_burst_status = LOCK_BURST;
                
                consume_each(consume);
                return 0;
              }
              else if((d_samp_size + (consume-tmp_bgn)) > d_cap){
                d_state = false;
                d_samp_size = 0;
                consume_each(nin);
                return 0;
              }
            }
          }
          if(d_state){
            //there is somthing left in inputs
            consume = nin;
            int buf_len = nin-tmp_bgn;
            if( (buf_len + d_samp_size)>d_cap ){
              d_state = false;
              d_samp_size =0;
              consume_each(consume);
              return 0;
            }
            if(buf_len>0)
              memcpy(d_sample_buffer+d_samp_size,in+tmp_bgn,sizeof(gr_complex)*buf_len);
            d_samp_size+=buf_len;
          }
        break;
        }
        case LOCK_BURST:
        {
          int offset = d_sps;
          int sub_size = d_samp_size/2;
          // should do decimation first
          decimation_filter();
          squaring_core(d_dec_out+offset,sub_size);
          float cfo_est = coarse_cfo_estimation(d_in_pwr,sub_size);
          // corrected to sample base cfo
          cfo_est = cfo_est * TWO_PI
          cfo_est/=(float)d_sps;
          float phase_correction = 0;
          for(int i=0;i<d_samp_size;++i){
            d_sample_buffer[i]*=gr_expj(-phase_correction);
            phase_correction+=cfo_est;
          }
          d_out_counter = 0;
          d_burst_status = OUTPUT_BURST;
          nout = 0;
        break;
        }
        case OUTPUT_BURST:
        {
          int samp_left = d_samp_size - d_out_counter;
          nout = (samp_left > noutput_items) ? noutput_items : samp_left;
          if(nout>0){
            if(d_out_counter == 0 ){
              add_item_tag(0,nitems_written(0),pmt::intern("burst_begin"),pmt::PMT_T);
            }
            memcpy(out,d_sample_buffer+d_out_counter,sizeof(gr_complex)*nout);
          }
          d_out_counter += nout;
          if(d_out_counter ==d_samp_size){
            d_burst_status = FIND_BURST;
            d_samp_size = 0;
          }
        break;
        }
        default:
        {
          throw std::runtime_error("Entering wrong state");
          break;
        }
      }
      
      consume_each(consume);
      return nout;

      //this is still a working project, should change the I/O flow in more complete scenario.
    }

  } /* namespace lsa */
} /* namespace gr */

