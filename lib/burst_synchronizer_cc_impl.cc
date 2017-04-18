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

#define TWO_PI (M_PI * 2.0f)
static const float B_arr[16] = {0,1,0,0,
                               -1.0F/3.0,-0.5,1.0F,-1.0F/6.0,
                               0.5,-1.0F,0.5,0,
                               -1.0F/6.0F,0.5,-0.5,1.0F/6.0};

    enum burstStatus{
      FIND_BURST,
      LOCK_BURST,
      OUTPUT_BURST
    };

    burst_synchronizer_cc::sptr
    burst_synchronizer_cc::make(int min_len, int sps, const std::vector<float>& window,
    int arity, float loop_bw)
    {
      return gnuradio::get_initial_sptr
        (new burst_synchronizer_cc_impl(min_len,sps,window,arity,loop_bw));
    }

    /*
     * The private constructor
     */
    static int fft_size = 512;
    burst_synchronizer_cc_impl::burst_synchronizer_cc_impl(
      int min_len, int sps, const std::vector<float>& window,
      int arity, float loop_bw)
      : gr::block("burst_synchronizer_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
        d_cap(8192), d_omega(2.0), d_omega_mid(2.0),
        d_p_2T(0),d_p_1T(0),d_p_0T(0), d_c_2T(0), d_c_1T(0), d_c_0T(0),
        d_omega_relative_limit(1e-2)
    {
      if(!window.empty() && window.size()!=fft_size){
        throw std::runtime_error("window size should be equal to fft size");
      }
      d_window = window;
      d_fft = new gr::fft::fft_complex(fft_size,true,1);
      d_fft_out = (gr_complex*)volk_malloc(sizeof(gr_complex)*2048,volk_get_alignment());
      d_interp_out = (gr_complex*) volk_malloc(sizeof(gr_complex)*8192,volk_get_alignment());

      d_sps = sps;
      d_min_len = min_len;
      d_sample_buffer = (gr_complex*)volk_malloc(sizeof(gr_complex)*d_cap,volk_get_alignment());
      d_samp_size = 0;
      d_state = false;
      d_arity = arity;
      d_burst_status = FIND_BURST;
      d_out_counter = 0;
      set_tag_propagation_policy(TPP_DONT);

      //clock recovery
      d_mu = 0.5;
      float crit = sqrt(2.0)/2.0;
      float denom = 1+2*crit*loop_bw+loop_bw*loop_bw;
      float Kp = 2.7;
      //d_gain_omega= 4*crit*loop_bw/denom/Kp;
      //d_gain_mu = 4*loop_bw*loop_bw/denom/Kp;
      d_gain_omega=0.25*0.175*0.175;
      d_gain_mu = 0.175;
      d_interp_size = 0;
    }

    /*
     * Our virtual destructor.
     */
    burst_synchronizer_cc_impl::~burst_synchronizer_cc_impl()
    {
      delete d_fft;
      volk_free(d_sample_buffer);
      volk_free(d_fft_out);
      volk_free(d_interp_out);
    }
    
    gr_complex
    burst_synchronizer_cc_impl::interp_3(const gr_complex* in, const float& mu)
    {
      gr_complex v[4];
      for(int i=0;i<4;++i){
        v[i]=0;
        for(int j=0;j<4;++j){
          v[i] += B_arr[i*4+j]*in[j];
        }
      }
      return ((v[3]*mu+v[2])*mu+v[1])*mu+v[0];
    }

    static inline gr_complex
    slice(gr_complex x)
    {
      return gr_complex(((x.real()>1.0)? 1.0F : -1.0F),
      ((x.imag()>1.0)?1.0F:-1.0F) );
    }

    void
    burst_synchronizer_cc_impl::mm_time_recovery(gr_complex* out, const gr_complex* in, int size)
    {
      d_interp_size = 0;
      d_omega_mid = 2.0;
      d_mu = 1.0/d_omega_mid;
      d_omega = 2.0;
      d_p_2T = 0;d_p_1T = 0;d_p_0T = 0;
      d_c_2T = 0;d_c_1T = 0;d_c_0T = 0;
      int ii =0;
      int oo=0;
      float error=0;
      gr_complex u;
      while(ii< (size-3) )
      {
        d_p_2T = d_p_1T;
        d_p_1T = d_p_0T;
        d_p_0T = interp_3(&in[ii], d_mu);
        
        d_c_2T = d_c_1T;
        d_c_1T = d_c_0T;
        d_c_0T = slice(d_p_0T);

        u = (d_p_0T - d_p_2T) * conj(d_c_1T) - (d_c_0T-d_c_2T)*conj(d_p_1T);
        error = u.real();
        out[oo++] = d_p_0T;

        error = gr::branchless_clip(error,1.0);
        d_omega = d_omega + d_gain_omega*error;
        d_omega = d_omega_mid + gr::branchless_clip(d_omega-d_omega_mid,d_omega_relative_limit);

        d_mu = d_mu + d_omega + d_gain_mu*error;
        ii+=(int)floor(d_mu);
        d_mu -= floor(d_mu);
        if(ii<0){
          ii=0;
        }
      }
      d_interp_size = oo;
    }

    void
    burst_synchronizer_cc_impl::constellation_remove(gr_complex* in, int size)
    {
      // specific for PSK modulation
      if(size>d_cap){
        throw std::runtime_error("squaring size larger than available buffer size");
      }
      float tmp_arg;
      for(int i=0;i<size;++i){
        tmp_arg = arg(in[i]) * d_arity;
        while(tmp_arg>TWO_PI)
          tmp_arg-=TWO_PI;
        while(tmp_arg<-TWO_PI)
          tmp_arg+=TWO_PI;
        in[i] = abs(in[i])*gr_expj(tmp_arg);
      }
      //change to phase scalar;
      //volk_32fc_s32f_power_32fc(d_in_pwr,in,d_arity,size);
    }


    float
    burst_synchronizer_cc_impl::coarse_cfo_estimation(const gr_complex* in, int input_data_size)
    {
      /*gr_complex* cos_test = d_fft->get_inbuf();
      float test_f = 0.1;
      for(int i=0;i<fft_size;++i){
        cos_test[i] = gr_expj(TWO_PI*test_f*i);
      }
      d_fft->execute();
      unsigned int len = (unsigned int)(ceil(fft_size/2.0));
      memcpy(&d_fft_out[0],&d_fft->get_outbuf()[len],sizeof(gr_complex)*(fft_size-len));
      memcpy(&d_fft_out[fft_size-len],&d_fft->get_outbuf()[0],sizeof(gr_complex)*len);
      return 0;*/
      
      gr_complex* dst = d_fft->get_inbuf();
      if(input_data_size<fft_size){
        memset(&dst[0],0,fft_size);
      }
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
      return (max_idx-fft_size/2.0f)/denom; //M-PSK arity;
      
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
          consume=nin;
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
                memcpy(d_sample_buffer+d_samp_size,in+tmp_bgn,sizeof(gr_complex)*con_len);
                d_samp_size += con_len;
                d_burst_status = LOCK_BURST; 
                consume_each(consume);
                return 0;
              }
              else{
                d_samp_size = 0;
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
          
          // there may be bugs in decimator and timing loop
          //decimation_filter(d_dec_out,d_sample_buffer, d_samp_size);
          mm_time_recovery(d_interp_out,d_sample_buffer, d_samp_size);
          constellation_remove(d_interp_out,d_interp_size);
          //change to phase scalar
          int offset = 0;
          float cfo_est = coarse_cfo_estimation(d_interp_out+offset,fft_size);
          //std::cout<<"******************************************"<<std::endl;
          //std::cout<<"checking fft outputs:"<<std::endl;
          //for(int i=0;i<fft_size;++i){
            //std::cout<<d_fft_out[i].real()<<","<<d_fft_out[i].imag()<<std::endl;
          //}
          // corrected to sample base cfo
          cfo_est = cfo_est * TWO_PI;
          cfo_est/=(float)d_sps;
          float phase_correction = 0;
          for(int i=0;i<d_samp_size;++i){
            d_sample_buffer[i]*=gr_expj(-phase_correction);
            phase_correction+=cfo_est;
            while(phase_correction>TWO_PI)
              phase_correction-=TWO_PI;
            while(phase_correction<-TWO_PI)
              phase_correction+=TWO_PI;
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
            d_state = false;
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
    }

  } /* namespace lsa */
} /* namespace gr */

