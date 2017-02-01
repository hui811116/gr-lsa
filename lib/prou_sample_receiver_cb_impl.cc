/* -*- c++ -*- */
/* 
 * Copyright 2016 <+YOU OR YOUR COMPANY+>.
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
#include "prou_sample_receiver_cb_impl.h"
#include <volk/volk.h>

#include <gnuradio/math.h>
#include <gnuradio/expj.h>
#include <gnuradio/sincos.h>

#include <cstdio>
#include <cmath>
#include <algorithm>


namespace gr {
  namespace lsa {

    prou_sample_receiver_cb::sptr
    prou_sample_receiver_cb::make(
      const gr::digital::constellation_sptr& su_hdr_const,
      int su_pld_bps,
      const std::string& su_accesscode,
      double plf_sps,
      float plf_loop_bw,
      const std::vector<float> &plf_taps,
      int su_nfilts,
      float costas_loop_bw,
      int costas_order,
      bool mode,
      bool debug)
    {
      return gnuradio::get_initial_sptr
        (new prou_sample_receiver_cb_impl(
          su_hdr_const,
          su_pld_bps,
          su_accesscode,
          plf_sps,
          plf_loop_bw,
          plf_taps,
          su_nfilts,
          costas_loop_bw,
          costas_order,
          mode,
          debug));
    }

    enum proURxMode{
      STANDARD,
      INTERFERENCE_CANCELLATION,
    };
    enum suSyncState{
      SEARCH_ACCESSCODE,
      HEADER_WAIT,
      PAYLOAD_WAIT
    };
    enum intfState{
      CLEAR,
      RETRANSMISSION
    };
    enum intfSyncState{
      INTF_SEARCH,
      INTF_HEADER,
      INTF_PAYLOAD_WAIT
    };
    
    /*
     * The private constructor
     */
    prou_sample_receiver_cb_impl::prou_sample_receiver_cb_impl(
      const gr::digital::constellation_sptr& su_hdr_const,
      int su_pld_bps,
      const std::string& su_accesscode,
      double plf_sps,
      float plf_loop_bw,
      const std::vector<float>& plf_taps,
      int su_nfilts,
      float costas_loop_bw,
      int costas_order,
      bool mode,
      bool debug)
      : gr::block("prou_sample_receiver_cb",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
      gr::blocks::control_loop(costas_loop_bw, 1.0,-1.0),
      d_costas_phase_detector(NULL),
      d_process_buffer(NULL),
      d_sample_buffer(NULL),
      d_su_bps(su_hdr_const->bits_per_symbol()),
      d_su_pld_bps(su_pld_bps),
      d_sample_cap_init(128*1024),
      d_cap_init(floor(128*1024/plf_sps)),
      d_su_hdr_bits_len((4+2+2)*8)
    {
      d_sample_cap = d_sample_cap_init;
      d_process_cap = d_cap_init;
      reset_buffer();

      d_mode = (mode)? INTERFERENCE_CANCELLATION : STANDARD;
      d_state = SEARCH_ACCESSCODE;
      d_intf_state = CLEAR;

      // DEBUG Purpose
      d_debug = debug;
      d_debug_port = pmt::mp("debug");      

      if(!set_su_accesscode(su_accesscode)){
        std::invalid_argument("ProU RX: Invalid SU accesscode");
      }

      d_su_hdr_const = su_hdr_const->base();
      
      //output sample buffer initialization
      d_output_buffer_cap = d_cap_init;
      d_output_buffer = new gr_complex[d_output_buffer_cap];
      d_output_buffer_size = 0;
      d_output_buffer_idx = 0;

      message_port_register_out(d_debug_port);
      set_tag_propagation_policy(TPP_DONT);
      
      d_var_eng_buffer = (float *) volk_malloc(sizeof(float)*d_cap_init, volk_get_alignment());
      d_eng_buffer = (float *) volk_malloc(sizeof(float)*d_cap_init, volk_get_alignment());

      //su sync---polyphase clock sync
      if(plf_taps.empty()){
        throw std::runtime_error("ProU RX::Polyphase Sync::filter length should not be 0");
      }
      d_plf_nfilts = su_nfilts;
      d_plf_sps = floor(plf_sps);
      d_plf_damping = 2*d_plf_nfilts;
      d_plf_loop_bw = plf_loop_bw;
      d_plf_max_dev = 1.5;//FIXME use var here
      d_plf_osps = 1;

      d_plf_k = su_nfilts/2.0; // init phase
      d_plf_rate = (d_plf_sps-floor(d_plf_sps))*(double)d_plf_nfilts;
      d_plf_rate_i = (int)floor(d_plf_rate);
      d_plf_rate_f = d_plf_rate - (float)d_plf_rate_i;
      d_plf_filtnum = (int)floor(d_plf_k);

      d_plf_filters = std::vector<gr::filter::kernel::fir_filter_ccf*>(d_plf_nfilts);
      d_plf_diff_filters = std::vector<gr::filter::kernel::fir_filter_ccf*>(d_plf_nfilts);

      std::vector<float> vtaps(1,0);
      for(int i=0 ; i < d_plf_nfilts; i++) {
        d_plf_filters[i] = new gr::filter::kernel::fir_filter_ccf(1, vtaps);
        d_plf_diff_filters[i] = new gr::filter::kernel::fir_filter_ccf(1, vtaps);
      }

      std::vector<float> dtaps;
      plf_create_diff_taps(plf_taps, dtaps);
      plf_set_taps(plf_taps, d_plf_taps, d_plf_filters);
      plf_set_taps(dtaps, d_plf_dtaps, d_plf_diff_filters);

      float denom = (1.0 + 2.0*d_plf_damping*d_plf_loop_bw + d_plf_loop_bw*d_plf_loop_bw);
      d_plf_alpha = (4*d_plf_damping*d_plf_loop_bw) / denom;
      d_plf_beta = (4*d_plf_loop_bw*d_plf_loop_bw) / denom;
      d_plf_out_idx = 0;
      d_plf_error = 0;
      
      //su sync---costas loop
      
      d_costas_loop_bw = costas_loop_bw;
      d_costas_order = costas_order;
      d_costas_error = 0.0;
      d_costas_noise = 1.0;
      switch(d_costas_order)
      {
        case 2:
          d_costas_phase_detector = &prou_sample_receiver_cb_impl::phase_detector_2;
        break;
        case 4:
          d_costas_phase_detector = &prou_sample_receiver_cb_impl::phase_detector_4;
        break;
        case 8:
          d_costas_phase_detector = &prou_sample_receiver_cb_impl::phase_detector_8;
        break;
        default:
          std::invalid_argument("ProU RX::Costas_loop::Order should be 2,4,8");
        break;
      }

      d_plf_time_error = new float[4096];
      d_costas_phase_error = new float[4096];
      d_plf_symbol_buffer = new gr_complex[4096];
    }

    /*
     * Our virtual destructor.
     */
    prou_sample_receiver_cb_impl::~prou_sample_receiver_cb_impl()
    {
      for(int i=0;i<d_plf_nfilts;++i){
        delete d_plf_filters[i];
        delete d_plf_diff_filters[i];
      }
      delete [] d_sample_buffer;
      delete [] d_process_buffer;
      delete [] d_output_buffer;
      volk_free(d_var_eng_buffer);
      volk_free(d_eng_buffer);

      delete [] d_plf_time_error;
      delete [] d_costas_phase_error;
      delete [] d_plf_symbol_buffer;
    }

    bool
    prou_sample_receiver_cb_impl::parse_su_header(uint8_t& qidx, uint8_t& qsize, uint16_t& pld_len, const std::vector<unsigned char>& input)
    {
      uint16_t len0,len1;
      len0 = _get_bit16(0,input);
      len1 = _get_bit16(16,input);
      if(len0 == len1)
      {
        pld_len = len0;
        qidx = _get_bit8(48,input);
        qsize = _get_bit8(56,input);
        return true;
      }
      return false;
    }
    //SU SYNC--- POLYPHASE CLOCK SYNC
    void
    prou_sample_receiver_cb_impl::plf_create_diff_taps(
      const std::vector<float> &newtaps,
      std::vector<float> &difftaps)
    {
      std::vector<float> diff_filter(3);
      diff_filter[0] = -1;
      diff_filter[1] = 0;
      diff_filter[2] = 1;
      float pwr=0;
      difftaps.clear();
      difftaps.push_back(0);
      for(unsigned int i=0;i<newtaps.size()-2; i++){
        float tap = 0;
        for(unsigned int j=0;j<diff_filter.size();j++){
          tap += diff_filter[j]*newtaps[i+j];
        }
        difftaps.push_back(tap);
        pwr += fabsf(tap);
      }
      difftaps.push_back(0);
      for (unsigned int i=0;i<difftaps.size();i++){
        difftaps[i] *= d_plf_nfilts/pwr;
        if(difftaps[i] != difftaps[i]) {
          throw std::runtime_error("ProU RX::Polyphase Clock Sync::create_diff_taps produced NaN");
        }
      }
    }

    void
    prou_sample_receiver_cb_impl::plf_set_taps(
      const std::vector<float> &newtaps,
      std::vector< std::vector<float> > &ourtaps,
      std::vector< gr::filter::kernel::fir_filter_ccf*> &ourfilter)
    {
      int i,j;
      unsigned int ntaps = newtaps.size();
      d_plf_taps_per_filter = (unsigned)ceil((double)ntaps/(double)d_plf_nfilts);

      ourtaps.resize(d_plf_nfilts);

      std::vector<float> tmp_taps;
      tmp_taps = newtaps;
      while((float)(tmp_taps.size()) < d_plf_nfilts*d_plf_taps_per_filter)
      {
        tmp_taps.push_back(0.0);
      }
      for(i=0;i<d_plf_nfilts;i++){
        ourtaps[i] = std::vector<float>(d_plf_taps_per_filter,0);
        for(j=0;j<d_plf_taps_per_filter; j++){
          ourtaps[i][j] = tmp_taps[i+j*d_plf_nfilts];
        }
        ourfilter[i]->set_taps(ourtaps[i]);
      }
      //FIXME (history can be handled through internal queue)
      //set_history(d_plf_taps_per_filter + d_plf_sps + d_plf_sps);
      d_plf_history = d_plf_taps_per_filter + d_plf_sps + d_plf_sps;
      //set_output_multiples(d_plf_osps);
    }

    int
    prou_sample_receiver_cb_impl::plf_core(
      gr_complex* out,
      float* error,
      const gr_complex* in,
      int nsample,
      int& nconsume)
    {
      int i =0, count =0;
      float error_r, error_i;
      while(i < nsample) {
        while(d_plf_out_idx < d_plf_osps){
          d_plf_filtnum = (int) floor(d_plf_k);
          while(d_plf_filtnum >= d_plf_nfilts){
            d_plf_k -= d_plf_nfilts;
            d_plf_filtnum -= d_plf_nfilts;
            count += 1;
          }
          while(d_plf_filtnum <0){
            d_plf_k += d_plf_nfilts;
            d_plf_filtnum += d_plf_nfilts;
            count -= 1;
          }
          out[i+d_plf_out_idx] = d_plf_filters[d_plf_filtnum]->filter(&in[count+d_plf_out_idx]);
          d_plf_k = d_plf_k + d_plf_rate_i + d_plf_rate_f;
          d_plf_out_idx++;
          if(i+d_plf_out_idx >= nsample) {
            nconsume = count;
            return i; // output numbers
          }
        }
        //reset
        d_plf_out_idx = 0;
        gr_complex diff = d_plf_diff_filters[d_plf_filtnum]->filter(&in[count]);
        error_r = out[i].real() * diff.real();
        error_i = out[i].imag() * diff.imag();
        d_plf_error = (error_i+error_r)/2.0;

        for(int s =0; s<d_plf_sps;s++){
          d_plf_rate_f = d_plf_rate_f + d_plf_beta*d_plf_error;
          d_plf_k = d_plf_k + d_plf_rate_f + d_plf_alpha*d_plf_error;
        }
        // what
        d_plf_rate_f = gr::branchless_clip(d_plf_rate_f, d_plf_max_dev);
        i+=d_plf_osps;
        count += (int)floor(d_plf_sps);
      }
      nconsume = count;
      return i;
    }
    //SU FREQUENCY/PHASE SYNC::COSTAS_LOOP
    float
    prou_sample_receiver_cb_impl::phase_detector_8(const gr_complex& sample) const
    {
      float K = sqrt(2.0)-1;
      if(fabsf(sample.real()) >= fabsf(sample.imag())){
        return ((sample.real() > 0?1.0:-1.0) * sample.imag() - (sample.imag()>0 ? 1.0:-1.0) * sample.real()*K);
      }
      else{
        return ((sample.real() > 0?1.0:-1.0) * sample.imag() *K-(sample.imag()>0?1.0:-1.0)*sample.real());
      }
    }
    float
    prou_sample_receiver_cb_impl::phase_detector_4(const gr_complex& sample) const
    {
      return ((sample.real()>0?1.0:-1.0) *sample.imag() - (sample.imag()>0?1.0:-1.0) * sample.real());
    }
    float
    prou_sample_receiver_cb_impl::phase_detector_2(const gr_complex& sample) const
    {
      return (sample.real()*sample.imag());
    }
    int 
    prou_sample_receiver_cb_impl::costas_core(
      gr_complex* out,
      float* error_ang,
      const gr_complex* in,
      int nsample
      )
    {
      gr_complex nco_out;
      for(int i=0;i<nsample;++i){
        nco_out = gr_expj(-d_phase);
        out[i]=in[i] * nco_out;

        d_costas_error = (*this.*d_costas_phase_detector)(out[i]);
        d_costas_error = gr::branchless_clip(d_costas_error,1.0);

        advance_loop(d_costas_error);
        phase_wrap();
        frequency_limit();

        error_ang[i] = d_freq;
      }
      return nsample;
    }
    //INTERFERENCE CANCELLATION HELPER FUNCTIONS
    uint16_t
    prou_sample_receiver_cb_impl::_get_bit16(int begin_idx, const std::vector<unsigned char>& input)
    {
      unsigned long tmp=0UL;
      for(int i=0;i<16;++i){
        tmp |= (input[begin_idx+i] & 0x0001) << (15-i);  // NOTE: this is how bits arrangement should be like
      }
      return tmp;
    }

    uint8_t
    prou_sample_receiver_cb_impl::_get_bit8(int begin_idx, const std::vector<unsigned char>& input)
    {
      unsigned char tmp =0;
      for(int i=0;i<8;++i){
        tmp |= (input[begin_idx+i] & 0x01) << (7-i);
      }
      return tmp;
    }

    //SET FUNCTIONS
    bool
    prou_sample_receiver_cb_impl::set_su_accesscode(const std::string& su_accesscode)
    {
      d_su_code_len = su_accesscode.length();
      if(d_su_code_len > 64){
        return false;
      }
      d_su_code_mask = ((~0ULL) >> (64-d_su_code_len) );
      for(unsigned i=0;i<d_su_code_len;++i){
        d_su_accesscode = (d_su_accesscode << 1) | (su_accesscode[i] & 1);
      }
      return true;
    }

    //GET FUNCTIONS
    uint64_t
    prou_sample_receiver_cb_impl::su_accesscode() const
    {
      return d_su_accesscode;
    }

    //BUFFER HELPER FUNCTION
    void
    prou_sample_receiver_cb_impl::reduce_sample(int divider)
    {
      int nleft = d_sample_size/divider;
      int nleft_symbol = d_process_size/divider;

      gr_complex tmp[d_sample_size];
      memcpy(tmp,d_sample_buffer+nleft,sizeof(gr_complex)*(d_sample_size-nleft));
      memcpy(d_sample_buffer, tmp, sizeof(gr_complex)*(d_sample_size-nleft));
      d_sample_size -= nleft;
      if(d_sample_idx >nleft){
        d_sample_idx -= nleft;
      }

      gr_complex tmp_symbol[d_process_size];
      memcpy(tmp_symbol, d_process_buffer+nleft_symbol,sizeof(gr_complex)*(d_process_size-nleft_symbol));
      memcpy(d_process_buffer, tmp_symbol, sizeof(gr_complex)*(d_process_size-nleft_symbol));
      d_process_size-= nleft_symbol;
      if(d_process_idx > nleft_symbol){
        d_process_idx-= nleft_symbol;
      }
    }

    void
    prou_sample_receiver_cb_impl::double_cap()
    {
      gr_complex tmp_sample[d_sample_size];
      gr_complex tmp[d_process_size];

      memcpy(tmp_sample, d_sample_buffer, sizeof(gr_complex)*d_sample_size);
      delete [] d_sample_buffer;
      d_sample_buffer = new gr_complex[d_sample_cap*2];
      memcpy(d_sample_buffer, tmp_sample, sizeof(gr_complex)*d_sample_size);
      d_sample_cap*=2;
      
      memcpy(tmp, d_process_buffer, sizeof(gr_complex)*d_process_size);
      delete [] d_process_buffer;
      d_process_buffer = new gr_complex[d_process_cap*2];
      memcpy(d_process_buffer, tmp, sizeof(gr_complex)*d_process_size);
      d_process_cap*=2;
      //size remain the same, index also
    }

    void
    prou_sample_receiver_cb_impl::reset_buffer()
    {
      if(d_process_buffer!=NULL)
        delete [] d_process_buffer;
      d_process_buffer = new gr_complex[d_cap_init];
      d_process_cap = d_cap_init;
      d_process_size = 0;
      d_process_idx = 0;

      if(d_sample_buffer!=NULL)
        delete [] d_sample_buffer;
      d_sample_buffer = new gr_complex[d_sample_cap_init];
      d_sample_cap = d_sample_cap_init;
      d_sample_size = 0;
      d_sample_idx = 0;
    }

    //INTERFERENCE CANCELLER FUNCTIONS

    void
    prou_sample_receiver_cb_impl::update_retx_info(bool test_voe)
    {
      switch(d_intf_state)
      {
        case RETRANSMISSION:
          if( !test_voe){
            if(d_retx_pkt_len[d_qidx] == 0){
              d_retx_count++;
            }
            d_retx_buf_idx[d_qidx] = d_su_pkt_begin;
            d_retx_pkt_len[d_qidx] = d_pld_len*8/d_su_pld_bps + d_su_hdr_bits_len/d_su_bps;
          }
        break;
        case CLEAR:
        default:
          return;
        break;
      }
    }

    void
    prou_sample_receiver_cb_impl::reset_intf_reg()
    {
      //reset the registers required to decide whether retransmission is done or not.
      d_retx_count=0;
      d_retx_pkt_len.clear();
      d_retx_buf_idx.clear();
      d_cei_buf_idx.clear();
      d_cei_pkt_len.clear();
      d_cei_qidx.clear();
    }

    bool
    prou_sample_receiver_cb_impl::calc_var_energy(const gr_complex* in, size_t length, float threshold_db)
    {
      assert(length < 1024*1024);
      float voe = -1000;

      int alignment = volk_get_alignment();
      float * mean = (float*) volk_malloc(sizeof(float), alignment);
      float * stddev = (float*) volk_malloc(sizeof(float), alignment);
      
      volk_32fc_magnitude_squared_32f(d_eng_buffer, in, length);
      volk_32f_stddev_and_mean_32f_x2(stddev,mean, d_eng_buffer, length);
      
      float var_eng_db = 20.0*log10(*stddev);
      volk_free(mean);
      volk_free(stddev);

      return (var_eng_db > threshold_db);
    }

    void
    prou_sample_receiver_cb_impl::calc_cei_all()
    {
      unsigned symbol;
      int state = INTF_SEARCH;
      uint64_t bit_reg = ~0ULL;
      uint8_t qidx,qsize;
      uint16_t pld_len;
      int pld_count;
      int pkt_len;
      std::vector<unsigned char> input;
      for(int i=0;i<d_process_size;++i){
        symbol = d_su_hdr_const->decision_maker(&d_process_buffer[i]);
        switch(state)
        {
          case INTF_SEARCH:
            for(int j=0;j<d_su_bps;++j){
              bit_reg = (bit_reg << 1) | ((symbol >> (d_su_bps-1-j)) & 0x01);
            }
            if( ((bit_reg ^ d_su_accesscode)& d_su_code_mask) == 0 ){
              input.clear();
              state = INTF_HEADER;
            }
          break;
          case INTF_HEADER:
            for(int j=0;j<d_su_bps;++j){
              input.push_back( (symbol >> (d_su_bps-1-j)) & 0x01);
            }
            if(input.size() == d_su_hdr_bits_len){
              if(parse_su_header(qidx,qsize,pld_len,input)){
                pld_count = pld_len*8/d_su_pld_bps;
                pkt_len = pld_count + (d_su_hdr_bits_len + d_su_code_len)/d_su_bps;
                state = INTF_PAYLOAD_WAIT;
              }
              else{
                state = INTF_SEARCH;
              }
            }
          break;
          case INTF_PAYLOAD_WAIT:
            if(pld_count==0){
              d_cei_buf_idx.push_back(i-pkt_len+1);
              d_cei_pkt_len.push_back(pkt_len);
              d_cei_qidx.push_back(qidx);
              state = INTF_SEARCH;
            }
            pld_count--;
          break;
          default:
            std::runtime_error("PROU RX: calc_cei_all--Entering wrong state");
          break;
        }

      }
    }

    void
    prou_sample_receiver_cb_impl::do_interference_cancellation()
    {
      //TODO
      //First step, synchroniza with partial header information
      assert(!d_cei_buf_idx.empty());
      int total_len = 0;
      int max_len = d_retx_pkt_len[0];
      for(int i=0;i<d_retx_pkt_len.size();++i){
        total_len += d_retx_pkt_len[i];
        if(d_retx_pkt_len[i]>max_len){
          max_len = d_retx_pkt_len[i];
        }
      }
      total_len = 0;
      gr_complex* retx_all = (gr_complex*) malloc(sizeof(gr_complex)* total_len);
      std::vector<int> retx_idx;

      //NOTE: this version does not replace the header part of the retransmitted packets
      //<FIXME>
      for(int i=0;i<d_retx_buf_idx.size();++i){
        int tmp_len = d_retx_pkt_len[i];
        int tmp_idx = d_retx_buf_idx[i];
        memcpy(retx_all + total_len, d_process_buffer + tmp_idx, sizeof(gr_complex)* d_retx_buf_idx[i]);
        retx_idx.push_back(total_len);
        total_len += tmp_len;
      }

      int qsize = d_retx_buf_idx.size();
      int last_good_idx = d_cei_buf_idx[d_cei_buf_idx.size()-1];
      int last_qidx  = d_cei_qidx[d_cei_qidx.size()-1];
      int last_len = d_cei_pkt_len[d_cei_pkt_len.size()-1];

      while(last_good_idx + last_len < d_process_size){
        last_qidx = (last_qidx+1) % qsize;
        last_good_idx += last_len;
        last_len = d_retx_pkt_len[last_qidx];
      }

      last_good_idx--;//shift to valid index
      int pkt_count = d_retx_pkt_len[last_qidx]-1;

      while(last_good_idx >=0){
        d_process_buffer[last_good_idx] -= (retx_all[ retx_idx[last_qidx] + pkt_count] );
        pkt_count--;
        if(pkt_count < 0){
          last_good_idx--;
          last_good_idx %= qsize;
          pkt_count = d_retx_pkt_len[last_good_idx];
        }
        last_good_idx--;
      }

      free(retx_all);
    }

    //OUTPUT BUFFER FUNCTIONS
    int
    prou_sample_receiver_cb_impl::output_samples(gr_complex* out, int noutput_items)
    {
      int true_noutput_items;
      if(noutput_items > d_output_buffer_size){
        memcpy(out, d_output_buffer + d_output_buffer_idx, sizeof(gr_complex)*d_output_buffer_size);
        true_noutput_items = d_output_buffer_size;
        d_output_buffer_size = 0;
        d_output_buffer_idx = 0;
        _out_buffer_reset_cap();
      }
      else{
        memcpy(out, d_output_buffer + d_output_buffer_idx, sizeof(gr_complex)*noutput_items);
        true_noutput_items = noutput_items;
        d_output_buffer_idx += noutput_items;
        d_output_buffer_size -= noutput_items;
      }
      return true_noutput_items;
    }

    void 
    prou_sample_receiver_cb_impl::extract_samples_ed(std::vector<gr_complex>& out, double ed_db)
    {
      //NOTE: this may lose some transition sample which are parts of a symbol
      double ed_tmp;
      for(int i=0;i<d_process_size;++i){
        ed_tmp = 10* log10(norm(d_process_buffer[i]));
        if(ed_tmp > ed_db){
          out.push_back(d_process_buffer[i]);
        }
      }
    }

    void 
    prou_sample_receiver_cb_impl::_out_buffer_double_cap()
    {
      gr_complex tmp[d_output_buffer_size];
      memcpy(tmp, d_output_buffer, sizeof(gr_complex)*d_output_buffer_size);
      delete [] d_output_buffer;
      d_output_buffer_cap *= 2;
      d_output_buffer = new gr_complex[d_output_buffer_cap];
      memcpy(d_output_buffer, tmp, sizeof(gr_complex)*d_output_buffer_size);
    }

    void 
    prou_sample_receiver_cb_impl::_out_buffer_reset_cap()
    {
      if(d_output_buffer_cap == d_cap_init){
        return;
      }
      assert(d_output_buffer_size < d_cap_init);
      gr_complex tmp[d_output_buffer_size];
      memcpy(tmp, d_output_buffer,sizeof(gr_complex)*d_output_buffer_size);
      d_output_buffer_cap = d_cap_init;
      delete [] d_output_buffer;
      d_output_buffer = new gr_complex[d_output_buffer_cap];
      memcpy(d_output_buffer, tmp, sizeof(gr_complex)* d_output_buffer_size);
      
    }

    //FLOW CONTROL FUNCTIONS

    void
    prou_sample_receiver_cb_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      if(d_output_buffer_size != 0){
        ninput_items_required[0] = 0;  
      }
      else{
        assert(d_process_cap >= d_process_size);
        int max_sample = d_process_cap - d_process_size;
        ninput_items_required[0] = (noutput_items > max_sample) ? max_sample: noutput_items;  
      }
    }


    // MAIN FLOW
    bool
    prou_sample_receiver_cb_impl::intf_decision_maker()
    {
      //cei will be kept until this function ends
      int pkt_symbol_len = d_pld_len*8/d_su_pld_bps + d_su_hdr_bits_len/d_su_bps;
      bool test_voe = calc_var_energy(d_process_buffer + d_su_pkt_begin, pkt_symbol_len,-40.0);
      switch(d_intf_state)
      {
        case CLEAR:
          if(d_qsize!=0x00){
            d_intf_state = RETRANSMISSION;
            reset_intf_reg();
            d_retx_buf_idx.resize(d_qsize);
            d_retx_pkt_len.resize(d_qsize,0);
            //calc all cancellation enabling information
            calc_cei_all();            
          }
        break;
        case RETRANSMISSION:
          if( (d_qsize == 0x00)|| (d_qsize != d_retx_buf_idx.size()) ){
            //new pkt transmitting, retransmission ends
            if(d_retx_count != d_retx_buf_idx.size()){
              //fail to receive all retransmission, but still have chances to cancel interference
            }
            d_intf_state = CLEAR;
          }

        break;
        default:
          std::runtime_error("ProU RX: function<intf_decision_maker> Entering wrong state");
        break;
      }
      update_retx_info(test_voe);
      if((d_retx_count!=0) && (d_retx_count == d_retx_pkt_len.size()) ){
        do_interference_cancellation();
        d_retx_count=0;
        return true;
      }
      return false;
    }

    bool
    prou_sample_receiver_cb_impl::process_symbols()
    {
      pmt::pmt_t debug_info = pmt::make_dict();
      unsigned char symbol;
      uint64_t check_bits;
      for(d_process_idx ;d_process_idx < d_process_size; d_process_idx++){
        symbol = d_su_hdr_const->decision_maker(&d_process_buffer[ d_process_idx]);
        switch(d_state)
        {
          case SEARCH_ACCESSCODE:
            for(int i=0;i<d_su_bps;++i){
              d_su_sync_reg = (d_su_sync_reg << 1) | ((symbol >> (d_su_bps-1-i) ) & 0x01);
            }
            check_bits = (d_su_sync_reg ^ d_su_accesscode) & d_su_code_mask;
            if(check_bits == 0){
              d_state = HEADER_WAIT;
              d_su_bit_input.clear();
              d_su_pkt_begin = (d_process_idx) - (d_su_code_len)/d_su_bps;
            }
          break;
          case HEADER_WAIT:
            for(int i=0;i<d_su_bps;++i){
              d_su_bit_input.push_back( ((symbol >> (d_su_bps-1-i)) & 0x01) );
            }
            if(d_su_bit_input.size() == d_su_hdr_bits_len){
              if(parse_su_header(d_qidx,d_qsize,d_pld_len, d_su_bit_input)){
                d_state = PAYLOAD_WAIT;
                d_su_pld_counter = ( d_pld_len*8)/d_su_pld_bps;
                if(d_debug){
                  debug_info = pmt::dict_add(debug_info, pmt::intern("ProU_RX"), pmt::string_to_symbol("process_symbols"));
                  debug_info = pmt::dict_add(debug_info, pmt::intern("payload_len"), pmt::from_long(d_pld_len));
                  debug_info = pmt::dict_add(debug_info, pmt::intern("qidx"), pmt::from_long(d_qidx));
                  debug_info = pmt::dict_add(debug_info, pmt::intern("qsize"), pmt::from_long(d_qsize));
                  message_port_pub(d_debug_port, debug_info);
                }

              }
              else{
                d_state = SEARCH_ACCESSCODE;
                if(d_debug){
                  debug_info = pmt::dict_add(debug_info, pmt::intern("ProU RX"), pmt::string_to_symbol("accesscode found but failed"));
                  message_port_pub(d_debug_port, debug_info);
                }
              }
            }
          break;
          case PAYLOAD_WAIT:
            assert(d_su_pld_counter>=0);
            if(d_su_pld_counter==0){
              d_state = SEARCH_ACCESSCODE;
              if(intf_decision_maker()){
                std::vector<gr_complex> out;
                //<FIXME>: threshold of the interference detector
                extract_samples_ed(out, -50);
                int samp_size = out.size();
                if(d_output_buffer_size + samp_size > d_output_buffer_cap){
                  _out_buffer_double_cap();
                }
                memcpy(d_output_buffer + d_output_buffer_size,out.data(),sizeof(gr_complex)*samp_size);
                d_output_buffer_size += samp_size;
                return true;
              }
            }
            d_su_pld_counter--;
          break;
          default:
            std::runtime_error("ProU RX::process_symbols::entering wrong state");
          break;
        }
      }
      //update
      return d_output_buffer_size!=0;
    }

    void
    prou_sample_receiver_cb_impl::su_sample_sync(const std::vector<bool>& sensing_result, int window)
    {
      //int plf_consume, plf_total_consume=0;
      int plf_consume;
      //int plf_out, costas_out; // costas is sync block, input num = output num

      int c_count=0, p_count=0;
      //in original design, noutput_items meant for output buffer size!!!!
      int nsample = (sensing_result.size() * window)/d_plf_sps; // equivalent available output size
      if(nsample==0){
        return;
      }
      int sample_start = (d_sample_idx >= d_plf_history)? d_sample_idx - d_plf_history : 0;
      p_count = plf_core(d_plf_symbol_buffer, d_plf_time_error, d_sample_buffer+sample_start,nsample, plf_consume);
      c_count = costas_core(d_process_buffer+d_process_size,d_costas_phase_error, d_plf_symbol_buffer, p_count);
      d_sample_idx += plf_consume;
      d_process_size += c_count;

      /*
      if(d_debug){
        pmt::pmt_t debug_info = pmt::make_dict();
        debug_info = pmt::dict_add(debug_info, pmt::intern("polyphase output"),pmt::from_long(p_count));
        debug_info = pmt::dict_add(debug_info, pmt::intern("polyphase consume"), pmt::from_long(plf_consume));
        debug_info = pmt::dict_add(debug_info, pmt::intern("costas output"),pmt::from_long(c_count));
        message_port_pub(d_debug_port,debug_info);
      }*/

      /*
      int info_iter = sensing_result.size();
      if(info_iter==0)
        return;
      int max_len = info_iter*window;
      int info_count=0;
      while( (info_count<info_iter) && ( (plf_total_consume + window) < max_len)){
          if(!sensing_result[info_count]){
            plf_out = plf_core(d_plf_symbol_buffer+p_count,d_plf_time_error+plf_total_consume, d_sample_buffer+d_sample_idx+plf_total_consume,window, plf_consume);
            costas_out = costas_core(d_process_buffer+d_process_idx+c_count, d_costas_phase_error+c_count,d_plf_symbol_buffer+p_count, plf_out);
            p_count += plf_out;
            plf_total_consume += plf_consume;
            c_count += costas_out;
          }
          else{
            plf_total_consume += window;
            p_count += (window / d_plf_sps);
            c_count += (window / d_plf_sps);
          }
          //skip since this window is interfered by su
          info_count = floor(plf_total_consume/ window);
      }
      d_sample_idx += (plf_total_consume > max_len)? plf_total_consume-window : plf_total_consume;
      d_process_size += (plf_total_consume > max_len)? c_count - (window/d_plf_sps):c_count;
      */
      //TODO: record freq and time error and rebuild interfering signal for cancellation
    }

    void
    prou_sample_receiver_cb_impl::interference_detector(std::vector<bool>& result, int window)
    {
      double thres_db = -50.0;
      int start_index = d_sample_idx;
      int iter = (d_sample_size-d_sample_idx)/window;
      for(int i=0;i<iter;++i){
        result.push_back(calc_var_energy(d_sample_buffer+d_sample_idx+i*window,window,thres_db));
      }
      
    }

    bool
    prou_sample_receiver_cb_impl::append_samples(const gr_complex* in, int size, int& consume)
    {
      // forecast should handle the input sample length carefully
      /*
      while(d_sample_size +size > d_sample_cap)
      {
        double_cap(); // sample and symbol both update
        if(d_sample_cap > 64*d_sample_cap_init)
        {
          return false;//failed, force reset
        }
      }
      */
      
      switch(d_intf_state)
      {
        case CLEAR:
          if(d_sample_size == d_sample_cap){
            reduce_sample(2);
          }
        break;
        case RETRANSMISSION:
          while(d_sample_size +size > d_sample_cap){
            double_cap(); // sample and symbol both update
            if(d_sample_cap > 64*d_sample_cap_init){
              //failed, force reset
              return false;
            }
          }
        break;
        default:
          std::runtime_error("ProU RX: Func<append_samples>: Entering wrong state");
        break;
      }
      
      consume = (size > (d_sample_cap - d_sample_size))? (d_sample_cap - d_sample_size) : size;
      memcpy(d_sample_buffer + d_sample_size, in, sizeof(gr_complex)*consume);
      d_sample_size += consume;
      
      return true;
    }


    int
    prou_sample_receiver_cb_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
      int true_output = 0;
      noutput_items = (noutput_items > ninput_items[0])? ninput_items[0] : noutput_items;
      // Do <+signal processing+>
      int consume=0;
      std::vector<bool> sensing_result;
      int window = 256;

      switch(d_mode)
      {
        case STANDARD:
          //do nothing
          memcpy(out, in, sizeof(gr_complex)*noutput_items);
          true_output = noutput_items;
        break;

        case INTERFERENCE_CANCELLATION:
          // append samples to process_buffer
          if(!append_samples(in, noutput_items, consume)){
            //avoid memory overflow, force reset
            reset_buffer();
            reset_intf_reg();
            d_state = SEARCH_ACCESSCODE;
          }
            interference_detector(sensing_result,window);
            su_sample_sync(sensing_result,window);
          if(process_symbols()){
            //handling the output flow
            true_output = output_samples(out, noutput_items);
          }
        break;
        default:
          std::runtime_error("ProU RX: Wrong Receiver Mode!");
        break;
      }

      /*
      if(d_debug){
        pmt::pmt_t debug_info = pmt::make_dict();
        debug_info = pmt::dict_add(debug_info, pmt::intern("sensing size"),pmt::from_long(sensing_result.size()));
        debug_info = pmt::dict_add(debug_info, pmt::intern("sample_size"),pmt::from_long(d_sample_size));
        debug_info = pmt::dict_add(debug_info, pmt::intern("symbol_size"),pmt::from_long(d_process_size));
        message_port_pub(d_debug_port,debug_info);
      }
      */
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (consume);

      // Tell runtime system how many output items we produced.
      return true_output;
    }

  } /* namespace lsa */
} /* namespace gr */

