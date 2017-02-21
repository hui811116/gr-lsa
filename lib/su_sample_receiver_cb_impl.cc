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
#include "su_sample_receiver_cb_impl.h"
#include <gnuradio/blocks/pdu.h>
#include <gnuradio/math.h>
#include <gnuradio/sincos.h>
#include <gnuradio/expj.h>
#include <sstream>

namespace gr {
  namespace lsa {

#define M_TWOPI (2.0f*M_PI)

    su_sample_receiver_cb::sptr
    su_sample_receiver_cb::make(
      const std::string& sensing_tag_id,
      const std::string& accesscode,
      const gr::digital::constellation_sptr& hdr_const,
      const gr::digital::constellation_sptr& pld_const,
      double sps,
      float loop_bw,
      const std::vector<float>&taps,
      unsigned int filter_size,
      float init_phase,
      float max_rate_deviation,
      int osps,
      int cos_order,
      bool debug,
      bool sync)
    {
      return gnuradio::get_initial_sptr
        (new su_sample_receiver_cb_impl(
          sensing_tag_id,
          accesscode,
          hdr_const,
          pld_const,
          sps,
          loop_bw,
          taps,
          filter_size,
          init_phase,
          max_rate_deviation,
          osps,
          cos_order,
          debug,
          sync));
    }

    enum SuRxState{
      SU_ONLY,
      INTERFERING
    };

    enum SyncState{
      SEARCH_SYNC_CODE,
      SYNC_WAIT_HEADER,
      WAIT_PAYLOAD
    };
    /*
     * The private constructor
     */
    su_sample_receiver_cb_impl::su_sample_receiver_cb_impl(
      const std::string& sensing_tag_id,
      const std::string& accesscode, 
      const gr::digital::constellation_sptr& hdr_const,
      const gr::digital::constellation_sptr& pld_const,
      double sps,
      float loop_bw,
      const std::vector<float>&taps,
      unsigned int filter_size,
      float init_phase,
      float max_rate_deviation,
      int osps,
      int cos_order,
      bool debug,
      bool sync)
      : gr::block("su_sample_receiver_cb",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(0, 0, 0)),
        d_src_id(pmt::intern(alias())),
        d_nfilters(filter_size),
        d_max_dev(max_rate_deviation),
        d_osps(osps),
        d_error(0),
        d_out_idx(0),
        d_cos_order(cos_order),
        d_cos_error(0)
    {
      d_hdr_sptr = hdr_const->base();
      d_pld_sptr = pld_const->base();
      d_hdr_bps = hdr_const->bits_per_symbol();
      d_pld_bps = pld_const->bits_per_symbol();

      d_state = SU_ONLY;
      d_bit_state = SEARCH_SYNC_CODE;

      d_msg_port = pmt::mp("info");
      d_pkt_port = pmt::mp("packet");
      d_debug_port = pmt::mp("debug");  //debug
      d_sensing_tag_id = pmt::string_to_symbol(sensing_tag_id);

      d_cap = 1024*64;
      d_byte_count = 0;
  
      //set_max_noutput_items(d_cap);
      if(!set_accesscode(accesscode)){
        throw std::runtime_error("SU Receiver: Setting access code failed");
      }
      d_byte_reg = new unsigned char[d_cap];
      d_symbol_to_bytes = new unsigned char[d_cap];
      d_debug = debug;
      d_use_sync = sync;
      set_min_noutput_items(512);
      message_port_register_out(d_msg_port);
      message_port_register_out(d_pkt_port);
      message_port_register_out(d_debug_port); //debug
      set_tag_propagation_policy(TPP_DONT);
      //synchronizer
      d_nfilters = filter_size;
      d_sps = floor(sps);

      d_damping = 2*d_nfilters;
      d_loop_bw = loop_bw;
      d_k = d_nfilters/2.0;
      d_rate = (sps-floor(sps))*(double)d_nfilters;
      d_rate_i = (int)floor(d_rate);
      d_rate_f = d_rate - (float)d_rate_i;
      d_filtnum = (int)floor(d_k);

      d_filters = std::vector<gr::filter::kernel::fir_filter_ccf*>(d_nfilters);
      d_diff_filters = std::vector<gr::filter::kernel::fir_filter_ccf*>(d_nfilters);

      std::vector<float> vtaps(1,0);
      for(int i=0;i<d_nfilters;i++){
        d_filters[i] = new gr::filter::kernel::fir_filter_ccf(1,vtaps);
        d_diff_filters[i] = new gr::filter::kernel::fir_filter_ccf(1, vtaps);
      }

      //update gains
      float denom = (1.0 + 2.0*d_damping*d_loop_bw +d_loop_bw*d_loop_bw);
      d_alpha = (4*d_damping*d_loop_bw) / denom;
      d_beta = (4*d_loop_bw*d_loop_bw) / denom;

      const size_t max_size = 8192;
      d_plf_symbols = new gr_complex[max_size];
      d_plf_size = 0;

      // costas loop
      d_cos_symbols = new gr_complex[max_size];
      d_cos_damping = sqrtf(2.0f)/2.0f;
      denom = (1.0 + 2.0*d_cos_damping*d_loop_bw+d_loop_bw * d_loop_bw);
      d_cos_alpha = (4* d_cos_damping*d_loop_bw)/ denom;
      d_cos_beta = (4*d_loop_bw*d_loop_bw)/denom;
      d_cos_phase =0 ;
      d_cos_freq = 0;
      switch(d_cos_order)
      {
        case 2:
          d_phase_detector = &su_sample_receiver_cb_impl::phase_detector_2;
        break;
        case 4:
          d_phase_detector = &su_sample_receiver_cb_impl::phase_detector_4;
        break;
        default:
          throw std::invalid_argument("order must be 2 or 4");
        break;
      }
      if(d_use_sync){
        set_relative_rate((float)d_osps/(float)d_sps);
        std::vector<float> dtaps;
        create_diff_taps(taps, dtaps);
        set_taps(taps, d_taps, d_filters);
        set_taps(dtaps, d_dtaps, d_diff_filters);
      }
    }

    /*
     * Our virtual destructor.
     */
    su_sample_receiver_cb_impl::~su_sample_receiver_cb_impl()
    {
      delete [] d_byte_reg;
      delete [] d_symbol_to_bytes;
      for(int i=0;i<d_nfilters;++i){
        delete d_filters[i];
        delete d_diff_filters[i];
      }
      delete [] d_plf_symbols;
      delete [] d_cos_symbols;
    }

    //synchronizer
    float
    su_sample_receiver_cb_impl::phase_detector_2(const gr_complex& sample) const
    {
      return (sample.real()*sample.imag());
    }
    float
    su_sample_receiver_cb_impl::phase_detector_4(const gr_complex& sample) const
    {
      return ((sample.real()>0 ? 1.0 : -1.0) * sample.imag() -
        (sample.imag()>0 ? 1.0 : -1.0)*sample.real());
    }
    int
    su_sample_receiver_cb_impl::costas_core(
      gr_complex* out,
      const gr_complex* in,
      int noutput_items,
      float* error,
      float* outphase,
      float* outfreq,
      bool output_error,
      std::vector<tag_t>& tags)
    {
      int state_copy = d_state;
      gr_complex nco_out;
      for(int i=0;i<noutput_items;++i){
        if(!tags.empty()){
          if(tags[0].offset == (size_t)i){
            state_copy = (pmt::to_bool(tags[0].value) ? INTERFERING : SU_ONLY);
            if(state_copy == INTERFERING){
              d_cos_prev_error = d_cos_error;
              d_cos_prev_phase = d_cos_phase;
              d_cos_prev_freq = d_cos_freq;
            }
            else if(state_copy == SU_ONLY){
              d_cos_error = d_cos_prev_error;
              d_cos_phase = d_cos_prev_phase;
              d_cos_freq = d_cos_prev_freq;
            }
            tags.erase(tags.begin());
          }
        }
        nco_out = gr_expj(-d_cos_phase);
        out[i] = in[i] * nco_out;

        d_cos_error = (*this.*d_phase_detector)(out[i]);
        d_cos_error = gr::branchless_clip(d_cos_error,1.0);

        d_cos_freq = d_cos_freq + d_cos_error * d_cos_beta;
        d_cos_phase = d_cos_phase + d_cos_error * d_cos_alpha + d_cos_freq;
        while(d_cos_phase > M_TWOPI){
          d_cos_phase -= M_TWOPI;
        }
        while(d_cos_phase< -M_TWOPI){
          d_cos_phase += M_TWOPI;
        }
        if(d_cos_freq > 1.0){
          d_cos_freq = 1.0;
        }
        else if(d_cos_freq< -1.0){
          d_cos_freq = -1.0;
        }
        if(output_error){
          error[i] = d_cos_error;
          outphase[i] = d_cos_phase;
          outfreq[i] = d_cos_freq;
        }
      }
      return noutput_items;
    }
    //polyphase
    void
    su_sample_receiver_cb_impl::create_diff_taps(
      const std::vector<float>& newtaps,
      std::vector<float>&difftaps)
    {
      std::vector<float> diff_filter(3);
      diff_filter[0]=-1;
      diff_filter[1]=0;
      diff_filter[2]=1;

      float pwr = 0;
      difftaps.clear();
      difftaps.push_back(0);
      for(unsigned int i =0;i <newtaps.size()-2;i++){
        float tap =0;
        for(unsigned int j=0;j<diff_filter.size();j++){
          tap += diff_filter[j]*newtaps[i+j];
        }
        difftaps.push_back(tap);
        pwr += fabsf(tap);
      }
      difftaps.push_back(0);

      for(unsigned int i=0;i< difftaps.size();i++){
        difftaps[i]*= d_nfilters/pwr;
        if(difftaps[i] != difftaps[i]){
          throw std::runtime_error("SU_RX::create_diff_taps produced NaN");
        }
      }
    }

    void
    su_sample_receiver_cb_impl::set_taps(
      const std::vector<float>&newtaps,
      std::vector< std::vector<float> >&ourtaps,
      std::vector<gr::filter::kernel::fir_filter_ccf*> &ourfilter)
    {
      int i,j;

      unsigned int ntaps = newtaps.size();
      d_taps_per_filter = (unsigned int) ceil((double)ntaps/(double)d_nfilters);

      ourtaps.resize(d_nfilters);
      std::vector<float> tmp_taps = newtaps;
      while((float)(tmp_taps.size()) < d_nfilters*d_taps_per_filter){
        tmp_taps.push_back(0);
      }

      for(i=0;i<d_nfilters;i++){
        ourtaps[i] = std::vector<float>(d_taps_per_filter,0);
        for(j=0; j<d_taps_per_filter;j++){
          ourtaps[i][j] = tmp_taps[i+ j*d_nfilters];
        }
        ourfilter[i]->set_taps(ourtaps[i]);
      }
      set_history(d_taps_per_filter + d_sps + d_sps);
      set_output_multiple(d_osps);
    }

    int
    su_sample_receiver_cb_impl::plf_core(
      gr_complex* out,
      const gr_complex* in,
      int noutput_items,
      float* error,
      float* outf,
      float* outk,
      bool output_error,
      int ninput_items,
      int& nconsume,
      std::vector<tag_t>& plf_tags)
    {
      int i=0, count = 0;
      float error_r,error_i;
      int state_copy = d_state;
      // NOTE::if there are time_est tags, refer to original code and shift d_k here

      std::vector<tag_t> tags;
      get_tags_in_window(tags, 0,0,d_sps*noutput_items, d_sensing_tag_id);
      while(i < noutput_items) {
        if(!tags.empty()){
          size_t offset = tags[0].offset-nitems_read(0);
          //if(d_debug){
            //std::stringstream ss;
            //ss <<"offset"<< tags[0].offset<<"key="<<tags[0].key<< " value="<<tags[0].value;
            //GR_LOG_DEBUG(d_logger,ss.str());  
          //}
          if((offset >= (size_t)count) && (offset<(size_t)(count + d_sps))) {
            if(pmt::to_bool(tags[0].value) && (state_copy == SU_ONLY)){
              d_prev_k = d_k;
              d_prev_rate_f = d_rate_f;
              d_prev_filtnum = d_filtnum;
              d_prev_error = d_error;
              d_prev_out_idx = d_out_idx;
              tag_t tag;
              tag.offset = i;
              tag.value = pmt::from_bool(true);
              plf_tags.push_back(tag);
              state_copy = !state_copy;
            }
            else if((!pmt::to_bool(tags[0].value)) && (state_copy == INTERFERING)){
              //GR_LOG_DEBUG(d_logger,"ERROR: should not enter this case");
              d_k = d_prev_k;
              d_rate_f = d_prev_rate_f;
              d_filtnum = d_prev_filtnum;
              d_error = d_prev_error;
              d_out_idx = d_prev_out_idx;
              tag_t tag;
              tag.offset = i;
              tag.value = pmt::from_bool(false);
              plf_tags.push_back(tag);
              state_copy = !state_copy;
            }
            tags.erase(tags.begin());
          }
        }
        while(d_out_idx < d_osps){
          d_filtnum = (int)floor(d_k);
          while(d_filtnum >=d_nfilters){
            d_k -= d_nfilters;
            d_filtnum -= d_nfilters;
            count +=1;
          }
          while(d_filtnum < 0){
            d_k += d_nfilters;
            d_filtnum += d_nfilters;
            count -= 1;
          }
          out[i+d_out_idx] = d_filters[d_filtnum]->filter(&in[count+d_out_idx]);
          d_k = d_k + d_rate_i + d_rate_f;

          d_out_idx++;
          if(output_error){
            error[i] = d_error;
            outf[i] = d_rate_f;
            outk[i] = d_k;
          }

          if(i+d_out_idx >= noutput_items){
            nconsume = count;
            return i;
          }
        }
        d_out_idx = 0;

        gr_complex diff = d_diff_filters[d_filtnum]->filter(&in[count]);
        error_r = out[i].real() * diff.real();
        error_i = out[i].imag() * diff.imag();
        d_error = (error_i + error_r)/2.0 ;

        for(int s=0;s<d_sps;s++){
          d_rate_f = d_rate_f + d_beta*d_error;
          d_k = d_k +d_alpha*d_error + d_rate_f;
        }
        d_rate_f = gr::branchless_clip(d_rate_f, d_max_dev);
        i+= d_osps;
        count += (int)floor(d_sps);
      }
      nconsume = count;
      return i;
    }

    void
    su_sample_receiver_cb_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
       
      ninput_items_required[0]= (noutput_items + history()) * (d_sps/d_osps);
    }

    void
    su_sample_receiver_cb_impl::pub_byte_pkt()
    {  
        int bits_count = d_pld_bps * d_byte_count;
        int byte_count = bits_count/8;
        unsigned char tmp;   
        
        for(int i=0;i<bits_count;++i){
          if(i%8==0){
            d_symbol_to_bytes[i/8]=0x00;
          }
          tmp = (d_byte_reg[i/d_pld_bps] >> (d_pld_bps-1-i%d_pld_bps)) & 0x01;
          d_symbol_to_bytes[i/8] =  ((d_symbol_to_bytes[i/8] >> (7-i%8)) | tmp ) << 7-i%8;
        }

        pmt::pmt_t pdu_meta = pmt::make_dict();
        pdu_meta = pmt::dict_add(pdu_meta, pmt::intern("payload_len"), pmt::from_long(d_payload_len));
        pdu_meta = pmt::dict_add(pdu_meta, pmt::intern("counter"), pmt::from_long(d_counter));
        
        d_pdu_vector = gr::blocks::pdu::make_pdu_vector(gr::blocks::pdu::byte_t,d_symbol_to_bytes,byte_count);
        pmt::pmt_t msg = pmt::cons(pdu_meta, d_pdu_vector);
        message_port_pub(d_pkt_port, msg);
    }


    void
    su_sample_receiver_cb_impl::feedback_info(bool type)
    {
      pmt::pmt_t sen_back = pmt::make_dict();
      //interfering case
      sen_back = pmt::dict_add(sen_back,d_sensing_tag_id,pmt::from_bool(type));
      if(!type)
      {
        //su pkt received
        sen_back = pmt::dict_add(sen_back,pmt::intern("payload"),pmt::from_long(d_payload_len));
        sen_back = pmt::dict_add(sen_back,pmt::intern("queue_index"),pmt::from_long(d_qidx));
        sen_back = pmt::dict_add(sen_back,pmt::intern("queue_size"),pmt::from_long(d_qsize));
        sen_back = pmt::dict_add(sen_back,pmt::intern("counter"),pmt::from_long(d_counter));
      }
      if(d_debug){
          std::stringstream ss;
          ss<<"SU_RX_feedback----->"<<((type)? "INTERFERING":"CLEAR");
          GR_LOG_DEBUG(d_logger, ss.str());
        }
      message_port_pub(d_msg_port,sen_back);
    }

    void
    su_sample_receiver_cb_impl::data_reg_reset()
    {
      d_bit_state = SEARCH_SYNC_CODE;
      d_byte_count=0;
    }

    size_t
    su_sample_receiver_cb_impl::header_nbits() const
    {
      return d_accesscode_len + 32+16+8+8;
    }

    bool
    su_sample_receiver_cb_impl::set_accesscode(const std::string& accesscode)
    {
      d_accesscode_len = accesscode.length();
      if(d_accesscode_len >64){
        return false;
      }
      d_mask = ((~0ULL) >> (64- d_accesscode_len) );
      d_accesscode=0;
      for(unsigned i=0;i<d_accesscode_len;++i){
        d_accesscode = (d_accesscode << 1) | (accesscode[i] & 1);
      }
      return true;
    }

    uint64_t
    su_sample_receiver_cb_impl::accesscode() const
    {
      return d_accesscode;
    }
    //helper function for header parsing
    uint16_t
    su_sample_receiver_cb_impl::_get_bit16(int begin_idx)
    {
      unsigned long tmp=0UL;
      for(int i=0;i<16;++i){
        tmp |= ((d_input[begin_idx+i])? 1 : 0) << (15-i);  // NOTE: this is how bits arrangement should be like
      }
      return tmp;
    }
    uint8_t
    su_sample_receiver_cb_impl::_get_bit8(int begin_idx)
    {
      unsigned char tmp=0;
      for(int i=0;i<8;++i){
        tmp |= ((d_input[begin_idx+i])? 1:0 ) << (7-i);
      }
      return tmp;
    }

    bool
    su_sample_receiver_cb_impl::parse_header()
    {
      uint16_t len0,len1;    
      len0 = _get_bit16(0);
      len1 = _get_bit16(16);
      if(len0 == len1){
        d_payload_len=len0;
        d_counter = _get_bit16(32);
        d_qidx = _get_bit8(48);
        d_qsize = _get_bit8(56);
        if(d_debug){
          std::stringstream ss;
          ss<<"Packet found-->"<<"payload_len:"<<len0<<", counter:"<<d_counter<<", qidx:"<<(int)d_qidx<<", qsize:"<<(int)d_qsize;
          GR_LOG_DEBUG(d_logger,ss.str());
        }
        return true;
      }
      return false;
    }

    bool
    su_sample_receiver_cb_impl::insert_parse_byte(const gr_complex& sample)
    {
      unsigned char byte_hold; 
      if(d_byte_count == d_cap){
        d_byte_count =0;
        GR_LOG_CRIT(d_logger, "SU Receiver: Reaching maximum capacity, reset to initial.");
      }
      switch(d_bit_state)
      {
        case SEARCH_SYNC_CODE:
          byte_hold = d_hdr_sptr->decision_maker(&sample);
          for(int i=0;i<d_hdr_bps;++i)
          {
            uint64_t check_bits = (~0ULL);
            d_data_reg = (d_data_reg << 1) | ((byte_hold >> (d_hdr_bps-1-i) )& 0x01 );
            check_bits = (d_data_reg ^ d_accesscode) & d_mask;
            if(check_bits == 0){
              d_bit_state = SYNC_WAIT_HEADER;
              d_input.clear();
            }
          } 
        break;
        case SYNC_WAIT_HEADER:
          byte_hold = d_hdr_sptr->decision_maker(&sample);
          for(int i=0;i<d_hdr_bps;++i){
            d_input.push_back( (((byte_hold >> (d_hdr_bps-1-i)) & 0x01)==0x00 )? false : true);
          }
          if(d_input.size() == (header_nbits()-d_accesscode_len) )
          {
            if(parse_header()){
              d_bit_state = WAIT_PAYLOAD;
            }
            else{
              d_bit_state = SEARCH_SYNC_CODE;
            }
            d_byte_count=0;
          }
        break;
        case WAIT_PAYLOAD:
          d_byte_reg[d_byte_count++] = d_pld_sptr->decision_maker(&sample);
          if(d_payload_len*8 == (d_byte_count * d_pld_sptr->bits_per_symbol()))
          {
            pub_byte_pkt();
            d_bit_state = SEARCH_SYNC_CODE;
            return true;
          }
        break;
        default:
          std::runtime_error("SU Receiver: Entering wrong bit processing state");
        break;
      }
      //MSB parsing
      return false;
    }

    int
    su_sample_receiver_cb_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      const gr_complex *out = (const gr_complex *) input_items[0];
      int count = ninput_items[0];
      int nin = ninput_items[0];
      float* plf_error;
      float* plf_outk;
      float* plf_outf;
      float* cos_error;
      float* cos_phase;
      float* cos_freq;
      std::vector<tag_t> plf_tags, state_handle;
      

      if(d_use_sync){
        d_plf_size = plf_core(d_plf_symbols, in, noutput_items, plf_error, plf_outf, plf_outk, 
          false, ninput_items[0], count, plf_tags);  
        state_handle = plf_tags;
        nin = costas_core(d_cos_symbols, d_plf_symbols, d_plf_size, cos_error, cos_phase, cos_freq, false, plf_tags);
        out = d_cos_symbols;
      }
      else{
        get_tags_in_window(plf_tags, 0,0,count, d_sensing_tag_id);
        for(int i=0;i<plf_tags.size();++i){
          tag_t tmp_tag = plf_tags[i];
          tmp_tag.offset  = tmp_tag.offset - nitems_read(0);
          state_handle.push_back(tmp_tag);
          /*if(d_debug){
            std::stringstream ss;  
            ss<<"offset="<<tmp_tag.offset<<" key="<<tmp_tag.key << " value="<<tmp_tag.value;
            GR_LOG_DEBUG(d_logger,ss.str());
          }*/           
        }
      }
      
      int idx_count;

      for(int i=0;i<nin;++i){
        if(!state_handle.empty()){
          idx_count = state_handle[0].offset;
          if(idx_count == i){
            d_state = (pmt::to_bool(state_handle[0].value) ? INTERFERING : SU_ONLY);
            if(d_state == INTERFERING){
              feedback_info(true);
              data_reg_reset();
            }
            state_handle.erase(state_handle.begin());
          }
        }
        switch(d_state)
        {
          case SU_ONLY:
          if(insert_parse_byte(out[i])){
            feedback_info(false);
            data_reg_reset();
          }
          break;
          case INTERFERING:
          break;
          default:
            std::runtime_error("SU_RX_general_work::entering wrong state");
          break;
        }
      }
      consume_each(count);
      return noutput_items;
      
    }

  } /* namespace lsa */
} /* namespace gr */

