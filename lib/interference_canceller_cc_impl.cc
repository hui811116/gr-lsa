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
#include "interference_canceller_cc_impl.h"

namespace gr {
  namespace lsa {

    enum cancellerState{
      RECEIVE,
      OUTPUT
    };

    interference_canceller_cc::sptr
    interference_canceller_cc::make(const std::vector<gr_complex>& clean_preamble,
      const std::string& sensing_tagname,
      int sps,
      bool debug)
    {
      return gnuradio::get_initial_sptr
        (new interference_canceller_cc_impl(clean_preamble,
          sensing_tagname,
          sps,
          debug));
    }

    /*
     * The private constructor
     */
    interference_canceller_cc_impl::interference_canceller_cc_impl(const std::vector<gr_complex>& clean_preamble,
      const std::string& sensing_tagname,
      int sps,
      bool debug)
      : gr::block("interference_canceller_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make2(1, 2, sizeof(gr_complex), sizeof(float))),
      d_clean_preamble(clean_preamble),
      d_sps(sps)
    {
      d_sensing_tagname = pmt::string_to_symbol(sensing_tagname);
      const size_t capacity = 1024*4096;
      d_sample_buffer = new gr_complex[capacity];
      d_sample_size =0;
      d_sample_idx =0;
      d_output_buffer = new gr_complex[capacity];
      d_output_size =0;
      d_output_idx = 0;

      d_retx_buffer.clear(); 
      d_retx_pkt_size.clear();
      d_buffer_info.clear();

      d_state = RECEIVE;
      d_debug = debug;
      set_tag_propagation_policy(TPP_DONT);
    }

    /*
     * Our virtual destructor.
     */
    interference_canceller_cc_impl::~interference_canceller_cc_impl()
    {
      delete [] d_sample_buffer;
      delete [] d_output_buffer;
      for(int i=0;i<d_retx_buffer.size();++i){
        delete [] d_retx_buffer[i];
      }
      d_retx_buffer.clear();
    }

    void
    interference_canceller_cc_impl::retx_handler(pmt::pmt_t hdr_info, const gr_complex* in)
    {
      int retx_size, retx_idx, payload_size=0;
      //float phase_est,freq_est, time_rate_est,time_k_est;
      if(pmt::dict_has_key(hdr_info, pmt::intern("retx_size"))){
        retx_size = pmt::to_long(pmt::dict_ref(hdr_info, pmt::intern("retx_size"), pmt::PMT_NIL));
      }
      if(pmt::dict_has_key(hdr_info, pmt::intern("retx_idx"))){
        retx_idx = pmt::to_long(pmt::dict_ref(hdr_info, pmt::intern("retx_idx"), pmt::PMT_NIL));
      }
      if(pmt::dict_has_key(hdr_info, pmt::intern("payload"))){
        payload_size = pmt::to_long(pmt::dict_ref(hdr_info, pmt::intern("payload"), pmt::PMT_NIL));
      }
      if(d_retx_buffer.empty()){
        d_retx_buffer.resize(retx_size);
        d_retx_pkt_size.resize(retx_size);
        d_retx_count =0;
      }
      if(payload_size ==0){
        throw std::runtime_error("no payload length found");
      }
      //FIXME
      // can add a header length;
      gr_complex* retx = new gr_complex[payload_size];
      //FIXME
      for(int i=0;i<payload_size;++i){
        retx[i] = in[i];
      }
      d_retx_buffer[retx_idx] = retx;
      d_retx_count++;
      //FIXME
      // can add a header length
      d_retx_pkt_size[retx_idx] = payload_size;
    }

    void
    interference_canceller_cc_impl::header_handler(pmt::pmt_t hdr_info, int index)
    {
      int payload_len=0;
      //for debugging not relevent to interference cancellation
      int counter, qidx, qsize;
      float freq, phase, time_freq, time_phase;
      bool sensing_info=true;
      if(pmt::dict_has_key(hdr_info, d_sensing_tagname)){
        sensing_info = pmt::to_bool(pmt::dict_ref(hdr_info, d_sensing_tagname,pmt::PMT_NIL));
      }
      if(pmt::dict_has_key(hdr_info, pmt::intern("payload"))){
        payload_len = pmt::to_long(pmt::dict_ref(hdr_info, pmt::intern("payload"),pmt::PMT_NIL));
      }
      if(pmt::dict_has_key(hdr_info, pmt::intern("counter"))){
        counter = pmt::to_long(pmt::dict_ref(hdr_info, pmt::intern("counter"),pmt::PMT_NIL));
      }
      if(pmt::dict_has_key(hdr_info, pmt::intern("queue_index"))){
        qidx = pmt::to_long(pmt::dict_ref(hdr_info, pmt::intern("queue_index"),pmt::PMT_NIL));
      }
      if(pmt::dict_has_key(hdr_info, pmt::intern("queue_size"))){
        qsize = pmt::to_long(pmt::dict_ref(hdr_info, pmt::intern("queue_size"),pmt::PMT_NIL));
      }
      if(pmt::dict_has_key(hdr_info, pmt::intern("time_rate_f_est"))){
        time_freq = pmt::to_float(pmt::dict_ref(hdr_info, pmt::intern("time_rate_f_est"), pmt::PMT_NIL));
      }
      if(pmt::dict_has_key(hdr_info, pmt::intern("time_k_est"))){
        time_phase = pmt::to_float(pmt::dict_ref(hdr_info, pmt::intern("time_k_est"), pmt::PMT_NIL)); 
      }
      if(pmt::dict_has_key(hdr_info, pmt::intern("freq_est"))){
        freq = pmt::to_float(pmt::dict_ref(hdr_info, pmt::intern("freq_est"), pmt::PMT_NIL));
      }
      if(pmt::dict_has_key(hdr_info, pmt::intern("phase_est"))){
        phase = pmt::to_float(pmt::dict_ref(hdr_info, pmt::intern("phase_est"), pmt::PMT_NIL));  
      }

      d_buffer_info.push_back(hdr_info);
      d_info_index.push_back(index);

    }

    void
    interference_canceller_cc_impl::do_interference_cancellation()
    {
      if(d_debug){
        std::cout<<"<Debug>: do interference cancellation"<<std::endl;
        std::cout<<"header info:"<<std::endl;
        for(int i=0;i<d_buffer_info.size();++i){
          pmt::pmt_t tmp_dict=d_buffer_info[i];
          tmp_dict = pmt::dict_delete(tmp_dict, pmt::intern("time_rate_f_est"));
          tmp_dict = pmt::dict_delete(tmp_dict, pmt::intern("time_k_est"));
          tmp_dict = pmt::dict_delete(tmp_dict, pmt::intern("freq_est"));
          tmp_dict = pmt::dict_delete(tmp_dict, pmt::intern("phase_est"));
          std::cout<<"index:"<<d_info_index[i]<<" content:"<<tmp_dict<<std::endl;
        }
        std::cout<<"retx count:"<<d_retx_count<<std::endl;  
      }
      


      for(int i=0;i<d_sample_size;++i){
        d_output_buffer[i] = d_sample_buffer[i];
      }
      d_output_size = d_sample_size;
      d_output_idx =0;
    }

    void
    interference_canceller_cc_impl::output_result(int noutput_items, gr_complex* out)
    {
      if((d_output_idx == d_output_size) || d_output_size ==0 ){
        produce(0,0);
        return;
      }
      int out_count =0;
      int nout = (noutput_items > (d_output_size-d_output_idx)) ? (d_output_size-d_output_idx) : noutput_items;
      for(int i=0;i<nout;++i){
        out[i] = d_output_buffer[d_output_idx++];
      }
      produce(0,nout);
    }

    void
    interference_canceller_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      switch(d_state)
      {
        case RECEIVE:
          for(int i=0;i<ninput_items_required.size();++i){
             ninput_items_required[i] = noutput_items;
          }    
        break;
        case OUTPUT:
          for(int i=0;i<ninput_items_required.size();++i)
            ninput_items_required[i] = 0;
        break;
        default:
          std::runtime_error("Entering wrong state");
        break;
      }
      
    }

    int
    interference_canceller_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
      float* eng =  (float*) output_items[1];
      bool have_eng = output_items.size()>=2;
      int nin = (ninput_items[0]>noutput_items) ? noutput_items : ninput_items[0];
      // Do <+signal processing+>
      std::vector<tag_t> tags;
      std::vector<tag_t> end_tags;
      std::vector<tag_t> hdr_tags;
      get_tags_in_window(tags, 0, 0, nin, pmt::intern("begin_of_retx"));
      get_tags_in_window(end_tags, 0,0 ,nin, pmt::intern("end_of_retx"));
      get_tags_in_window(hdr_tags, 0,0 ,nin, pmt::intern("header_found"));
      if(!tags.empty()){
        int offset = tags.back().offset - nitems_read(0);
        d_sample_idx = 0;
        d_sample_size = 0;
      }
      
      for(int i=0;i<nin;++i){
        if(!hdr_tags.empty()){
          int offset = hdr_tags[0].offset - nitems_read(0);
          if(i==offset){
            std::vector<tag_t> tmp_tags;
            //window size?
            get_tags_in_window(tmp_tags, 0,i, i+4);
            pmt::pmt_t tmp_dict=pmt::make_dict();
            for(int j=0;j<tmp_tags.size();++j){
              tmp_dict = pmt::dict_add(tmp_dict, tmp_tags[j].key, tmp_tags[j].value);
            }
            if(pmt::dict_has_key(tmp_dict, pmt::intern("retx_idx"))){
              retx_handler(tmp_dict, in+i);
            }
            if(pmt::dict_has_key(tmp_dict, pmt::intern("header_found"))){
              header_handler(tmp_dict, d_sample_idx);
            }
            hdr_tags.erase(hdr_tags.begin());
          }
        }
        d_sample_buffer[d_sample_idx++] = in[i];
      }
      
      

      if(!end_tags.empty()){
        int offset = end_tags.back().offset - nitems_read(0);
        //output
        do_interference_cancellation();
        d_retx_count=0;
        for(int i=0;i<d_retx_buffer.size();++i){
          delete [] d_retx_buffer[i];
        }
        d_retx_buffer.clear();
        d_retx_pkt_size.clear();
        d_buffer_info.clear();
        d_info_index.clear();
        d_sample_idx = 0;
        d_sample_size =0;
      }

      output_result(noutput_items, out);

      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (nin);

      // Tell runtime system how many output items we produced.
      return WORK_CALLED_PRODUCE;
    }

  } /* namespace lsa */
} /* namespace gr */

