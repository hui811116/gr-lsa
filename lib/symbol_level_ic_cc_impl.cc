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
#include "symbol_level_ic_cc_impl.h"
#include <algorithm>
#include <map>
#include <volk/volk.h>

namespace gr {
  namespace lsa {

    symbol_level_ic_cc::sptr
    symbol_level_ic_cc::make(
      const std::string& accesscode,
      const std::vector<gr_complex>& clean_preamble,
      int bps,
      bool debug)
    {
      return gnuradio::get_initial_sptr
        (new symbol_level_ic_cc_impl(accesscode,
          clean_preamble,
          bps,
          debug));
    }

    /*
     * The private constructor
     */
    static int ios[] = {sizeof(gr_complex), sizeof(float),sizeof(float),sizeof(float),sizeof(float)};
    static std::vector<int> iosig(ios, ios+ sizeof(ios)/sizeof(int));
    symbol_level_ic_cc_impl::symbol_level_ic_cc_impl(
      const std::string& accesscode,
      const std::vector<gr_complex>& clean_preamble,
      int bps,
      bool debug
      )
      : gr::block("symbol_level_ic_cc",
              gr::io_signature::makev(1, 5, iosig),
              gr::io_signature::make2(1, 2, sizeof(gr_complex), sizeof(float))),
      d_cap(1024*128),
      d_clean_preamble(clean_preamble)
    {
      if(!set_accesscode(accesscode)){
        throw std::invalid_argument("Invalid accesscode");
      }

      d_buffer = new gr_complex[d_cap];
      d_buf_size =0;
      d_buf_idx =0;

      d_output_buffer = new gr_complex[d_cap];
      d_out_size = 0;
      d_out_idx = 0;
      
      d_bps = bps;
      d_hdr_len = (d_accesscode_len+64) / d_bps; //symbol length
      d_debug = debug;

      d_info_count =0;

      d_eng = (float*) volk_malloc(sizeof(float)*d_cap, volk_get_alignment());
      d_sensing = pmt::string_to_symbol("sensing");
    }

    /*
     * Our virtual destructor.
     */
    symbol_level_ic_cc_impl::~symbol_level_ic_cc_impl()
    {
      delete [] d_buffer;
      delete [] d_output_buffer;
      volk_free(d_eng);
    }

    bool
    symbol_level_ic_cc_impl::set_accesscode(const std::string& accesscode)
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

    bool
    symbol_level_ic_cc_impl::ic_detector()
    {
      int qidx, qsize, pld;
      bool reset=false;
      for(d_info_count;d_info_count<d_buf_info.size();d_info_count++)
      {
        qidx = pmt::to_long(pmt::dict_ref(d_buf_info[d_info_count], pmt::intern("queue_index"), pmt::PMT_NIL));
        qsize = pmt::to_long(pmt::dict_ref(d_buf_info[d_info_count], pmt::intern("queue_size"), pmt::PMT_NIL));
        pld = pmt::to_long(pmt::dict_ref(d_buf_info[d_info_count], pmt::intern("payload"), pmt::PMT_NIL));
        if(d_info_idx[d_info_count]+pld > d_buf_size){
          //symbols not enough
          break;
        }
        if( (!d_retx_buffer.empty()) && (qsize!=d_retx_buffer.size())   ){
          reset = true;
          //update buffer queue only if previous queue size not matched
          break;
        }
        if(qsize!=0 && d_retx_buffer.empty()){
          d_retx_buffer.resize(qsize,NULL);
          d_retx_idx.resize(qsize,0);
          d_retx_pld.resize(qsize,0);
        }
        if(!d_retx_buffer.empty() && (qsize == d_retx_buffer.size()) ){
          if(qidx>qsize){
            GR_LOG_WARN(d_logger, "queue index greater than size, skip");
            continue;
            //FIXME
          }
          if(d_retx_buffer[qidx]==NULL){
            d_retx_buffer[qidx] = new gr_complex[d_hdr_len+pld+d_bps]; // d_bps as guard
            d_retx_idx[qidx] = d_info_idx[d_info_count];
            d_retx_count++;
            d_retx_pld[qidx] = pld;
          }  
        }
        if(d_retx_count!=0 && (d_retx_count== d_retx_buffer.size()) ){
          // interference cancellation available!
          // push end index.
          d_end_idx.push_back(d_info_idx[d_info_count]+pld);
          return true;
        }
      }
      if(reset){
        clean_buffer();
        clean_retx();
      }
      return false;
    }

    void
    symbol_level_ic_cc_impl::clean_buffer()
    {
        int end_idx = d_info_idx[d_info_count] - d_hdr_len+1;
        if(end_idx<0){
          GR_LOG_WARN(d_logger, "Begin index is negative, resort to 0");
          end_idx =0;
        }
        memcpy(d_buffer, d_buffer+end_idx, sizeof(gr_complex)*(d_buf_size-end_idx));
        d_buf_size-=end_idx;
        d_buf_idx = 0;
        
        if(d_info_count!=0){
          d_buf_info.erase(d_buf_info.begin(),d_buf_info.begin()+d_info_count);
          d_info_idx.erase(d_info_idx.begin(),d_info_idx.begin()+d_info_count);
          d_info_count =0;
          for(int i=0;i<d_info_idx.size();++i){
            d_info_idx[i]-=end_idx;
          }  
        }
    }
    void
    symbol_level_ic_cc_impl::clean_retx()
    {
      for(int j=0;j<d_retx_buffer.size();++j){
            if(d_retx_buffer[j]!=NULL){
              delete [] d_retx_buffer[j];
            }
          }
      d_retx_count=0;
      d_retx_buffer.clear();
      d_retx_idx.clear();
      d_retx_pld.clear();
    }
    void
    symbol_level_ic_cc_impl::do_interference_cancellation()
    {
      int end_idx = d_end_idx.front();
      d_end_idx.erase(d_end_idx.begin());
      memcpy(d_output_buffer+d_out_idx, d_buffer, sizeof(gr_complex)*end_idx);
      d_out_size+=end_idx;
    }

    void
    symbol_level_ic_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      int items_reqd = d_cap - d_buf_size;
      items_reqd = (noutput_items>items_reqd)? items_reqd : noutput_items;
      for(int i=0;i<ninput_items_required.size();++i){
        ninput_items_required[i] = items_reqd;
      }
    }

    int
    symbol_level_ic_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
      const float *freq, *phase, *rate_f,*rate_k;
      bool have_sync = (input_items.size()>=5);
      bool out_eng = (output_items.size()>=2);
      float* eng;
      if(out_eng){
        eng =(float*) output_items[1];
      }
      
      if(have_sync){
        freq = (const float*) input_items[1];
        phase = (const float*) input_items[2];
        rate_f = (const float*) input_items[3];
        rate_k = (const float*) input_items[4]; 
      }
      int consume = d_cap-d_buf_size;
      int count=0;
      consume = (ninput_items[0]>consume) ? consume : ninput_items[0];
      memcpy(d_buffer, in, sizeof(gr_complex)* consume);
      d_buf_size+=consume;
      std::vector<tag_t> tags, qs_tags, qi_tags, c_tags, pld_tags;

      get_tags_in_window(tags,0,0,consume,pmt::intern("LSA_hdr"));
      get_tags_in_window(qs_tags,0,0,consume,pmt::intern("queue_size"));
      get_tags_in_window(qi_tags,0,0,consume,pmt::intern("queue_index"));
      get_tags_in_window(c_tags,0,0,consume,pmt::intern("counter"));
      get_tags_in_window(pld_tags,0,0,consume,pmt::intern("payload"));
      while(!tags.empty()){
        pmt::pmt_t tmp_dict = pmt::make_dict();
        tmp_dict = pmt::dict_add(tmp_dict,qs_tags[0].key,qs_tags[0].value);
        tmp_dict = pmt::dict_add(tmp_dict,qi_tags[0].key,qi_tags[0].value);
        tmp_dict = pmt::dict_add(tmp_dict,c_tags[0].key,c_tags[0].value);
        tmp_dict = pmt::dict_add(tmp_dict,pld_tags[0].key,pld_tags[0].value);

        d_buf_info.push_back(tmp_dict);
        d_info_idx.push_back(tags[0].offset - nitems_read(0)+d_buf_size);
        tags.erase(tags.begin());
        qs_tags.erase(qs_tags.begin());
        qi_tags.erase(qi_tags.begin());
        c_tags.erase(c_tags.begin());
        pld_tags.erase(pld_tags.begin());
      }
      if(ic_detector()){
        //do_interference_cancellation;
        do_interference_cancellation();
        clean_retx();
        clean_buffer();
        //reset after cancellation
      }
      // Do <+signal processing+>
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (consume);

      // Tell runtime system how many output items we produced.
      if(d_out_idx<d_out_size){
        count = d_out_size-d_out_idx;
        count = (noutput_items>count)?count: noutput_items;
        memcpy(out, d_output_buffer+d_out_idx, sizeof(gr_complex)*count);
        if(out_eng){
          volk_32fc_magnitude_squared_32f(d_eng, d_output_buffer+d_out_idx, count);
          memcpy(eng, d_eng, sizeof(float)*count);
        }
      }
      if(2*d_out_idx>d_cap){
        memcpy(d_output_buffer, d_output_buffer+d_out_idx, sizeof(gr_complex)* (d_cap-d_out_idx));
        d_out_size -= d_out_idx;
        d_out_idx = 0;
      }

      return count;
    }

  } /* namespace lsa */
} /* namespace gr */

