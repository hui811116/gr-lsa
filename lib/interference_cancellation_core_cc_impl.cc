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
#include "interference_cancellation_core_cc_impl.h"

namespace gr {
  namespace lsa {

#define DEBUG d_debug && std::cout

    static const pmt::pmt_t d_ring_tag= pmt::intern("VoE_detected");
    static const int MAXLEN = (127+6)*8*8/2*4;
    static const int MEMCAP = MAXLEN*128;

    interference_cancellation_core_cc::sptr
    interference_cancellation_core_cc::make(bool debug)
    {
      return gnuradio::get_initial_sptr
        (new interference_cancellation_core_cc_impl(debug));
    }

    /*
     * The private constructor
     */
    interference_cancellation_core_cc_impl::interference_cancellation_core_cc_impl(bool debug)
      : gr::block("interference_cancellation_core_cc",
              gr::io_signature::make3(3, 3, sizeof(gr_complex),sizeof(float),sizeof(float)),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
              d_mem_cap(MEMCAP)
    {
      d_in_mem = new gr_complex[d_mem_cap];
      d_out_mem= new gr_complex[d_mem_cap];
      d_phase_mem=new float[d_mem_cap];
      d_debug = debug;
      d_in_mem_idx =0;
      d_in_mem_size=0;
      d_phase_idx =0;
      d_phase_size=0;
      d_in_tlist.clear();
      d_out_tlist.clear();
    }

    /*
     * Our virtual destructor.
     */
    interference_cancellation_core_cc_impl::~interference_cancellation_core_cc_impl()
    {
      delete [] d_in_mem;
      delete [] d_out_mem;
      delete [] d_phase_mem;
    }

    void
    interference_cancellation_core_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      for(int i=0;i<ninput_items_required.size();++i){
        ninput_items_required[i] = noutput_items;
      }
    }

    int
    interference_cancellation_core_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      const float* phase_in= (const float*) input_items[1];
      const float* time_in = (const float*) input_items[2];
      gr_complex *out = (gr_complex*) output_items[0];
      int nin_s = std::min(ninput_items[0],(d_mem_cap-d_in_mem_size) );
      int nin_p = std::min(ninput_items[1],std::min(ninput_items[2],(d_mem_cap-d_phase_size)));
      int nout = 0;
      bool reset_s = false;
      bool reset_p = false;
      bool reset_ready = false;
      std::vector<tag_t> tags_s;
      std::vector<tag_t> tags_p;
      std::vector<tag_t> hdr_tag;
      get_tags_in_range(tags_s,0,nitems_read(0),nitems_read(0)+nin_s, d_ring_tag);
      get_tags_in_range(tags_p,1,nitems_read(1),nitems_read(1)+nin_p, d_ring_tag);
      if(!tags_s.empty()){
        int offset = tags_s[0].offset - nitems_read(0);
        if(offset < nin_s){
          //DEBUG<<"<IC Core DEBUG>Found ring tag port<0>:"<<tags_s[0].key<<" ,value="<<tags_s[0].value<<" ,at:"<<offset+d_in_mem_size<<std::endl;
          nin_s = offset;
          reset_s = true;
        }
      }
      if(!tags_p.empty()){
        int offset = tags_p[0].offset - nitems_read(1);
        if(offset < nin_p){
          //DEBUG<<"<IC Core DEBUG>Found ring tag part<1>:"<<tags_p[0].key<<" ,value="<<tags_p[0].value<<" ,at:"<<offset+d_phase_size<<std::endl;
          nin_p = offset;
          reset_p = true;
        }
      }
      reset_ready = (reset_s && reset_p) || (d_in_mem_size==d_mem_cap && d_phase_size==d_mem_cap);
      get_tags_in_range(hdr_tag,1,nitems_read(1),nitems_read(1)+nin_p);
      tagObject tmp_obj;
      while(!hdr_tag.empty()){
        int offset = hdr_tag[0].offset - nitems_read(1);
        if(!pmt::eqv(d_ring_tag,hdr_tag[0].key)){
          // exclude ring tag
          if(tmp_obj.index() == (offset+d_phase_size) ){
            // same object
            tmp_obj.add_msg(hdr_tag[0].key,hdr_tag[0].value);
          }else{
            if(!tmp_obj.empty()){
              DEBUG<<"<IC Core>New tag object added---"<<tmp_obj<<std::endl;
              d_in_tlist.push_back(tmp_obj);
              tmp_obj.reset();
            }else{
              tmp_obj.init_dict();
              tmp_obj.add_msg(hdr_tag[0].key,hdr_tag[0].value);
              tmp_obj.set_idx(offset+d_phase_size);
            }
          }
        }
        hdr_tag.erase(hdr_tag.begin());
      }
      if(!tmp_obj.empty()){
        DEBUG<<"<IC Core>One tag object added---"<<tmp_obj<<std::endl;
        d_in_tlist.push_back(tmp_obj);
      }
      memcpy(d_in_mem+d_in_mem_size,in,sizeof(gr_complex)*nin_s);
      memcpy(d_phase_mem+d_phase_size,phase_in,sizeof(float)*nin_p);
      d_in_mem_size += nin_s;
      d_phase_size  += nin_p;
      if(reset_ready){
        // end of a possible ic opportunities
        if(d_in_mem_size == d_mem_cap){
          DEBUG<<"<IC Core>signal buffer full..."<<std::endl;
        }else{
          DEBUG <<"<IC Core>Reset signal are all ready, clear queue... sig_size:"
          <<d_in_mem_size<<" ,phase_size:"<<d_phase_size<<std::endl;
          nin_s++;
          nin_p++;
        }
        // handling output
        d_in_tlist.clear();
        d_in_mem_idx =0;
        d_in_mem_size=0;
        d_phase_idx =0;
        d_phase_size=0;
      }
      consume(0,nin_s);
      consume(1,nin_p);
      consume(2,nin_p);
      return nout;
    }

  } /* namespace lsa */
} /* namespace gr */

