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
#include "prou_ring_queue_cc_impl.h"

namespace gr {
  namespace lsa {

#define DEBUG d_debug && std::cout

    enum RINGSTATE{
      SEARCH,
      RING,
      COPY
    };

    static const int VOELEN = 128;
    static const int MAXLEN = (127+6)*8*8/2*4;
    static const int COPYMAX= MAXLEN *1024;

    prou_ring_queue_cc::sptr
    prou_ring_queue_cc::make(float thres, bool debug)
    {
      return gnuradio::get_initial_sptr
        (new prou_ring_queue_cc_impl(thres,debug));
    }

    /*
     * The private constructor
     */
    prou_ring_queue_cc_impl::prou_ring_queue_cc_impl(float thres, bool debug)
      : gr::block("prou_ring_queue_cc",
              gr::io_signature::make2(2, 2, sizeof(gr_complex),sizeof(float)),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
              d_mem_cap(MAXLEN*8)
    {
      d_ring_mem = new gr_complex[d_mem_cap];
      enter_search();
      set_voe_threshold(thres);
      d_debug = debug;
      d_debug_cnt =0;
    }

    /*
     * Our virtual destructor.
     */
    prou_ring_queue_cc_impl::~prou_ring_queue_cc_impl()
    {
      delete [] d_ring_mem;
    }

    void
    prou_ring_queue_cc_impl::enter_search()
    {
      d_state = SEARCH;
      d_ring_idx = 0;
      d_ring_cnt = 0;
      d_voe_cnt = 0;
    }
    void
    prou_ring_queue_cc_impl::enter_ring()
    {
      d_state = RING;
      d_ring_cnt =0;
    }
    void
    prou_ring_queue_cc_impl::enter_copy()
    {
      d_state = COPY;
      d_cpy_cnt = 0;
      d_voe_cnt = 0;
    }

    void
    prou_ring_queue_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      int item_reqd=0;
      switch(d_state){
        case SEARCH:
          item_reqd = noutput_items;
        break;
        case RING:
          item_reqd = 0;
        break;
        case COPY:
          item_reqd= noutput_items;
        break;
        default:
          throw std::runtime_error("Worng state");
        break;
      }
      for(int i=0;i<ninput_items_required.size();++i){
        ninput_items_required[i] = noutput_items;
      }
    }

    int
    prou_ring_queue_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      const float * voe_in = (const float*) input_items[1];
      gr_complex *out = (gr_complex *) output_items[0];
      int nin = std::min(ninput_items[0],ninput_items[1]);
      int count = 0;
      int nout = 0;
      const uint64_t nread = nitems_read(0);
      const uint64_t nwrite= nitems_written(0);

      switch(d_state){
        case SEARCH:
          while(count<nin){
            if(voe_in[count++]>d_voe_thres){
              d_voe_cnt++;
              if(d_voe_cnt >= VOELEN){
                DEBUG<<"<RING queue>VoE greater than minmum length threshold! change state"<<std::endl;
                //if(d_debug){
                  add_item_tag(0,nwrite,pmt::intern("VoE_detected"),pmt::from_long(d_debug_cnt++));
                //}
                enter_ring();
                break;
              }
            }else{
              d_voe_cnt =0;
            }
          }
          for(int i=0;i<count;i++){
            d_ring_mem[d_ring_idx++] = in[i];
            d_ring_idx %= d_mem_cap;
          }
          consume_each(count);
          return 0;
        break;
        case RING:
          while(nout<noutput_items && d_ring_cnt<d_mem_cap)
          {
            int eq_idx = (d_ring_idx+(d_ring_cnt++) )%d_mem_cap;
            out[nout++] = d_ring_mem[eq_idx];
          }
          if(d_ring_cnt==d_mem_cap){
            DEBUG<<"<Ring Queue>Ring memory released, change state to copy..."<<std::endl;
            enter_copy();
          }
          consume_each(0);
          return nout;
        break;
        case COPY:
          while(nout<noutput_items && count<nin && d_cpy_cnt<COPYMAX){
            if(voe_in[count]>d_voe_thres){
              d_voe_cnt++;
              if(d_voe_cnt>=VOELEN && d_cpy_cnt>=MAXLEN){
                d_voe_cnt = 0;
                d_cpy_cnt =0;
              }
            }else{
              d_voe_cnt = 0;
            }
            out[nout++] = in[count++];
            d_cpy_cnt++;
          }
          if(d_cpy_cnt == COPYMAX){
            DEBUG<<"<Ring Queue>Copy complete, resume to search state"<<std::endl;
            enter_search();
          }
          consume_each(count);
          return nout;
        break;
        default:
          throw std::runtime_error("Wrong state");
        break;
      }
    }// end general 

  } /* namespace lsa */
} /* namespace gr */

