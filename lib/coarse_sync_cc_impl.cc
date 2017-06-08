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
#include "coarse_sync_cc_impl.h"
#include <algorithm>
#include <gnuradio/expj.h>
#include <gnuradio/math.h>

namespace gr {
  namespace lsa {

#define TWO_PI M_PI*2.0F

    enum COARSESTATE{
      SEARCH_AUTO,
      COPY
    };
    static const int AUTOLEN = 32;
    static const int MAXLEN = (127+6)*8*8/2*4;
    static const int MINGAP = (8+12*8)*8/2*4;
    static const pmt::pmt_t d_voe_tag = pmt::intern("voe_tag");

    inline void phase_wrap(float& phase){
      while(phase>=TWO_PI)
        phase-=TWO_PI;
      while(phase<=-TWO_PI)
        phase+=TWO_PI;
    }

    coarse_sync_cc::sptr
    coarse_sync_cc::make(float threshold, int delay)
    {
      return gnuradio::get_initial_sptr
        (new coarse_sync_cc_impl(threshold, delay));
    }

    /*
     * The private constructor
     */
     static int ios[] = {sizeof(gr_complex),sizeof(gr_complex),sizeof(float)};
     static std::vector<int> iosig(ios,ios+sizeof(ios)/sizeof(int));

    coarse_sync_cc_impl::coarse_sync_cc_impl(float threshold, int delay)
      : gr::block("coarse_sync_cc",
              gr::io_signature::makev(3, 3, iosig),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
              d_valid_len(AUTOLEN),
              d_mingap(MINGAP),
              d_maxlen(MAXLEN),
              d_cfo_key(pmt::string_to_symbol("cfo_est"))
    {
      d_state = SEARCH_AUTO;
      if(threshold > 1 || threshold<0){
        throw std::invalid_argument("Threshold should between 0 and 1");
      }
      d_threshold = threshold;
      if(delay<0){
        throw std::invalid_argument("delay cannot be negative");
      }
      d_delay = delay;
      d_phase = 0;
      d_coarse_cfo=0;
      d_auto_cnt =0;
      d_copy_cnt = 0;
      set_tag_propagation_policy(TPP_DONT);
      d_voe_state = false;
    }

    /*
     * Our virtual destructor.
     */
    coarse_sync_cc_impl::~coarse_sync_cc_impl()
    {
    }

    void
    coarse_sync_cc_impl::update_tag_state(int idx)
    {
      const uint64_t nread = nitems_read(0);
      while(!d_tags.empty()){
        int offset = d_tags[0].offset - nread;
        if(offset == idx){
          // DEBUG
          //bool tmp_state = d_voe_state;
          d_voe_state = pmt::to_bool(d_tags[0].value);
          //if(tmp_state!=d_voe_state){
            //std::cout<<"<Coarse DEBUG>Change voe state: after--"<<d_voe_state<<" --prev--"<<tmp_state<<std::endl;
          //}
          d_tags.erase(d_tags.begin());
          break;
        }else if(idx >offset){
          throw std::runtime_error("WTF, tags may need sorting...");
        }
        else{
          break;
        }
      }
    }

    void
    coarse_sync_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
          for(int i=0;i<ninput_items_required.size();++i){
            ninput_items_required[i] = noutput_items;
          } 
    }

    int
    coarse_sync_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in_samp = (const gr_complex *) input_items[0];
      const gr_complex *in_corr = (const gr_complex *) input_items[1];
      const float * in_mag_norm = (const float *) input_items[2];
      gr_complex *out = (gr_complex *) output_items[0];
      int nin = std::min(ninput_items[0],std::min(ninput_items[1],ninput_items[2])) ;
      int count =0;
      int nout= 0;
      d_tags.clear();
      get_tags_in_window(d_tags,0,0,nin,d_voe_tag);
      std::vector<tag_t> tags = d_tags;
      const uint64_t nwrite= nitems_written(0);
      switch(d_state){
        case SEARCH_AUTO:
        while(nout<noutput_items && count<nin){
          update_tag_state(count);
          if(in_mag_norm[count]>d_threshold){
              d_auto_cnt++;
              if(d_auto_cnt >=d_valid_len){
                if(!d_voe_state){
                  d_state = COPY;
                  d_auto_cnt =0;
                  // divide by delay
                  d_coarse_cfo = arg(in_corr[count])/(float)d_delay;
                  d_copy_cnt =0;
                  add_item_tag(0,nwrite+count,d_cfo_key,pmt::from_float(d_coarse_cfo));
                  //std::cout<<"<Coarse DEBUG>send tag"<<std::endl;
                  break;
                }else{
                  // interfering, do not change cfo estimate...
                  d_auto_cnt=0;
                }
              }
            }
            else{
              d_auto_cnt =0;
            }
            out[nout++] = in_samp[count++] * gr_expj(-d_phase);
            d_phase+=d_coarse_cfo;
            phase_wrap(d_phase);
        }
        for(int i=0;i<tags.size();++i){
          int offset = tags[i].offset - nitems_read(0);
          if(offset <count){
            add_item_tag(0,nwrite+offset,tags[i].key,tags[i].value);
          }
        }
          consume_each(count);
          return nout;
        break;
        case COPY:
          while(count<nin && nout<noutput_items && d_copy_cnt < d_maxlen){
            update_tag_state(count);
            if(in_mag_norm[count]>d_threshold){
              d_auto_cnt++;
              if(d_auto_cnt < d_valid_len){
                d_auto_cnt++;
              }
              else if(d_copy_cnt>=d_mingap){
                if(!d_voe_state){
                  d_coarse_cfo = arg(in_corr[count])/(float)d_delay;
                  d_auto_cnt =0;
                  d_copy_cnt =0;
                  add_item_tag(0,nwrite+nout,d_cfo_key,pmt::from_float(d_coarse_cfo));
                  //DEBUG
                  //std::cout<<"<Coarse DEBUG>send tag"<<std::endl;
                }else{
                  // interfering, do not change state...
                  d_auto_cnt=0;
                }
              }
            }
            else{
              d_auto_cnt= 0;
            }
            out[nout++] = in_samp[count++] * gr_expj(-d_phase); 
            d_phase += d_coarse_cfo;
            phase_wrap(d_phase);
            d_copy_cnt++;
            if(d_copy_cnt>=d_maxlen){
              d_state = SEARCH_AUTO;
              d_copy_cnt =0;
              d_auto_cnt =0;
              break;
            }
          }
          for(int i=0;i<tags.size();++i){
          int offset = tags[i].offset - nitems_read(0);
          if(offset <count){
            add_item_tag(0,nwrite+offset,tags[i].key,tags[i].value);
          }
        }
          consume_each(count);
          return nout;
        break;
        default:
          throw std::runtime_error("Entering undefined state");
        break;
      }
    }

  } /* namespace lsa */
} /* namespace gr */

