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

namespace gr {
  namespace lsa {

    enum COARSESTATE{
      SEARCH,
      COPY
    };

    static const int AUTOLEN = 32;
    static const int MAXLEN = 127*32;
    static const int MINGAP = 128;

    coarse_sync_cc::sptr
    coarse_sync_cc::make(float threshold, int sps)
    {
      return gnuradio::get_initial_sptr
        (new coarse_sync_cc_impl(threshold, sps));
    }

    /*
     * The private constructor
     */
    coarse_sync_cc_impl::coarse_sync_cc_impl(float threshold, int sps)
      : gr::block("coarse_sync_cc",
              gr::io_signature::make3(3, 3, sizeof(gr_complex),sizeof(gr_complex),sizeof(float)),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
              d_valid_len(AUTOLEN*sps),
              d_mingap(MINGAP*sps),
              d_maxlen(MAXLEN*sps),
              d_cfo_key(pmt::string_to_symbol("cfo_est")),
              d_edend_tagname(pmt::intern("ed_end"))
    {
      d_state = SEARCH;
      if(threshold > 1 || threshold<0){
        throw std::invalid_argument("Threshold should between 0 and 1");
      }
      d_threshold = threshold;
      if(sps<0){
        throw std::invalid_argument("Sps cannot be negative");
      }
      d_sps = sps;
      set_tag_propagation_policy(TPP_DONT);
    }

    /*
     * Our virtual destructor.
     */
    coarse_sync_cc_impl::~coarse_sync_cc_impl()
    {
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

      int nin = std::min(ninput_items[0],std::min(ninput_items[1],ninput_items[2]));
      int count =0;
      int nout= 0;

      std::vector<tag_t> tags, ed_tags;
      const uint64_t nread = nitems_read(0);
      const uint64_t nwrite = nitems_written(0);
      get_tags_in_range(tags,0,nread,nread+nin);
      get_tags_in_range(ed_tags,0,nread,nread+nin,d_edend_tagname);


      switch(d_state){
        case SEARCH:
          for(count;count<nin;++count){
            if(in_mag_norm[count]>d_threshold){
              d_auto_cnt++;
              if(d_auto_cnt >=d_valid_len){
                d_state = COPY;
                d_auto_cnt =0;
                d_coarse_cfo = arg(in_corr[count])/(float)d_valid_len;
                d_copy_cnt =0;
                add_item_tag(0,nwrite,d_cfo_key,pmt::from_float(d_coarse_cfo));
                while(!ed_tags.empty()){
                  if(ed_tags[0].offset-nread >= count){
                    break;
                  }
                  ed_tags.erase(ed_tags.begin());
                }
                break;
              }
            }
            else{
              d_auto_cnt =0;
            } 
          }
          consume_each(count);
          return 0;
        break;
        case COPY:
          while(nout<noutput_items&& nout<nin && d_copy_cnt<d_maxlen){
            if(in_mag_norm[nout]>d_threshold){
              if(d_auto_cnt < d_valid_len){
                d_auto_cnt++;
              }
              else if(d_copy_cnt>=d_mingap){
                d_coarse_cfo = arg(in_corr[nout])/(float)d_valid_len;
                d_auto_cnt = 0;
                add_item_tag(0,nwrite+nout,d_cfo_key,pmt::from_float(d_coarse_cfo));
                while(!ed_tags.empty()){
                  if(ed_tags[0].offset-nread >= nout){
                    break;
                  }
                  ed_tags.erase(ed_tags.begin());
                }
              }
            }
            else{
              d_auto_cnt = 0;
            }
            
            out[nout] = in_samp[nout] * gr_expj(-d_coarse_cfo * (d_copy_cnt++));
            if(d_copy_cnt >= d_maxlen){
              d_state = SEARCH;
              d_copy_cnt =0;
              d_auto_cnt =0;
            }
            if(!ed_tags.empty()){
              int offset = ed_tags[0].offset - nread;
              if(offset == nout){
                d_state = SEARCH;
                d_copy_cnt =0;
                d_auto_cnt =0;
                ed_tags.erase(ed_tags.begin());
              }
            }
            nout++;
          }
          consume_each(nout);
          return nout;
        break;
        default:
          throw std::runtime_error("Undefined state");
        break;
      }

/*
      for(count=0;count<nin;++count){
        switch(d_state){
          case SEARCH:
            if(in_mag_norm[count]>d_threshold){
              d_auto_cnt++;
            }
            else{
              d_auto_cnt=0;
            }
            if(d_auto_cnt >= d_valid_len){
              d_state = COPY;
              d_coarse_cfo = arg(in_corr[count])/AUTOLEN;
              d_copy_cnt = 0;
              add_item_tag(0,nwrite,d_cfo_key,pmt::from_float(d_coarse_cfo));
              while(!ed_tags.empty()){
                if(ed_tags[0].offset-nread >= count){
                  break;
                }
                ed_tags.erase(ed_tags.begin());
              }
            }
          break;
          case COPY:
            if(!ed_tags.empty()){
              int offset = ed_tags[0].offset - nread;
              if(count == offset){
                d_state = SEARCH;
                d_auto_cnt = 0;
              }
            }
            out[nout++] = in_samp[count] * gr_expj( -d_coarse_cfo*(d_copy_cnt++) );
            if(d_copy_cnt>=MAXLEN){
              d_state = SEARCH;
              d_auto_cnt =0;
            }
          break;
          default:
            throw std::runtime_error("Undefined state");
          break;
        }
      }

      consume_each (count);
      // Tell runtime system how many output items we produced.
      return nout;
      */
    }

  } /* namespace lsa */
} /* namespace gr */

