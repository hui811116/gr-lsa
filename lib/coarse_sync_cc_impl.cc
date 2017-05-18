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
      SEARCH_ED,
      SEARCH_AUTO,
      COPY
    };

    static const int AUTOLEN = 32;
    static const int MAXLEN = 127*32;
    static const int MINGAP = 128;
    static const int ED_LENGTH= 32;

    coarse_sync_cc::sptr
    coarse_sync_cc::make(float threshold, int sps, float ed_thres)
    {
      return gnuradio::get_initial_sptr
        (new coarse_sync_cc_impl(threshold, sps, ed_thres));
    }

    /*
     * The private constructor
     */
     static int ios[] = {sizeof(gr_complex),sizeof(gr_complex),sizeof(float),sizeof(float)};
     static std::vector<int> iosig(ios,ios+sizeof(ios)/sizeof(int));

    coarse_sync_cc_impl::coarse_sync_cc_impl(float threshold, int sps, float ed_thres)
      : gr::block("coarse_sync_cc",
              gr::io_signature::makev(4, 4, iosig),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
              d_valid_len(AUTOLEN*sps),
              d_mingap(MINGAP*sps),
              d_maxlen(MAXLEN*sps),
              d_cfo_key(pmt::string_to_symbol("cfo_est")),
              d_ed_tagname(pmt::intern("ed_tag")),
              d_out_port(pmt::mp("msg_out"))
    {
      d_state = SEARCH_ED;
      if(threshold > 1 || threshold<0){
        throw std::invalid_argument("Threshold should between 0 and 1");
      }
      d_threshold = threshold;
      if(sps<0){
        throw std::invalid_argument("Sps cannot be negative");
      }
      d_sps = sps;

      d_ed_cnt = 0;
      d_auto_cnt =0;
      d_copy_cnt = 0;

      set_ed_threshold(ed_thres);
      set_tag_propagation_policy(TPP_DONT);
      message_port_register_out(d_out_port);
    }

    /*
     * Our virtual destructor.
     */
    coarse_sync_cc_impl::~coarse_sync_cc_impl()
    {
    }

    void
    coarse_sync_cc_impl::set_ed_threshold(const float& ed_thres)
    {
      d_ed_threshold = ed_thres;
    }
    float
    coarse_sync_cc_impl::ed_threshold() const
    {
      return d_ed_threshold;
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
      const float * in_eng      = (const float *) input_items[3];
      gr_complex *out = (gr_complex *) output_items[0];

      int nin = std::min(std::min(ninput_items[0],std::min(ninput_items[1],ninput_items[2])),ninput_items[3]) ;
      //nin = std::min(noutput_items,nin);
      int count =0;
      int nout= 0;

      const uint64_t nread = nitems_read(0);
      const uint64_t nwrite= nitems_written(0);

      switch(d_state){
        case SEARCH_ED:
          for(count;count<nin;++count){
            if(in_eng[count]>d_ed_threshold){
              d_ed_cnt++;
              if(d_ed_cnt >= ED_LENGTH){
                add_item_tag(0,nwrite,d_ed_tagname,pmt::PMT_T);
                d_state = SEARCH_AUTO;
                d_ed_cnt =0;
                d_free_cnt =0;
                break;
              }
            }
            else{
              d_ed_cnt =0;
            }
          }
          consume_each(count);
          return 0;
        break;
        case SEARCH_AUTO:
          while(nout<noutput_items && count<nin){
            if(in_mag_norm[count]>d_threshold){
              d_auto_cnt++;
              d_ed_cnt =0;
              if(d_auto_cnt >=d_valid_len){
                d_state = COPY;
                d_auto_cnt =0;
                // divide by delay
                d_coarse_cfo = arg(in_corr[count])/(float)d_valid_len;
                d_copy_cnt =0;
                add_item_tag(0,nwrite+count,d_cfo_key,pmt::from_float(d_coarse_cfo));
                out[nout++] = in_samp[count++] * gr_expj(-d_coarse_cfo * (d_copy_cnt++));
                break;
              }
            }
            else{
              d_auto_cnt =0;
              if(in_eng[count]<d_ed_threshold){
                d_ed_cnt++;
                if(d_ed_cnt>=ED_LENGTH){
                  d_state = SEARCH_ED;
                  d_ed_cnt =0;
                  count++;
                  break;
                }
              }
              else{
                d_ed_cnt =0;
              }
            }
            out[nout++] = in_samp[count++] * gr_expj(-d_coarse_cfo * (d_free_cnt++));
          }
          consume_each(count);
          return nout;
        break;
        case COPY:
          while(count<nin && nout<noutput_items && d_copy_cnt < d_maxlen){
            if(in_mag_norm[count]>d_threshold){
              d_auto_cnt++;
              if(d_auto_cnt < d_valid_len){
                d_auto_cnt++;
              }
              else if(d_copy_cnt>=d_mingap){
                d_coarse_cfo = arg(in_corr[count])/(float)d_valid_len;
                d_auto_cnt =0;
                d_copy_cnt =0;
                add_item_tag(0,nwrite+nout,d_cfo_key,pmt::from_float(d_coarse_cfo));
              }
            }
            else{
              d_auto_cnt= 0;
            }
            if(in_eng[count]<d_ed_threshold){
              d_ed_cnt++;
              if(d_ed_cnt>=ED_LENGTH){
                d_state = SEARCH_ED;
                d_copy_cnt =0;
                d_auto_cnt =0;
                d_ed_cnt =0;
                count++;
                break;
              }
            }
            out[nout++] = in_samp[count++] * gr_expj(d_coarse_cfo * (d_copy_cnt++)); 
            if(d_copy_cnt>=d_maxlen){
              d_state = (d_ed_cnt==0)? SEARCH_AUTO : SEARCH_ED;
              d_copy_cnt =0;
              d_auto_cnt =0;
              d_ed_cnt =0;
              break;
            }
          }
          consume_each(count);
          return nout;
        break;
        default:
          throw std::runtime_error("Entering undefined state");
        break;
      }
/*
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
                message_port_pub(d_out_port,pmt::intern("coarse"));
                break;
              }
            }
            else{
              d_auto_cnt =0;
            }
          }
          consume_each(count);
          return count;
        break;
        case COPY:
          while( (nout<noutput_items)&& (nout<nin) && (d_copy_cnt<d_maxlen) ){
            if(in_mag_norm[nout]>d_threshold){
              if(d_auto_cnt < d_valid_len){
                d_auto_cnt++;
              }
              else if(d_copy_cnt>=d_mingap){
                d_coarse_cfo = arg(in_corr[nout])/(float)d_valid_len;
                d_auto_cnt = 0;
                d_copy_cnt = 0;
                add_item_tag(0,nwrite+nout,d_cfo_key,pmt::from_float(d_coarse_cfo));
                message_port_pub(d_out_port,pmt::intern("coarse"));
              }
            }
            else{
              d_auto_cnt = 0;
            }    
            out[nout] = in_samp[nout] * gr_expj(-d_coarse_cfo * (d_copy_cnt++));
            // end condition 1
            if(in_eng[nout]<d_ed_threshold){
              d_ed_cnt++;
              if(d_ed_cnt>= ED_LENGTH){
                d_ed_cnt = 0;
                d_copy_cnt = 0;
                d_auto_cnt =0;
                d_state = SEARCH;
                nout++;
                break;
              }
            }
            else{
              d_ed_cnt = 0;
            }
            // end condition 2
            if(d_copy_cnt >= d_maxlen){
              d_state = SEARCH;
              d_copy_cnt =0;
              d_auto_cnt =0;
              nout++;
              break;
            }
            nout++;
          }
          consume_each(nout);
          return nout;
        break;
        default:
          throw std::runtime_error("Undefined state");
        break;
      }*/

    }

  } /* namespace lsa */
} /* namespace gr */

