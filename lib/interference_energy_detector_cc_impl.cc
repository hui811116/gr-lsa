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
#include "interference_energy_detector_cc_impl.h"
#include <volk/volk.h>
#include <pmt/pmt.h>
#include <numeric>

namespace gr {
  namespace lsa {


    const pmt::pmt_t d_voe_begin_tag = pmt::intern("voe_begin");
    const pmt::pmt_t d_voe_end_tag = pmt::intern("voe_end");

    interference_energy_detector_cc::sptr
    interference_energy_detector_cc::make(
      float ed_threshold,
      float voe_threshold,
      size_t blocklength,
      int minlen,
      bool debug)
    {
      return gnuradio::get_initial_sptr
        (new interference_energy_detector_cc_impl(
          ed_threshold,
          voe_threshold,
          blocklength,
          minlen,
          debug));
    }

    /*
     * The private constructor
     */
    interference_energy_detector_cc_impl::interference_energy_detector_cc_impl(
      float ed_threshold,
      float voe_threshold,
      size_t blocklength,
      int minlen,
      bool debug)
      : gr::block("interference_energy_detector_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make3(1, 3, sizeof(gr_complex), sizeof(float),sizeof(float))),
      d_src_id(pmt::intern(alias())),
      d_ed_tagname(pmt::intern("ed_tag")),
      d_voe_tagname(pmt::intern("voe_tag"))
    {
      set_blocklength(blocklength);
      set_ed_threshold(ed_threshold);
      set_voe_threshold(voe_threshold);
      set_min_length(minlen);
      d_voe_cnt =0;
      d_debug = debug;

      const size_t max_size = 32*1024;
      d_energy_reg = (float*) volk_malloc( sizeof(float)*max_size, volk_get_alignment());

      set_tag_propagation_policy(TPP_DONT);
      
      d_state_ed = false;
      d_state_voe = false;
      v_stddev = (float*) volk_malloc(sizeof(float),volk_get_alignment());
      v_mean = (float*) volk_malloc(sizeof(float),volk_get_alignment());
    }

    /*
     * Our virtual destructor.
     */
    interference_energy_detector_cc_impl::~interference_energy_detector_cc_impl()
    {
      volk_free(d_energy_reg);
      volk_free(v_stddev);
      volk_free(v_mean);
    }

    void
    interference_energy_detector_cc_impl::set_ed_threshold(float threshold_db)
    {
      d_ed_thres = pow(10.0,threshold_db/10.0f);
    }

    float
    interference_energy_detector_cc_impl::ed_threshold() const
    {
      return 10.0f*log10(d_ed_thres);
    }

    void
    interference_energy_detector_cc_impl::set_voe_threshold(float threshold_db)
    {
      d_voe_thres = pow(10.0,threshold_db/10.0f);
    }

    float
    interference_energy_detector_cc_impl::voe_threshold() const
    {
      return 10.0f*log10(d_voe_thres);
    }

    void
    interference_energy_detector_cc_impl::set_blocklength(size_t blocklength)
    {
      if(blocklength<0){
        throw std::invalid_argument("Block length cannot be negative");
      }
      d_blocklength = blocklength;
      set_history(d_blocklength);
    }

    size_t
    interference_energy_detector_cc_impl::blocklength() const
    {
      return d_blocklength;
    }

    void
    interference_energy_detector_cc_impl::set_min_length(int minlen)
    {
      if(minlen<0){
        throw std::invalid_argument("minimum length cannot be negative");
      }
      d_minlen = minlen;
    }
    
    int
    interference_energy_detector_cc_impl::min_length() const
    {
      return d_minlen;
    }

    void
    interference_energy_detector_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      for(int i=0;i<ninput_items_required.size();++i)
        ninput_items_required[0] = noutput_items + history();
    }

    int
    interference_energy_detector_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
      bool have_ed = (output_items.size()>=3);
      float ed_test;
      float var =0;
      float* voe_out, *ed_out;
      int nin = std::min(ninput_items[0]-d_blocklength+1,noutput_items);
      const uint64_t nread = nitems_read(0);
      memcpy(out, in, sizeof(gr_complex) * nin);
      volk_32fc_magnitude_squared_32f(d_energy_reg, in, nin+d_blocklength);
      if(have_ed){
        ed_out = (float*)output_items[1];
        voe_out   = (float*)output_items[2];
      }
      for(int i=0;i<nin;++i){
        volk_32f_stddev_and_mean_32f_x2(v_stddev, v_mean, d_energy_reg+i,d_blocklength);
        var = pow(*(v_stddev),2);
        ed_test = std::accumulate(d_energy_reg+i,d_energy_reg+i+d_blocklength-1,0.0)/(float)d_blocklength;
        if(have_ed){
          voe_out[i] = var;
          ed_out[i] = ed_test;
        }
        if( ed_test >= d_ed_thres){
          if(!d_state_ed){
            add_item_tag(0,nitems_written(0)+i,d_ed_tagname,pmt::PMT_T);
            add_item_tag(0,nitems_written(0)+i,pmt::intern("ed_begin"),pmt::PMT_T);
            if(have_ed){
              add_item_tag(1,nitems_written(1)+i,pmt::intern("ed_begin"),pmt::PMT_T);
              add_item_tag(2,nitems_written(2)+i,pmt::intern("ed_begin"),pmt::PMT_T);
            }
            d_state_ed = true;
          }  
        }
        else{
          if(d_state_ed){
            add_item_tag(0,nitems_written(0)+i,d_ed_tagname,pmt::PMT_F);
            d_state_ed = false;
          }
        }
        if(var > d_voe_thres){
          d_voe_cnt++;
          if(d_voe_cnt>=d_minlen && !d_state_voe){
            add_item_tag(0,nitems_written(0)+i,d_voe_begin_tag,pmt::PMT_T);
            if(have_ed){
              add_item_tag(1,nitems_written(1)+i,d_voe_begin_tag,pmt::PMT_T);
              add_item_tag(2,nitems_written(2)+i,d_voe_begin_tag,pmt::PMT_T);
            }
            d_voe_cnt=0;
            d_state_voe = true;
          }
        }
        else{
          if(d_state_voe){
            d_voe_cnt++;
            if(d_voe_cnt >=d_minlen){
              add_item_tag(0,nitems_written(0)+i,d_voe_end_tag,pmt::PMT_T);
              if(have_ed){
              add_item_tag(1,nitems_written(1)+i,d_voe_end_tag,pmt::PMT_T);
              add_item_tag(2,nitems_written(2)+i,d_voe_end_tag,pmt::PMT_T);
            }
              d_state_voe = false;
              d_voe_cnt =0;
            }
          }
          else{
            d_voe_cnt =0;
          }
        }
        /*
        if( var >= d_voe_thres){
          if(!d_state_voe){
            add_item_tag(0,nitems_written(0)+i,d_voe_tagname,pmt::PMT_T);
            if(have_ed){
              add_item_tag(1,nitems_written(1)+i,d_voe_tagname,pmt::PMT_T);
              add_item_tag(2,nitems_written(2)+i,d_voe_tagname,pmt::PMT_T);
            }
            d_state_voe =true;
          }
        }
        else{
          if(d_state_voe){
            add_item_tag(0,nitems_written(0)+i,d_voe_tagname,pmt::PMT_F);
            if(have_ed){
              add_item_tag(1,nitems_written(1)+i,d_voe_tagname,pmt::PMT_F);
              add_item_tag(2,nitems_written(2)+i,d_voe_tagname,pmt::PMT_F);
            }
            d_state_voe = false;
          }
        }*/
      }

      // each input stream.
      consume_each (nin);
      
      // Tell runtime system how many output items we produced.
      return nin;
    }

  } /* namespace lsa */
} /* namespace gr */

