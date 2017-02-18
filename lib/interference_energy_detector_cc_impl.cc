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

namespace gr {
  namespace lsa {

    enum detMode{
      BLOCK,
      SLIDE
    };

    interference_energy_detector_cc::sptr
    interference_energy_detector_cc::make(
      const std::string& ed_tagname,
      const std::string& voe_tagname,
      float ed_threshold,
      float voe_threshold,
      size_t blocklength,
      int mode,
      bool debug)
    {
      return gnuradio::get_initial_sptr
        (new interference_energy_detector_cc_impl(
          ed_tagname,
          voe_tagname,
          ed_threshold,
          voe_threshold,
          blocklength,
          mode,
          debug));
    }

    /*
     * The private constructor
     */
    interference_energy_detector_cc_impl::interference_energy_detector_cc_impl(
      const std::string& ed_tagname,
      const std::string& voe_tagname,
      float ed_threshold,
      float voe_threshold,
      size_t blocklength,
      int mode,
      bool debug)
      : gr::block("interference_energy_detector_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make2(1, 2, sizeof(gr_complex), sizeof(float))),
      d_src_id(pmt::intern(alias())),
      d_ed_tagname(pmt::string_to_symbol(ed_tagname)),
      d_voe_tagname(pmt::string_to_symbol(voe_tagname))
    {
      if(blocklength==0){
        std::invalid_argument("Invalid argument: blocklength must be greater than 0");
        return;
      }
      set_blocklength(blocklength);
      set_ed_threshold(ed_threshold);
      set_voe_threshold(voe_threshold);
      d_debug = debug;

      const size_t max_size = 32*1024;
      d_energy_reg = (float*) volk_malloc( sizeof(float)*max_size, volk_get_alignment());

      set_tag_propagation_policy(TPP_DONT);
      set_output_multiple(d_blocklength);

      d_debug_port = pmt::mp("debug");
      message_port_register_out(d_debug_port);
      d_mode = mode;
      switch(d_mode){
        case BLOCK:
        break;
        case SLIDE:
          set_history(d_blocklength);
        break;
        default:
        break;
      }
      d_state = false;
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
      d_ed_thres_db = threshold_db;
    }

    float
    interference_energy_detector_cc_impl::ed_threshold() const
    {
      return d_ed_thres_db;
    }

    void
    interference_energy_detector_cc_impl::set_voe_threshold(float threshold_db)
    {
      d_voe_thres_db = threshold_db;
    }

    float
    interference_energy_detector_cc_impl::voe_threshold() const
    {
      return d_voe_thres_db;
    }

    void
    interference_energy_detector_cc_impl::set_blocklength(size_t blocklength)
    {
      d_blocklength = blocklength;
    }

    size_t
    interference_energy_detector_cc_impl::blocklength() const
    {
      return d_blocklength;
    }

    void
    interference_energy_detector_cc_impl::print(float var_db,float ed_db)
    {
      std::cout << "<Interference detected>";
      std::cout << " variance: "<<var_db<<" dB,";
      std::cout << " energy: " <<ed_db<<" dB,";
      std::cout << " threshold (VoE, Ed): (" <<d_voe_thres_db<<","<<d_ed_thres_db<<")";
      std::cout << std::endl;
    }

    void
    interference_energy_detector_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      switch(d_mode)
      {
        case BLOCK:
          ninput_items_required[0] = d_blocklength * (noutput_items/d_blocklength);
        break;
        case SLIDE:
          ninput_items_required[0] = noutput_items + history();
        break;
        default:
          std::runtime_error("Entering wrong mode");
        break;
      }
      
      if(d_debug){
        std::stringstream ss;
        ss<<"<forecast>"<<"ninput_items_required[0]="<<ninput_items_required[0];
        GR_LOG_DEBUG(d_logger, ss.str());
      }
    }

    int
    interference_energy_detector_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
      float* ed_val=NULL;
      if(output_items.size()==2){
        ed_val = (float*) output_items[1];
      }
      // Do <+signal processing+>
      int iter=0;
      bool tmp_state =false;
      //cal energy val
      
      float var =0;
      switch(d_mode)
      {
        case BLOCK:
        noutput_items = ((noutput_items < ninput_items[0]) ? noutput_items:ninput_items[0]);
        iter = noutput_items /d_blocklength;
        noutput_items = iter*d_blocklength;
          volk_32fc_magnitude_squared_32f(d_energy_reg, in, noutput_items);
        if(ed_val!=NULL){
          for(int i=0;i<noutput_items;++i){
            ed_val[i] = 10.0 * log10(d_energy_reg[i]);
          }
        }
        for(int i=0; i < iter ; ++i){
          //cal mean and std of energy
          volk_32f_stddev_and_mean_32f_x2(v_stddev, v_mean, d_energy_reg + i*d_blocklength, d_blocklength);
          var = pow(*(v_stddev),2) ; // variance
        
          add_item_tag(0,nitems_written(0)+i*d_blocklength,pmt::intern("ed_val"),pmt::from_float(10.0f*log10(*v_mean)),d_src_id);
          add_item_tag(0,nitems_written(0)+i*d_blocklength,d_ed_tagname,pmt::from_bool(10.0*log10(*v_mean) >= d_ed_thres_db),d_src_id);

          add_item_tag(0,nitems_written(0)+i*d_blocklength,pmt::intern("intf_val"),pmt::from_float(10.0f*log10(var)),d_src_id);
          add_item_tag(0,nitems_written(0)+i*d_blocklength,d_voe_tagname,pmt::from_bool(10.0*log10(var) >= d_voe_thres_db),d_src_id); 
         }
        break;
        case SLIDE:
          noutput_items = ((noutput_items + d_blocklength)<= ninput_items[0] ? noutput_items : ninput_items[0]-d_blocklength);
          iter = (noutput_items<0)? 0 : noutput_items;
          volk_32fc_magnitude_squared_32f(d_energy_reg, in, iter+d_blocklength);
          for(int i=0;i<iter;++i){
            volk_32f_stddev_and_mean_32f_x2(v_stddev, v_mean, d_energy_reg+i,d_blocklength);
            var = pow(*(v_stddev),2);
            tmp_state = (10.0*log10(*v_mean)>= d_ed_thres_db) && (10.0*log10(var) >= d_voe_thres_db);
            if(tmp_state != d_state){
              add_item_tag(0,nitems_written(0)+i,pmt::intern("ed_val"),pmt::from_float(10.0f*log10(*v_mean)),d_src_id);
              add_item_tag(0,nitems_written(0)+i,d_ed_tagname,pmt::from_bool(10.0*log10(*v_mean) >= d_ed_thres_db),d_src_id);

              add_item_tag(0,nitems_written(0)+i,pmt::intern("intf_val"),pmt::from_float(10.0f*log10(var)),d_src_id);
              add_item_tag(0,nitems_written(0)+i,d_voe_tagname,pmt::from_bool(10.0*log10(var) >= d_voe_thres_db),d_src_id);     
              d_state = tmp_state;
            }
            if(ed_val!=NULL){
              ed_val[i] = 10.0 * log10(*v_mean);
            }
          }
        break;
        default:
          std::runtime_error("Entering wrong state");
        break;
      }
      
      
      
      // Tell runtime system how many input items we consumed on
      memcpy(out, in, sizeof(gr_complex) * noutput_items);
      // each input stream.
      consume_each (noutput_items);
      if(d_debug){
        std::stringstream ss;
        ss << "consume/output samples-->"<<noutput_items;
        GR_LOG_DEBUG(d_logger, ss.str());
      }
      
      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace lsa */
} /* namespace gr */

