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
#include "modified_costas_loop_cc_impl.h"
#include <gnuradio/expj.h>
#include <gnuradio/sincos.h>
#include <gnuradio/math.h>

namespace gr {
  namespace lsa {

    modified_costas_loop_cc::sptr
    modified_costas_loop_cc::make(float loop_bw, int order, bool use_snr, 
                                  const std::string& intf_tagname,
                                  bool burst_mode)
    {
      return gnuradio::get_initial_sptr
        (new modified_costas_loop_cc_impl(loop_bw, order,use_snr,
                                          intf_tagname,
                                          burst_mode));
    }

    /*
     * The private constructor
     */
    static int ios[]  = {sizeof(gr_complex), sizeof(float), sizeof(float)};
    static std::vector<int> iosig(ios, ios + sizeof(ios)/sizeof(int));
    modified_costas_loop_cc_impl::modified_costas_loop_cc_impl(float loop_bw, int order, bool use_snr, 
    const std::string& intf_tagname,
    bool burst_mode)
      : gr::block("modified_costas_loop_cc",
              gr::io_signature::make2(1,2, sizeof(gr_complex), sizeof(float)),
              gr::io_signature::makev(1,3,iosig)),
      blocks::control_loop(loop_bw, 1.0, -1.0),
  d_order(order), d_error(0), d_noise(1.0), d_phase_detector(NULL)
    {
      d_burst_mode = burst_mode;
      d_found_burst = false;

      d_intf_tagname = pmt::string_to_symbol(intf_tagname);
      d_prev_freq = 0;
      d_intf_state = false;

      switch(d_order) {
      case 2:
        if(use_snr)
          d_phase_detector = &modified_costas_loop_cc_impl::phase_detector_snr_2;
  else
          d_phase_detector = &modified_costas_loop_cc_impl::phase_detector_2;
  break;

      case 4:
        if(use_snr)
          d_phase_detector = &modified_costas_loop_cc_impl::phase_detector_snr_4;
  else
          d_phase_detector = &modified_costas_loop_cc_impl::phase_detector_4;
  break;

      case 8:
        if(use_snr)
          d_phase_detector = &modified_costas_loop_cc_impl::phase_detector_snr_8;
  else
          d_phase_detector = &modified_costas_loop_cc_impl::phase_detector_8;
  break;

      default:
  throw std::invalid_argument("order must be 2, 4, or 8");
  break;
      }

      message_port_register_in(pmt::mp("noise"));
      set_msg_handler(
        pmt::mp("noise"),
        boost::bind(&modified_costas_loop_cc_impl::handle_set_noise,
                    this, _1));
    }
    float
    modified_costas_loop_cc_impl::phase_detector_8(gr_complex sample) const
    {
      /* This technique splits the 8PSK constellation into 2 squashed
   QPSK constellations, one when I is larger than Q and one
   where Q is larger than I. The error is then calculated
   proportionally to these squashed constellations by the const
   K = sqrt(2)-1.

   The signal magnitude must be > 1 or K will incorrectly bias
   the error value.

   Ref: Z. Huang, Z. Yi, M. Zhang, K. Wang, "8PSK demodulation for
   new generation DVB-S2", IEEE Proc. Int. Conf. Communications,
   Circuits and Systems, Vol. 2, pp. 1447 - 1450, 2004.
      */

      float K = (sqrt(2.0) - 1);
      if(fabsf(sample.real()) >= fabsf(sample.imag())) {
  return ((sample.real()>0 ? 1.0 : -1.0) * sample.imag() -
    (sample.imag()>0 ? 1.0 : -1.0) * sample.real() * K);
      }
      else {
  return ((sample.real()>0 ? 1.0 : -1.0) * sample.imag() * K -
    (sample.imag()>0 ? 1.0 : -1.0) * sample.real());
      }
    }

    float
    modified_costas_loop_cc_impl::phase_detector_4(gr_complex sample) const
    {
      return ((sample.real()>0 ? 1.0 : -1.0) * sample.imag() -
        (sample.imag()>0 ? 1.0 : -1.0) * sample.real());
    }

    float
    modified_costas_loop_cc_impl::phase_detector_2(gr_complex sample) const
    {
      return (sample.real()*sample.imag());
    }

    float
    modified_costas_loop_cc_impl::phase_detector_snr_8(gr_complex sample) const
    {
      float K = (sqrt(2.0) - 1);
      float snr = abs(sample)*abs(sample) / d_noise;
      if(fabsf(sample.real()) >= fabsf(sample.imag())) {
  return ((blocks::tanhf_lut(snr*sample.real()) * sample.imag()) -
          (blocks::tanhf_lut(snr*sample.imag()) * sample.real() * K));
      }
      else {
  return ((blocks::tanhf_lut(snr*sample.real()) * sample.imag() * K) -
          (blocks::tanhf_lut(snr*sample.imag()) * sample.real()));
      }
    }

    float
    modified_costas_loop_cc_impl::phase_detector_snr_4(gr_complex sample) const
    {
      float snr = abs(sample)*abs(sample) / d_noise;
      return ((blocks::tanhf_lut(snr*sample.real()) * sample.imag()) -
              (blocks::tanhf_lut(snr*sample.imag()) * sample.real()));
    }

    float
    modified_costas_loop_cc_impl::phase_detector_snr_2(gr_complex sample) const
    {
      float snr = abs(sample)*abs(sample) / d_noise;
      return blocks::tanhf_lut(snr*sample.real()) * sample.imag();
    }

    float
    modified_costas_loop_cc_impl::error() const
    {
      return d_error;
    }

    void
    modified_costas_loop_cc_impl::handle_set_noise(pmt::pmt_t msg)
    {
      if(pmt::is_real(msg)) {
        d_noise = pmt::to_double(msg);
        d_noise = powf(10.0f, d_noise/10.0f);
      }
    }

    /*
     * Our virtual destructor.
     */
    modified_costas_loop_cc_impl::~modified_costas_loop_cc_impl()
    {
    }

    void
    modified_costas_loop_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      for(int i=0;i<ninput_items_required.size();++i){
        ninput_items_required[i] = noutput_items;
      }
    }

    int
    modified_costas_loop_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *iptr = (gr_complex *) input_items[0];
      const float* plf_phase = NULL;
      gr_complex *optr = (gr_complex *) output_items[0];  
      float *poptr = NULL;
      float *poly_phase = NULL;
      bool write_foptr = output_items.size() >= 3;
      bool have_poly = input_items.size() >=2;
      if(have_poly && write_foptr){
        plf_phase = (const float*)input_items[1];
        poptr = (float*) output_items[1];
        poly_phase = (float*) output_items[2];
      }
      gr_complex nco_out;
      std::vector<tag_t> intf_tags, ed_tags;
      
      get_tags_in_range(intf_tags, 0, nitems_read(0), nitems_read(0)+noutput_items, d_intf_tagname);
      get_tags_in_range(ed_tags, 0, nitems_read(0), nitems_read(0)+noutput_items, pmt::intern("ed_tag"));

      std::vector<tag_t> tags;
      get_tags_in_range(tags, 0, nitems_read(0),
                        nitems_read(0)+noutput_items,
                        pmt::intern("phase_est"));
      if(write_foptr && have_poly) {
        for(int i = 0; i < noutput_items; i++) {
          if(!tags.empty()) {
            if( (tags[0].offset-nitems_read(0)) == i) {
              d_phase = (float)pmt::to_double(tags[0].value);
              tags.erase(tags.begin());
            }
          }
          if(!intf_tags.empty()){
            if( (intf_tags[0].offset-nitems_read(0)) == i) {
            if(!d_intf_state && pmt::to_bool(intf_tags[0].value)){
              d_prev_freq = d_freq;
            }
            else if(d_intf_state && !pmt::to_bool(intf_tags[0].value)){
              d_freq = d_prev_freq;
            }
            intf_tags.erase(intf_tags.begin());
            }
          }
          if(!ed_tags.empty()){
            if( ed_tags[0].offset-nitems_read(0) == i){
              d_found_burst = pmt::to_bool(ed_tags[0].value);
              ed_tags.erase(ed_tags.begin());
            }
          }
          
          nco_out = gr_expj(-d_phase);
          optr[i] = iptr[i] * nco_out;

          d_error = (*this.*d_phase_detector)(optr[i]);
          d_error = gr::branchless_clip(d_error, 1.0);

          if(d_burst_mode && !d_found_burst){
              d_error = 0;
          }

          advance_loop(d_error);
          phase_wrap();
          frequency_limit();
          poptr[i] = d_phase;
          // direct copy
        }
        memcpy(poly_phase, plf_phase, sizeof(float)*noutput_items);
      }
        
      else {
        for(int i = 0; i < noutput_items; i++) {
          if(!tags.empty()) {
            if( (tags[0].offset-nitems_read(0)) == i) {
              d_phase = (float)pmt::to_double(tags[0].value);
              tags.erase(tags.begin());
            }
          }
          if(!intf_tags.empty()){
            if((intf_tags[0].offset-nitems_read(0)) == i) {
            if(!d_intf_state && pmt::to_bool(intf_tags[0].value)){
              d_prev_freq = d_freq;
            }
            else if(d_intf_state && !pmt::to_bool(intf_tags[0].value)){
              d_freq = d_prev_freq;
            }
            intf_tags.erase(intf_tags.begin());
            }
          }
          if(!ed_tags.empty()){
            if( ed_tags[0].offset-nitems_read(0) == i){
              d_found_burst = pmt::to_bool(ed_tags[0].value);
              ed_tags.erase(ed_tags.begin());
            }
          }

          nco_out = gr_expj(-d_phase);
          optr[i] = iptr[i] * nco_out;

          if(d_burst_mode && !d_found_burst){
              d_error = 0;
          }

          d_error = (*this.*d_phase_detector)(optr[i]);
          d_error = gr::branchless_clip(d_error, 1.0);

          advance_loop(d_error);
          phase_wrap();
          frequency_limit();
        }
      }
      consume_each(noutput_items);
      return noutput_items;
    }

  } /* namespace lsa */
} /* namespace gr */

