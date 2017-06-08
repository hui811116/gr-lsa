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

    static const pmt::pmt_t d_voe_tag = pmt::intern("voe_tag");
    static const pmt::pmt_t d_block_tag=pmt::intern("block_tag");

    modified_costas_loop_cc::sptr
    modified_costas_loop_cc::make(float loop_bw, int order, bool use_snr)
    {
      return gnuradio::get_initial_sptr
        (new modified_costas_loop_cc_impl(loop_bw, order,use_snr));
    }

    /*
     * The private constructor
     */
    static int ios[]  = {sizeof(gr_complex), sizeof(float), sizeof(float)};
    static std::vector<int> iosig(ios, ios + sizeof(ios)/sizeof(int));
    modified_costas_loop_cc_impl::modified_costas_loop_cc_impl(float loop_bw, int order, bool use_snr)
      : gr::block("modified_costas_loop_cc",
              gr::io_signature::make(1,1, sizeof(gr_complex)),
              gr::io_signature::makev(1,3,iosig)),
      blocks::control_loop(loop_bw, 1.0, -1.0),
  d_order(order), d_error(0), d_noise(1.0), d_phase_detector(NULL)
    {
      set_tag_propagation_policy(TPP_DONT);
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
      gr_complex *optr = (gr_complex *) output_items[0];  
      float *poptr = NULL;
      float *foptr = NULL;
      bool write_foptr = output_items.size() >= 3;
      if(write_foptr){
        poptr = (float*) output_items[1];
        foptr = (float*) output_items[2];
      }
      int nin = std::min(ninput_items[0],noutput_items);
      int nout =0 ;
      const uint64_t nread =nitems_read(0);
      const uint64_t nwrite=nitems_written(0);

      gr_complex nco_out;
      std::vector<tag_t> tags,voe_tags, block_tags;
      get_tags_in_range(voe_tags,0,nread,nread+nin,d_voe_tag);
      get_tags_in_range(block_tags,0,nread,nread+nin,d_block_tag);
      get_tags_in_range(tags, 0, nread,
                        nread+nin,
                        pmt::intern("phase_est"));

      for(int i=0;i<voe_tags.size();++i){
        int offset = voe_tags[i].offset - nread;
        add_item_tag(0,nwrite+offset,voe_tags[i].key,voe_tags[i].value);
      }
      for(int i=0;i<block_tags.size();++i){
        int offset = block_tags[i].offset - nread;
        add_item_tag(0,nwrite+offset,block_tags[i].key,block_tags[i].value);
      }

      if(write_foptr) {
        for(int i = 0; i < nin; i++) {
          if(!tags.empty()) {
            if( (tags[0].offset-nread) == i) {
              d_phase = (float)pmt::to_double(tags[0].value);
              tags.erase(tags.begin());
            }
          }
          nco_out = gr_expj(-d_phase);
          optr[nout] = iptr[i] * nco_out;
          d_error = (*this.*d_phase_detector)(optr[i]);
          d_error = gr::branchless_clip(d_error, 1.0);
          advance_loop(d_error);
          phase_wrap();
          frequency_limit();
          poptr[nout] = d_phase;
          foptr[nout] = d_freq;
          nout++;
          // direct copy
        }
      }
      else {
        for(int i = 0; i < nin; i++) {
          if(!tags.empty()) {
            if( (tags[0].offset-nread) == i) {
              d_phase = (float)pmt::to_double(tags[0].value);
              tags.erase(tags.begin());
            }
          }
          nco_out = gr_expj(-d_phase);
          optr[nout] = iptr[i] * nco_out;
          d_error = (*this.*d_phase_detector)(optr[i]);
          d_error = gr::branchless_clip(d_error, 1.0);
          advance_loop(d_error);
          phase_wrap();
          frequency_limit();
          nout++;
        }
      }
      consume_each(nin);
      if(write_foptr){
        produce(0,nout);
        produce(1,nout);
        produce(2,nout);
      }else{
        produce(0,nout);
        produce(1,0);
        produce(2,0);
      }
      return WORK_CALLED_PRODUCE;
    }

  } /* namespace lsa */
} /* namespace gr */

