/* -*- c++ -*- */
/* 
 * Copyright 2016 <+YOU OR YOUR COMPANY+>.
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
#include "su_sample_receiver_cb_impl.h"

namespace gr {
  namespace lsa {

    su_sample_receiver_cb::sptr
    su_sample_receiver_cb::make()
    {
      return gnuradio::get_initial_sptr
        (new su_sample_receiver_cb_impl());
    }

    enum SuRxState{
      SU_ONLY,
      INTERFERING
    };
    /*
     * The private constructor
     */
    su_sample_receiver_cb_impl::su_sample_receiver_cb_impl(const string& sensing_tag_id, const gr::digital::constellation_sptr& hdr_sptr)
      : gr::block("su_sample_receiver_cb",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(0, 1, sizeof(gr_complex))),
        d_src_id(pmt::intern(alias()))
    {
      d_hdr_sptr = hdr_sptr->base();
      d_state = SU_ONLY;

      d_msg_port = pmt::mp("sensing_info");
      d_debug_port = pmt::mp("debug");
      d_sensing_tag_id = pmt::string_to_symbol(sensing_tag_id);

      const size_t nitems = 1024;
      set_max_noutput_items(nitems);
      d_byte_ref = (unsigned char*) malloc(sizeof(char)*nitems);
      // d_name = (gr_complex*) volk_malloc(sizeof(gr_complex)*nitems, volk_get_alignment());
      //int nsamples
      //set_output_multiple(nsamples);

      //set_history(1);
      //declare_sample_delay(port, delay);


      message_port_register_out(d_msg_port);
      message_port_register_out(d_debug_port);
      set_tag_propagation_policy(TPP_DONT);
    }

    /*
     * Our virtual destructor.
     */
    su_sample_receiver_cb_impl::~su_sample_receiver_cb_impl()
    {
    }

    void
    su_sample_receiver_cb_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      ninput_items_required[0]=noutput_items;
    }

    bool
    su_sample_receiver_cb_impl::symbol_segment(std::vector<tag_t>& tags)
    {
      int offset;
      bool sensing_result;
      for(int i=0;i<tags.size();++i)
      {
        if(tags[i].key == d_sensing_tag_id){
          sensing_result = tags[i].value.to_bool();
          if((d_state == SU_ONLY) && sensing_result)
          { //interfering ProU, should discard and feedback sensing info to TX

          }
          else if((d_state == INTERFERING) && !sensing_result){
            //change from interfering to clear, should feedback the received index back to TX
          }
        }
      }

      return bool;
    }

    int
    su_sample_receiver_cb_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_compelx *in = (const gr_complex *) input_items[0];
      if(!output_items.empty()){
        gr_complex *out = (gr_complex *) output_items[0];  
      }
      // assume input is complex symbols, with tags of sensing info to be feedback
      // relating the state of sensing info. Labeling results and publish to msg port
      // output is optional, meant for debugging purpose.
      //unsigned char* demod_bytes;
      std::vector<tag_t> tags;
      get_tags_in_range(tags,0,nitems_read(0),nitems_read(0)+noutput_items);


      for(int i=0;i<noutput_items;++i)
      {
        d_bytes_reg[i]=d_hdr_sptr->decision_maker(&in[i]);
      }

      // Do <+signal processing+>
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (noutput_items);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace lsa */
} /* namespace gr */

