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
#include "lsa_queue_cc_impl.h"

namespace gr {
  namespace lsa {

    lsa_queue_cc::sptr
    lsa_queue_cc::make()
    {
      return gnuradio::get_initial_sptr
        (new lsa_queue_cc_impl());
    }

    /*
     * The private constructor
     */
    lsa_queue_cc_impl::lsa_queue_cc_impl()
      : gr::block("lsa_queue_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(gr_complex)))
    {
      //d_queue_size=0;
      d_status=false;
      d_queue_capacity=1024;
      d_q_ptr=new std::vector<std::complex<float> >;
    }

    /*
     * Our virtual destructor.
     */
    lsa_queue_cc_impl::~lsa_queue_cc_impl()
    {
      d_q_ptr->clear();
      delete d_q_ptr;
    }
    
    //void
    //lsa_queue_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    //{
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
    //}

    int
    lsa_queue_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
      int nin=ninput_items[0];
      // Do <+signal processing+>
      enqueue(in,nin);
      noutput_items=0;
      if(check_status())
      {
        for(int i=0;i<d_q_ptr->size();++i){
          out[i]=d_q_ptr->at(i);
        }
        noutput_items=(int)d_q_ptr->size();
      }
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (nin);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

    void
    lsa_queue_cc_impl::set_capacity(int capacity)
    {
      if(capacity<d_q_ptr->size())
        dequeue_n(d_q_ptr->size()-capacity);
      d_queue_capacity=capacity;
    }
    void
    lsa_queue_cc_impl::enqueue(const gr_complex* in, int nin)
    {
      if(nin<(d_queue_capacity-d_q_ptr->size()))
      {
        for(int i=0;i<nin;++i)
          d_q_ptr->push_back(in[i]);
      }
      else
      {
        d_q_ptr->clear();
        for(int i=nin-d_queue_capacity;i<nin;++i)
          d_q_ptr->push_back(in[i]);
      }
    }
    void
    lsa_queue_cc_impl::dequeue_n(int nSample)
    {
      if(nSample < d_q_ptr->size())
        d_q_ptr->erase(d_q_ptr->begin(),d_q_ptr->begin()+nSample);
      else
        d_q_ptr->clear();
    }
    size_t 
    lsa_queue_cc_impl::get_queue_size() const
    {
      return d_q_ptr->size();
    }

    bool
    lsa_queue_cc_impl::check_status()
    {
      return d_status;
    }
    void
    lsa_queue_cc_impl::set_status(bool new_sta)
    {
      d_status=new_sta;
    }


  } /* namespace lsa */
} /* namespace gr */

