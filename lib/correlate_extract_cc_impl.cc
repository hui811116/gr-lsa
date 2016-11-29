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
#include "correlate_extract_cc_impl.h"

namespace gr {
  namespace lsa {

    correlate_extract_cc::sptr
    correlate_extract_cc::make(const std::vector<gr_complex>& symbols, float threshold)
    {
      //d_symbols=symbols;
      //d_threshold=( (threshold>=0) && (threshold<=1) )? threshold : 0.5;
      return gnuradio::get_initial_sptr
        (new correlate_extract_cc_impl(symbols,threshold));
    }

    /*
     * The private constructor
     */
    correlate_extract_cc_impl::correlate_extract_cc_impl(const std::vector<gr_complex>& symbols,
                                                         float threshold)
      : gr::block("correlate_extract_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(2, 2, sizeof(gr_complex)))
    {
      d_symbols=symbols;
      d_threshold=( (threshold>=0) && (threshold<=1) )? threshold : 0.5;
    }

    /*
     * Our virtual destructor.
     */
    correlate_extract_cc_impl::~correlate_extract_cc_impl()
    {
    }
    /*
    void
    correlate_extract_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      // <+forecast+> e.g. ninput_items_required[0] = noutput_items 
    }
    */
    void
    correlate_extract_cc_impl::calc_corr(gr_complex* corr,const gr_complex* in, int in_size)
    {
      int ns=d_symbols.size();
      gr_complex temp(0,0);
      for(int i=0;i<in_size;++i){
        //temp=gr_complex(0,0);
        corr[i]=gr_complex(0,0);
        for(int j=1;j<ns;++j){
          //temp+=in[i+j]*conj(d_symbols[j]);
          corr[i]+=in[i+j]*conj(d_symbols[j]);
        }
        //out[i]=temp;
      }
    }

    void
    correlate_extract_cc_impl::set_symbols(const std::vector<gr_complex>& symbols)
    {
      d_symbols=symbols;
    }

    std::vector<gr_complex>
    correlate_extract_cc_impl::symbols() const
    {
      return d_symbols;
    }

    void
    correlate_extract_cc_impl::set_threshold(float threshold)
    {
      d_threshold= ((threshold>=0)&&(threshold<=1))? threshold : d_threshold;
    }

    float
    correlate_extract_cc_impl::get_threshold() const
    {
      return d_threshold;
    }

    int
    correlate_extract_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
      gr_complex *corr_out = (gr_complex*) output_items[1];

      int nin=ninput_items[0];
      int ns=d_symbols.size();
      int n_out=((nin-ns+1)>0)?nin-ns+1:0;
      //gr_complex corr[n_out];
      calc_corr(corr_out,in,n_out);
      // Do <+signal processing+>
      int iter=0;bool found=false;
      int index=nin;
      while((!found)&&(iter<n_out)){
        if(abs(corr_out[iter])>=d_threshold){
          index=iter;
          found=true;
        }
          iter++;
      }
      noutput_items=((nin-index)>0)?nin-index:0;
      if(noutput_items!=0){
        for(int i=0;i<noutput_items;++i)
          out[i]=in[index+i];
      }
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (n_out);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace lsa */
} /* namespace gr */

