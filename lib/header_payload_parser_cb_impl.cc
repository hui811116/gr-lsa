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
#include "header_payload_parser_cb_impl.h"

#include <cmath>

namespace gr {
  namespace lsa {

    header_payload_parser_cb::sptr
    header_payload_parser_cb::make(constellation_sptr hdr_constellation,
                                   constellation_sptr pld_constellation,
                                   const std::vector<gr_complex>& symbols,
                                   const std::vector<unsigned char>& accessbytes
                                   )
    {
      return gnuradio::get_initial_sptr
        (new header_payload_parser_cb_impl(hdr_constellation,
                                           pld_constellation,
                                           symbols,
                                           accessbytes));
    }

    /*
     * The private constructor
     */
    header_payload_parser_cb_impl::header_payload_parser_cb_impl(constellation_str hdr_constellation,
                                                                 constellation_str pld_constellation,
                                                                 const std::vector<gr_complex>& symbols,
                                                                 const std::vector<unsigned char>& accessbytes,
                                                                 double threshold
                                                                 )
            : gr::block("header_payload_parser_cb",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(unsigned char))),
              d_hdr_const(hdr_constellation),
              d_pld_const(pld_constellation),
              d_threshold(threshold),
              d_symbols(symbols)
    {
      symbol_norm=0.0;
      for(int i=0;i<symbols.size();++i)
        symbol_norm+=norm(symbols[i]);
    }

    /*
     * Our virtual destructor.
     */
    header_payload_parser_cb_impl::~header_payload_parser_cb_impl()
    {
    }

    //void
    //header_payload_parser_cb_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    //{
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
    //}
    void
    header_payload_parser_cb_impl::cal_correlation(std::vector<gr_complex>& corr,const gr_complex* in,int n_size)
    {      
      //calaulate cross correlation
      int iter=n_size-d_symbols.size()+1;
      //gr_complex corr_sum;
      corr.resize(iter);
      double tmp_norm;
      for(int i=0;i<iter;++i){
        corr[i]=gr_complex(0,0);
        tmp_norm=0;
        for(int j=0;j<d_symbols.size();++j){
          corr[i]+=in[i+j]*conj(d_symbols[j]);
          tmp_norm+=norm(in[i+j]);
        }
        if(abs(corr[i])>1e-16){
          corr[i]=corr[i]/std::sqrt(symbol_norm)/std::sqrt(tmp_norm);
        }
        else
          corr[i]=1e-16;
      }
    }

    bool
    header_payload_parser_cb_impl::corr_thres_locate_pkt(std::vector<int>& indices,const std::vector<gr_complex>& corr)
    {
      int begin=0,end=0;
      for(int i=0;i<corr.size();++i){
        if(abs(corr[i])>=d_threshold){
          begin=i;
          end=i;
          while(i<corr.size() && end==begin){
            if(abs(corr[i])<d_threshold){
              end=i;
              indices.push_back(i);
            }
            i++;
          }
        }
      }
      return !indices.empty();
    }

    void
    header_payload_parser_cb_impl::sync_accessbytes(std::vector<int>& pos,unsigned char* in, int i_size)
    {
      int niter=i_size-d_accessbytes.size()+1;
      int count;
      for(int i=0;i<niter;++i)
      {
        count=0;
        for(int j=0;j<d_accessbytes.size();++j)
        {
          if(in[i+j]==d_accessbytes[j])
            count++;
        }
        if(count==d_accessbytes.size())
            pos.push_back(i);
      }
    }

    bool
    header_payload_parser_cb_impl::hdr_demux(std::vector<int>& positions,const gr_complex * in, int i_size)
    {
      //int hdr_dim=d_hdr_const->dimensionality();
      unsigned char tmp_bytes[i_size];
      std::vector<gr_complex> corr;
      cal_coorelation(corr,in,i_size);
      std::vector<int> indices;
      if(corr_thres_locate_pkt(indices,corr))
      {
        for(int i=0;i<i_size;++i){
          tmp_bits[i]=d_hdr_const->decision_maker(&(in[i]));
        }
        //std::vector<int> positions;
        sync_accesscode(positions, tmp_bytes, i_size);
        return !positions.empty();
      }
      return false;
    }


    std::vector<unsigned char>
    header_payload_parser_cb_impl::accessbytes() const
    {
      return d_accessbytes;
    }

    void
    header_payload_parser_cb_impl::set_accessbytes(const std::vector<unsigned char>& accessbytes)
    {
      d_accessbytes=accessbytes;
    }

    double
    header_payload_parser_cb_impl::threshold() const
    {
      return d_threshold;
    }

    void
    header_payload_parser_cb_impl::set_threshold(double threshold)
    {
      d_threshold=threshold;
    }

    std::vector<gr_complex>
    header_payload_parser_cb_impl::symbols() const
    {
      return d_symbols;
    }

    void
    header_payload_parser_cb_impl::set_symbols(const std::vector<gr_complex>& symbols)
    {
      d_symbols=symbols;
    }

    int
    header_payload_parser_cb_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      //const gr_complex *corr= (const gr_complex *) input_items[1];
      unsigned char *out = (unsigned char *) output_items[0];

      // Do <+signal processing+>
      std::vector<int> positions;
      if(!hdr_demux(positions,in,ninput_items[0])){
        //how to parse the infomation in the header field
      }
      else{

      }
      

      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (noutput_items);

      // Tell runtime system how many output items we produced.
      return noutput_items;
      
    }

  } /* namespace lsa */
} /* namespace gr */

