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
#include <gnuradio/blocks/pdu.h>
#include <gnuradio/digital/constellation.h>
#include "header_payload_parser_cb_impl.h"

#include <cmath>

namespace gr {
  namespace lsa {

    header_payload_parser_cb::sptr
    header_payload_parser_cb::make(gr::digital::constellation_sptr hdr_constellation,
                                   gr::digital::constellation_sptr pld_constellation,
                                   const std::vector<gr_complex>& symbols,
                                   const std::vector<unsigned char>& accessbits,
                                   double threshold
                                   )
    {
      return gnuradio::get_initial_sptr
        (new header_payload_parser_cb_impl(hdr_constellation,
                                           pld_constellation,
                                           symbols,
                                           accessbits,
                                           threshold));
    }

    /*
     * The private constructor
     */
    enum HeaderType{DEFAULT,
                    COUNTER,
                    DEV};

    header_payload_parser_cb_impl::header_payload_parser_cb_impl(gr::digital::constellation_sptr hdr_constellation,
                                                                 gr::digital::constellation_sptr pld_constellation,
                                                                 const std::vector<gr_complex>& symbols,
                                                                 const std::vector<unsigned char>& accessbits,
                                                                 double threshold
                                                                 )
            : gr::block("header_payload_parser_cb",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(unsigned char))),
              d_hdr_const(hdr_constellation),
              d_pld_const(pld_constellation),
              d_threshold(threshold),
              d_symbols(symbols),
              d_accessbits(accessbits)
    {
      symbol_norm=0.0;
      for(int i=0;i<symbols.size();++i)
        symbol_norm+=norm(symbols[i]);
      d_msg_port=pmt::mp("payload");
      message_port_register_out(d_msg_port);
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
          corr[i]/=(std::sqrt(symbol_norm)/std::sqrt(tmp_norm));
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
    header_payload_parser_cb_impl::sync_accessbits(std::vector<int>& pos,const unsigned char* in, int i_size)
    {
      int niter=i_size-d_accessbits.size()+1;
      int count;
      for(int i=0;i<niter;++i)
      {
        count=0;
        for(int j=0;j<d_accessbits.size();++j)
        {
          if(in[i+j]==d_accessbits[j])
            count++;
        }
        if(count==d_accessbits.size())
            pos.push_back(i);
      }
    }
/*
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
        std::vector<int> positions;
        sync_accessbits(positions, tmp_bytes, i_size);
        return !positions.empty();
      }
      return false;
    }
*/
    //void
    //header_payload_parser_cb_impl::parse_packet_length()
    //{
    //}

    void
    header_payload_parser_cb_impl::repack_bits_lsb(unsigned char* out, unsigned char* in, unsigned int len, unsigned int bps)
    {
      int total_len=len*bps;
      for(int i=0;i<total_len;++i)
        out[i]=(0x01 & (in[i/bps] >> (i%bps)));        
    }


    std::vector<unsigned char>
    header_payload_parser_cb_impl::accessbits() const
    {
      return d_accessbits;
    }

    void
    header_payload_parser_cb_impl::set_accessbits(const std::vector<unsigned char>& accessbits)
    {
      d_accessbits=accessbits;
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
      gr_complex *out = (gr_complex *) output_items[0];
      memcpy(out,in,sizeof(gr_complex)*noutput_items);
      // Do <+signal processing+>
      
      unsigned char tmp[ninput_items[0]];
      for(int i=0;i<ninput_items[0];++i){
        tmp[i]=d_hdr_const->decision_maker(&(in[i]));
      } 
      //unpacking to k bits
      int total_len=ninput_items[0]*d_hdr_const->bits_per_symbol();
      unsigned char repack_bits[total_len];
      repack_bits_lsb(repack_bits,tmp,ninput_items[0],d_hdr_const->bits_per_symbol());
      std::vector<int> positions;
      sync_accessbits(positions, repack_bits, total_len);
      //parse_packet_length()
      /*
       this version meant for default header type, for other implementation,
       reuse the enum type defined in the begin of this file.
      */
      unsigned short first,second;
      int pos_reg;
      int consume_idx=ninput_items[0];
      pmt::pmt_t pdu_out;
      gr::blocks::pdu::vector_type d_type = gr::blocks::pdu::byte_t;
      //pmt::pmt_t info;
      for(int i=0;i<positions.size();++i){
        //default packet length in header 16+16
        if(positions[i]+d_accessbits.size()+32<total_len){
          first=0x0000;
          second=0x0000;
          pos_reg=positions[i]+d_accessbits.size();
          for(int j=0;j<16;++j){
            first |= (repack_bits[pos_reg+j] & 0x01)<<j;
            second |=(repack_bits[pos_reg+16+j] & 0x01)<<j;
          }
          if(first==second){
            //finding payload symbols
            if((pos_reg+32)/d_hdr_const->bits_per_symbol()+first*8/d_pld_const->bits_per_symbol()<ninput_items[0]){
              consume_idx=(pos_reg+32)/d_hdr_const->bits_per_symbol()+first*8/d_pld_const->bits_per_symbol();
              for(int j=0;j<first;++j){
                tmp[j]=d_pld_const->decision_maker(&(in[(pos_reg+32)/d_hdr_const->bits_per_symbol()+j]));
              }
              pdu_out=pmt::make_dict();
              pdu_out=pmt::dict_add(pdu_out, pmt::intern("payload"), pmt::from_long((long)first));
              pdu_out=gr::blocks::pdu::make_pdu_vector(d_type, tmp, first);
              message_port_pub(d_msg_port,pdu_out);
            }//matched payload
          }//packet length match (deault header setup)
        }//total len matched
      }

      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (consume_idx);

      // Tell runtime system how many output items we produced.
      return noutput_items;
      
    }

  } /* namespace lsa */
} /* namespace gr */

