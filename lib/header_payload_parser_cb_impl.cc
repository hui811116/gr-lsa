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
    using namespace digital;

    header_payload_parser_cb::sptr
    
    header_payload_parser_cb::make(gr::digital::constellation_sptr hdr_constellation,
                                   gr::digital::constellation_sptr pld_constellation,
                                   const std::vector<gr_complex>& symbols,
                                   const std::string & accesscode,
                                   float threshold
                                   )
    {
      return gnuradio::get_initial_sptr
        (new header_payload_parser_cb_impl(hdr_constellation,
                                           pld_constellation,
                                           symbols,
                                           accesscode,
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
                                                                 const std::string & accesscode,
                                                                 float threshold
                                                                 )
            : gr::block("header_payload_parser_cb",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make2(1, 2, sizeof(gr_complex), sizeof(char))),
              d_hdr_const_ptr(hdr_constellation),
              d_pld_const_ptr(pld_constellation),
              d_threshold(threshold),
              d_symbols(symbols)
    {
      set_accessbits(accesscode);
      symbol_norm=0.0;
      for(int i=0;i<symbols.size();++i)
        symbol_norm+=norm(symbols[i]);
      d_msg_port=pmt::mp("payload");
      message_port_register_out(d_msg_port);
      set_tag_propagation_policy(TPP_DONT);
    }

    /*
     * Our virtual destructor.
     */
    header_payload_parser_cb_impl::~header_payload_parser_cb_impl()
    {
    }

    void
    header_payload_parser_cb_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      for(int i=0;i<ninput_items_required.size();++i)
        ninput_items_required[i]=noutput_items;
    }
    void
    header_payload_parser_cb_impl::cal_correlation(std::vector<gr_complex>& corr,const gr_complex* in,int n_size)
    {      
      //calaulate cross correlation
      int iter=n_size-d_symbols.size()+1;
      //gr_complex corr_sum;
      corr.resize(iter);
      float tmp_norm;
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

    void
    header_payload_parser_cb_impl::repack_bits_lsb(unsigned char* out, unsigned char* in, unsigned int len, unsigned int bps)
    {
      int total_len=len*bps;
      for(int i=0;i<total_len;++i)
        out[i]=(0x01 & (in[i/bps] >> (i%bps)));        
    }


    std::string
    header_payload_parser_cb_impl::accessbits() const
    {
      //return d_accessbits;
      std::string str;
      std::string _one("1");
      std::string _zero("0");
      for(int i=0;i<d_accessbits.size();++i)
        str+=(d_accessbits[i]==0x01) ? _one : _zero;
      return str;
    }

    void
    header_payload_parser_cb_impl::set_accessbits(const std::string & accesscode)
    {
      d_accessbits.clear();
      for(int i=0;i<accesscode.length();++i){
        d_accessbits.push_back((accesscode[i]!='0')? 0x01 : 0x00);
      }
      //d_accessbits=accessbits;
    }

    float
    header_payload_parser_cb_impl::threshold() const
    {
      return d_threshold;
    }

    void
    header_payload_parser_cb_impl::set_threshold(float threshold)
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
      gr_complex *out = (gr_complex *) output_items[0];
      unsigned char* out_port1;
      if(output_items.size()>1)
       out_port1 = (unsigned char*) output_items[1];

      memcpy(out,in,sizeof(gr_complex)*noutput_items);
      // Do <+signal processing+>
      
      unsigned char tmp[noutput_items];
      for(int i=0;i<noutput_items;++i){
        tmp[i]=d_hdr_const_ptr->decision_maker(&(in[i]));
      } 

      if(output_items.size()>1)
        memcpy(out_port1,tmp,sizeof(unsigned char)*noutput_items);
      //unpacking to k bits
      int total_len=noutput_items*d_hdr_const_ptr->bits_per_symbol();
      unsigned char repack_bits[total_len];
      repack_bits_lsb(repack_bits,tmp,ninput_items[0],d_hdr_const_ptr->bits_per_symbol());
      std::vector<int> positions;
      sync_accessbits(positions, repack_bits, total_len);
      /*
       this version meant for default header type, for other implementation,
       reuse the enum type defined in the begin of this file.
      */
      unsigned short first,second;
      int pos_reg;
      int consume_idx=noutput_items;
      pmt::pmt_t pdu_out;
      gr::blocks::pdu::vector_type d_type = gr::blocks::pdu::byte_t;
      for(int i=0;i<positions.size();++i){
        //default packet length in header 16+16 (4bytes)
        //add_item_tag(0,nitems_written(0)+positions[i],pmt::intern("index:"),pmt::from_long(positions[i]),pmt::intern(alias()));
        if(positions[i]+d_accessbits.size()+32<total_len){
          first=0x0000;
          second=0x0000;
          pos_reg=positions[i]+d_accessbits.size();
          for(int j=0;j<16;++j){
            first |= (repack_bits[pos_reg+j] & 0x01)<<15-j;
            second |=(repack_bits[pos_reg+16+j] & 0x01)<<15-j;
          }
          add_item_tag(0,nitems_written(0)+pos_reg,pmt::intern("length1"),pmt::from_long(first),pmt::intern(alias()));
          add_item_tag(0,nitems_written(0)+pos_reg,pmt::intern("length2"),pmt::from_long(second),pmt::intern(alias()));
          if(first==second){
            //finding payload symbols
            if((pos_reg+32)/d_hdr_const_ptr->bits_per_symbol()+first*8/d_pld_const_ptr->bits_per_symbol()<ninput_items[0]){
              consume_idx=(pos_reg+32)/d_hdr_const_ptr->bits_per_symbol()+first*8/d_pld_const_ptr->bits_per_symbol();
              for(int j=0;j<first;++j){
                tmp[j]=d_pld_const_ptr->decision_maker(&(in[(pos_reg+32)/d_hdr_const_ptr->bits_per_symbol()+j]));
              }
              pdu_out=pmt::make_dict();
              pdu_out=pmt::dict_add(pdu_out, pmt::intern("payload"), pmt::from_long((long)first));
              pdu_out=gr::blocks::pdu::make_pdu_vector(d_type, tmp, first);
              message_port_pub(d_msg_port,pdu_out);
              add_item_tag(0,nitems_written(0)+consume_idx,pmt::intern("payload"),pmt::from_long(first),pmt::intern(alias()));
            }//matched payload
          }//packet length match (deault header setup)
        }//total len matched

        if(output_items.size()>1){
          add_item_tag(1,nitems_written(1)+positions[i],pmt::intern("debug"),pmt::string_to_symbol(accessbits()),pmt::intern(alias()));
        }
      }

      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (consume_idx);
      produce(0,noutput_items); // for output port 0
      if(output_items.size()>1)
        produce(1,noutput_items); // for output port 1
      // Tell runtime system how many output items we produced.
      return WORK_CALLED_PRODUCE;
      //return ninput_items[0];
      
    }

  } /* namespace lsa */
} /* namespace gr */

