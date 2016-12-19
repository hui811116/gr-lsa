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
#include "su_header_prefix_impl.h"
#include <string>

namespace gr {
  namespace lsa {

    su_header_prefix::sptr
    su_header_prefix::make(const std::string& accesscode, const std::string & lengthtagname, bool mode)
    {
      return gnuradio::get_initial_sptr
        (new su_header_prefix_impl(accesscode,lengthtagname,mode));
    }

    /*
     * The private constructor
     */
    su_header_prefix_impl::su_header_prefix_impl(const std::string & accesscode, const std::string& lengthtagname,bool mode)
      : gr::tagged_stream_block("su_header_prefix",
              gr::io_signature::make(1, 1, sizeof(unsigned char)),
              gr::io_signature::make(1, 1, sizeof(unsigned char)), lengthtagname),
      d_mode(mode)
    {
      if(!set_accesscode(accesscode)){
        throw std::runtime_error("SU header prefix:setting accesscode failed");
      }
      set_tag_propagation_policy(TPP_DONT);
    }

    /*
     * Our virtual destructor.
     */
    su_header_prefix_impl::~su_header_prefix_impl()
    {
    
    }

    int
    su_header_prefix_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
    {
      int noutput_items = ninput_items[0]+header_nbytes();/* <+set this+> */; 
      return noutput_items ;
    }

    bool
    su_header_prefix_impl::set_accesscode(const std::string & accesscode)
    {
      //note that this version only implement reading string from index '0'
      d_accesscode_len=accesscode.length();
      if(d_accesscode_len >64){
        return false;
      }
      d_mask = ((~0ULL) >> (64- d_accesscode_len) );
      d_accesscode=0;
      for(unsigned i=0;i<d_accesscode_len;++i){
        d_accesscode = (d_accesscode << 1) | (accesscode[i] & 1);
      }
      return true;   
    }
    unsigned long long
    su_header_prefix_impl::accesscode() const
    {
      return d_accesscode;
    }

    size_t 
    su_header_prefix_impl::header_nbits() const
    {
      return d_accesscode_len + 16*2 + 8*2 + 16;
    }

    size_t 
    su_header_prefix_impl::header_nbytes() const
    {
      return (size_t)(ceil(d_accesscode_len/(float)8.0)+8);
    }

    void
    su_header_prefix_impl::gen_header(unsigned char* out, uint16_t payload_size)
    {
      int acc_bytes=ceil(d_accesscode_len/(float)8.0);
      unsigned char* acc=(unsigned char*)&d_accesscode;
      for(int i=0;i<acc_bytes;++i){
        out[i]=acc[acc_bytes-1-i];
      }
      unsigned char* pay_size=(unsigned char*)&payload_size;
      for(int i=0;i<2;++i){
        out[acc_bytes+i]=pay_size[1-i];
        out[acc_bytes+2+i]=pay_size[1-i];
      }
      unsigned char re_size=0x00;
      unsigned char re_indx=0x00;
      uint16_t count=0xffff;
      if(d_mode){
        re_size=0xff;
        re_indx=0x11;
      }
      out[acc_bytes+4]=re_size;
      out[acc_bytes+5]=re_indx;
      unsigned char * count_u8=(unsigned char*)&count;
      out[acc_bytes+6]=count_u8[1];
      out[acc_bytes+7]=count_u8[0];
    }

    int
    su_header_prefix_impl::work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const unsigned char *in = (const unsigned char *) input_items[0];
      unsigned char*out = (unsigned char *) output_items[0];

      // Do <+signal processing+>
      //memcpy(out,d_accessbytes,sizeof(unsigned char)*d_prefix_bytes);
      //memcpy(out+d_prefix_bytes,in,sizeof(unsigned char)*ninput_items[0]);
      gen_header(out, (uint16_t)ninput_items[0]);
      memcpy(out+header_nbytes(),in,sizeof(char)*ninput_items[0]);

      std::vector<tag_t> tags;
      get_tags_in_range(tags,0,nitems_read(0),nitems_read(0)+ninput_items[0]);
      for(int i=0;i<tags.size();++i){        
        tags[i].offset -= nitems_read(0);
        add_item_tag(0, nitems_written(0)+tags[i].offset, tags[i].key, tags[i].value);
        //add_item_tag(0, nitems_written(0)+tags[i].offset, pmt::intern("mode"),pmt::from_bool(d_mode));
      }
      // Tell runtime system how many output items we produced.
      //return noutput_items;
      return ninput_items[0] + header_nbytes();
    }

  } /* namespace lsa */
} /* namespace gr */
 
