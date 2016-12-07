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
    su_header_prefix::make(const std::string& accesscode, const std::string & lengthtagname)
    {
      return gnuradio::get_initial_sptr
        (new su_header_prefix_impl(accesscode,lengthtagname));
    }

    /*
     * The private constructor
     */
    su_header_prefix_impl::su_header_prefix_impl(const std::string & accesscode, const std::string& lengthtagname)
      : gr::tagged_stream_block("su_header_prefix",
              gr::io_signature::make(1, 1, sizeof(unsigned char)),
              gr::io_signature::make(1, 1, sizeof(unsigned char)), lengthtagname)
    {
      d_prefix_bytes=accesscode.length()/8;
      d_accessbytes= new unsigned char[accesscode.length()/8];
      init_accesscode(accesscode);
      set_tag_propagation_policy(TPP_DONT);
      //std::cout << "prefix bits" << d_prefix_bytes<<std::endl;
    }

    /*
     * Our virtual destructor.
     */
    su_header_prefix_impl::~su_header_prefix_impl()
    {
      delete [] d_accessbytes;
    }

    int
    su_header_prefix_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
    {
      int noutput_items = ninput_items[0]+d_prefix_bytes;/* <+set this+> */;
      //int noutput_items = ninput_items[0];
      return noutput_items ;
    }

    void
    su_header_prefix_impl::init_accesscode(const std::string & accesscode)
    {
      //note that this version only implement reading string from index '0'
      //for(int i=0;i<accesscode.length()/8;++i)
        //accessbytes[i]=0xff;
      //unsigned char tmp;
      for(int i=0;i<d_prefix_bytes*8;++i){
        if(i%8==0)
          d_accessbytes[i/8]=0x00;
        d_accessbytes[i/8] |= ((accesscode[i]!='0')?0x01:0x00) << (7- i%8);
      }
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
      //memcpy(out,in,sizeof(unsigned char)*ninput_items[0]);
      memcpy(out,d_accessbytes,sizeof(unsigned char)*d_prefix_bytes);
      memcpy(out+d_prefix_bytes,in,sizeof(unsigned char)*ninput_items[0]);

      std::vector<tag_t> tags;
      get_tags_in_range(tags,0,nitems_read(0),nitems_read(0)+ninput_items[0]);
      for(int i=0;i<tags.size();++i){
        //std::cerr << "****************DEBUG******************" << std::endl;
        //std::cerr << "offset: "<<tags[i].offset<<std::endl;
        //std::cerr << "key   : "<<pmt::symbol_to_string(tags[i].key)<<std::endl;
        //std::cerr << "value : "<<pmt::to_long(tags[i].value)<<std::endl;
        //std::cerr << "***************************************" << std::endl;
        //add_item_tag(0, nitems_written(0)+tags[i].offset,tags[i].key, tags[i].value ,pmt::intern(alias()));
        //add_item_tag(0, nitems_written(0)+tags[i].offset,pmt::intern("hdr_bytes"),pmt::from_long((long)d_prefix_bytes),pmt::intern(alias()));
        tags[i].offset -= nitems_read(0);
        add_item_tag(0, nitems_written(0)+tags[i].offset, tags[i].key, tags[i].value);
      }
      // Tell runtime system how many output items we produced.
      //return noutput_items;
      return ninput_items[0] + d_prefix_bytes;
    }

  } /* namespace lsa */
} /* namespace gr */
 
