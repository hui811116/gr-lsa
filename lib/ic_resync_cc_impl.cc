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
#include "ic_resync_cc_impl.h"
#include <volk/volk.h>

namespace gr {
  namespace lsa {
    #define DEBUG d_debug && std::cout
    #define CAPACITY 128*1024*1024
    #define FIRCAPACITY 256*128
    static int d_sps = 4;
    static int d_prelen = 128; // symbols 16*8
    static int d_phylen = 192; // 0x00,0x00,0x00,0x00,0xe6,0xXX
    enum VOESTATE{
      VOE_CLEAR,
      VOE_TRIGGERED
    };

    ic_resync_cc::sptr
    ic_resync_cc::make(const std::vector<float>& taps)
    {
      return gnuradio::get_initial_sptr
        (new ic_resync_cc_impl(taps));
    }

    /*
     * The private constructor
     */
    ic_resync_cc_impl::ic_resync_cc_impl(const std::vector<float>& taps)
      : gr::block("ic_resync_cc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(2, 2, sizeof(gr_complex))),
              d_in_port(pmt::intern("pkt_in")),
              d_cap(CAPACITY)
    {
      message_port_register_in(d_in_port);
      set_msg_handler(d_in_port,boost::bind(&ic_resync_cc_impl::msg_in,this,_1));
      d_fir_buffer = (gr_complex*)volk_malloc(sizeof(gr_complex)*FIRCAPACITY,volk_get_alignment());
      d_in_mem = (gr_complex*)volk_malloc(sizeof(gr_complex)*d_cap,volk_get_alignment());
      d_in_idx =0;
      d_out_idx=0;
      d_intf_idx=0;
      d_intf_mem = new gr_complex[d_cap];
      d_out_mem = new gr_complex[d_cap];
      d_state = VOE_CLEAR;
    }

    /*
     * Our virtual destructor.
     */
    ic_resync_cc_impl::~ic_resync_cc_impl()
    {
      volk_free(d_fir_buffer);
      volk_free(d_in_mem);
      delete [] d_intf_mem;
      delete [] d_out_mem;
    }

    bool
    ic_resync_cc_impl::voe_update(int idx)
    {
      if(d_voe_tags.empty()){
        int offset = d_voe_tags[0].offset - nitems_read(0);
        if(offset == idx){
          d_voe_tags.erase(d_voe_tags.begin());
          bool result = pmt::to_bool(d_voe_tags[0].value);
          if(d_state == VOE_CLEAR && result){
            return true;
          }else if(d_state == VOE_TRIGGERED && !result){
            return true;
          }
        }
      }
      return false;
    }

    void
    ic_resync_cc_impl::msg_in(pmt::pmt_t msg)
    {
      gr::thread::scoped_lock guard(d_mutex);
      pmt::pmt_t k = pmt::car(msg);
      pmt::pmt_t v = pmt::cdr(msg);
      assert(pmt::is_dict(k));
      assert(pmt::is_blob(v));
      size_t io(0);
      const uint8_t* uvec = u8vector_elements(v,io);
      uint64_t block = pmt::to_uint64(pmt::dict_ref(k,pmt::intern("block"),pmt::from_uint64(0)));
      int offset = pmt::to_long(pmt::dict_ref(k,pmt::intern("offset"),pmt::from_long(0)));
      offset*= d_sps;
      int payload = pmt::to_long(pmt::dict_ref(k,pmt::intern("payload"),pmt::from_long(0)));
      int pktlen = (payload+d_phylen)*d_sps;
      matching_pkt(block,offset,pktlen);
    }

    bool
    ic_resync_cc_impl::matching_pkt(uint64_t bid, int offset, int pktlen)
    {
      std::list< std::pair<uint64_t, int> >::reverse_iterator rit;
      return true;
    }

    void
    ic_resync_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = noutput_items;
    }

    int
    ic_resync_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
      gr_complex *demo= (gr_complex *) output_items[1];
      int nin = ninput_items[0];
      int nout = 0;
      int count =0;
      d_voe_tags.clear();
      std::vector<tag_t> tags;
      get_tags_in_window(tags,0,0,nin,pmt::intern("block_tag"));
      get_tags_in_window(d_voe_tags,0,0,nin,pmt::intern("voe_tag"));
      d_nex_block_idx = d_block_idx+nin;
      if(!tags.empty()){
        count = tags[0].offset - nitems_read(0);
        count++;
        d_block_list.push_back(std::make_pair(d_block,d_block_idx+count));
        d_nex_block_idx=0;
        d_nex_block = pmt::to_uint64(tags[0].value);
      }
      while(count<nin){
        switch(d_state)
        {
          case VOE_CLEAR:
            while(count<nin){
              if(voe_update(count)){
                d_state = VOE_TRIGGERED;
                // try to locate a header in advance?
                // use SFD? Correlation?
                // should record the block and idx of the event for back tracking
                break;
              }
              d_in_mem[d_in_idx++] = in[count++];
              d_in_idx%=d_cap;
            }
          break;
          case VOE_TRIGGERED:
            while(count<nin){
              if(voe_update(count)){
                d_state = VOE_CLEAR;
                // should record the block and idx of the event for back tracking
                break;
              }
              d_in_mem[d_in_idx++] = in[count];
              d_in_idx%=d_cap;
              count = (count+1)%d_cap;
            }
          break;
          default:
            throw std::runtime_error("Undefined state");
          break;
        }
      }
      d_block_idx=d_nex_block_idx;
      d_block = d_nex_block;
      consume_each (count);
      return nout;
    }

  } /* namespace lsa */
} /* namespace gr */

