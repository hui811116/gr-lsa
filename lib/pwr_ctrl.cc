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
#include <lsa/pwr_ctrl.h>
#include <gnuradio/block_detail.h>

namespace gr {
  namespace lsa {
  	#define d_debug false
    #define dout d_debug && std::cout
    #define EVENT_COLLISION 2
    #define EVENT_CLEAR 3
    static const unsigned char d_collision_bytes[] = {0xff,0x00};
    static const unsigned char d_clear_bytes[] = {0x00,0xff,0x0f};
    class pwr_ctrl_impl: public pwr_ctrl
    {
    public:
    	pwr_ctrl_impl():block("pwr_ctrl",
    		gr::io_signature::make(0,0,0),
    		gr::io_signature::make(0,0,0)),
    		d_pwr_in(pmt::mp("pwr_in")),
    		d_state_out(pmt::mp("state_out"))
    	{
    		message_port_register_in(d_pwr_in);
    		message_port_register_out(d_state_out);
    		set_msg_handler(d_pwr_in,boost::bind(&pwr_ctrl_impl::msg_in,this,_1));
    		d_clear_msg = pmt::cons(pmt::PMT_NIL,pmt::make_blob(d_clear_bytes,EVENT_CLEAR));
    		d_colli_msg = pmt::cons(pmt::PMT_NIL,pmt::make_blob(d_collision_bytes,EVENT_COLLISION));
    	}
    	~pwr_ctrl_impl()
    	{

    	}
    	void msg_in(pmt::pmt_t msg)
    	{
    		pmt::pmt_t k = pmt::car(msg);
    		pmt::pmt_t v = pmt::cdr(msg);
    		// is number
    		if(!pmt::is_number(v))
    			return;
            int ctrl_type = pmt::to_long(v);
            if(ctrl_type == EVENT_COLLISION){
                d_cur_msg = d_colli_msg;
                message_port_pub(d_state_out,d_cur_msg);
            }else if(ctrl_type == EVENT_CLEAR){
                d_cur_msg = d_clear_msg;
                message_port_pub(d_state_out,d_cur_msg);
            }else{
                return;
            }
    	}
    private:
    	const pmt::pmt_t d_pwr_in;
    	const pmt::pmt_t d_state_out;
    	pmt::pmt_t d_clear_msg;
    	pmt::pmt_t d_colli_msg;
    	gr::thread::mutex d_mutex;
    	pmt::pmt_t d_cur_msg;
    };

    pwr_ctrl::sptr
    pwr_ctrl::make()
    {
    	return gnuradio::get_initial_sptr(new pwr_ctrl_impl());
    }

  } /* namespace lsa */
} /* namespace gr */

