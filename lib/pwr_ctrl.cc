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
    	enum PWR_CTRL_STATE{
    		CLEAR_HIGH,
    		COLLISION_DELAY,
    		CLEAR_LOW,
    	};
    	pwr_ctrl_impl(float high_thres,float low_thres):block("pwr_ctrl",
    		gr::io_signature::make(0,0,0),
    		gr::io_signature::make(0,0,0)),
    		d_pwr_in(pmt::mp("pwr_in")),
    		d_state_out(pmt::mp("state_out"))
    	{
    		message_port_register_in(d_pwr_in);
    		message_port_register_out(d_state_out);
    		set_msg_handler(d_pwr_in,boost::bind(&pwr_ctrl_impl::msg_in,this,_1));
    		set_threshold(high_thres,low_thres);
    		d_cd_cnt =0;
    		d_state = CLEAR_HIGH;
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
    		float pwr_db = pmt::to_float(v);
    		// should be pwr in db
    		switch(d_state){
    			case CLEAR_HIGH:
    				if(pwr_db>d_high_thres){
    					d_state = COLLISION_DELAY;
    					d_cd_cnt++;
    					d_cur_msg = d_colli_msg;
    					message_port_pub(d_state_out,d_cur_msg);
    				}
    			break;
    			case COLLISION_DELAY:
    				if(pwr_db>d_high_thres){
    					message_port_pub(d_state_out,d_cur_msg);
    					d_cd_cnt++;
    				}else if(pwr_db<d_high_thres && pwr_db<d_low_thres){
    					d_state = CLEAR_LOW;
    				}
    			break;
    			case CLEAR_LOW:
    				if(pwr_db>d_low_thres){
    					// maybe found a pu signal passing by
    					d_cd_cnt--;
    					if(d_cd_cnt==0){
    						d_state = CLEAR_HIGH;
    						d_cur_msg = d_clear_msg;
    						message_port_pub(d_state_out,d_cur_msg);
    					}
    				}
    			break;
    			default:
    				throw std::runtime_error("Undefined state");
    			break;
    		}
    	}
    	void set_threshold(float high_thres,float low_thres)
    	{
    		gr::thread::scoped_lock guard(d_mutex);
    		if(high_thres<low_thres){
    			d_high_thres = low_thres;
    			d_low_thres = high_thres;
    		}else{
    			d_high_thres = high_thres;
    			d_low_thres = low_thres;
    		}
    	}
    private:
    	const pmt::pmt_t d_pwr_in;
    	const pmt::pmt_t d_state_out;
    	pmt::pmt_t d_clear_msg;
    	pmt::pmt_t d_colli_msg;
    	int d_listen_slots;
    	gr::thread::mutex d_mutex;
    	int d_state;
    	int d_cd_cnt;
    	float d_high_thres;
    	float d_low_thres;
    	pmt::pmt_t d_cur_msg;
    };

    pwr_ctrl::sptr
    pwr_ctrl::make(float high_thres,float low_thres)
    {
    	return gnuradio::get_initial_sptr(new pwr_ctrl_impl(high_thres,low_thres));
    }

  } /* namespace lsa */
} /* namespace gr */

