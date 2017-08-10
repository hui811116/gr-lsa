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
#include "stop_n_wait_rx_ctrl_cc_impl.h"
#include <volk/volk.h>

namespace gr {
  namespace lsa {

    stop_n_wait_rx_ctrl_cc::sptr
    stop_n_wait_rx_ctrl_cc::make(float ed_thres,const std::vector<gr_complex>& samples)
    {
      return gnuradio::get_initial_sptr
        (new stop_n_wait_rx_ctrl_cc_impl(ed_thres,samples));
    }

    /*
     * The private constructor
     */
    stop_n_wait_rx_ctrl_cc_impl::stop_n_wait_rx_ctrl_cc_impl(float ed_thres,const std::vector<gr_complex>& samples)
      : gr::block("stop_n_wait_rx_ctrl_cc",
              gr::io_signature::make2(2, 2, sizeof(gr_complex),sizeof(float)),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
              d_out_port(pmt::mp("ctrl_out")),
              d_src_id(pmt::intern(alias()))
    {
      
      if(samples.size()==0){
        throw std::invalid_argument("Sync words empty...");
      }
      d_samples = samples;
      volk_32fc_x2_conjugate_dot_prod_32fc(&d_sample_eng,samples.data(),samples.data(),samples.size());
      set_ed_threshold(ed_thres);
      enter_listen();
      message_port_register_out(d_out_port);
      set_tag_propagation_policy(TPP_DONT);
      d_buf = (gr_complex*) volk_malloc(sizeof(gr_complex)*(1024+samples.size()),volk_get_alignment());
    }

    /*
     * Our virtual destructor.
     */
    stop_n_wait_rx_ctrl_cc_impl::~stop_n_wait_rx_ctrl_cc_impl()
    {
      volk_free(d_buf);
    }
    
    void
    stop_n_wait_rx_ctrl_cc_impl::enter_listen()
    {
      d_state = ED_LISTEN;
      d_ed_cnt=0;
      d_voe_cnt=0;
      DEBUG<<"<SNS CTRL DEBUG>enter listen"<<std::endl;
    }
    void
    stop_n_wait_rx_ctrl_cc_impl::enter_sfd()
    {
      d_state = ED_SFD;
      d_ed_cnt=0;
      DEBUG<<"<SNS CTRL DEBUG>enter sfd"<<std::endl;
    }
    void
    stop_n_wait_rx_ctrl_cc_impl::enter_ed_pu()
    {
      d_state = ED_DETECT_PU;
      d_ed_cnt=0;
      DEBUG<<"<SNS CTRL DEBUG>enter ed pu"<<std::endl;
    }
    void
    stop_n_wait_rx_ctrl_cc_impl::enter_ed_sns()
    {
      d_state = ED_DETECT_SNS;
      d_ed_cnt=0;
      DEBUG<<"<SNS CTRL DEBUG>enter ed sns"<<std::endl;
    }
    void
    stop_n_wait_rx_ctrl_cc_impl::enter_silent()
    {
      d_state = ED_SILENT;
      d_silent_trig = false;
      d_ed_cnt=0;
      DEBUG<<"<SNS CTRL DEBUG>enter silent"<<std::endl;
    }
    void
    stop_n_wait_rx_ctrl_cc_impl::notify_clear()
    {
      //message_port_pub(d_out_port,pmt::cons(pmt::PMT_NIL,pmt::make_blob(d_sns_clear,3)));
      gr::thread::thread(&stop_n_wait_rx_ctrl_cc_impl::pub_clear,this).detach();
    }
    void
    stop_n_wait_rx_ctrl_cc_impl::pub_clear()
    {
      int count =0;
      while(count<2){
        message_port_pub(d_out_port,pmt::cons(pmt::PMT_NIL,pmt::make_blob(d_sns_clear,3)));
        boost::this_thread::sleep(boost::posix_time::milliseconds(25));
        count++;
      }
      DEBUG<<"<SNS CTRL DEBUG>pub message complete"<<std::endl;
    }
    void
    stop_n_wait_rx_ctrl_cc_impl::set_ed_threshold(float thres)
    {
      d_ed_thres = thres;
    }
    float
    stop_n_wait_rx_ctrl_cc_impl::ed_threshold()const 
    {
      return d_ed_thres;
    }
    
    void
    stop_n_wait_rx_ctrl_cc_impl::tags_handler(int count)
    {
      if(d_tags.empty()){
        return;
      }else{
        int offset = d_tags[0].offset-nitems_read(0);
        if(offset==count){
          bool test_voe = pmt::to_bool(d_tags[0].value);
          d_voe_cnt = (test_voe) ? d_voe_cnt+1 : d_voe_cnt;
          add_item_tag(0,nitems_written(0)+offset,d_tags[0].key,d_tags[0].value,d_src_id);
          d_tags.erase(d_tags.begin());
        }
      }
    }
    void
    stop_n_wait_rx_ctrl_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      if(d_state == ED_SFD){
        int sample_size = d_samples.size();
        for(int i=0;i<ninput_items_required.size();++i)
          ninput_items_required[i] = std::max(noutput_items,512+sample_size);
      }else{
        for(int i=0;i<ninput_items_required.size();++i)
          ninput_items_required[i] = noutput_items;
      }
    }

    int
    stop_n_wait_rx_ctrl_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      const float *ed = (const float*) input_items[1];
      gr_complex *out = (gr_complex *) output_items[0];
      int nin = std::min(std::min(ninput_items[0],ninput_items[1]),noutput_items);
      int count =0;
      d_tags.clear();
      get_tags_in_window(d_tags,0,0,nin,d_voe_tag);
      gr_complex corr_val, eng, corr_norm;
      while(count<nin){
        switch(d_state){
          case ED_LISTEN:
            while(count<nin){
              tags_handler(count);
              if(ed[count++]>d_ed_thres){
                d_ed_cnt++;
                if(d_ed_cnt==d_valid){
                  // triggered
                  enter_sfd();
                  break;
                }
              }else{
                d_ed_cnt=0;
              } 
            }
          break;
          case ED_SFD:
            if( (noutput_items-count)>=512 && (nin-count)>=(512+d_samples.size()) ){
              // enough samples for searching preamble
              for(int i=0;i<512;++i){
                tags_handler(count+i);
              }
              for(int i=0;i<512;++i){
                volk_32fc_x2_conjugate_dot_prod_32fc(&corr_val,in+count+i,d_samples.data(),d_samples.size());
                volk_32fc_x2_conjugate_dot_prod_32fc(&eng, in+count+i,in+count+i,d_samples.size());
                d_buf[i] = corr_val/(std::sqrt(eng*d_sample_eng)+gr_complex(1e-8,0));
              }
              uint16_t max_idx;
              volk_32fc_index_max_16u(&max_idx,d_buf,512);
              if(std::abs(d_buf[max_idx])>0.9){
                DEBUG<<"<SNS RX CTRL>found SFD in first 512 samples declare SNS signal"<<std::endl;
                enter_ed_sns();
              }else{
                DEBUG<<"<SNS RX CTRL>no sfd found in first 512, ProU signal detected"<<std::endl;
                enter_ed_pu();
              }
              count+=512;
            }else{
              memcpy(out,in,sizeof(gr_complex)*count);
              consume_each(count);
              return count;
            }
          break;
          case ED_DETECT_PU:
            while(count<nin){
              tags_handler(count);
              if(ed[count++]<d_ed_thres){
                d_ed_cnt++;
                if(d_ed_cnt>=d_valid){
                  if(d_voe_cnt){
                    enter_silent();
                  }else{
                    enter_listen();
                  }
                  break;
                }
              }else{
                d_ed_cnt=0;
              }
            }
          break;
          case ED_DETECT_SNS:
            while(count<nin){
              tags_handler(count);
              if(ed[count++]<d_ed_thres){
                d_ed_cnt++;
                if(d_ed_cnt>=d_valid){
                  if(d_voe_cnt){
                    enter_silent();
                  }else{
                    enter_listen();
                  }
                  break;
                }
              }else{
                d_ed_cnt=0;
              }
            }
          break;
          case ED_SILENT:
            while(count<nin){
              tags_handler(count);
              if(d_silent_trig){
                if(ed[count++]<d_ed_thres){
                  d_ed_cnt++;
                  if(d_ed_cnt>=d_valid){
                    d_voe_cnt = (d_voe_cnt==0)? 0 : d_voe_cnt-1;
                    if(d_voe_cnt==0){
                      notify_clear();
                      enter_listen();
                      break;
                    }else{
                      DEBUG<<"<SNS RX CTRL DEBUG>Triggered set to false"<<std::endl;
                      d_silent_trig = false;
                      d_ed_cnt=0;
                    }
                  }
                }else{
                  d_ed_cnt=0;
                }
              }else{
                if(ed[count++]>d_ed_thres){
                  d_ed_cnt++;
                  if(d_ed_cnt>=d_valid){
                    DEBUG<<"<SNS RX CTRL DEBUG>Triggered set to true"<<std::endl;
                    d_silent_trig = true;
                    d_ed_cnt=0;
                  }
                }else{
                  d_ed_cnt=0;
                }
              }
            }
          break;
          default:
            throw std::runtime_error("undefined state");
          break;
        }
      }
      tags_handler(count);
      memcpy(out,in,sizeof(gr_complex)*count);
      consume_each (count);
      return count;
    }

  } /* namespace lsa */
} /* namespace gr */

