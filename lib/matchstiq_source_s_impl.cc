/* -*- c++ -*- */
/* 
 * Copyright 2013 Epiq Solutions.
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

#include <gr_io_signature.h>
#include "matchstiq_source_s_impl.h"

#include <stdio.h>

namespace gr {
  namespace matchstiq {

#define     MATCHSTIQ_SAMPLES_PER_PACKET        (1024)
// *2 for I & Q
#define BUF_LEN      (MATCHSTIQ_SAMPLES_PER_PACKET*sizeof(short)*2)

    matchstiq_source_s::sptr
    matchstiq_source_s::make(const std::string ip_address, uint32_t port)
    {
	return gnuradio::get_initial_sptr 
	    (new matchstiq_source_s_impl(ip_address, port));
    }
      
    /*
     * The private constructor
     */
    matchstiq_source_s_impl::matchstiq_source_s_impl(const std::string ip_address, 
						     uint32_t port)
      : gr_sync_block("matchstiq_source_s",
		      gr_make_io_signature(0, 0, 0),
		      gr_make_io_signature(1, 1, sizeof (short)))
    {
	rcv.reset( new matchstiq(ip_address.c_str(), port) );
	set_output_multiple(MATCHSTIQ_SAMPLES_PER_PACKET*2);
    }

    /*
     * Our virtual destructor.
     */
    matchstiq_source_s_impl::~matchstiq_source_s_impl()
    {
	rcv->stop();
    }

    bool
    matchstiq_source_s_impl::stop(void)
    {
	rcv->stop();
	return true;
    }

    bool
    matchstiq_source_s_impl::start(void)
    {
	rcv->start();
	return true;
    }

    uint64_t
    matchstiq_source_s_impl::set_center_freq(uint64_t freq)
    {
	return (rcv->set_rx_freq(freq));
    }

    uint64_t
    matchstiq_source_s_impl::set_center_freq(float freq)
    {
	return (rcv->set_rx_freq(freq));
    }

    uint64_t
    matchstiq_source_s_impl::center_freq(void)
    {
	return (rcv->rx_freq());
    }

    uint32_t
    matchstiq_source_s_impl::set_sample_rate(uint32_t sample_rate)
    {
	return (rcv->set_rx_sample_rate(sample_rate));
    }

    uint32_t
    matchstiq_source_s_impl::set_sample_rate(float sample_rate)
    {
	return (rcv->set_rx_sample_rate(sample_rate));
    }

    uint32_t
    matchstiq_source_s_impl::sample_rate(void)
    {
	return (rcv->rx_sample_rate());
    }


    STATUS
    matchstiq_source_s_impl::set_front_lna(STATUS enable)
    {
	return (rcv->set_rx_front_lna_status(enable));
    }

    STATUS
    matchstiq_source_s_impl::front_lna(void)
    {
	return (rcv->rx_front_lna_status());
    }

    float
    matchstiq_source_s_impl::set_second_lna_gain(float gain)
    {
	return (rcv->set_rx_second_lna_gain(gain));
    }

    float
    matchstiq_source_s_impl::second_lna_gain(void)
    {
	return (rcv->rx_second_lna_gain());
    }

    float 
    matchstiq_source_s_impl::set_step_atten(float atten)
    {
	return (rcv->set_rx_step_atten(atten));
    }

    float 
    matchstiq_source_s_impl::step_atten(void)
    {
	return (rcv->rx_step_atten());
    }

    uint8_t
    matchstiq_source_s_impl::set_rxvga1_gain(uint8_t gain1)
    {
	return (rcv->set_rx_rxvga1_gain(gain1));
    }

    uint8_t
    matchstiq_source_s_impl::rxvga1_gain(void)
    {
	return (rcv->rx_rxvga1_gain());
    }

    uint8_t
    matchstiq_source_s_impl::set_rxvga2_gain(uint8_t gain2)
    {
	return (rcv->set_rx_rxvga2_gain(gain2));
    }

    uint8_t
    matchstiq_source_s_impl::rxvga2_gain(void)
    {
	return (rcv->rx_rxvga2_gain());
    }

    uint16_t
    matchstiq_source_s_impl::set_tcvcxo_warp_voltage(uint16_t warp_voltage)
    {
	return (rcv->set_warp_voltage(warp_voltage));
    }

    uint16_t
    matchstiq_source_s_impl::tcvcxo_warp_voltage(void)
    {
	return (rcv->warp_voltage());
    }

    LOWPASS
    matchstiq_source_s_impl::set_low_pass_filter(LOWPASS filter)
    {
	return (rcv->set_rx_low_pass_filter(filter));
    }

    LOWPASS
    matchstiq_source_s_impl::low_pass_filter(void)
    {
	return (rcv->rx_low_pass_filter());
    }

    uint8_t
    matchstiq_source_s_impl::set_decimation(uint8_t decimation)
    {
	return (rcv->set_decimation(decimation));
    }

    uint8_t
    matchstiq_source_s_impl::decimation(void)
    {
	return (rcv->decimation());
    }

    PRESELECT
    matchstiq_source_s_impl::set_preselect_filter(PRESELECT preselect)
    {
	return (rcv->set_rx_preselect_filter(preselect));
    }

    PRESELECT
    matchstiq_source_s_impl::preselect_filter(void)
    {
	return (rcv->rx_preselect_filter());
    }

    ONE_PPS
    matchstiq_source_s_impl::set_one_pps_source(ONE_PPS source)
    {
	return (rcv->set_one_pps_source(source));
    }

    ONE_PPS
    matchstiq_source_s_impl::one_pps_source(void)
    {
	return (rcv->one_pps_source());
    }

    std::string
    matchstiq_source_s_impl::set_cic_coefficients(const std::string filename)
    {
	return (rcv->set_cic_coefficients(filename));
    }

    std::string
    matchstiq_source_s_impl::cic_coefficients(void)
    {
	return (rcv->cic_coefficients());
    }

    int
    matchstiq_source_s_impl::work(int noutput_items,
				  gr_vector_const_void_star &input_items,
				  gr_vector_void_star &output_items)
    {
	signed short* out1 =(signed short*) output_items[0];
	char buffer[BUF_LEN];
	int num_bytes_rcvd = 0;
	int out_idx = 0;
	int recv_result = 0;
	
	for(int i=0; i<floor(noutput_items*1.0/(2*MATCHSTIQ_SAMPLES_PER_PACKET));i++)
	{
	    
	    recv_result = rcv->read( &buffer[0], BUF_LEN );
	    if( recv_result < 0 )
	    {
		// no data available, send back 0
		goto end_work;
	    }
	    num_bytes_rcvd += recv_result;
	    out_idx = num_bytes_rcvd/2;
	    memcpy(&out1[out_idx], buffer, MATCHSTIQ_SAMPLES_PER_PACKET * sizeof(short)*2);
	}	

    end_work:
	return (num_bytes_rcvd/2);
    }
      
  } /* namespace matchstiq */
} /* namespace gr */

