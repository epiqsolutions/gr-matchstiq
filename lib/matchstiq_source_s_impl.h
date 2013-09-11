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

#ifndef INCLUDED_MATCHSTIQ_SOURCE_C_IMPL_H
#define INCLUDED_MATCHSTIQ_SOURCE_C_IMPL_H

#include <matchstiq/matchstiq_source_s.h>
#include <matchstiq/matchstiq_defs.h>
#include "matchstiq.h"

namespace gr {
  namespace matchstiq {



    class matchstiq_source_s_impl;
    typedef boost::shared_ptr<matchstiq_source_s_impl> matchstiq_source_s_impl_sptr;

    class matchstiq_source_s_impl : public matchstiq_source_s
    {
    private:
	boost::scoped_ptr<matchstiq> rcv;

    public:
	matchstiq_source_s_impl(const std::string ip_address, uint32_t port);
	~matchstiq_source_s_impl();
	
	bool stop();
	bool start();

	int work(int noutput_items,
		 gr_vector_const_void_star &input_items,
		 gr_vector_void_star &output_items);

	uint64_t set_center_freq(uint64_t freq);
	uint64_t set_center_freq(float freq);
	uint64_t center_freq(void);

	uint32_t set_sample_rate(uint32_t sample_rate);
	uint32_t set_sample_rate(float sample_rate);
	uint32_t sample_rate(void);

	STATUS set_front_lna(STATUS enable);
	STATUS front_lna(void);

	float set_second_lna_gain(float gain);
	float second_lna_gain(void);

	float set_step_atten(float atten);
	float step_atten(void);

	uint8_t set_rxvga1_gain(uint8_t gain1);
	uint8_t rxvga1_gain(void);

	uint8_t set_rxvga2_gain(uint8_t gain2);
	uint8_t rxvga2_gain(void);

	uint16_t set_tcvcxo_warp_voltage(uint16_t warp_voltage);
	uint16_t tcvcxo_warp_voltage(void);

	LOWPASS set_low_pass_filter(LOWPASS filter);
	LOWPASS low_pass_filter(void);

	uint8_t set_decimation(uint8_t decimation);
	uint8_t decimation(void);

	PRESELECT set_preselect_filter(PRESELECT preselect);
	PRESELECT preselect_filter(void);

	ONE_PPS set_one_pps_source(ONE_PPS pps_src);
	ONE_PPS one_pps_source(void);

	std::string set_cic_coefficients(const std::string filename); 
	std::string cic_coefficients(void); 
    };

  } // namespace matchstiq
} // namespace gr

#endif /* INCLUDED_MATCHSTIQ_SOURCE_C_IMPL_H */

