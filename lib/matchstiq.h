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

#ifndef MATCHSTIQ_H
#define MATCHSTIQ_H

#include <matchstiq/matchstiq_defs.h>
#include <boost/scoped_ptr.hpp>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <stdint.h>
#include <string>
#include <netdb.h>
#include <string>
#include <map>

#include "srfs_interface.h"

namespace gr {
    namespace matchstiq {
	/*! String representation of STATUS enum */
	const std::string status_str[STATUS_INVALID] = {
	    "enabled",
	    "disabled"
	};
	
	/*! String representation of PRESELECT enum */
	const std::string preselect_str[PRESELECT_INVALID] = {
	    "300-700",
	    "625-1080",
	    "1000-2100",
	    "1700-2500",
	    "2200-3800",
	    "bypass",
	    "auto"
	};

	/*! String representation of LOWPASS enum */
	const std::string lowpass_str[LOWPASS_INVALID] = {
	    "14000000",
	    "10000000",
	    "7000000",
	    "6000000",
	    "5000000",
	    "4375000",
	    "3500000",
	    "3000000",
	    "2750000",
	    "2500000",
	    "1920000",
	    "1500000",
	    "1375000",
	    "1250000",
	    "875000",
	    "750000",
	};

	/*!< String representation of ONE_PPS enum */
	const std::string onepps_str[ONE_PPS_INVALID] = {
	    "internal",
	    "external"
	};

	/*! Implements interface to matchstiq */
	class matchstiq {
	    
	public:
	    
	    /*!< state of matchstiq connection */
	    enum state {
		STATE_STOPPED, STATE_STARTED,
	    };
	    
	    matchstiq(const char* ip_addr, unsigned short port);
	    ~matchstiq();

	    uint64_t set_rx_freq( uint64_t freq );	    
	    uint64_t rx_freq( void );	    

	    uint32_t set_rx_sample_rate( uint32_t sample_rate );
	    uint32_t rx_sample_rate( void );

	    STATUS set_rx_front_lna_status( STATUS lna_status );
	    STATUS rx_front_lna_status( void );

	    float set_rx_second_lna_gain( float lna_gain );
	    float rx_second_lna_gain( void );

	    float set_rx_step_atten( float atten );
	    float rx_step_atten( void );

	    uint8_t set_rx_rxvga1_gain( uint8_t gain1 );
	    uint8_t rx_rxvga1_gain( void );

	    uint8_t set_rx_rxvga2_gain( uint8_t gain2 );
	    uint8_t rx_rxvga2_gain( void );

	    uint16_t set_warp_voltage( uint16_t warp_voltage );
	    uint16_t warp_voltage( void );

	    LOWPASS set_rx_low_pass_filter( LOWPASS lp_filter );
	    LOWPASS rx_low_pass_filter( void );

	    uint8_t set_decimation( uint8_t );
	    uint8_t decimation( void );

	    PRESELECT set_rx_preselect_filter( PRESELECT presel_filter );
	    PRESELECT rx_preselect_filter( void );

	    ONE_PPS set_one_pps_source( ONE_PPS pps_src );
	    ONE_PPS one_pps_source( void );

	    std::string set_cic_coefficients( const std::string filename );
	    std::string cic_coefficients( void );

	    void start();
	    void stop();

	    // read the data from matchstiq
	    int read(char*, int);

	private:
	    // matchstiq parameters
	    uint64_t d_rx_freq;
	    uint32_t d_rx_sample_rate;
	    STATUS d_rx_front_lna_status;
	    float d_rx_step_atten;
	    float d_rx_second_lna_gain;
	    uint8_t d_rxvga1_gain;
	    uint8_t d_rxvga2_gain;
	    uint16_t d_warp_voltage;
	    LOWPASS d_rx_low_pass_filter;
	    uint8_t d_decimation;
	    PRESELECT d_rx_preselect_filter;
	    ONE_PPS d_one_pps;
	    std::string d_cic_coefficients;

	    STATUS d_srfs_src_status;

	    // sockets
	    int d_sock;
	    int d_iq_sock;
	    int d_iq_port;
	    int d_src_port;
	    state d_state;
	    hostent *d_server;
	    sockaddr_in d_server_addr;
	    sockaddr_in d_iq_server_addr;
	    socklen_t d_addr_len;

	    bool first; // indicates if first block of data

            uint32_t d_usleep_period;

	    // string to srfs_param_t map
	    typedef std::map<const std::string, srfs::srfs_param_t> param_map;
	    // pair string to srfs_param_t
	    typedef std::pair<const std::string, srfs::srfs_param_t> param_pair;

	    // map of valid matchstiq parameters and associated strings
	    param_map matchstiq_params;

	    // establishes connection to SRFS
	    void open_srfs();
	    // closes connection to SRFS
	    void close_srfs();
	    // initializes matchstiq_params map
	    void init_srfs_params(void);
	    // adds a parameter to the matchstiq_params map
	    void add_srfs_param( const std::string token,
				 srfs::SRFS_DATATYPES data_type,
				 void *p_value,
				 int64_t min_value,
				 int64_t max_value,
				 float resolution,
				 const std::string *p_strings );

	    // configures parameter specified by token to the value provided
	    void set_param( const std::string token, void *pValue );

	    // configures all matchstiq parameters
	    void config_src();	
	    // sends a message to matchstiq
	    void send_msg( char * );
	    // receive a response from matchstiq
	    int receive_msg( char *, int );
	};
    } // namespace matchstiq
} // namespace gr

#endif
