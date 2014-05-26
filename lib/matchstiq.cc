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
#include <config.h>
#endif
#include "matchstiq.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#ifdef HAVE_ARPA_INET_H
#include <arpa/inet.h>
#endif
#ifdef HAVE_NETINET_IN_H
#include <netinet/in.h>
#endif
#ifdef HAVE_SYS_SOCKET_H
#include <sys/socket.h>
#endif

#include <netdb.h>
#include <string>
#include <math.h>
#include <stdexcept>
#include "srfs_interface.h"

namespace gr {
    namespace matchstiq {

#define DEBUG_MATCHSTIQ 0
#define DEBUG(A)    if( DEBUG_MATCHSTIQ ) printf("=debug=> %s\n", A)

#define IQ_HEADER_SIZE (sizeof(srfs::BINARY_IQ))
#define BINARY_HEADER_SIZE (sizeof(srfs::BINARY))

#define BUF_SIZE (65536)

#define NUM_RECV_ATTEMPTS (3)

#define FREQUENCY_MIN  300000000ULL
#define FREQUENCY_MAX 3740000000ULL
#define FREQUENCY_RESOLUTION 1

#define SAMPLE_RATE_MIN   160000
#define SAMPLE_RATE_MAX 40000000
#define SAMPLE_RATE_RESOLUTION 1

#define STEP_ATTEN_MIN 0
#define STEP_ATTEN_MAX 31.5
#define STEP_ATTEN_RESOLUTION 0.5

#define SECOND_LNA_GAIN_MIN -6
#define SECOND_LNA_GAIN_MAX 6
#define SECOND_LNA_GAIN_RESOLUTION 6

#define RXVGA1_GAIN_MIN 2
#define RXVGA1_GAIN_MAX 120
#define RXVGA1_GAIN_RESOLUTION 1

#define RXVGA2_GAIN_MIN 0
#define RXVGA2_GAIN_MAX 30
#define RXVGA2_GAIN_RESOLUTION 3

#define WARP_VOLTAGE_MIN 683
#define WARP_VOLTAGE_MAX 3413
#define WARP_VOLTAGE_RESOLUTION 1

#define DECIMATION_MIN 1
#define DECIMATION_MAX 128
#define DECIMATION_RESOLUTION 1

matchstiq::matchstiq(const char* addr, unsigned short port)
{
    struct timeval tv;
    tv.tv_sec = 10;
    tv.tv_usec = 0;

    d_server = gethostbyname( addr );

    if ((d_sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
	throw std::runtime_error("unable to create socket");
    }

    if( setsockopt( d_sock, SOL_SOCKET, SO_RCVTIMEO, 
		    (char*)(&tv), sizeof(struct timeval)) != 0 )
    {
	throw std::runtime_error("unable to set socket options");
    }

    memset( &d_server_addr, 0, sizeof(d_server_addr) );
    d_server_addr.sin_family = AF_INET;
    d_server_addr.sin_port = htons(port);
    d_server_addr.sin_addr.s_addr = INADDR_ANY;
    memcpy( &d_server_addr.sin_addr.s_addr,
            d_server->h_addr,
            d_server->h_length );

    if ( 0 != connect( d_sock,
                       (struct sockaddr *)&d_server_addr,
                       sizeof( d_server_addr ) ) ) {
	throw std::runtime_error("unable to connect to matchstiq");
    }

    d_addr_len = sizeof(struct sockaddr);
	
    // set default values
    d_rx_freq =             FREQUENCY_MIN;
    d_rx_sample_rate =      SAMPLE_RATE_MIN;
    d_rx_front_lna_status = STATUS_ENABLED;
    d_rx_second_lna_gain =  SECOND_LNA_GAIN_MAX;
    d_rx_step_atten =       STEP_ATTEN_MIN;
    d_rxvga1_gain =         100;
    d_rxvga2_gain =         21;
    d_warp_voltage =        2048;
    d_rx_low_pass_filter =  LOWPASS_750kHz;
    d_decimation =          DECIMATION_MIN;
    d_rx_preselect_filter = PRESELECT_AUTO;
    d_one_pps             = ONE_PPS_INTERNAL;
    d_cic_coefficients =    "default";
    d_srfs_src_status =   STATUS_DISABLED;

    d_state = STATE_STOPPED;

    init_srfs_params();

    // Get SRFS spun up
    open_srfs();
}

matchstiq::~matchstiq()
{
    matchstiq::stop();
    close_srfs();
    close(d_sock);
}

void
matchstiq::init_srfs_params(void)
{
    // frequency
    add_srfs_param( "frequency",
		    srfs::SRFS_UINT64,
		    (void*)(&d_rx_freq),
		    FREQUENCY_MIN,
		    FREQUENCY_MAX,
		    FREQUENCY_RESOLUTION,
		    NULL );

    // sample rate
    add_srfs_param( "sample_rate",
		    srfs::SRFS_UINT32,
		    (void*)(&d_rx_sample_rate),
		    SAMPLE_RATE_MIN,
		    SAMPLE_RATE_MAX,
		    SAMPLE_RATE_RESOLUTION,
		    NULL );

    // front lna
    add_srfs_param( "front_lna",
		    srfs::SRFS_ENUM,
		    (void*)(&d_rx_front_lna_status),
		    0,
		    STATUS_INVALID,
		    1, 
		    status_str );
		    
    // step atten
    add_srfs_param( "step_atten",
		    srfs::SRFS_FLOAT,
		    (void*)(&d_rx_step_atten),
		    STEP_ATTEN_MIN,
		    STEP_ATTEN_MAX,
		    STEP_ATTEN_RESOLUTION,
		    NULL );

    // second lna gain
    add_srfs_param( "second_lna_gain",
		    srfs::SRFS_FLOAT,
		    (void*)(&d_rx_second_lna_gain),
		    SECOND_LNA_GAIN_MIN,
		    SECOND_LNA_GAIN_MAX,
		    SECOND_LNA_GAIN_RESOLUTION,
		    NULL );

    // rxvga1 gain
    add_srfs_param( "rxvga1_gain",
		    srfs::SRFS_UINT8,
		    (void*)(&d_rxvga1_gain),
		    RXVGA1_GAIN_MIN,
		    RXVGA1_GAIN_MAX,
		    RXVGA1_GAIN_RESOLUTION,
		    NULL );

    // rxvga2 gain
    add_srfs_param( "rxvga2_gain",
		    srfs::SRFS_UINT8,
		    (void*)(&d_rxvga2_gain),
		    RXVGA2_GAIN_MIN,
		    RXVGA2_GAIN_MAX,
		    RXVGA2_GAIN_RESOLUTION,
		    NULL );

    // warp voltage
    add_srfs_param( "warp_voltage",
		    srfs::SRFS_UINT16,
		    (void*)(&d_warp_voltage),
		    WARP_VOLTAGE_MIN,
		    WARP_VOLTAGE_MAX,
		    WARP_VOLTAGE_RESOLUTION,
		    NULL );

    // low pass filter
    add_srfs_param( "rx_lpf",
		    srfs::SRFS_ENUM,
		    (void*)(&d_rx_low_pass_filter),
		    0,
		    LOWPASS_INVALID,
		    1,
		    lowpass_str );

    // decimation
    add_srfs_param( "decimation",
		    srfs::SRFS_UINT8,
		    (void*)(&d_decimation),
		    DECIMATION_MIN,
		    DECIMATION_MAX,
		    DECIMATION_RESOLUTION,
		    NULL );

    // preselect filter
    add_srfs_param( "preselect",
		    srfs::SRFS_ENUM,
		    (void*)(&d_rx_preselect_filter),
		    0,
		    PRESELECT_INVALID,
		    1,
		    preselect_str );

    // one pps
    add_srfs_param( "1pps_source",
		    srfs::SRFS_ENUM,
		    (void*)(&d_one_pps),
		    0, 
		    ONE_PPS_INVALID,
		    1,
		    onepps_str );

    // Note: cic coeff is a string so we don't do any validation on it
}

void 
matchstiq::add_srfs_param( const std::string token,
			   srfs::SRFS_DATATYPES data_type,
			   void *p_value,
			   int64_t min_value,
			   int64_t max_value,
			   float resolution,
			   const std::string *p_strings )
{
    srfs::srfs_param_t param;

    param.data_type = data_type;
    param.p_value = p_value;
    param.min_value = min_value;
    param.max_value = max_value;
    param.resolution = resolution;
    param.p_strings = p_strings;

    matchstiq_params[token] = param;
}

void
matchstiq::set_param( const std::string token, void *pValue )
{
    param_map::iterator iter;

    if( d_state == STATE_STARTED ) {
	iter = matchstiq_params.find(token);
	if( iter != matchstiq_params.end() ) {
	    if( srfs::set_param(&(iter->second), pValue) ) {
		config_src();
	    }
	}
	else {
	    throw std::invalid_argument("unknown parameter specified");
	}
    }
    else {
	throw std::runtime_error("no matchstiq connection when trying to set param");
    }
}

uint64_t
matchstiq::set_rx_freq(uint64_t rx_freq)
{
    set_param( "frequency", &rx_freq );
    return d_rx_freq;
}

uint64_t
matchstiq::rx_freq(void)
{
    return d_rx_freq;
}

uint32_t
matchstiq::set_rx_sample_rate(uint32_t rx_sample_rate)
{
    set_param("sample_rate", &rx_sample_rate);
    return d_rx_sample_rate;
}

uint32_t
matchstiq::rx_sample_rate(void)
{
    return d_rx_sample_rate;
}

STATUS
matchstiq::set_rx_front_lna_status( STATUS rx_lna_status )
{
    set_param("front_lna", &rx_lna_status);
    return d_rx_front_lna_status;
}

STATUS
matchstiq::rx_front_lna_status( void )
{
    return d_rx_front_lna_status;
}

float
matchstiq::set_rx_step_atten( float rx_step_atten )
{
    set_param("step_atten", &rx_step_atten);
    return d_rx_step_atten;
}

float
matchstiq::rx_step_atten( void )
{
    return d_rx_step_atten;
}

float
matchstiq::set_rx_second_lna_gain( float rx_second_lna_gain )
{
    set_param("second_lna_gain", &rx_second_lna_gain);
    return d_rx_second_lna_gain;
}

float
matchstiq::rx_second_lna_gain( void )
{
    return d_rx_second_lna_gain;
}

uint8_t
matchstiq::set_rx_rxvga1_gain( uint8_t rx_gain )
{
    set_param("rxvga1_gain", &rx_gain);
    return d_rxvga1_gain;
}

uint8_t
matchstiq::rx_rxvga1_gain( void )
{
    return d_rxvga1_gain;
}

uint8_t
matchstiq::set_rx_rxvga2_gain( uint8_t rx_gain )
{
    set_param("rxvga2_gain", &rx_gain);
    return d_rxvga2_gain;
}

uint8_t
matchstiq::rx_rxvga2_gain( void )
{
    return d_rxvga2_gain;
}

uint16_t
matchstiq::set_warp_voltage( uint16_t warp_voltage )
{
    set_param("warp_voltage", &warp_voltage);
    return d_warp_voltage;
}

uint16_t
matchstiq::warp_voltage( void )
{
    return d_warp_voltage;
}

LOWPASS
matchstiq::set_rx_low_pass_filter( LOWPASS low_pass_filter )
{
    set_param("rx_lpf", &low_pass_filter );
    return d_rx_low_pass_filter;
}

LOWPASS
matchstiq::rx_low_pass_filter( void )
{
    return d_rx_low_pass_filter;
}

uint8_t
matchstiq::set_decimation( uint8_t decimation )
{
    set_param("decimation", &decimation);
    return d_decimation;
}

uint8_t
matchstiq::decimation( void )
{
    return d_decimation;
}

PRESELECT
matchstiq::set_rx_preselect_filter( PRESELECT rx_preselect_filter )
{
    set_param("preselect", &rx_preselect_filter);
    return d_rx_preselect_filter;
}

PRESELECT
matchstiq::rx_preselect_filter( void )
{
    return d_rx_preselect_filter;
}

ONE_PPS
matchstiq::set_one_pps_source( ONE_PPS one_pps )
{
    set_param("1pps_source", &one_pps);
    return d_one_pps;
}

ONE_PPS
matchstiq::one_pps_source( void )
{
    return d_one_pps;
}

std::string
matchstiq::set_cic_coefficients( const std::string cic_coeff )
{
    if( !cic_coeff.empty() ) {
	d_cic_coefficients = cic_coeff;
    }

    return d_cic_coefficients;
}

std::string
matchstiq::cic_coefficients( void )
{
    return d_cic_coefficients;
}

void 
matchstiq::open_srfs()
{
    char cmd[1024];
    char rcv[1024];

    std::string str1;
    std::string str2;
    std::string str3;

    // reset to the default state
    snprintf(cmd, 1024, "reset!\n");
    send_msg( cmd );
    receive_msg( rcv, 1024 ); // clear out rcv buf

    // subscribe to MATCHSTIQ-RX
    snprintf(cmd, 1024, "subscribe! block MATCHSTIQ-RX\n");
    send_msg( cmd );
    receive_msg( rcv, 1024 );
    
    // parse out source port
    str1 = std::string( rcv );
    str2 = str1.substr( str1.find(":") + 1 ); // Move one past :
    str3 = str2.substr( 0, str2.find(" ") ); // src_port is between : and space
    d_src_port = atoi( str3.c_str() );

    // subscribe to IQ
    snprintf(cmd, 1024, "subscribe! block IQ\n"); 
    send_msg( cmd );
    receive_msg( rcv, 1024 );
    
    // parse out IQ port
    str1 = std::string( rcv );
    str2 = str1.substr( str1.find(":") + 1 ); // Move one past :
    str3 = str2.substr( 0, str2.find(" ") ); // iq_port is between : and space
    d_iq_port = atoi( str3.c_str() );

    // configure IQ to MATCHSTIQ-RX
    snprintf(cmd, 1024, "config! block IQ:%d input MATCHSTIQ-RX:%d\n", 
	     d_iq_port, d_src_port);
    send_msg( cmd );
    receive_msg( rcv, 1024 );
    
    // configure to continuous, set timestamp to 0
    snprintf(cmd, 1024, 
	     "config! block IQ:%d duty_cycle continuous block_timestamp 0\n", 
	     d_iq_port);
    send_msg( cmd );
    receive_msg( rcv, 1024 );

    d_state = STATE_STARTED;
    d_srfs_src_status = STATUS_ENABLED;
}

void 
matchstiq::start()
{
    char cmd[1024];
    char rcv[1024];

    if( DEBUG_MATCHSTIQ ) { 
	printf("starting\n");
    }

    // make sure it's configured
    config_src();

    //###################################
    //#     Now Open new IQ Port        #
    //###################################
    d_iq_sock = socket( AF_INET, SOCK_STREAM, 0 );

    memset( &d_iq_server_addr, 0, sizeof(d_iq_server_addr) );
    d_iq_server_addr.sin_family = AF_INET;
    d_iq_server_addr.sin_port = htons( d_iq_port );
    memcpy( &d_iq_server_addr.sin_addr.s_addr,
        d_server->h_addr,
        d_server->h_length );

    if ( 0 != connect( d_iq_sock,
                       (struct sockaddr *)&d_iq_server_addr,
                       sizeof(d_iq_server_addr) ) ) {
	throw std::runtime_error("unable to connect to IQ socket");
    }

    // enable the IQ data
    snprintf( cmd, 1024,
	     "config! block IQ:%d status enabled starting_timestamp 0x0000000000000000\n",
	      d_iq_port );
    send_msg(cmd);
    receive_msg(rcv, 1024);

    first = true;
    
    if( DEBUG_MATCHSTIQ ) {
	printf("Successfully opened both ports\n");
    }
}

void 
matchstiq::close_srfs()
{
    char cmd[1024];
    char rcv[1024];

    if ( d_state == STATE_STARTED ) {

	// unsubscribe from the RX block
        snprintf(cmd, 1024, "unsubscribe! block MATCHSTIQ-RX:%d\n", d_src_port);
        send_msg( cmd );
        receive_msg( rcv, 1024 ); // clear out rcv buf
	// unsubscribe from IQ block
        snprintf(cmd, 1024, "unsubscribe! block IQ:%d\n", d_iq_port);
        send_msg( cmd );
        receive_msg( rcv, 1024 ); // clear out rcv buf
    }
}

void 
matchstiq::stop()
{
    char cmd[1024];
    char rcv[1024];
    char data[BUF_SIZE];
    ssize_t num_bytes = BUF_SIZE;
    uint8_t count = 0;

    if( DEBUG_MATCHSTIQ ) {
	printf("stopping\n");
    }

    if( d_state == STATE_STARTED ) {
	snprintf( cmd, 1024,
		 "config! block IQ:%d status disabled\n",
		 d_iq_port );
	send_msg( cmd );
	receive_msg( rcv, 1024 );

	// make sure that the IQ socket is flushed out
	while( count < 2 ) {
	    num_bytes = recv( d_iq_sock,
			      data,
			      BUF_SIZE,
			      MSG_DONTWAIT );
	    if( num_bytes != -1 ) {
		count = 0;
	    }
	    else {
		count++;
		usleep(100*1000);
	    }
	}
	// shutdown the IQ socket
	shutdown(d_iq_port, 2);
    }
    d_state = STATE_STOPPED;
}

void 
matchstiq::send_msg( char * cmd )
{
    int flags = 0;
    if (DEBUG_MATCHSTIQ) {
	printf("sending: %s\n", cmd);
    }

    sendto( d_sock, cmd, strlen(cmd)+1, flags,
	    (const sockaddr*)&d_server_addr, d_addr_len);
    // Make sure to not send messages too fast
    usleep(200000);
}

int 
matchstiq::receive_msg( char * rcv, int size )
{
    int flags = 0;
    int num_bytes;

    // TODO: add timeout, fail and check for fail

    memset( rcv, 0, size );
    num_bytes = recvfrom( d_sock, rcv, size, flags,
	                    (struct sockaddr *)&d_server_addr, &d_addr_len);
    if( num_bytes <= 0 )
    {
	throw std::runtime_error("failed to receive response");
    }

    if (DEBUG_MATCHSTIQ) {
	printf("receive: %s\n", rcv);
    }

    return num_bytes;
}    
    
void 
matchstiq::config_src()
{
    char cmd[1024];
    char rcv[1024];

    char *pParam;
    char *pValue;
    
    int index=0;

    param_map::iterator iter;

    index = snprintf(cmd, 1024, "config! block MATCHSTIQ-RX:%d", d_src_port);
    // configure all of the parameters
    for( iter=matchstiq_params.begin(); 
	 iter != matchstiq_params.end(); 
	 iter++ ) {
	index += snprintf( &cmd[index], 1024-index, " %s ", (iter->first).c_str() );
	// format the parameters based on data_type
	switch( iter->second.data_type ) {
	    case srfs::SRFS_UINT64:
		index += snprintf( &cmd[index], 1024-index, "%lu", 
				   (*(uint64_t*)(iter->second.p_value)) );
		break;

	    case srfs::SRFS_UINT32:
		index += snprintf( &cmd[index], 1024-index, "%u", 
				   (*(uint32_t*)(iter->second.p_value)) );
		break;

	    case srfs::SRFS_UINT16:
		index += snprintf( &cmd[index], 1024-index, "%hu", 
				   (*(uint16_t*)(iter->second.p_value)) );
		break;

	    case srfs::SRFS_UINT8:
		index += snprintf( &cmd[index], 1024-index, "%hhu", 
				   (*(uint8_t*)(iter->second.p_value)) );
		break;

	    case srfs::SRFS_FLOAT:
		index += snprintf( &cmd[index], 1024-index, "%f", 
				   (*(float*)(iter->second.p_value)) );
		break;

	    case srfs::SRFS_ENUM:
		index += snprintf( &cmd[index], 1024-index, "%s", 
				   (iter->second.p_strings[(*(int*)(iter->second.p_value))]).c_str() );
		break;
	}
    }
    index += snprintf( &cmd[index], 1024-index, 
		       " cic_coefficients %s status %s\n", 
		       d_cic_coefficients.c_str(), 
		       status_str[d_srfs_src_status].c_str());
    send_msg( cmd );
    receive_msg( rcv, 1024 );

    // parse the response, update the returned parameters
    pParam = strtok( rcv, " "  );
    while( pParam != NULL )	{
	iter = matchstiq_params.find(pParam);
	if( iter != matchstiq_params.end() ) {
	    pParam = strtok( NULL, " " );
	    update_param( &(iter->second), (const char*)(pParam) );
	}
	else if( strncmp(pParam, "NOK", 4) == 0 ) {
	    throw std::invalid_argument("unexpected error with config_src");
	}
	// get the next parameter
	pParam = strtok( NULL, " " );
    } // end parsing
}

int 
matchstiq::read(char* buf, int size)
{
    static char data[BUF_SIZE];
    static uint32_t dataIndex=BUF_SIZE;  // initialize to max, forcing data retrieval
    static uint64_t old_timestamp;

    uint64_t timestamp_diff;
    ssize_t num_bytes;
    char header[IQ_HEADER_SIZE];
    bool bMoreData = false;
    int num_bytes_processed = 0;
    int recv_result;
    int count = 0;

    // see if this is the first block we're receiving
    if( first ) {
	// reset the index back to max to force more data retrieval
	dataIndex = BUF_SIZE;
	first = false;
    }

    int16_t *tmp = (int16_t*)(&data[dataIndex]);
    int16_t *tmpBuf = (int16_t*)(buf);
    uint32_t i=0;
    for( i=0; (i<(size/2)) && (dataIndex < BUF_SIZE); i++ ) {
	tmpBuf[i] = be16toh( tmp[i] );
	num_bytes_processed += 2;
	dataIndex += 2;
    } 

    // see if we've ran out of buffer space
    if( dataIndex >= BUF_SIZE ) {
	bMoreData = true;
    }
    
    // need to provide more data, try to retrieve it
    if( bMoreData ) {
	num_bytes = 0;

	// try to receive just the IQ header
	while( num_bytes < IQ_HEADER_SIZE ) {
	    // try to get data but don't block
	    recv_result = recv( d_iq_sock, 
				header + num_bytes,
				IQ_HEADER_SIZE - num_bytes,
				MSG_DONTWAIT );
	    if( recv_result < 0 ) {
		count++;
		if( count >= NUM_RECV_ATTEMPTS ) {
		    num_bytes_processed = -1;
		    first = true;
		    goto end_recv;
		}
		usleep(10*1000);
	    }
	    else
	    {
		count = 0;
		num_bytes += recv_result;
	    }
	}
	srfs::BINARY_IQ* binary_iq = (srfs::BINARY_IQ*)(header);
	// convert to host format
	srfs::BINARY_IQ_to_host( binary_iq );
	// get the length of the payload from the header
	uint32_t length = binary_iq->binary.length - IQ_HEADER_SIZE + BINARY_HEADER_SIZE;
	num_bytes = 0;
	dataIndex = 0;
	count = 0;
	// receive the payload
	while( num_bytes < length ) {
	    // read in the data but don't block
	    recv_result = recv( d_iq_sock,
				data + num_bytes,
				length - num_bytes,
				MSG_DONTWAIT );
	    if( recv_result < 0 ) {
		count++;
		if( count >= NUM_RECV_ATTEMPTS ) {
		    num_bytes_processed = -1;
		    first = true;
		    goto end_recv;
		}
		usleep(10*1000);
	    }
	    else {
		count = 0;
		num_bytes += recv_result;
	    }
	}
	// Check for dropped samples
	timestamp_diff = binary_iq->timestamp - old_timestamp;
	if ( timestamp_diff > (16384*d_decimation) )
	{
	    printf("Dropped %lu samples\n", timestamp_diff-16384 );
	}
	old_timestamp = binary_iq->timestamp;
    }

end_recv:
    return num_bytes_processed;
}

    } // namespace matchstiq
} // namespace gr
