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

#ifndef SRFS_INTERFACE_H
#define SRFS_INTERFACE_H

#include <stdio.h>
#include <string>
#include <unistd.h>
#include <sys/socket.h>
#include <stdint.h>
#include <netinet/in.h>

namespace gr {
    namespace srfs {

	typedef enum 
	{
	    SRFS_UINT64,
	    SRFS_UINT32,
	    SRFS_UINT16,
	    SRFS_UINT8,
	    SRFS_FLOAT,
	    SRFS_ENUM,
	} SRFS_DATATYPES;
	    
	typedef struct
	{
	    SRFS_DATATYPES data_type;
	    void *p_value; 
	    int64_t min_value;
	    int64_t max_value;
	    float resolution;
	    const std::string *p_strings;
	} srfs_param_t;

	typedef struct __attribute__((__packed__))
	{
	    uint16_t indicator; /* always 0 */
	    uint16_t type;
	    uint32_t length; /* length of message in octets */
	    uint8_t message[0];
	} BINARY;

	typedef struct __attribute__((__packed__))
	{
	    BINARY binary;
	    uint32_t config_id;
	    uint16_t format;
	    uint64_t timestamp;
	    int16_t iq[0]; /* FIXME type varies depending on format */
	} BINARY_IQ;
	
	bool BINARY_to_host( BINARY * const binary );
	uint32_t BINARY_IQ_length_iq_in_pairs( const BINARY_IQ * const binary_iq );
	bool BINARY_IQ_to_host( BINARY_IQ * const binary_iq );
	
	bool set_param( srfs_param_t *p_param, void* p_value );
	
	bool set_uint64( srfs_param_t *p_param, uint64_t value );
	bool set_uint32( srfs_param_t *p_param, uint32_t value );
	bool set_uint16( srfs_param_t *p_param, uint16_t value );
	bool set_uint8( srfs_param_t *p_param, uint8_t value );
	bool set_float( srfs_param_t *p_param, float value );

	void update_param( srfs_param_t *p_param, const char* p_value );
	
	uint32_t convert_str_to_enum( const char* pString, 
				      const std::string *pStrings, 
				      uint32_t invalid_index );
    } // namespace SRFS
} // namespace gr

#endif
