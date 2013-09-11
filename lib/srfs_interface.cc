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

#include "srfs_interface.h"

#include <math.h>
#include <string.h>
#include <stdexcept>

namespace gr {
    namespace srfs {

      bool
      BINARY_to_host( BINARY * const binary )
      {
	  bool return_value = false;
	  if ( NULL != binary ) {
	      binary->indicator = ntohs( binary->indicator );
	      binary->type = ntohs( binary->type );
	      binary->length = ntohl( binary->length );
	      return_value = true;
	  }
	    return ( return_value );
      } /* end BINARY_to_host */

      uint32_t
      BINARY_IQ_length_iq_in_pairs( const BINARY_IQ * const binary_iq )
      {
	  uint32_t length_iq = 0;
	  if ( NULL != binary_iq ) {
	      length_iq = ( binary_iq->binary.length -
			    ( sizeof(*binary_iq) -
			      sizeof(binary_iq->binary) ) ) /
		  ( 2 * sizeof(*binary_iq->iq) ); /* FIXME depends on type */
	  }
	  return ( length_iq );
      } /* end BINARY_IQ_length_iq */

      bool
      BINARY_IQ_to_host ( BINARY_IQ * const binary_iq )
      {
	  bool return_value = false;
	  uint32_t i;
	  int16_t * tmp;
	  uint32_t length;
	  
	  if ( NULL != binary_iq ) {
	      return_value = BINARY_to_host( &binary_iq->binary );
	      binary_iq->config_id = be32toh( binary_iq->config_id );
	      binary_iq->format = be16toh( binary_iq->format );
	      binary_iq->timestamp = be64toh( binary_iq->timestamp );
	      tmp = binary_iq->iq;
	      length = BINARY_IQ_length_iq_in_pairs(binary_iq);
	      
	      /* NOTE: it is the responsibility of the caller to convert the actual data */
	  }
	  return ( return_value );
      } /* end BINARY_IQ_to_network */

      void
      update_param( srfs_param_t *p_param, const char *p_value )
      {
	  if( p_value != NULL ) {
	      switch( p_param->data_type ) {
		  case SRFS_UINT64:
		      sscanf( p_value, "%lu", 
			      (uint64_t*)(p_param->p_value) );
		      break;
		
		  case SRFS_UINT32:
		      sscanf( p_value, "%u", 
			      (uint32_t*)(p_param->p_value) );
		      break;
		
		  case SRFS_UINT16:
		      sscanf( p_value, "%hu", 
			      (uint16_t*)(p_param->p_value) );
		      break;
		
		  case SRFS_UINT8:
		      sscanf( p_value, "%hhu", 
			      (uint8_t*)(p_param->p_value) );
		      break;
		
		  case SRFS_FLOAT:
		      sscanf( p_value, "%f", 
			      (float*)(p_param->p_value) );
		      break;

		  case SRFS_ENUM:
		      *((uint32_t*)(p_param->p_value)) = 
			  convert_str_to_enum( p_value,
					       p_param->p_strings, 
					       p_param->max_value );
		      break;
		
		  default:
		      throw std::invalid_argument("invalid data_type on update");
		      break;
	      }
	  }
      }

      bool
      set_param( srfs_param_t *p_param, void *p_value )
      {
	  bool b_valid = false;
	  
	  switch( p_param->data_type ) {
	      case SRFS_UINT64:
		  b_valid = set_uint64( p_param, 
					*((uint64_t*)(p_value)) );
		  break;

	      case SRFS_UINT32:
		  b_valid = set_uint32( p_param, 
					*((uint32_t*)(p_value)) );
		  break;

	      case SRFS_UINT16:
		  b_valid = set_uint16( p_param, 
					*((uint16_t*)(p_value)) );
		  break;

	      case SRFS_UINT8:
		  b_valid = set_uint8( p_param, 
				       *((uint8_t*)(p_value)) );
		  break;

	      case SRFS_FLOAT:
		  b_valid = set_float( p_param, 
				       *((float*)(p_value)) );
		  break;

	      case SRFS_ENUM:
		  // just treat as uint32...should be ok
		  b_valid = set_uint32( p_param, 
					*((uint32_t*)(p_value)) );
		  break;
		  
	      default:
		  throw std::invalid_argument("invalid data_type on set");
		  break;
	  }
	  return b_valid;
      }

      bool 
      set_uint64( srfs_param_t *p_param, uint64_t value )
      {
	  bool b_valid = false;

	  if( value >= p_param->min_value &&
	      value <= p_param->max_value &&
	      (value % (uint64_t)(p_param->resolution)) == 0 ) {

	      *((uint64_t*)(p_param->p_value)) = value; 
	      b_valid = true;
	  }
	  else
	  {
	      char e[200];
	      snprintf( e, 200,
			"out of range, valid range %lu-%lu in increments of %lu",
			(uint64_t)(p_param->min_value), 
			(uint64_t)(p_param->max_value), 
			(uint64_t)(p_param->resolution) );
	      throw std::out_of_range(e);
	  }
	  return b_valid;
      }

      bool 
      set_uint32( srfs_param_t *p_param, uint32_t value )
      {
	  bool b_valid = false;
	  if( value >= (uint32_t)(p_param->min_value) &&
	      value <= (uint32_t)(p_param->max_value) &&
	      (value % (uint32_t)(p_param->resolution)) == 0 ) {

	      *((uint32_t*)(p_param->p_value)) = value;
	      b_valid = true;
	  }
	  else {
	      char e[200];
	      snprintf( e, 200,
			"out of range, valid range %u-%u in increments of %u",
			(uint32_t)(p_param->min_value), 
			(uint32_t)(p_param->max_value), 
			(uint32_t)(p_param->resolution) );
	      throw std::out_of_range(e);
	  }
	  return b_valid;
      }

      bool 
      set_uint16( srfs_param_t *p_param, uint16_t value )
      {
	  bool b_valid = false;
	  if( value >= (uint16_t)(p_param->min_value) &&
	      value <= (uint16_t)(p_param->max_value) &&
	      (value % (uint16_t)(p_param->resolution)) == 0 ) {

	      *((uint16_t*)(p_param->p_value)) = value;
	      b_valid = true;
	  }
	  else {
	      char e[200];
	      snprintf( e, 200,
			"out of range, valid range %hu-%hu in increments of %u",
			(uint16_t)(p_param->min_value), 
			(uint16_t)(p_param->max_value), 
			(uint16_t)(p_param->resolution) );
	      throw std::out_of_range(e);
	  }
	  return b_valid;
      }

      bool 
      set_uint8( srfs_param_t *p_param, uint8_t value )
      {
	  bool b_valid = false;
	  if( value >= (uint8_t)(p_param->min_value) &&
	      value <= (uint8_t)(p_param->max_value) &&
	      (value % (uint8_t)(p_param->resolution)) == 0 ) {

	      *((uint8_t*)(p_param->p_value)) = value;
	      b_valid = true;
	  }
	  else {
	      char e[200];
	      snprintf( e, 200,
			"out of range, valid range %hhu-%hhu in increments of %hhu",
			(uint8_t)(p_param->min_value), 
			(uint8_t)(p_param->max_value), 
			(uint8_t)(p_param->resolution) );
	      throw std::out_of_range(e);
	  }
	  return b_valid;
      }

      bool 
      set_float( srfs_param_t *p_param, float value )
      {
	  bool b_valid = false;
	  if( value >= (float)(p_param->min_value) &&
	      value <= (float)(p_param->max_value) &&
	      (fmodf(value,p_param->resolution)) == 0 ) {
	      
	      *((float*)(p_param->p_value)) = value; 
	      b_valid = true;
	  }
	  else {
	      char e[200];
	      snprintf( e, 200,
			"out of range, valid range %f-%f in increments of %f",
			(float)(p_param->min_value), 
			(float)(p_param->max_value), 
			(float)(p_param->resolution) );
	      throw std::out_of_range(e);
	  }
	  return b_valid;
      }

      uint32_t
      convert_str_to_enum( const char* pString, 
			   const std::string *pStrings, 
			   uint32_t invalid_index )
      {
	  uint32_t index = invalid_index;
	  
	  for( index=0; index<invalid_index; index++ ) {
	      if( strncmp(pString, 
			    pStrings[index].c_str(),
			  strlen(pStrings[index].c_str())) == 0 ) {
		  break;
	      }
	  }
	  if( invalid_index == index ) {
	      throw std::out_of_range("out_of_range, string not in enum");
	  }
	  return index;
      }
    } // namespace srfs
} // namespace gr


