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

#ifndef MATCHSTIQ_DEFS_H
#define MATCHSTIQ_DEFS_H

namespace gr {
    namespace matchstiq {

	/*!
	 * Status enable / disable 
	 */
	typedef enum 
	{ 
	    STATUS_ENABLED = 0,    
	    STATUS_DISABLED,       
	    STATUS_INVALID         
	} STATUS;
	
	/*!
	 * Preselect filter options
	 */
	typedef enum
	{ 
	    PRESELECT_300_700 = 0,  /*!< 300-700 MHz        */
	    PRESELECT_625_1080,     /*!< 625-1080 MHz       */
	    PRESELECT_1000_2100,    /*!< 1000-2100 MHz      */
	    PRESELECT_1700_2500,    /*!< 1700-2500 MHz      */
	    PRESELECT_2200_3800,    /*!< 2200-3800 MHz      */
	    PRESELECT_BYPASS,       /*!< filter bypass      */
	    PRESELECT_AUTO,         /*!< filter auto select */
	    PRESELECT_INVALID       /*!< invalid            */
	} PRESELECT;

	/*!
	 * Lowpass filter options
	 */
	typedef enum 
	{ 
	    LOWPASS_14MHz = 0,     
	    LOWPASS_10MHz,         
	    LOWPASS_7MHz,          
	    LOWPASS_6MHz,           
	    LOWPASS_5MHz,           
	    LOWPASS_4375kHz,
	    LOWPASS_3500kHz,
	    LOWPASS_3000kHz,
	    LOWPASS_2750kHz,
	    LOWPASS_2500kHz,
	    LOWPASS_1920kHz,
	    LOWPASS_1500kHz,
	    LOWPASS_1375kHz,
	    LOWPASS_1250kHz,
	    LOWPASS_875kHz,
	    LOWPASS_750kHz,
	    LOWPASS_INVALID
	} LOWPASS;

	/*!
	 * 1PPS source options
	 */
	typedef enum
	{
	    ONE_PPS_INTERNAL = 0,
	    ONE_PPS_EXTERNAL,
	    ONE_PPS_INVALID
	} ONE_PPS;

    } // namespace matchstiq
} // namespace gr

#endif
