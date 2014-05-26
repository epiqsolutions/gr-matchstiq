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


#ifndef INCLUDED_MATCHSTIQ_SOURCE_S_H
#define INCLUDED_MATCHSTIQ_SOURCE_S_H

#include <matchstiq/api.h>
#include <matchstiq/matchstiq_defs.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace matchstiq {

    /*!
     * \brief Implementation of Matchstiq as a source block
     * \ingroup matchstiq
     *
     */
    class MATCHSTIQ_API matchstiq_source_s : virtual public gr::sync_block
    {
    public:
       typedef boost::shared_ptr<matchstiq_source_s> sptr;

       /*!
        * \brief Return a shared_ptr to a new instance of 
	* matchstiq::matchstiq_source_s.
       * 
        * To avoid accidental use of raw pointers, 
	* matchstiq::matchstiq_source_s's constructor is in a private 
	* implementation class. matchstiq::matchstiq_source_s::make is
	* the public interface for creating new instances.
        */
       static sptr make(const std::string ip_address, uint32_t port);

       /*! 
	* \brief Set center frequency with Hz resolution.
	* \param freq The frequency in Hz
	* \return the actual center frequency
	*
	* Set the center frequency of the Matchstiq.
	*/
       virtual uint64_t set_center_freq(uint64_t freq) = 0;

       /*! 
	* \brief Set frequency with Hz resolution.
	* \param freq The frequency in Hz
	* \return the actual center frequency
	*
	* Convenience function that uses float parameter to all 
	* engineering notation to be used in GRC.
	*/
       virtual uint64_t set_center_freq(float freq) = 0;

       /*! 
	* \brief Get center frequency with Hz resolution.
	* \return the actual center frequency
	*
	* Get the center frequency of the Matchstiq.
	*/
       virtual uint64_t center_freq(void) = 0;

       /*! 
	* \brief Set the sample rate
	* \param sample_rate The sample rate
	* \return the actual sample rate
	*
	* Set the sample rate of the Matchstiq. To calculate the rate 
	* of samples delivered, the decimation stage needs to be
	* factored in.
	*/
       virtual uint32_t set_sample_rate(uint32_t sample_rate) = 0;

       /*! 
	* \brief Set the sample rate
	* \param sample_rate The sample rate
	* \return the actual sample rate
	*
	* Convenience function that uses float parameter to all engineering
	* notation to be used in GRC.
	*/
       virtual uint32_t set_sample_rate(float sample_rate) = 0;

       /*! 
	* \brief Get the sample rate
	* \return the actual sample rate
	*
	* Get the sample rate of the Matchstiq. To calculate
	* the rate of samples delivered, the decimation stage needs to be
	* factored in.
	*/
       virtual uint32_t sample_rate(void) = 0;


       /*! 
	* \brief Enable/disable the front LNA
	* \param enable Enable/disable front LNA
	* \return state of front LNA
	*
	* Enable or disable the front gain LNA (~15 dB)
	*/
       virtual STATUS set_front_lna(STATUS enable) = 0;

       /*! 
	* \brief Get the front LNA state
	* \return state of front LNA
	*
	* Get the state of the front gain LNA (~15 dB)
	*/
       virtual STATUS front_lna(void) = 0;

       /*! 
	* \brief Set the second LNA gain
	* \param gain LNA gain
	* \return second LNA gain
	* 
	* Set the second LNA gain of the RX lineup
	*/
       virtual float set_second_lna_gain(float gain) = 0;

       /*! 
	* \brief Get the second LNA gain
	* \return second LNA gain
	* 
	* Get the second LNA gain of the RX lineup
	*/
       virtual float second_lna_gain(void) = 0;

       /*! 
	* \brief Set the attenuation in dB
	* \param atten attenuation 
	* \return actual attenuation
	*
	* Set the attenuation in dB (0-42.5 dB in 0.5 steps)
	*/
       virtual float set_step_atten(float atten) = 0;

       /*! 
	* \brief Get the attenuation in dB
	* \return actual attenuation
	*
	* Get the attenuation in dB
	*/
       virtual float step_atten(void) = 0;

       /*!
	* \brief Set the gain of the first variable gain ampliefier
	* \param gain1 gain of first stage
	* \return actual gain
	*
	* Set the first variable gain amplifier stage in the mixer of
	* RF lineup (rxvga1). Gain provides ~25 dB gain but the mapping is not 
	* log-linear. (Gain mapping example: 2=~5 dB, 102=~19 dB, 120=~30 dB)
	*/
       virtual uint8_t set_rxvga1_gain(uint8_t gain1) = 0;

       /*!
	* \brief Get the gain of the first variable gain ampliefier
	* \return actual gain
	*
	* Get the first variable gain amplifier stage in the mixer of
	* RF lineup (rxvga1). Gain provides ~25 dB gain but the mapping is not 
	* log-linear. (Gain mapping example: 2=~5 dB, 102=~19 dB, 120=~30 dB)
	*/
       virtual uint8_t rxvga1_gain(void) = 0;

       /*!
	* \brief Set the gain of the second variable gain amplifier
	* \param gain2 gain of the second stage
	* \return actual gain
	* Set the second variable gain amplier stage in the mixer of 
	* RF lineup (rxvga2). Gain provides ~30 dB of gain in 3 dB steps.
	* 
	*/
       virtual uint8_t set_rxvga2_gain(uint8_t gain2) = 0;

       /*!
	* \brief Get the gain of the second variable gain amplifier
	* \return actual gain
	* Get the second variable gain amplier stage in the mixer of 
	* RF lineup (rxvga2). Gain provides ~30 dB of gain in 3 dB steps.
	* 
	*/
       virtual uint8_t rxvga2_gain(void) = 0;

       /*!
	* \brief Set the tcvcxo warp voltage for the ref clock oscillator
	* \param warp_voltage warp voltage
	* \return actual warp voltage
	*
	* Set the warp voltage for the ref clock oscillator.  The ref clock
	* can be pulled +/- 8 ppm by applying a 12-bit DAC voltage.
	* Valid warp voltages are 0.5-2.5V (values 683-3413)
	*/
       virtual uint16_t set_tcvcxo_warp_voltage(uint16_t warp_voltage) = 0;

       /*!
	* \brief Get the tcvcxo warp voltage for the ref clock oscillator
	* \return actual warp voltage
	*
	* Get the warp voltage for the ref clock oscillator.  The ref clock
	* can be pulled +/- 8 ppm by applying a 12-bit DAC voltage.
	* Valid warp voltages are 0.5-2.5V (values 683-3413)
	*/
       virtual uint16_t tcvcxo_warp_voltage(void) = 0;

       /*!
	* \brief Set the frequency range of the low pass filter 
	* \param filter value of low pass filter
	* \return actual low pass filter value
	*
        * Set the baseband low pass filter cutoff frequency.
	*/
       virtual LOWPASS set_low_pass_filter(LOWPASS filter) = 0;

       /*!
	* \brief Get the frequency range of the low pass filter 
	* \return actual low pass filter value
	*
        * Get the baseband low pass filter cutoff frequency.
	*/
       virtual LOWPASS low_pass_filter(void) = 0;

       /*!
	* \brief Set the CIC decimation stage
	* \param decimation value of decimation
	* \return actual decimation value
	*
	* Set the CIC decimation stage value
	*/
       virtual uint8_t set_decimation(uint8_t decimation) = 0;

       /*!
	* \brief Get the CIC decimation stage
	* \return actual decimation value
	*
	* Get the CIC decimation stage value
	*/
       virtual uint8_t decimation(void) = 0;

       /*!
	* \brief Set the preselect filter on the RX path
	* \param preselect the preselect filter
	* \return actual preselect filter
	*
	* Set the preselect filter on the RX path
	*/
       virtual PRESELECT set_preselect_filter(PRESELECT preselect) = 0;

       /*!
	* \brief Get the preselect filter on the RX path
	* \return actual preselect filter
	*
	* Get the preselect filter on the RX path
	*/
       virtual PRESELECT preselect_filter(void) = 0;

       /*!
	* \brief Set the source of the 1PPS signal
	* \param pps_source source of 1PPS signal
	* \return actual 1PPS signal source
	*
	* Set the source of 1PPS signal to be either internal 
	* (from GPS) or external.
	*/
       virtual ONE_PPS set_one_pps_source(ONE_PPS pps_source) = 0;

       /*!
	* \brief Get the source of the 1PPS signal
	* \return actual 1PPS signal source
	*
	* Get the source of 1PPS signal to be either internal 
	* (from GPS) or external.
	*/
       virtual ONE_PPS one_pps_source(void) = 0;

       /*!
	* \brief Set the full path of the CIC filter coefficients file.
	* \param filename full path of CIC filter coefficients file
	* \return the actual filename 
	*
	* Set the full path of the CIC filter coefficients file.  Note
	* that most users will not need to set this and the default 
	* parameters will be utilized in this case.  If the filename
	* of "default" is provided, this will default the CIC filter
	* coefficients.
	*/
       virtual std::string set_cic_coefficients(const std::string filename) = 0; 

       /*!
	* \brief Get the full path of the CIC filter coefficients file.
	* \return the actual filename 
	*
	* Get the full path of the CIC filter coefficients file.  
	*/
       virtual std::string cic_coefficients(void) = 0; 

    };

  } // namespace matchstiq
} // namespace gr

#endif /* INCLUDED_MATCHSTIQ_SOURCE_S_H */

