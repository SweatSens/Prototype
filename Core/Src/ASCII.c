/*
 * ASCII.c
 *
 *  Created on: May 7, 2024
 *      Author: rjned
 */

// includes
#include "main.h"

uint8_t float_to_ASCII(float val, uint8_t buf[]){
	// 5123.946 -> 5123 / -0.001 -> -0.001/ 12.00 -> 12.0

	// size is normally 3
	uint8_t size = 5;
	uint8_t ngtv_flg = 0;	// number is negative
	uint8_t cntr = 0;		// length of array
	float pos = 0.0;		// absolute value

	// if value is zero
	if(val == 0.0){
		buf[0] = 48;
		return 1;
	}

	// check if value is negative
	if(val < 0.0){
		ngtv_flg = 1;
		pos = val * -1.0;
		cntr += 1;
		buf[0] = 45;
	}
	else{
		pos = val;
	}

	// check how many integers there are
	uint8_t intgr = 0;
	float prfx = 1.0;
	while(pos >= prfx){
		prfx *= 10.0;
		intgr += 1;
	}

	// 51235.122 -> 51235
	if(intgr >= size){
		for(uint8_t j=cntr;j<(intgr+cntr); j++){
			buf[j] = (uint8_t) fmod(pos / (prfx/10), 10) + 48;
			prfx /= 10;
		}
		cntr = intgr + cntr;
	}
	// -12.96 -> -12.9 (transform to three numbers)
	if(intgr < size && intgr > 0){
		// get the 3 number containing information (12.9 -> 129)
		for(uint8_t j=intgr;j<size;j++){
			pos *= 10;
			prfx *= 10;
		}
		// add the point and the numbers holding information
		for(uint8_t j=cntr;j<(size+cntr+1); j++){
			if(j == intgr+cntr){
				buf[cntr+intgr] = 46;
			}
			else{
				buf[j] = (uint8_t) fmod(pos / (prfx/10), 10) + 48;
				prfx /= 10;
			}
		}
		cntr = cntr + size + 1;
	}
	// 0.0012345 -> 0.00123 / 0.99951 -> 0.999 / 0.100000 -> 0.100 (transform to three numbers)
	if(intgr <= 0){
		// steps are i.e. 0.000123 -> 0. + 000 + 123
		uint8_t frctnl = 0;
		// make 0.
		buf[cntr] = 48;
		cntr += 1;
		buf[cntr] = 46;
		cntr += 1;
		// get the three numbers that hold the information
		while(pos < 100){
			pos *= 10;
			frctnl += 1;
		}
		// pad zeros (+ 000)
		for(uint8_t i=cntr;i<(frctnl-1+ngtv_flg);i++){
			buf[i] = 48;
			cntr += 1;
		}
		// add the numbers containing information to the array (+ 123)
		prfx = 100;
		for(uint8_t j=cntr;j<(cntr+size); j++){
			buf[j] = (uint8_t) fmod(pos / (prfx), 10) + 48;
			prfx /= 10;
		}
		cntr = cntr + size;
	}
	return cntr;
}

float ASCII_to_float(uint8_t begin, uint8_t end){
	float val = 0.0;		// result
	uint8_t k = begin;		// increment
	uint8_t dot = 0;		// dot for decimal calculation
	float dcmlpnt = 1.0;	// decimal point counter
	uint8_t ngtv_flg = 0;	// number is negative

	if(UART_rx_buff[k] == 45){	// check if UART send 45 is '-' (negative sign)
	  ngtv_flg = 1;
	  k = begin + 1;
	}
	// loop through UART Rx buffer
	while(k != end){
	  if(UART_rx_buff[k] == 46){	// 46 is ASCII '.' (dot)
		  dot = 1;
		  k += 1;
	  }
	  // calculate value from UART uint8_t to float
	  if(dot == 0){
		  val = val * 10 + (float)UART_rx_buff[k] - 48.0;
	  }
	  else{
		  val = val + (((float)UART_rx_buff[k] - 48.0) / (10.0 * dcmlpnt));
		  dcmlpnt *= 10.0;
	  }
	  k += 1;
	}
	// make value negative
	if(ngtv_flg == 1){
	  val *= -1.0;
	}
	return val;
}
