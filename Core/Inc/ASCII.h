/*
 * ASCII.h
 *
 *  Created on: May 7, 2024
 *      Author: rjned
 */

#ifndef INC_ASCII_H_
#define INC_ASCII_H_

float ASCII_to_float(uint8_t begin, uint8_t end);
uint8_t float_to_ASCII(float val, uint8_t buf[]);

#endif /* INC_ASCII_H_ */
