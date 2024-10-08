/*
 * CLI.h
 *
 *  Created on: May 7, 2024
 *      Author: rjned
 */

#ifndef INC_CLI_H_
#define INC_CLI_H_

// includes
#include "math.h"
#include "ASCII.h"

// defines
#define OFF		0x00
#define ON		0x01

// extern global variables
extern UART_HandleTypeDef huart2;
extern uint8_t UART_tx_flag;
extern uint8_t UART_rx_flag;
extern uint8_t UART_rx[1];
extern uint8_t UART_rx_buff[100];

extern struct Channels channels;
extern struct pH p1;

// extern functions
extern void clear_UART_rx_buff(void);

// functions
void change_voltammetry_technique(uint8_t * val, uint8_t message[], uint8_t mssg_end); // lets change technique
void change_voltammetry_parameter(float * val, uint8_t message[], uint8_t mssg_end);	// lets change parameters
void voltammerty_write_SIM_parameter(uint8_t channel);	// write parameters for simultaneously voltammetry
void voltammerty_write_parameter(uint8_t channel);	// write parameters for individual channels
void voltammerty_individually(void);	// individual voltammetry is selected let user choose which channel
void voltammerty_simultaneously(void);	// simultaneously voltammetry is selected let user choose which channel
void start(uint8_t * dev, uint8_t message[], uint8_t mssg_end);
void change_voltammetry(void);	// show and change all the voltammetry settings
void show_parameter(float val, uint8_t message[], uint8_t mssg_end); // show all parameters in detail of voltammerty
void show_voltammetry(void);	// show all parameters if selected
void change_potentiostat(void);	// change the parameters
void show_potentiostat(void);	// show all parameters if selected
void UART_menu(void);	// start menu

#endif /* INC_CLI_H_ */
