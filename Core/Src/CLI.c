/*
 * CLI.c
 *
 *  Created on: May 7, 2024
 *      Author: rjned
 *      c file of the command line interface (CLI).
 */

// includes
#include "main.h"

void show_parameter(float val, uint8_t message[], uint8_t mssg_end){
	// show parameter
	while(UART_tx_flag != 0);
	HAL_UART_Transmit_DMA(&huart2, message, mssg_end);
	UART_tx_flag = 1;
	// communicate to console
	uint8_t buff2[10];
	uint8_t size = float_to_ASCII(val, buff2);
	while(UART_tx_flag != 0);
	HAL_UART_Transmit_DMA(&huart2, buff2, size);
	UART_tx_flag = 1;
	while(UART_tx_flag != 0);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"\n:", 1);
	UART_tx_flag = 1;

	return;
}

void change_voltammetry_parameter(float * val, uint8_t message[], uint8_t mssg_end){
	uint8_t size = 0;
	uint8_t i = 0;
	// send which parameter
	while(UART_tx_flag != 0);
	HAL_UART_Transmit_DMA(&huart2, message, mssg_end);
	UART_tx_flag = 1;
	// receive data and set it in a parameter
	while(UART_rx_flag != 1);
	i = 0;
	while(UART_rx_buff[i] != 10 && UART_rx_buff[i] != 13){
		i += 1;
	}
	// if no enter, there is data and set parameter
	if(i != 0){
		*val = ASCII_to_float(0,i);
	}
	clear_UART_rx_buff();
	UART_rx_flag = 0;
	// Communicate parameter back to console
	uint8_t buff1[10];
	size = float_to_ASCII(*val, buff1);
	while(UART_tx_flag != 0);
	HAL_UART_Transmit_DMA(&huart2, buff1, size);
	UART_tx_flag = 1;
	while(UART_tx_flag != 0);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"\n:", 1);
	UART_tx_flag = 1;

	return;
}

void change_voltammetry_technique(uint8_t * val, uint8_t message[], uint8_t mssg_end){
	uint8_t size = 0;
	uint8_t i = 0;
	// send which parameter
	while(UART_tx_flag != 0);
	HAL_UART_Transmit_DMA(&huart2, message, mssg_end);
	UART_tx_flag = 1;
	// receive data and set it in a parameter
	while(UART_rx_flag != 1);
	i = 0;
	while(UART_rx_buff[i] != 10 && UART_rx_buff[i] != 13){
		i += 1;
	}
	// if no enter, there is data and set parameter
	if(i != 0){
		*val = ASCII_to_float(0,i);
	}
	clear_UART_rx_buff();
	UART_rx_flag = 0;
	// Communicate parameter back to console
	uint8_t buff1[10];
	size = float_to_ASCII(*val, buff1);
	while(UART_tx_flag != 0);
	HAL_UART_Transmit_DMA(&huart2, buff1, size);
	UART_tx_flag = 1;

	return;
}

void voltammerty_write_SIM_parameter(uint8_t channel){
	// get the first broadcast channel settings
	for(uint8_t i=0;i<4;i++){
		if(channels.parameters[i].ch_brdcst_on == 1 && i != channel){
			switch(channels.parameters[channel].technique){
				case 1:
					channels.parameters[i].technique = 1;
					channels.parameters[i].cv.t_equilibration = channels.parameters[channel].cv.t_equilibration;
					channels.parameters[i].cv.E_begin = channels.parameters[channel].cv.E_begin;
					channels.parameters[i].cv.E_step = channels.parameters[channel].cv.E_step;
					channels.parameters[i].cv.E_vertex_1 = channels.parameters[channel].cv.E_vertex_1;
					channels.parameters[i].cv.E_vertex_2 = channels.parameters[channel].cv.E_vertex_2;
					channels.parameters[i].cv.scan_rate = channels.parameters[channel].cv.scan_rate;
					break;
				case 2:
					channels.parameters[i].technique = 2;
					channels.parameters[i].dpv.t_equilibration = channels.parameters[channel].dpv.t_equilibration;
					channels.parameters[i].dpv.E_begin = channels.parameters[channel].dpv.E_begin;
					channels.parameters[i].dpv.E_end = channels.parameters[channel].dpv.E_end;
					channels.parameters[i].dpv.E_pulse = channels.parameters[channel].dpv.E_pulse;
					channels.parameters[i].dpv.t_pulse = channels.parameters[channel].dpv.t_pulse;
					channels.parameters[i].dpv.Scan_rate = channels.parameters[channel].dpv.Scan_rate;
					channels.parameters[i].dpv.E_step = channels.parameters[channel].dpv.E_step;
					break;
				case 3:
					channels.parameters[i].technique = channels.parameters[channel].technique;
					channels.parameters[i].swv.t_equilibration = channels.parameters[channel].swv.t_equilibration;
					channels.parameters[i].swv.E_begin = channels.parameters[channel].swv.E_begin;
					channels.parameters[i].swv.E_end = channels.parameters[channel].swv.E_end;
					channels.parameters[i].swv.Frequency = channels.parameters[channel].swv.Frequency;
					channels.parameters[i].swv.Amplitude = channels.parameters[channel].swv.Amplitude;
					channels.parameters[i].swv.E_step = channels.parameters[channel].swv.E_step;
					break;
				default:
					channels.parameters[i].technique = 0;
					break;
			}
		}
		else if(i != channel){
			channels.parameters[i].technique = 0;
		}
	}

	return;
}

void voltammerty_write_parameter(uint8_t channel){

	// parameters
	change_voltammetry_technique(&(channels.parameters[channel].technique),(uint8_t*)" technique (0 = off, 1 = CV, 2 = DPV, 3= SWV): ",47);
	switch(channels.parameters[channel].technique){
		case 1:
			while(UART_tx_flag != 0);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)" - cyclic voltammetry\n", 22);
			UART_tx_flag = 1;
			change_voltammetry_parameter(&channels.parameters[channel].cv.t_equilibration, (uint8_t*)" t equilibration (s): ",22);
			change_voltammetry_parameter(&channels.parameters[channel].cv.E_begin, (uint8_t*)" E begin (V): ",14);
			change_voltammetry_parameter(&channels.parameters[channel].cv.E_step, (uint8_t*)" E step (V): ",13);
			change_voltammetry_parameter(&channels.parameters[channel].cv.E_vertex_1, (uint8_t*)" E Vertex 1 (V): ",17);
			change_voltammetry_parameter(&channels.parameters[channel].cv.E_vertex_2, (uint8_t*)" E Vertex 2 (V): ",17);
			change_voltammetry_parameter(&channels.parameters[channel].cv.scan_rate, (uint8_t*)" Scan rate (V/s): ",18);
			break;
		case 2:
			while(UART_tx_flag != 0);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)" - differential voltammetry\n", 28);
			UART_tx_flag = 1;
			change_voltammetry_parameter(&channels.parameters[channel].dpv.t_equilibration, (uint8_t*)" t equilibration (s): ",22);
			change_voltammetry_parameter(&channels.parameters[channel].dpv.E_begin, (uint8_t*)" E begin (V): ",14);
			change_voltammetry_parameter(&channels.parameters[channel].dpv.E_end, (uint8_t*)" E end (V): ",12);
			change_voltammetry_parameter(&channels.parameters[channel].dpv.E_step, (uint8_t*)" E step (V): ",13);
			change_voltammetry_parameter(&channels.parameters[channel].dpv.E_pulse, (uint8_t*)" E pulse (V): ",14);
			change_voltammetry_parameter(&channels.parameters[channel].dpv.t_pulse, (uint8_t*)" t pulse (s): ",14);
			change_voltammetry_parameter(&channels.parameters[channel].dpv.Scan_rate, (uint8_t*)" Scan rate (V/s): ",18);
			break;
		case 3:
			while(UART_tx_flag != 0);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)" - square wave voltammetry\n", 27);
			UART_tx_flag = 1;
			change_voltammetry_parameter(&channels.parameters[channel].swv.t_equilibration, (uint8_t*)" t equilibration (s): ",22);
			change_voltammetry_parameter(&channels.parameters[channel].swv.E_begin, (uint8_t*)" E begin (V): ",14);
			change_voltammetry_parameter(&channels.parameters[channel].swv.E_end, (uint8_t*)" E end (V): ",12);
			change_voltammetry_parameter(&channels.parameters[channel].swv.E_step, (uint8_t*)" E step (V): ",13);
			change_voltammetry_parameter(&channels.parameters[channel].swv.Amplitude, (uint8_t*)" Amplitude (V): ",16);
			change_voltammetry_parameter(&channels.parameters[channel].swv.Frequency, (uint8_t*)" Frequency (Hz): ",17);
			break;
		default:
			while(UART_tx_flag != 0);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)" - off\n", 7);
			UART_tx_flag = 1;
			break;
	}
	// end
	while(UART_tx_flag != 0);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"\n", 1);
	UART_tx_flag = 1;
	return;
}

void voltammerty_simultaneously(void){
	uint8_t channel = 0;
	uint8_t buf[1];
	while(UART_tx_flag != 0);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"Choose channels (0 = off & 1 = on): \n", 37);
	UART_tx_flag = 1;
	// channel
	while(channel != 4){
		while(UART_tx_flag != 0);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"Channel ", 8);
		UART_tx_flag = 1;
		buf[0] = channel + 49;
		while(UART_tx_flag != 0);
		HAL_UART_Transmit_DMA(&huart2, buf , 1);
		UART_tx_flag = 1;
		change_voltammetry_technique(&(channels.parameters[channel].ch_brdcst_on), (uint8_t*)": ",2);
		while(UART_tx_flag != 0);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"\n", 1);
		UART_tx_flag = 1;

		channel += 1;
	}
	uint8_t do_once = 0;
	for(uint8_t i=0;i<4;i++){
		if(channels.parameters[i].ch_brdcst_on == 1 && do_once == 0){
			voltammerty_write_parameter(i);
			voltammerty_write_SIM_parameter(i);
			do_once = 1;
		}
	}

	return;
}

void voltammerty_individually(void){
	uint8_t channel = 0;
	uint8_t buf[1];
	while(channel != 4){
		// show channel
		while(UART_tx_flag != 0);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"Channel ", 8);
		UART_tx_flag = 1;
		while(UART_tx_flag != 0);
		buf[0] = channel + 49;
		HAL_UART_Transmit_DMA(&huart2, buf, 1);
		UART_tx_flag = 1;
		while(UART_tx_flag != 0);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)": \n", 3);
		UART_tx_flag = 1;

		voltammerty_write_parameter(channel);

		channel += 1;
	}

	return;
}

void start(uint8_t * dev, uint8_t message[], uint8_t mssg_end){
	// write to the console
	while(UART_tx_flag != 0);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"Start ", 6);
	UART_tx_flag = 1;
	while(UART_tx_flag != 0);
	HAL_UART_Transmit_DMA(&huart2, message, mssg_end);
	UART_tx_flag = 1;
	while(UART_tx_flag != 0);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"? [y/n]\n\n", 9);
	UART_tx_flag = 1;
	// wait for answer
	while(UART_rx_flag != 1);
	if(UART_rx_buff[0] == 121){
		while(UART_tx_flag != 0);
		HAL_UART_Transmit_DMA(&huart2, message, mssg_end);
		UART_tx_flag = 1;
		while(UART_tx_flag != 0);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)" started\n\n", 10);
		UART_tx_flag = 1;
		*dev = ON;
	}
	else{
		while(UART_tx_flag != 0);
		HAL_UART_Transmit_DMA(&huart2, message, mssg_end);
		UART_tx_flag = 1;
		while(UART_tx_flag != 0);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)" cancelled\n\n", 12);
		UART_tx_flag = 1;
		*dev = OFF;
	}
	clear_UART_rx_buff();
	UART_rx_flag = 0;
	return;
}

void change_voltammetry(void){
	// write to the console
	while(UART_tx_flag != 0);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"Change parameters? [y/n].\n", 26);
	UART_tx_flag = 1;

		while(UART_rx_flag != 1);
		if(UART_rx_buff[0] == 121){
			clear_UART_rx_buff();
			UART_rx_flag = 0;
			// control simultaneously
			while(UART_tx_flag != 0);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"Control all channels simultaneously? [y/n]\n\n", 44);
			UART_tx_flag = 1;
			while(UART_rx_flag != 1);
			if(UART_rx_buff[0] == 121){
				clear_UART_rx_buff();
				UART_rx_flag = 0;
				channels.broadcast = 1;
				while(UART_tx_flag != 0);
				HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"Control all channels simultaneously!\n", 37);
				UART_tx_flag = 1;
				voltammerty_simultaneously();
			}
			else{
				clear_UART_rx_buff();
				UART_rx_flag = 0;
				channels.broadcast = 0;
				while(UART_tx_flag != 0);
				HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"Control all channels individually!\n", 35);
				UART_tx_flag = 1;
				voltammerty_individually();
			}
		}
		clear_UART_rx_buff();
		UART_rx_flag = 0;

	return;
}

void show_voltammetry(void){
	// write to the console
	while(UART_tx_flag != 0);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"Voltammetry! The following parameters are:\n\n", 44);
	UART_tx_flag = 1;

	uint8_t i = 0;
	uint8_t arr[1];

	while(i != 4){
		// show parameter
		switch(channels.parameters[i].technique){
		case 0:
			while(UART_tx_flag != 0);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"Channel ", 8);
			UART_tx_flag = 1;
			while(UART_tx_flag != 0);
			arr[0] = i + 49;
			HAL_UART_Transmit_DMA(&huart2, arr, 1);
			UART_tx_flag = 1;
			while(UART_tx_flag != 0);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)" is not used (technique = 0)\n\n", 30);
			UART_tx_flag = 1;
			break;
		case 1:
			while(UART_tx_flag != 0);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"Channel ", 8);
			UART_tx_flag = 1;
			while(UART_tx_flag != 0);
			arr[0] = i + 49;
			HAL_UART_Transmit_DMA(&huart2, arr, 1);
			UART_tx_flag = 1;
			while(UART_tx_flag != 0);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)" uses Cyclic voltammetry (technique = 1)\n", 41);
			UART_tx_flag = 1;
			show_parameter(channels.parameters[i].cv.t_equilibration, (uint8_t*)" t equilibration (s): ",22);
			show_parameter(channels.parameters[i].cv.E_begin, (uint8_t*)" E begin (V): ",14);
			show_parameter(channels.parameters[i].cv.E_step, (uint8_t*)" E step (V): ",13);
			show_parameter(channels.parameters[i].cv.E_vertex_1, (uint8_t*)" E Vertex 1 (V): ",17);
			show_parameter(channels.parameters[i].cv.E_vertex_2, (uint8_t*)" E Vertex 2 (V): ",17);
			show_parameter(channels.parameters[i].cv.scan_rate, (uint8_t*)" Scan rate (V/s): ",18);
			while(UART_tx_flag != 0);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"\n:", 1);
			UART_tx_flag = 1;
			break;
		case 2:
			while(UART_tx_flag != 0);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"Channel ", 8);
			UART_tx_flag = 1;
			while(UART_tx_flag != 0);
			arr[0] = i + 49;
			HAL_UART_Transmit_DMA(&huart2, arr, 1);
			UART_tx_flag = 1;
			while(UART_tx_flag != 0);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)" uses Differential voltammetry (technique = 2)\n", 47);
			UART_tx_flag = 1;
			show_parameter(channels.parameters[i].dpv.t_equilibration, (uint8_t*)" t equilibration (s): ",22);
			show_parameter(channels.parameters[i].dpv.E_begin, (uint8_t*)" E begin (V): ",14);
			show_parameter(channels.parameters[i].dpv.E_end, (uint8_t*)" E end (V): ",12);
			show_parameter(channels.parameters[i].dpv.E_step, (uint8_t*)" E step (V): ",13);
			show_parameter(channels.parameters[i].dpv.E_pulse, (uint8_t*)" E pulse (V): ",14);
			show_parameter(channels.parameters[i].dpv.t_pulse, (uint8_t*)" t pulse (s): ",14);
			show_parameter(channels.parameters[i].dpv.Scan_rate, (uint8_t*)" Scan rate (V/s): ",18);
			while(UART_tx_flag != 0);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"\n:", 1);
			UART_tx_flag = 1;
			break;
		case 3:
			while(UART_tx_flag != 0);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"Channel ", 8);
			UART_tx_flag = 1;
			while(UART_tx_flag != 0);
			arr[0] = i + 49;
			HAL_UART_Transmit_DMA(&huart2, arr, 1);
			UART_tx_flag = 1;
			while(UART_tx_flag != 0);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)" uses Square Wave voltammetry (technique = 3)\n", 46);
			UART_tx_flag = 1;
			show_parameter(channels.parameters[i].swv.t_equilibration, (uint8_t*)" t equilibration (s): ",22);
			show_parameter(channels.parameters[i].swv.E_begin, (uint8_t*)" E begin (V): ",14);
			show_parameter(channels.parameters[i].swv.E_end, (uint8_t*)" E end (V): ",12);
			show_parameter(channels.parameters[i].swv.E_step, (uint8_t*)" E step (V): ",13);
			show_parameter(channels.parameters[i].swv.Amplitude, (uint8_t*)" Amplitude (V): ",16);
			show_parameter(channels.parameters[i].swv.Frequency, (uint8_t*)" Frequency (Hz): ",17);
			while(UART_tx_flag != 0);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"\n:", 1);
			UART_tx_flag = 1;
			break;
		default:
			break;
		}
		i+= 1;
	}
	return;
}

void show_potentiostat(void){
	uint8_t size = 0;

	// write to the console
	while(UART_tx_flag != 0);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"Potentiostat! The following parameters are:\n t interval (s): ", 61);
	UART_tx_flag = 1;

	// communicate to console
	uint8_t buff1[10];
	size = float_to_ASCII(p1.t_interval, buff1);
	while(UART_tx_flag != 0);
	HAL_UART_Transmit_DMA(&huart2, buff1, size);
	UART_tx_flag = 1;
	while(UART_tx_flag != 0);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"\n:", 1);
	UART_tx_flag = 1;

	// show parameter
	while(UART_tx_flag != 0);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)" t run (s): ", 12);
	UART_tx_flag = 1;
	// communicate to console
	uint8_t buff2[10];
	size = float_to_ASCII(p1.t_run, buff2);
	while(UART_tx_flag != 0);
	HAL_UART_Transmit_DMA(&huart2, buff2, size);
	UART_tx_flag = 1;
	while(UART_tx_flag != 0);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"\n:", 1);
	UART_tx_flag = 1;

	return;
}

void change_potentiostat(void){
	uint8_t size = 0;
	uint8_t i = 0;

	// write to the console
	while(UART_tx_flag != 0);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"Change parameters? [y/n].\n", 26);
	UART_tx_flag = 1;
	while(UART_rx_flag != 1);
	if(UART_rx_buff[0] == 121){
		clear_UART_rx_buff();
		UART_rx_flag = 0;

		// send which parameter
		while(UART_tx_flag != 0);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)" t interval (s): ", 17);
		UART_tx_flag = 1;
		// receive data and set it in a parameter
		while(UART_rx_flag != 1);
		i = 0;
		while(UART_rx_buff[i] != 10 && UART_rx_buff[i] != 13){
			i += 1;
		}
		// if no enter, there is data and set parameter
		if(i != 0){
			p1.t_interval = ASCII_to_float(0,i);
		}
		clear_UART_rx_buff();
		UART_rx_flag = 0;
		// Communicate parameter back to console
		uint8_t buff1[10];
		size = float_to_ASCII(p1.t_interval, buff1);
		while(UART_tx_flag != 0);
		HAL_UART_Transmit_DMA(&huart2, buff1, size);
		UART_tx_flag = 1;

		// send which parameter
		while(UART_tx_flag != 0);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"\n t run (s): ", 13);
		UART_tx_flag = 1;
		// receive data and set it in a parameter
		while(UART_rx_flag != 1);
		i = 0;
		while(UART_rx_buff[i] != 10 && UART_rx_buff[i] != 13){
			i += 1;
		}
		// if no enter, there is data and set parameter
		if(i != 0){
			p1.t_run = ASCII_to_float(0,i);
		}
		clear_UART_rx_buff();
		UART_rx_flag = 0;
		// Communicate parameter back to console
		uint8_t buff2[10];
		size = float_to_ASCII(p1.t_run, buff2);
		while(UART_tx_flag != 0);
		HAL_UART_Transmit_DMA(&huart2, buff2, size);
		UART_tx_flag = 1;

		// end
		while(UART_tx_flag != 0);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"\n", 1);
		UART_tx_flag = 1;
	}
	clear_UART_rx_buff();
	UART_rx_flag = 0;

	return;
}

void UART_menu(void){
	p1.on = OFF;
	channels.on = OFF;
	// start menu
	while(UART_tx_flag != 0);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"\nInformation about settings can be found on palmsens.com\nType:\n 1. Potentiostat.\n 2. Voltammetry.\n\n", 99);
	UART_tx_flag = 1;

	while(UART_rx_flag != 1); // wait for new line
	switch (UART_rx_buff[0]) {
	case 49:
		clear_UART_rx_buff();
		UART_rx_flag = 0;
		show_potentiostat();
		change_potentiostat();
		start(&p1.on, (uint8_t*)"Potentiostat", 12);
		while(UART_tx_flag != 0);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"Channel, Time (s), Voltage Out (V) \n", 36);
		UART_tx_flag = 1;

		break;
	case 50:
		clear_UART_rx_buff();
		UART_rx_flag = 0;
		show_voltammetry();
		change_voltammetry();
		start(&channels.on, (uint8_t*)"Voltammetry", 11);
		while(UART_tx_flag != 0);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"Channel, Time (s), Voltage Applied (V), Voltage Out (V), Voltage Ref. (V) \n", 75);
		UART_tx_flag = 1;
	  break;
	default:
		break;
	}
	clear_UART_rx_buff();
	UART_rx_flag = 0;

	return;
}
