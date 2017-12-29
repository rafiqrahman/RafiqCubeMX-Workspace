/*
 * This file is subject to the terms of the GFX License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              http://ugfx.org/license.html
 */
#include "stm32f1xx_hal.h"

#ifndef GDISP_LLD_BOARD_H
#define GDISP_LLD_BOARD_H

#define SET_CS		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
#define CLR_CS		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
#define SET_RS		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
#define CLR_RS		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
#define SET_WR		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
#define CLR_WR		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
#define SET_RD		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
#define CLR_RD		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);

extern void GPIOE_Write(void);
extern void GPIOE_Read(void);

static GFXINLINE void init_board(GDisplay *g) {
	GPIO_InitTypeDef GPIO_InitStruct;
	
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	
	// As we are not using multiple displays we set g->board to NULL as we don't use it.
	g->board = 0;

	switch(g->controllerdisplay) {
	
		case 0:											// Set up for Display 0

		GPIOE_Write();
		GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
		
		// Configure the pins to a well know state
		SET_RS;
		SET_RD;
		SET_WR;
		CLR_CS;
		break;
		
	}
}

static GFXINLINE void post_init_board(GDisplay *g) {
	(void) g;
}

static GFXINLINE void setpin_reset(GDisplay *g, bool_t state) {
	(void) g;
	(void) state;
}

static GFXINLINE void set_backlight(GDisplay *g, uint8_t percent) {
	(void) g;
	(void) percent;
}

static GFXINLINE void acquire_bus(GDisplay *g) {
	(void) g;
}

static GFXINLINE void release_bus(GDisplay *g) {
	(void) g;
}

static GFXINLINE void write_index(GDisplay *g, uint16_t index) {
	(void) g;
	GPIOE->ODR = index;
	CLR_RS; CLR_WR; SET_WR; SET_RS;
}

static GFXINLINE void write_data(GDisplay *g, uint16_t data) {
	(void) g;
	GPIOE->ODR = data;
	CLR_WR; SET_WR;
}

static GFXINLINE void setreadmode(GDisplay *g) {
	(void) g;
	GPIOE_Read();
}

static GFXINLINE void setwritemode(GDisplay *g) {
	(void) g;
	GPIOE_Write();
}

static GFXINLINE uint16_t read_data(GDisplay *g) {
	uint16_t	value;
	(void) g;
	
	CLR_RD;
	value = GPIOE->IDR;
	SET_RD;
	
	return value;
}

#endif /* GDISP_LLD_BOARD_H */
