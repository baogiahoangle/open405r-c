/*****************************************************************************
* | File      	:		touch.h
* | Function    :   Touch API
* | Info        :
*----------------
* |	This version:   V1.0
* | Date        :   2025-06-24
* | Info        :
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
******************************************************************************/

#ifndef __TOUCH_H_
#define __TOUCH_H_

#include "stm32f4xx.h"
#include <stdbool.h>  

#define TP_PRESS_DOWN           0x80
#define TP_PRESSED              0x40
bool tp_get_touch_point(uint16_t *x, uint16_t *y);
typedef struct {
	uint16_t hwXpos0;
	uint16_t hwYpos0;
	uint16_t hwXpos;
	uint16_t hwYpos;
	uint8_t chStatus;
	uint8_t chType;
	short iXoff;
	short iYoff;
	float fXfac;
	float fYfac;
} tp_dev_t;

extern void tp_init(void);
extern void tp_adjust(void);
extern void tp_dialog(void);
extern void tp_draw_board(void);


#endif



