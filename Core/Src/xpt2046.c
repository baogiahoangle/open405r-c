/*****************************************************************************
* | File      	:		xpt2046.c
* | Author      :   Waveshare team
* | Function    :   Touch API
* | Info        :
*----------------
* |	This version:   V1.0
* | Date        :   2019-05-05
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

#include "xpt2046.h"


/******************************************************************************
function :	write eight bits' data to xpt2046
parameter:
  chData : send data
******************************************************************************/
uint8_t xpt2046_write_byte(uint8_t chData)
{
    //return XPT2046_WRITE_BYTE(chData);
    uint8_t received = 0;
    HAL_SPI_TransmitReceive(&hspi1, &chData, &received, 1, HAL_MAX_DELAY);
    return received;
}

/******************************************************************************
function :	initial xpt2046
parameter:
******************************************************************************/
void xpt2046_init(void)
{
	uint16_t hwXpos, hwYpos;
		
	XPT2046_CS_H();

	xpt2046_read_xy(&hwXpos, &hwYpos);
}

/******************************************************************************
function :	read ad value
parameter:
  chCmd  : send data
******************************************************************************/
uint16_t xpt2046_read_ad_value(uint8_t chCmd)
{
    uint16_t hwData = 0;
    
    XPT2046_CS_L();
    xpt2046_write_byte(chCmd);
    hwData = xpt2046_write_byte(0x00);
    hwData <<= 8;
    hwData |= xpt2046_write_byte(0x00);
    hwData >>= 3;
    XPT2046_CS_H();
    
    return hwData;
}

/******************************************************************************
function :	read ad average value
parameter:
  chCmd  : send data
******************************************************************************/
#define READ_TIMES  5
#define LOST_NUM    1
uint16_t xpt2046_read_average(uint8_t chCmd)
{
    uint8_t i, j;
    uint16_t hwbuffer[READ_TIMES], hwSum = 0, hwTemp;

    for (i = 0; i < READ_TIMES; i ++) {
        hwbuffer[i] = xpt2046_read_ad_value(chCmd);
    }
    for (i = 0; i < READ_TIMES - 1; i ++) {
        for (j = i + 1; j < READ_TIMES; j ++) {
            if (hwbuffer[i] > hwbuffer[j]) {
                hwTemp = hwbuffer[i];
                hwbuffer[i] = hwbuffer[j];
                hwbuffer[j] = hwTemp;
            }
        }
    }
    for (i = LOST_NUM; i < READ_TIMES - LOST_NUM; i ++) {
        hwSum += hwbuffer[i];
    }
    hwTemp = hwSum / (READ_TIMES - 2 * LOST_NUM);

    return hwTemp;
}

/******************************************************************************
function :	read lcd x and y axis
parameter:
  phwXpos: ponit to x axis
  phwYpos: ponit to y axis
******************************************************************************/
void xpt2046_read_xy(uint16_t *phwXpos, uint16_t *phwYpos)
{
	*phwXpos = xpt2046_read_average(0x90);
	*phwYpos = xpt2046_read_average(0xD0);
}

/******************************************************************************
function :	read lcd x and y axis twice
parameter:
  phwXpos: point to x axis
  phwYpos: point to y axis
******************************************************************************/
#define ERR_RANGE 50
uint8_t xpt2046_twice_read_xy(uint16_t *phwXpos, uint16_t *phwYpos)
{
	uint16_t hwXpos1, hwYpos1, hwXpos2, hwYpos2;

	xpt2046_read_xy(&hwXpos1, &hwYpos1);
	xpt2046_read_xy(&hwXpos2, &hwYpos2);

	if (((hwXpos2 <= hwXpos1 && hwXpos1 < hwXpos2 + ERR_RANGE) || (hwXpos1 <= hwXpos2 && hwXpos2 < hwXpos1 + ERR_RANGE))
	&& ((hwYpos2 <= hwYpos1 && hwYpos1 < hwYpos2 + ERR_RANGE) || (hwYpos1 <= hwYpos2 && hwYpos2 < hwYpos1 + ERR_RANGE))) {
		*phwXpos = (hwXpos1 + hwXpos2) >> 1;
		*phwYpos = (hwYpos1 + hwYpos2) >> 1;
		return 1;
	}

	return 0;
}


