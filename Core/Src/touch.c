/*****************************************************************************
* | File      	:		touch.c
* | Author      :   Waveshare team
* | Function    :   Touch API
* | Info        :
*----------------
* |	This version:   V1.0
* | Date        :   2019-04-30
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
#include "touch.h"
#include "LCD_Driver.h"
#include "xpt2046.h"
#include <stdlib.h>
#include <math.h>

static tp_dev_t s_tTouch;

/******************************************************************************
function :	initial touch lcd
parameter:
  phwXpos: ponit to x axis
  phwYpos: ponit to y axis
******************************************************************************/
void tp_init(void)
{
	xpt2046_init();
}

/******************************************************************************
function :	draw a touch point on lcd 
parameter:
  phwXpos: x axis
  phwYpos: y axis
	hwColor: point color
******************************************************************************/
void tp_draw_touch_point(uint16_t hwXpos, uint16_t hwYpos, uint16_t hwColor)
{
	lcd_draw_line(hwXpos - 12, hwYpos, hwXpos + 13, hwYpos, hwColor);
	lcd_draw_line(hwXpos, hwYpos - 12, hwXpos, hwYpos + 13, hwColor);
	lcd_draw_dot(hwXpos + 1, hwYpos + 1, hwColor);
	lcd_draw_dot(hwXpos - 1, hwYpos + 1, hwColor);
	lcd_draw_dot(hwXpos + 1, hwYpos - 1, hwColor);
	lcd_draw_dot(hwXpos - 1, hwYpos - 1, hwColor);
	lcd_draw_circle(hwXpos, hwYpos, 6, hwColor);
}

/******************************************************************************
function :	draw a big touch point on lcd 
parameter:
  phwXpos: x axis
  phwYpos: y axis
	hwColor: point color
******************************************************************************/
void tp_draw_big_point(uint16_t hwXpos, uint16_t hwYpos, uint16_t hwColor)
{
	lcd_draw_dot(hwXpos, hwYpos, hwColor);
	lcd_draw_dot(hwXpos + 1, hwYpos, hwColor);
	lcd_draw_dot(hwXpos, hwYpos + 1, hwColor);
	lcd_draw_dot(hwXpos + 1, hwYpos + 1, hwColor);
}

/******************************************************************************
function :	show information on lcd
parameter:
 phwXpos0: x0 coordinate
 phwYpos0: y0 coordinate
 phwXpos1: x1 coordinate
 phwYpos1: y1 coordinate
 phwXpos2: x2 coordinate
 phwYpos2: y2 coordinate
 phwXpos3: x3 coordinate
 phwYpos3: y3 coordinate
		hwFac: fac value
******************************************************************************/
void tp_show_info(uint16_t hwXpos0, uint16_t hwYpos0,
								  uint16_t hwXpos1, uint16_t hwYpos1,
								  uint16_t hwXpos2, uint16_t hwYpos2,
								  uint16_t hwXpos3, uint16_t hwYpos3, uint16_t hwFac)
{

	lcd_display_string(40, 160, (const uint8_t *)"x1", 16, RED);
	lcd_display_string(40 + 80, 160, (const uint8_t *)"y1", 16, RED);

	lcd_display_string(40, 180, (const uint8_t *)"x2", 16, RED);
	lcd_display_string(40 + 80, 180, (const uint8_t *)"y2", 16, RED);

	lcd_display_string(40, 200, (const uint8_t *)"x3", 16, RED);
	lcd_display_string(40 + 80, 200, (const uint8_t *)"y3", 16, RED);

	lcd_display_string(40, 220, (const uint8_t *)"x4", 16, RED);
	lcd_display_string(40 + 80, 220, (const uint8_t *)"y4", 16, RED);

	lcd_display_string(40, 240, (const uint8_t *)"fac is:", 16, RED);

	lcd_display_num(40 + 24, 160, hwXpos0, 4, 16, RED);
	lcd_display_num(40 + 24 + 80, 160, hwYpos0, 4, 16, RED);

	lcd_display_num(40 + 24, 180, hwXpos1, 4, 16, RED);
	lcd_display_num(40 + 24 + 80, 180, hwYpos1, 4, 16, RED);

	lcd_display_num(40 + 24, 200, hwXpos2, 4, 16, RED);
	lcd_display_num(40 + 24 + 80, 200, hwYpos2, 4, 16, RED);

	lcd_display_num(40 + 24, 220, hwXpos3, 4, 16, RED);
	lcd_display_num(40 + 24 + 80, 220, hwYpos3, 4, 16, RED);

	lcd_display_num(40 + 56, 240, hwFac, 3, 16, RED);
}

/******************************************************************************
	function :	scan touch
	parameter:
chCoordType:
******************************************************************************/
//uint8_t tp_scan(uint8_t chCoordType)
//{
//	if (!(XPT2046_IRQ_READ())) {
//		if (chCoordType) {
//			xpt2046_twice_read_xy(&s_tTouch.hwXpos, &s_tTouch.hwYpos);
//		} else if (xpt2046_twice_read_xy(&s_tTouch.hwXpos, &s_tTouch.hwYpos)) {
//			s_tTouch.hwXpos = s_tTouch.fXfac * abs(s_tTouch.hwXpos) + s_tTouch.iXoff;
//			s_tTouch.hwYpos = s_tTouch.fYfac * abs(s_tTouch.hwYpos) + s_tTouch.iYoff;
//		}
//		if (0 == (s_tTouch.chStatus & TP_PRESS_DOWN)) {
//			s_tTouch.chStatus = TP_PRESS_DOWN | TP_PRESSED;
//			s_tTouch.hwXpos0 = s_tTouch.hwXpos;
//			s_tTouch.hwYpos0 = s_tTouch.hwYpos;
//		}
//	} else {
//		if (s_tTouch.chStatus & TP_PRESS_DOWN) {
//			s_tTouch.chStatus &= ~(1 << 7);
//		} else {
//			s_tTouch.hwXpos0 = 0;
//			s_tTouch.hwYpos0 = 0;
//			s_tTouch.hwXpos = 0xffff;
//			s_tTouch.hwYpos = 0xffff;
//		}
//	}
//
//	return (s_tTouch.chStatus & TP_PRESS_DOWN);
//}

uint8_t tp_scan(uint8_t chCoordType)
{
    if (!(XPT2046_IRQ_READ())) {
        uint16_t raw_x, raw_y;
        if (xpt2046_twice_read_xy(&raw_x, &raw_y)) {
            if (raw_x > 4096 || raw_y > 4096) { // Thêm lọc giá trị thô
                s_tTouch.chStatus &= ~(1 << 7);
                s_tTouch.hwXpos = s_tTouch.hwYpos = 0xffff;
                return 0;
            }
            s_tTouch.hwXpos = raw_x;
            s_tTouch.hwYpos = raw_y;
            if (!chCoordType) {
            	// Print adc value
//            	tp_dialog();
//            	lcd_display_num(50, 30, s_tTouch.hwXpos, 4, FONT_1608, BLUE);
//            	lcd_display_num(100, 30, s_tTouch.hwYpos, 4, FONT_1608, BLUE);

                s_tTouch.hwXpos = s_tTouch.fXfac * raw_y + s_tTouch.iXoff;
                s_tTouch.hwYpos = s_tTouch.fYfac * raw_x + s_tTouch.iYoff;
            }
            if (0 == (s_tTouch.chStatus & TP_PRESS_DOWN)) {
                s_tTouch.chStatus = TP_PRESS_DOWN | TP_PRESSED;
                s_tTouch.hwXpos0 = s_tTouch.hwXpos;
                s_tTouch.hwYpos0 = s_tTouch.hwYpos;
            }
        }
    } else {
        if (s_tTouch.chStatus & TP_PRESS_DOWN) {
            s_tTouch.chStatus &= ~(1 << 7);
        } else {
            s_tTouch.hwXpos0 = 0;
            s_tTouch.hwYpos0 = 0;
            s_tTouch.hwXpos = 0xffff;
            s_tTouch.hwYpos = 0xffff;
        }
    }
    return (s_tTouch.chStatus & TP_PRESS_DOWN);
}

/******************************************************************************
function :	adjust touch lcd
parameter:
******************************************************************************/
void tp_adjust(void)
{	
	uint8_t  cnt = 0;
	uint16_t hwTimeout = 0, d1, d2, pos_temp[4][2];
	uint32_t tem1, tem2;
	float fac;				

	lcd_clear_screen(WHITE);
	lcd_display_string(40, 40, (const uint8_t *)"Please use the stylus click the cross on the screen. The cross will always move until the screen adjustment is completed.",
					16, RED);
	tp_draw_touch_point(20, 20, RED);
	s_tTouch.chStatus = 0;
	s_tTouch.fXfac = 0;
	// ***********Hard code assign*****************************
//	s_tTouch.fYfac = -0.078144f; 			//   Dung
//	s_tTouch.iYoff = 320;
//	s_tTouch.fXfac = -0.058608f;
//	s_tTouch.iXoff = 240;
	s_tTouch.fXfac = -0.070588f;		// Finalllllllllll
	s_tTouch.iXoff = 265;
	s_tTouch.fYfac = -0.094117f;
	s_tTouch.iYoff = 355;
//	s_tTouch.fXfac = -0.06122f;				// Final
//	s_tTouch.iXoff = 240;
//	s_tTouch.fYfac = -0.08816f;

	return;
	// ********************************************************

	while (1) {
		tp_scan(1);
		if((s_tTouch.chStatus & 0xC0) == TP_PRESSED) {	
			hwTimeout = 0;
			s_tTouch.chStatus &= ~(1 << 6);
						   			   
			pos_temp[cnt][0] = s_tTouch.hwXpos;
			pos_temp[cnt][1] = s_tTouch.hwYpos;
			cnt ++;	  
			switch(cnt) {			   
				case 1:						 
					tp_draw_touch_point(20, 20, WHITE);
					tp_draw_touch_point(LCD_WIDTH - 20, 20, RED);
					break;
				case 2:
					tp_draw_touch_point(LCD_WIDTH - 20, 20, WHITE);
					tp_draw_touch_point(20, LCD_HEIGHT - 20, RED);
					break;
				case 3:
					tp_draw_touch_point(20, LCD_HEIGHT - 20, WHITE);
					tp_draw_touch_point(LCD_WIDTH - 20, LCD_HEIGHT - 20, RED);
					break;
				case 4:	
					tem1=abs((int16_t)(pos_temp[0][0]-pos_temp[1][0]));//x1-x2
					tem2=abs((int16_t)(pos_temp[0][1]-pos_temp[1][1]));//y1-y2
					tem1*=tem1;
					tem2*=tem2;
					tem1+=tem2;
					d1=sqrt(tem1);

					tem1=abs((int16_t)(pos_temp[2][0]-pos_temp[3][0]));//x3-x4
					tem2=abs((int16_t)(pos_temp[2][1]-pos_temp[3][1]));//y3-y4
					tem1*=tem1;
					tem2*=tem2;
					tem1+=tem2;
					d2=sqrt(tem1);
					fac=(float)d1/d2;
					if(fac<0.85||fac>1.15||d1==0||d2==0) {
						cnt=0;
 						tp_show_info(pos_temp[0][0],pos_temp[0][1],pos_temp[1][0],pos_temp[1][1],pos_temp[2][0],pos_temp[2][1],pos_temp[3][0],pos_temp[3][1],(uint16_t)fac*100);
						delay_ms(1000);
						lcd_fill_rect(96, 240, 24, 16, WHITE);
						tp_draw_touch_point(LCD_WIDTH - 20, LCD_HEIGHT - 20, WHITE);
						tp_draw_touch_point(20, 20, RED);
						continue;
					}

					tem1=abs((int16_t)(pos_temp[0][0]-pos_temp[2][0]));//x1-x3
					tem2=abs((int16_t)(pos_temp[0][1]-pos_temp[2][1]));//y1-y3
					tem1*=tem1;
					tem2*=tem2;
					tem1+=tem2;
					d1=sqrt(tem1);//

					tem1=abs((int16_t)(pos_temp[1][0]-pos_temp[3][0]));//x2-x4
					tem2=abs((int16_t)(pos_temp[1][1]-pos_temp[3][1]));//y2-y4
					tem1*=tem1;
					tem2*=tem2;
					tem1+=tem2;
					d2=sqrt(tem1);//
					fac=(float)d1/d2;
					if(fac<0.85||fac>1.15) {
						cnt=0;
 						tp_show_info(pos_temp[0][0],pos_temp[0][1],pos_temp[1][0],pos_temp[1][1],pos_temp[2][0],pos_temp[2][1],pos_temp[3][0],pos_temp[3][1],(uint16_t)fac*100);//??��o?��oy?Y
						delay_ms(1000);
						lcd_fill_rect(96, 240, 24, 16, WHITE);
						tp_draw_touch_point(LCD_WIDTH - 20, LCD_HEIGHT - 20, WHITE);
						tp_draw_touch_point(20, 20, RED);
						continue;
					}//
								   
					tem1=abs((int16_t)(pos_temp[1][0]-pos_temp[2][0]));//x2-x3
					tem2=abs((int16_t)(pos_temp[1][1]-pos_temp[2][1]));//y2-y3
					tem1*=tem1;
					tem2*=tem2;
					tem1+=tem2;
					d1=sqrt(tem1);//

					tem1=abs((int16_t)(pos_temp[0][0]-pos_temp[3][0]));//x1-x4
					tem2=abs((int16_t)(pos_temp[0][1]-pos_temp[3][1]));//y1-y4
					tem1*=tem1;
					tem2*=tem2;
					tem1+=tem2;
					d2=sqrt(tem1);//
					fac=(float)d1/d2;
					if(fac<0.85||fac>1.15) {
						cnt=0;	
 						tp_show_info(pos_temp[0][0],pos_temp[0][1],pos_temp[1][0],pos_temp[1][1],pos_temp[2][0],pos_temp[2][1],pos_temp[3][0],pos_temp[3][1],(uint16_t)fac*100);//??��o?��oy?Y
						delay_ms(1000);
						lcd_fill_rect(96, 240, 24, 16, WHITE);
						tp_draw_touch_point(LCD_WIDTH - 20, LCD_HEIGHT - 20, WHITE);
						tp_draw_touch_point(20, 20, RED);
						continue;
					}

					s_tTouch.fXfac = (float)(LCD_WIDTH - 40) / (int16_t)(pos_temp[1][0] - pos_temp[0][0]);	
					s_tTouch.iXoff = (LCD_WIDTH - s_tTouch.fXfac * (pos_temp[1][0] + pos_temp[0][0])) / 2;

					s_tTouch.fYfac = (float)(LCD_HEIGHT - 40) / (int16_t)(pos_temp[2][1] - pos_temp[0][1]);	  
					s_tTouch.iYoff = (LCD_HEIGHT - s_tTouch.fYfac * (pos_temp[2][1] + pos_temp[0][1])) / 2;

					if(abs(s_tTouch.fXfac) > 2 || abs(s_tTouch.fYfac) > 2) {
						cnt=0;
 				    	tp_draw_touch_point(LCD_WIDTH - 20, LCD_HEIGHT - 20, WHITE);
						tp_draw_touch_point(20, 20, RED);								
						lcd_display_string(40, 26, (const uint8_t *)"TP Need readjust!", 16, RED);
						continue;
					}
					lcd_clear_screen(WHITE);
					lcd_display_string(35, 110, (const uint8_t *)"Touch Screen Adjust OK!", 16, BLUE);
					delay_ms(1000); 
 					lcd_clear_screen(WHITE);
 					// ******************* Print fXfac, iXoff, fYfac, iYOff ********************
 					lcd_display_num(220, 20, s_tTouch.fXfac, 4, FONT_1206, RED);
					lcd_display_num(220, 70, s_tTouch.iXoff, 4, FONT_1206, RED);
					lcd_display_num(250, 20, s_tTouch.fYfac, 4, FONT_1206, RED);
					lcd_display_num(250, 70, s_tTouch.iYoff, 4,FONT_1206, RED);
					// *************************************************************************

					return;				 
			}
		}
		delay_ms(10);
		if (++ hwTimeout >= 5000) {
			break;
		}
 	}
}

/******************************************************************************
function :	dialog touch lcd
parameter:
******************************************************************************/
void tp_dialog(void)
{
	lcd_clear_screen(WHITE);
	lcd_display_string(LCD_WIDTH - 40, 0, (const uint8_t *)"CLEAR", 16, BLUE);
}

void tp_draw_board(void)
{
	tp_scan(0);
	if (s_tTouch.chStatus & TP_PRESS_DOWN) {
		//tp_dialog();
//		lcd_display_num(50, 50, s_tTouch.hwXpos, 3, FONT_1608, RED);
//		lcd_display_num(100, 50, s_tTouch.hwYpos, 3, FONT_1608, RED);
		if (s_tTouch.hwXpos < LCD_WIDTH && s_tTouch.hwYpos < LCD_HEIGHT) {
			if (s_tTouch.hwXpos > (LCD_WIDTH - 40) && s_tTouch.hwYpos < 30) {
				tp_dialog();
			} else {
//				tp_draw_touch_point(s_tTouch.hwXpos, s_tTouch.hwYpos, RED);
				tp_draw_big_point(s_tTouch.hwXpos, s_tTouch.hwYpos, RED);
			}
		}
	}
}

/****************************************************************************
* Function Name: tp_get_touch_point
* Description: Gets touch coordinates if screen is touched
* Parameters:
*   x: Pointer to store X coordinate
*   y: Pointer to store Y coordinate
* Return:
*   true: If screen is touched
*   false: If screen is not touched
****************************************************************************/
bool tp_get_touch_point(uint16_t *x, uint16_t *y)
{
    // Scan for touch event
    if (tp_scan(0)) {
        // Get coordinates from touch structure
        *x = s_tTouch.hwXpos;
        *y = s_tTouch.hwYpos;
        return true;
    }
    return false;
}

