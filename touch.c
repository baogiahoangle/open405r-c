/******************************************************************************
* | File        :   touch.c
* | Description :  Touch screen API for XPT2046 controller
* | Version     :  V1.0
*
* This file provides functions to initialize, calibrate, and interact with a
* resistive touch screen using the XPT2046 controller. It includes routines for
* drawing touch points, handling calibration, scanning for touch events, and
* retrieving touch coordinates. The API is designed to work with an LCD display
* for visual feedback and debugging.
*
* Function Documentation:
*
* tp_init:
*   @brief  Initialize the touch controller hardware.
*
* tp_draw_touch_point:
*   @brief  Draw a cross and circle at the specified coordinates to indicate a touch point.
*   @param  hwXpos: X-axis position.
*   @param  hwYpos: Y-axis position.
*   @param  hwColor: Color of the touch point.
*
* tp_draw_big_point:
*   @brief  Draw a larger (2x2) dot at the specified coordinates.
*   @param  hwXpos: X-axis position.
*   @param  hwYpos: Y-axis position.
*   @param  hwColor: Color of the point.
*
* tp_show_info:
*   @brief  Display calibration and touch information on the LCD.
*   @param  hwXpos0, hwYpos0 ... hwXpos3, hwYpos3: Calibration points.
*   @param  hwFac: Calibration factor.
*
* tp_scan:
*   @brief  Scan for a touch event and update touch coordinates.
*   @param  chCoordType: 0 for screen coordinates, 1 for raw ADC values.
*   @retval 1 if touch is detected, 0 otherwise.
*
* tp_adjust:
*   @brief  Calibrate the touch screen by prompting the user to touch specific points.
*
* tp_dialog:
*   @brief  Display a dialog or clear area on the LCD for touch interaction.
*
* tp_draw_board:
*   @brief  Draw on the LCD based on touch input, including a clear area.
*
* tp_get_touch_point:
*   @brief  Get the current touch coordinates if the screen is touched.
*   @param  x: Pointer to store X coordinate.
*   @param  y: Pointer to store Y coordinate.
*   @retval true if touch is detected, false otherwise.
*
******************************************************************************/
#include "touch.h"
#include "LCD_Driver.h"
#include "xpt2046.h"
#include <stdlib.h>
#include <math.h>

static tp_dev_t s_tTouch;

void tp_init(void)
{
	xpt2046_init();
}

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

void tp_draw_big_point(uint16_t hwXpos, uint16_t hwYpos, uint16_t hwColor)
{
	lcd_draw_dot(hwXpos, hwYpos, hwColor);
	lcd_draw_dot(hwXpos + 1, hwYpos, hwColor);
	lcd_draw_dot(hwXpos, hwYpos + 1, hwColor);
	lcd_draw_dot(hwXpos + 1, hwYpos + 1, hwColor);
}

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
	s_tTouch.fXfac = -0.070588f;		
	s_tTouch.iXoff = 265;
	s_tTouch.fYfac = -0.094117f;
	s_tTouch.iYoff = 355;
	return;

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

void tp_dialog(void)
{
	lcd_clear_screen(WHITE);
	lcd_display_string(LCD_WIDTH - 40, 0, (const uint8_t *)"CLEAR", 16, BLUE);
}

void tp_draw_board(void)
{
	tp_scan(0);
	if (s_tTouch.chStatus & TP_PRESS_DOWN) {
		if (s_tTouch.hwXpos < LCD_WIDTH && s_tTouch.hwYpos < LCD_HEIGHT) {
			if (s_tTouch.hwXpos > (LCD_WIDTH - 40) && s_tTouch.hwYpos < 30) {
				tp_dialog();
			} else {
				tp_draw_big_point(s_tTouch.hwXpos, s_tTouch.hwYpos, RED);
			}
		}
	}
}

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

