#ifndef _LED_CONTROLLER_H_
#define _LED_CONTROLLER_H_

#define VERSION                 "LM20211123_002A"

#define LED_PANEL_COUNT         8   //1 pico with 8 port
#define COLOR_CHANNEL           3   //RGB channel
#define PANEL_WIDTH             80
#define PANEL_HEIGHT            24

//uint8_t led_rgb_buf[8][COLOR_CHANNEL*PANEL_WIDTH*PANEL_HEIGHT] = {0};
// double buffer for sync frame
uint8_t led_rgb_buf[3][8][COLOR_CHANNEL*PANEL_WIDTH*PANEL_HEIGHT] = {0};

unsigned int data_offset = 0;
uint8_t panel_id = -1;
int rgb_buf_write_idx = 0;
int rgb_buf_read_idx = 0;
#endif
