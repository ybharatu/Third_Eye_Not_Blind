/**
 * Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_gfx.h"

#include "nrf_lcd.h"
#include "nrf_drv_spi.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrfx_timer.h"

#include "app_pwm.h"
#include "bsp.h"
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"

#include "nrf_drv_gpiote.h"
#include "nrf_saadc.h"
#include "nrfx_saadc.h"

#include "nrf_drv_saadc.h"
#include "app_util_platform.h"
//#include "nrf_pwr_mgmt.h"


#define GRAY 0xC618
//#define BLUE 255
#define RED             0xF800
#define BLUE            0x001F
#define SPI_MOSI 
#define BUFFER_SIZE     60
#define LEFT 1
#define RIGHT 2
#define SENSE_PIN 3

#define TEST_STRING "Nordic"
#ifdef LVEZ4_AN_PIN
    #define PIN_IN LVEZ4_AN_PIN
#endif
#ifndef PIN_IN
    #error "Please indicate input pin"
#endif

#ifdef LVEZ4_AN_PIN
    #define PIN_OUT LVEZ4_AN_PIN
#endif
#ifndef PIN_OUT
    #error "Please indicate output pin"
#endif

#define SAMPLES_IN_BUFFER 1 

static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */
char * close = "CLOSE:    ";
char * far = "FAR:    ";
const char * text = "";

extern const nrf_gfx_font_desc_t orkney_24ptFontInfo;
extern nrf_lcd_t nrf_lcd_ili9341;
nrf_lcd_t* lcd = &nrf_lcd_ili9341;
static const nrf_gfx_font_desc_t * p_font = &orkney_24ptFontInfo;

const nrf_drv_timer_t TIMER_LED = NRF_DRV_TIMER_INSTANCE(1);
uint32_t seconds = 0;

uint8_t i = 0;
uint8_t rect_status = 0;
uint8_t object_close = 0;
uint8_t draw_right = 0;
uint8_t draw_left = 0;
uint8_t draw_text = 0;
uint8_t curr_drift = 0;
uint32_t distance = 0;
uint32_t atd_result = 0;
nrf_saadc_value_t p_value;
static nrf_saadc_value_t m_buffer[SAMPLES_IN_BUFFER];
//static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];

static uint8_t       drifting_values[BUFFER_SIZE] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2};
//static uint8_t       drifting_values[BUFFER_SIZE] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2};
//static uint8_t       drifting_values[15] = {0,0,0,1,1,1,0,0,0,2,2,2,0,0,0};
nrf_gfx_rect_t test_rect = NRF_GFX_RECT(10,100,20,150);

nrfx_timer_t timer_us = NRFX_TIMER_INSTANCE(1);

nrfx_timer_event_handler_t hc_sr04_measure(void);
nrfx_timer_event_handler_t LVEZ4_measure(void);

static void screen_clear(void)
{ 
    nrf_gfx_screen_fill(lcd, GRAY);
}

APP_PWM_INSTANCE(PWM1,3);                   // Create the instance "PWM1" using TIMER2.

static volatile bool ready_flag;            // A flag indicating PWM status.

void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    ready_flag = true;
}

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
//    if(nrf_gpio_pin_read(ECHO_PIN)){
//        NRF_TIMER2->TASKS_START = 1;
//    }

//    if(distance < 300){
//      if(text != close){
//          draw_text = 1;       
//      }
//      text = close;
//      object_close = 1;
//    }else{
//      if(text != far){
//          draw_text = 1;
//      }
//      text = far;
//      object_close = 0;
//    }
//
//    // EMULATING RASPBERRY PI
//    if(drifting_values[i] == 1 && rect_status != 1){
//      draw_right = 1;
//      draw_left = 0;
//      rect_status = 1;
//    }
//    else if(drifting_values[i] == 2 && rect_status != 2){
//      draw_right = 0;
//      draw_left = 1;
//      rect_status = 2;
//    }
//    else if(drifting_values[i] == 0){
//      draw_right = 0;
//      draw_left = 0;
//      rect_status = 0;
//    }
//    i = (i + 1) % BUFFER_SIZE;
//    nrfx_timer_clear(&timer_us);
   
}

void drifting_gpio_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
     if(nrf_gpio_pin_read(DRIFT_LEFT_PIN)){
        curr_drift = LEFT;
     }
     if(nrf_gpio_pin_read(DRIFT_RIGHT_PIN)){
        curr_drift = RIGHT;
     }
    return;
}

static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    err_code = nrf_drv_gpiote_out_init(PIN_OUT, &out_config);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

//    err_code = nrf_drv_gpiote_in_init(PIN_IN, &in_config, in_pin_handler);
//    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_in_init(DRIFT_LEFT_PIN, &in_config, drifting_gpio_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_in_init(DRIFT_RIGHT_PIN, &in_config, drifting_gpio_handler);
    APP_ERROR_CHECK(err_code);

    //nrf_gpio_cfg_input(PIN_IN, NRF_GPIO_PIN_NOPULL);

    //nrf_drv_gpiote_in_event_enable(PIN_IN, true);
}

void hc_sr04_init(void){
	nrf_gpio_cfg_output(TRIGGER_PIN);
	nrf_gpio_cfg_input(ECHO_PIN,NRF_GPIO_PIN_PULLUP);
	NRF_TIMER2->TASKS_STOP = 1;
	NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;
	NRF_TIMER2->PRESCALER = 0;
	NRF_TIMER2->BITMODE = 3;
	NRF_TIMER2->TASKS_CLEAR = 1;

}

void LVEZ4_init(void){
	NRF_TIMER2->TASKS_STOP = 1;
	NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;
	NRF_TIMER2->PRESCALER = 0;
	NRF_TIMER2->BITMODE = 3;
	NRF_TIMER2->TASKS_CLEAR = 1;
}

nrfx_timer_event_handler_t hc_sr04_measure(void){
      
//    while(!nrf_gpio_pin_read(ECHO_PIN)){}
//    NRF_TIMER2->TASKS_START = 1;
//    while(nrf_gpio_pin_read(ECHO_PIN)){}
//    NRF_TIMER2->TASKS_STOP = 1;
}

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);
        p_value = p_event->data.done.p_buffer[0];
        distance = p_value;
        //distance = p_value / 256;
        //distance = (1.0 / ((double)distance)) * ((double) VCC / 512.0);
        NRF_LOG_INFO("%d\r\n", p_event->data.done.p_buffer[0]);
    }
}

void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config 
        = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
    
    channel_config.gain = NRF_SAADC_GAIN1;

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer, SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}



nrfx_timer_event_handler_t LVEZ4_measure(void){
    int prev_p_value = 0;
    int flag = 0;
//    nrf_drv_saadc_sample();
//    nrf_drv_saadc_sample_convert(0,&p_value);
     
    if(p_value != prev_p_value){
      prev_p_value = p_value;
      flag = 1;
    }
    prev_p_value = p_value;
//    distance = (1.0 / ((double)p_value)) * ((double) VCC / 512.0);

    if(distance < 300){
      if(text != close){
          draw_text = 1;       
      }
      text = close;
      object_close = 1;
    }else{
      if(text != far){
          draw_text = 1;
      }
      text = far;
      object_close = 0;
    }

    // EMULATING RASPBERRY PI
    if(drifting_values[i] == 1 && rect_status != 1){
      draw_right = 1;
      draw_left = 0;
      rect_status = 1;
    }
    else if(drifting_values[i] == 2 && rect_status != 2){
      draw_right = 0;
      draw_left = 1;
      rect_status = 2;
    }
    else if(drifting_values[i] == 0){
      draw_right = 0;
      draw_left = 0;
      rect_status = 0;
    }
    i = (i + 1) % BUFFER_SIZE;
    nrfx_timer_clear(&timer_us);
   
    return;
}

void simple_timer_init(void){
    const nrfx_timer_config_t timer_config = {
    .frequency = NRF_TIMER_FREQ_31250Hz,
    .bit_width = NRF_TIMER_BIT_WIDTH_24,
    .interrupt_priority = 2,
    .mode = NRF_TIMER_MODE_TIMER,
    .p_context = NULL
    };

    nrfx_timer_init(&timer_us, &timer_config, LVEZ4_measure);
    nrfx_timer_compare(&timer_us, 0, 1563, true);

}
 
int main(void)
{
    
    bsp_board_init(BSP_INIT_LEDS);
    const char * test_text = "nRF52 family";
    
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    
    APP_ERROR_CHECK(nrf_gfx_init(lcd));
    //hc_sr04_init();
    LVEZ4_init();
    simple_timer_init();
    //gpio_init();
    screen_clear();
    saadc_init();

    

    nrfx_timer_enable(&timer_us);
    //nrfx_timer_clear(&timer_us);
    //nrfx_timer_compare_int_enable(&timer_us,0);


    NRF_TIMER1->TASKS_START = 1;

    //////////////////////////////////////////////////////////////////////////////
    // TO DO: Create PWM Init() function
    //////////////////////////////////////////////////////////////////////////////
  
    ret_code_t err_code;
    
    /* 2-channel PWM, 200Hz, output on DK LED pins. */
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_2CH(5000L, 23, 24);

    /* Switch the polarity of the second channel. */
    pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;

    /* Initialize and enable PWM. */
    err_code = app_pwm_init(&PWM1,&pwm1_cfg,pwm_ready_callback);
    APP_ERROR_CHECK(err_code);
    app_pwm_enable(&PWM1);
    
    nrf_gfx_rect_t wipe_text = NRF_GFX_RECT(5, nrf_gfx_height_get(lcd) - 50, nrf_gfx_width_get(lcd), 40);
    uint32_t value;

    while (true)
    {
        nrf_drv_saadc_sample();
        if(object_close){
          for (uint8_t i = 0; i < 40; ++i)
          {
              value = (i < 20) ? (i * 5) : (100 - (i - 20) * 5);

              ready_flag = false;
              /* Set the duty cycle - keep trying until PWM is ready... */
              while (app_pwm_channel_duty_set(&PWM1, 0, 50) == NRF_ERROR_BUSY);

              /* ... or wait for callback. */
              //while (!ready_flag);
              //APP_ERROR_CHECK(app_pwm_channel_duty_set(&PWM1, 1, value));
              nrf_delay_ms(25);
          }
        }
        else{
          app_pwm_channel_duty_set(&PWM1, 0, 0);
        }
        if(draw_text){
                    draw_text = 0;
            nrf_gfx_rect_draw(lcd,&wipe_text,10,GRAY,true);
            nrf_gfx_point_t text_start = NRF_GFX_POINT(5, nrf_gfx_height_get(lcd) - 50);
            APP_ERROR_CHECK(nrf_gfx_print(lcd, &text_start, 0, text, p_font, true));

            //APP_ERROR_CHECK(nrf_gfx_print(lcd, &text_start, 0, text, p_font, true));
        }
        if(draw_right){
          draw_right = 0;
          nrf_gfx_rect_draw(lcd, &test_rect, 10, GRAY, true);
          test_rect.x = 10;
          test_rect.y = 100;
          nrf_gfx_rect_draw(lcd, &test_rect, 10, RED, true);
        }
        else if(draw_left){
                  draw_left = 0;
          nrf_gfx_rect_draw(lcd, &test_rect, 10, GRAY, true);
          test_rect.x = ILI9341_WIDTH - 30;
          test_rect.y = 100;
          nrf_gfx_rect_draw(lcd, &test_rect, 10, BLUE, true);

        }
        else if(rect_status == 0){
          nrf_gfx_rect_draw(lcd, &test_rect, 10, GRAY, true);
        }
        nrf_delay_ms(50);
    }

}