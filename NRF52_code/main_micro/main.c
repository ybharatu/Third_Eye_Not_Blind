/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
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
/**
 * @brief BLE LED Button Service central and client application main file.
 *
 * This file contains the source code for a sample client application using the LED Button service.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_pwr_mgmt.h"
#include "app_timer.h"
#include "boards.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_lbs_c.h"
#include "nrf_ble_scan.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"
#include "nrf_saadc.h"
#include "nrfx_saadc.h"
#include "nrfx_timer.h"

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_state.h"
#include "boards.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_lbs.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

///////
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include <string.h>

//Gfx includes
#include "nrf_gfx.h"
#include "nrf_lcd.h"

#include "nrfx_timer.h"
#include "nrfx_timer.h"

#include "app_pwm.h"
#include "bsp.h"
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"

#include "nrfx_gpiote.h"

#include "app_util_platform.h"

#define CENTRAL_SCANNING_LED            BSP_BOARD_LED_0                     /**< Scanning LED will be on when the device is scanning. */
#define CENTRAL_CONNECTED_LED_1         BSP_BOARD_LED_1                     /**< Connected LED will be on when the device is connected. */
#define CENTRAL_CONNECTED_LED_2         BSP_BOARD_LED_3
#define LEDBUTTON_LED                   BSP_BOARD_LED_2                     /**< LED to indicate a change of state of the the Button characteristic on the peer. */

#define SCAN_INTERVAL                   0x00A0                              /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                     0x0050                              /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_DURATION                   0x0000                              /**< Timout when scanning. 0x0000 disables timeout. */

#define MIN_CONNECTION_INTERVAL         MSEC_TO_UNITS(7.5, UNIT_1_25_MS)    /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL         MSEC_TO_UNITS(30, UNIT_1_25_MS)     /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY                   0                                   /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT             MSEC_TO_UNITS(4000, UNIT_10_MS)     /**< Determines supervision time-out in units of 10 milliseconds. */

#define LEDBUTTON_BUTTON_PIN            BSP_BUTTON_0                        /**< Button that will write to the LED characteristic of the peer */
#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                 /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define APP_BLE_CONN_CFG_TAG            1                                   /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO           3                                   /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define SAMPLES_IN_BUFFER               1 

#define DRIFT_LEFT_PIN                  13
#define DRIFT_RIGHT_PIN                 12
#define ERROR_PIN                       11
#define BUFFER_SIZE                     60
#define CENTER                          0
#define LEFT                            1
#define RIGHT                           2

#define GRAY                            0xC618
#define RED                             0xF800
#define BLUE                            0x001F
#define BLACK                           0x0000

#define VCC 3.3

#define TEST_LED                        22

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                64                                      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */


#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**<  latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000)                  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

NRF_BLE_SCAN_DEF(m_scan);                                       /**< Scanning module instance. */
//BLE_LBS_C_DEF(m_ble_lbs_c);                                     /**< Main structure used by the LBS client module. */
NRF_BLE_GATT_DEF(m_gatt);                                       /**< GATT module instance. */
//BLE_DB_DISCOVERY_DEF(m_db_disc);                                /**< DB discovery module instance. */
BLE_LBS_C_ARRAY_DEF(m_ble_lbs_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);           /**< LED button client instances. */
BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc, NRF_SDH_BLE_CENTRAL_LINK_COUNT);  /**< Database discovery module instances. */

static char const m_target_periph_name[] = "Yash_Blinky";     /**< Name of the device we try to connect to. This name is searched in the scan report data*/

static nrf_saadc_value_t m_buffer[SAMPLES_IN_BUFFER];

uint8_t i = 0;
uint8_t rect_status = 0;
uint8_t object_close = 0;
uint8_t draw_right = 0;
uint8_t draw_left = 0;
uint8_t draw_text = 0;
uint8_t curr_drift = 0;
uint32_t distance = 0;
uint32_t atd_result = 0;
uint32_t drift_error = 0;

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */

nrf_gfx_rect_t test_rect = NRF_GFX_RECT(10,100,20,150);

nrfx_timer_t timer_us = NRFX_TIMER_INSTANCE(2);

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

char * close = "CLOSE:    ";
char * far = "FAR:    ";
const char * text = "";

extern const nrf_gfx_font_desc_t orkney_24ptFontInfo;
extern nrf_lcd_t nrf_lcd_ili9341;
nrf_lcd_t* lcd = &nrf_lcd_ili9341;
static const nrf_gfx_font_desc_t * p_font = &orkney_24ptFontInfo;

/**@brief Function to handle asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**@brief Function to start scanning.
 */
static void scan_start(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);

    bsp_board_led_off(CENTRAL_CONNECTED_LED_1);
//    nrf_gpio_pin_write(CENTRAL_CONNECTED_LED,0);
    bsp_board_led_on(CENTRAL_SCANNING_LED);
//    nrf_gpio_pin_write(CENTRAL_SCANNING_LED,0);
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;
    int value;

    // For readability.
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    if(ble_conn_state_central_conn_count() == NRF_SDH_BLE_CENTRAL_LINK_COUNT){
       bsp_board_led_on(CENTRAL_CONNECTED_LED_1);
       bsp_board_led_on(CENTRAL_CONNECTED_LED_2);
    }
    else if(ble_conn_state_central_conn_count() == 1){
       bsp_board_led_on(CENTRAL_CONNECTED_LED_1);
       bsp_board_led_off(CENTRAL_CONNECTED_LED_2);
    }
    else{
       bsp_board_led_off(CENTRAL_CONNECTED_LED_1);
       bsp_board_led_off(CENTRAL_CONNECTED_LED_2);
    }

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral has connected (HR or RSC), initiate DB
        // discovery, update LEDs status and resume scanning if necessary. */
        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("Connected.");
            err_code = ble_lbs_c_handles_assign(&m_ble_lbs_c[p_gap_evt->conn_handle], p_gap_evt->conn_handle, NULL);
            APP_ERROR_CHECK(err_code);

            err_code = ble_db_discovery_start(&m_db_disc[p_gap_evt->conn_handle], p_gap_evt->conn_handle);
            APP_ERROR_CHECK(err_code);
            
            if (ble_conn_state_central_conn_count() == NRF_SDH_BLE_CENTRAL_LINK_COUNT)
            {
                bsp_board_led_off(CENTRAL_SCANNING_LED);
            }
            else
            {
                // Resume scanning.
                bsp_board_led_on(CENTRAL_SCANNING_LED);
                scan_start();
            }

            // Update LEDs status, and check if we should be looking for more
            // peripherals to connect to.
//            bsp_board_led_on(CENTRAL_CONNECTED_LED);
//            nrf_gpio_pin_write(CENTRAL_CONNECTED_LED,1);
//            bsp_board_led_off(CENTRAL_SCANNING_LED);
//            nrf_gpio_pin_write(CENTRAL_SCANNING_LED,1);
        } break;

        // Upon disconnection, reset the connection handle of the peer which disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            value = ble_conn_state_central_conn_count();
            NRF_LOG_INFO("Disconnected, %d connections.", value);
            
            scan_start();
        } break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_DEBUG("Connection request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
        {
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_TIMEOUT:
        {
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    ret_code_t err_code;

    switch (pin_no)
    {
        case LEDBUTTON_BUTTON_PIN:
            err_code = ble_lbs_led_status_send(&m_ble_lbs_c, button_action);
            if (err_code != NRF_SUCCESS &&
                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            if (err_code == NRF_SUCCESS)
            {
                NRF_LOG_INFO("LBS write LED state %d", button_action);
            }
            break;

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}

/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    ret_code_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {LEDBUTTON_BUTTON_PIN, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_lbs_on_db_disc_evt(&m_ble_lbs_c[p_evt->conn_handle], p_evt);
}


/**@brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}

//GPIO Pin interrupt handler for the three pins from the raspberry pi
nrfx_gpiote_evt_handler_t drifting_gpio_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t state)
{
    NRF_LOG_INFO("GPIO INTRPT");

    int pin_val = nrf_gpio_pin_read(pin);

    if(pin == DRIFT_LEFT_PIN){
        if(pin_val == 1){
            curr_drift = LEFT;
            draw_right = 0;
            draw_left = 1;
            rect_status = 2;
        }else{
            curr_drift = CENTER;
            draw_right = 0;
            draw_left = 0;
            rect_status = 0;
        }
    }else if(pin == DRIFT_RIGHT_PIN){
        if(pin_val == 1){
            curr_drift = RIGHT;
            draw_right = 0;
            draw_left = 1;
            rect_status = 2;
        }else{
            curr_drift = CENTER;
            draw_right = 0;
            draw_left = 0;
            rect_status = 0;
        }
    }else if(pin == ERROR_PIN){
        if(pin_val == 1){
            drift_error = 1;
        }else{
            drift_error = 0;
        }
    }
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}

static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrfx_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLDOWN;

    err_code = nrfx_gpiote_in_init(DRIFT_LEFT_PIN, &in_config, (nrfx_gpiote_evt_handler_t) drifting_gpio_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_gpiote_in_init(DRIFT_RIGHT_PIN, &in_config, (nrfx_gpiote_evt_handler_t) drifting_gpio_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_gpiote_in_init(ERROR_PIN, &in_config, (nrfx_gpiote_evt_handler_t) drifting_gpio_handler);
    APP_ERROR_CHECK(err_code);

    nrf_gpio_cfg_output(TEST_LED);

    nrfx_gpiote_in_event_enable(DRIFT_LEFT_PIN, true);
    nrfx_gpiote_in_event_enable(DRIFT_RIGHT_PIN, true);
    nrfx_gpiote_in_event_enable(ERROR_PIN, true);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details Handle any pending log operation(s), then sleep until the next event occurs.
 */
static void idle_state_handle(void)
{
    NRF_LOG_FLUSH();
    nrf_pwr_mgmt_run();
}


/**@brief Handles events coming from the LED Button central module.
 */
static void lbs_c_evt_handler(ble_lbs_c_t * p_lbs_c, ble_lbs_c_evt_t * p_lbs_c_evt)
{
    switch (p_lbs_c_evt->evt_type)
    {
        case BLE_LBS_C_EVT_DISCOVERY_COMPLETE:
        {
            ret_code_t err_code;

            err_code = ble_lbs_c_handles_assign(&m_ble_lbs_c,
                                                p_lbs_c_evt->conn_handle,
                                                &p_lbs_c_evt->params.peer_db);
            NRF_LOG_INFO("LED Button service discovered on conn_handle 0x%x.", p_lbs_c_evt->conn_handle);

//            err_code = app_button_enable();
//            APP_ERROR_CHECK(err_code);

            // LED Button service discovered. Enable notification of Button.
            err_code = ble_lbs_c_button_notif_enable(p_lbs_c);
            APP_ERROR_CHECK(err_code);
        } break; // BLE_LBS_C_EVT_DISCOVERY_COMPLETE

        case BLE_LBS_C_EVT_BUTTON_NOTIFICATION:
        {
            draw_text = 1;
            NRF_LOG_INFO("Button state changed on peer to 0x%x.", p_lbs_c_evt->params.button.button_state);
//            if (p_lbs_c_evt->params.button.button_state)
//            {
////                bsp_board_led_on(LEDBUTTON_LED);
//                nrf_gpio_pin_write(LEDBUTTON_LED,0);
//            }
//            else
//            {
////                bsp_board_led_off(LEDBUTTON_LED);
//                nrf_gpio_pin_write(LEDBUTTON_LED,1);
//            }
              /************************************************
              * Actions if received "CLOSE" message from
              * peripheral micro:
              * 1. Change text to "Close"
              * 2. Change LED Status
              ************************************************/
              if (p_lbs_c_evt->params.button.button_state)
              {
                  text = close;
                  nrf_gpio_pin_write(TEST_LED,1);
                  NRF_LOG_INFO("Received LED ON!");
              }
              /************************************************
              * Actions if received "FAR" message from
              * peripheral micro:
              * 1. Change text to "Far"
              * 2. Change LED Status
              ************************************************/
              else
              {
                  text = far;
                  nrf_gpio_pin_write(TEST_LED,0);
                  NRF_LOG_INFO("Received LED OFF!");
              }
        } break; // BLE_LBS_C_EVT_BUTTON_NOTIFICATION

        default:
            // No implementation needed.
            break;
    }
}

/**@brief LED Button client initialization.
 */
static void lbs_c_init(void)
{
    ret_code_t       err_code;
    ble_lbs_c_init_t lbs_c_init_obj;

    lbs_c_init_obj.evt_handler = lbs_c_evt_handler;

//    err_code = ble_lbs_c_init(&m_ble_lbs_c, &lbs_c_init_obj);
//    APP_ERROR_CHECK(err_code);
    for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        err_code = ble_lbs_c_init(&m_ble_lbs_c[i], &lbs_c_init_obj);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);
//    nrf_gpio_cfg_output(LEDBUTTON_LED);
//    nrf_gpio_cfg_output(CENTRAL_CONNECTED_LED);
//    nrf_gpio_cfg_output(CENTRAL_SCANNING_LED);

//    nrf_gpio_pin_write(CENTRAL_CONNECTED_LED,0);
//    nrf_gpio_pin_write(CENTRAL_SCANNING_LED,1);
}

/**@brief Function for initializing the log.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing the Power manager. */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Scaning events.
 *
 * @param[in]   p_scan_evt   Scanning event.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
            err_code = p_scan_evt->params.connecting_err.err_code;
            APP_ERROR_CHECK(err_code);
            break;
        default:
          break;
    }
}

/**@brief Function for handling write events to the LED characteristic.
 *
 * @param[in] p_lbs     Instance of LED Button Service to which the write applies.
 * @param[in] led_state Written/desired state of the LED.
 */
static void led_write_handler(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t led_state)
{
    draw_text = 1;
    /************************************************
    * Actions if received "CLOSE" message from
    * peripheral micro:
    * 1. Change text to "Close"
    * 2. Change LED Status
    ************************************************/
    if (led_state)
    {
        text = close;
        nrf_gpio_pin_write(TEST_LED,1);
        NRF_LOG_INFO("Received LED ON!");
    }
    /************************************************
    * Actions if received "FAR" message from
    * peripheral micro:
    * 1. Change text to "Far"
    * 2. Change LED Status
    ************************************************/
    else
    {
        text = far;
        nrf_gpio_pin_write(TEST_LED,0);
        NRF_LOG_INFO("Received LED OFF!");
    }
}

static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    // Setting filters for scanning.
    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, m_target_periph_name);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

static void screen_clear(void)
{ 
    nrf_gfx_screen_fill(lcd, GRAY);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

int main(void)
{ 
    bsp_board_init(BSP_INIT_LEDS);


//    Initialize.
    gpio_init();
    log_init();
    //leds_init();
    timers_init();
    //buttons_init();

    nrf_gfx_init(lcd);
    power_management_init();

    //BLE init
    ble_stack_init();
    gatt_init();
    db_discovery_init();
    lbs_c_init();
    ble_conn_state_init();
    scan_init();
   
    screen_clear();

    //Initialize the Screen
    nrf_gfx_rect_t wipe_text = NRF_GFX_RECT(5, nrf_gfx_height_get(lcd) - 50, nrf_gfx_width_get(lcd), 40); //Creating the wipe text rectangle
    uint32_t value;

    //Graphics init for the 2 lane lines
    int offset = 100;
    int width = 20;
    int height = 40;

    nrf_gfx_rect_t right_rect_1 = NRF_GFX_RECT(50,offset,height,width);
    nrf_gfx_rect_t right_rect_2 = NRF_GFX_RECT(100,offset,height,width);
    nrf_gfx_rect_t right_rect_3 = NRF_GFX_RECT(150,offset,height,width);

    nrf_gfx_rect_t left_rect_1 = NRF_GFX_RECT(50,320-offset-width,height,width);
    nrf_gfx_rect_t left_rect_2 = NRF_GFX_RECT(100,320-offset-width,height,width);
    nrf_gfx_rect_t left_rect_3 = NRF_GFX_RECT(150,320-offset-width,height,width);


    //Graphics Init for the bluetooth Logo
    int scale = 10;
    int start_x = 30;
    int start_y = 40;
    int thickness = 4;

    nrf_gfx_line_t blue_1 = NRF_GFX_LINE(start_x,start_y,start_x + scale*2,start_y-scale*2,thickness);
    nrf_gfx_line_t blue_2 = NRF_GFX_LINE(start_x + scale*2,start_y - scale*2,start_x + scale*3,start_y - scale*1,thickness);
    nrf_gfx_line_t blue_3 = NRF_GFX_LINE(start_x - scale*1,start_y - scale*1,start_x + scale*3 + thickness,start_y - scale*1,thickness);
    nrf_gfx_line_t blue_4 = NRF_GFX_LINE(start_x - scale*1,start_y - scale*1,start_x + scale*0,start_y - scale*2,thickness);
    nrf_gfx_line_t blue_5 = NRF_GFX_LINE(start_x + scale*0,start_y - scale*2,start_x + scale*2,start_y - scale*0,thickness);

    //Graphics for the Red Circle over the logo
    int circ_thick = 5;

    nrf_gfx_line_t line_red = NRF_GFX_LINE(start_x,start_y - scale*3,start_x + scale*2,start_y + scale,circ_thick);


    // Start execution.
    NRF_LOG_INFO("Main Code started.");
    scan_start();
    bsp_board_led_on(CENTRAL_SCANNING_LED);

    nrf_gfx_line_draw(lcd, &blue_1,BLUE);
    nrf_gfx_line_draw(lcd, &blue_2,BLUE);
    nrf_gfx_line_draw(lcd, &blue_3,BLUE);
    nrf_gfx_line_draw(lcd, &blue_4,BLUE);
    nrf_gfx_line_draw(lcd, &blue_5,BLUE);

    nrf_gfx_rect_draw(lcd, &left_rect_1,2,BLACK,false);
    nrf_gfx_rect_draw(lcd, &left_rect_2,2,BLACK,false);
    nrf_gfx_rect_draw(lcd, &left_rect_3,2,BLACK,false);

    nrf_gfx_rect_draw(lcd, &right_rect_1,2,BLACK,false);
    nrf_gfx_rect_draw(lcd, &right_rect_2,2,BLACK,false);
    nrf_gfx_rect_draw(lcd, &right_rect_3,2,BLACK,false);
    
    while (true)
    {

      if(object_close)
      {
//        for (uint8_t i = 0; i < 40; ++i)
//        {
//          value = (i < 20) ? (i * 5) : (100 - (i - 20) * 5);
//          
//        }
      }

      if(draw_text){
            draw_text = 0;
            nrf_gfx_rect_draw(lcd,&wipe_text,10,GRAY,true);
            nrf_gfx_point_t text_start = NRF_GFX_POINT(5, nrf_gfx_height_get(lcd) - 50);
            APP_ERROR_CHECK(nrf_gfx_print(lcd, &text_start, 0, text, p_font, true));
      }
      if(draw_right){
          draw_right = 0;
          nrf_gfx_rect_draw(lcd, &right_rect_1,2,RED,true);
          nrf_gfx_rect_draw(lcd, &right_rect_2,2,RED,true);
          nrf_gfx_rect_draw(lcd, &right_rect_3,2,RED,true);
      }
      else if(draw_left){
          draw_left = 0;
          nrf_gfx_rect_draw(lcd, &left_rect_1,2,RED,true);
          nrf_gfx_rect_draw(lcd, &left_rect_2,2,RED,true);
          nrf_gfx_rect_draw(lcd, &left_rect_3,2,RED,true);
      }
      else if(rect_status == 0){
          rect_status = 1;
          nrf_gfx_rect_draw(lcd, &left_rect_1,2,GRAY,true);
          nrf_gfx_rect_draw(lcd, &left_rect_2,2,GRAY,true);
          nrf_gfx_rect_draw(lcd, &left_rect_3,2,GRAY,true);

          nrf_gfx_rect_draw(lcd, &right_rect_1,2,GRAY,true);
          nrf_gfx_rect_draw(lcd, &right_rect_2,2,GRAY,true);
          nrf_gfx_rect_draw(lcd, &right_rect_3,2,GRAY,true);

          nrf_gfx_rect_draw(lcd, &left_rect_1,2,BLACK,false);
          nrf_gfx_rect_draw(lcd, &left_rect_2,2,BLACK,false);
          nrf_gfx_rect_draw(lcd, &left_rect_3,2,BLACK,false);
                                                
          nrf_gfx_rect_draw(lcd, &right_rect_1,2,BLACK,false);
          nrf_gfx_rect_draw(lcd, &right_rect_2,2,BLACK,false);
          nrf_gfx_rect_draw(lcd, &right_rect_3,2,BLACK,false);
      }
      idle_state_handle();
    }

}


/************************************************
* Code that may be useful?
************************************************/
/*

        err_code = ble_lbs_led_status_send(&m_ble_lbs_c, send_bit);
        if (err_code != NRF_SUCCESS &&
            err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
            err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
        
//        if(send_bit == 20){
//          send_bit = 0;
//        }   
//        else{
//          send_bit = 20;
//        }
        send_bit = !send_bit;
        //err_code = ble_lbs_led_status_send(&m_ble_lbs_c, 20);
        nrf_delay_ms(1000);

*/