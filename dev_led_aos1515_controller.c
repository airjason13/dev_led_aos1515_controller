/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

// Pico
#include "pico/stdlib.h"

// For memcpy
#include <string.h>

// Include descriptor struct definitions
#include "usb_common.h"
// USB register definitions from pico-sdk
#include "hardware/regs/usb.h"
// USB hardware struct definitions from pico-sdk
#include "hardware/structs/usb.h"
// For interrupt enable and numbers
#include "hardware/irq.h"
// For resetting the USB controller
#include "hardware/resets.h"

// Device descriptors
#include "dev_lowlevel.h"

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "apa104.pio.h"
//#include "apa104.h"
#include "led_param.h"
#include "hardware/gpio.h" //for gpio irq
#include "led_controller.h"
#include "pico/sem.h"
#include "pico/critical_section.h"
#include "hardware/vreg.h"
#define usb_hw_set hw_set_alias(usb_hw)
#define usb_hw_clear hw_clear_alias(usb_hw)

//struct semaphore led_frame_sem;
critical_section_t c_s;
//int force_refresh = 1;
int force_refresh = 0;
uint32_t test_pattern = COLOR_WHITE;
bool b_color_test_mode = 0;//for test app
bool b_rgb_test_mode = 0;//self test RGB pattern
int width_interval = 0;
int height_interval = 0;
int LED_WIDTH = 40;
int LED_HEIGHT = 24;
uint32_t color_lsb = 0xFFFFFF;
uint32_t color_msb = 0xFFFFFF;


void gpio_callback(uint gpio, uint32_t events);
int32_t gpio_irq_enable(uint32_t gpio, void* callback, uint32_t condition);

int i_led_test_color = 0x0;

int64_t alarm_callback(alarm_id_t id, void *user_data) {
    printf("Timer %d fired!\n", (int) id);
    //timer_fired = true;
    gpio_irq_enable(15, &gpio_callback, GPIO_IRQ_EDGE_RISE );
    // Can return a value here in us to fire in the future
    return 0;
}

void gpio_callback(uint gpio, uint32_t events) {
    gpio_set_irq_enabled_with_callback(15, NULL, false, NULL);
    printf("%s\n", __func__);
    if(test_pattern == COLOR_WHITE){
    	test_pattern = COLOR_RED;
	printf("RED!\n");
    }else if(test_pattern == COLOR_RED){
    	test_pattern = COLOR_GREEN;
	printf("GREEN!\n");
    }else if(test_pattern == COLOR_GREEN){
    	test_pattern = COLOR_BLUE;
	printf("BLUE!\n");
    }else if(test_pattern == COLOR_BLUE){
    	test_pattern = COLOR_WHITE;
	printf("WHITE!\n");
    }
    add_alarm_in_ms(2000, alarm_callback, NULL, false);

}

int32_t gpio_irq_enable(uint32_t gpio, void* callback, uint32_t condition) {
    gpio_set_irq_enabled_with_callback(gpio, condition, true, callback);
}

// add for LED apa104 control
const int PIN_TX_0 = 4;
const int PIN_TX_1 = 5;
const int PIN_TX_2 = 6;
const int PIN_TX_3 = 7;

const int PIN_TX_4 = 16;
const int PIN_TX_5 = 17;
const int PIN_TX_6 = 18;
const int PIN_TX_7 = 19;
//PIO pio_0 = pio0
//PIO pio_1 = pio1

static inline void put_pixel_by_panel_64bits(uint8_t panel_id, uint32_t msb, uint32_t lsb) {
    switch(panel_id){
        case 0:
            pio_sm_put_blocking(pio0, 0, msb << 8u);
            pio_sm_put_blocking(pio0, 0, lsb << 8u);
            break;
        case 1:
            pio_sm_put_blocking(pio0, 1, msb << 8u);
            pio_sm_put_blocking(pio0, 1, lsb << 8u);
            break;
        case 2:
            pio_sm_put_blocking(pio0, 2, msb << 8u);
            pio_sm_put_blocking(pio0, 2, lsb << 8u);
            break;
        case 3:
            pio_sm_put_blocking(pio0, 3, msb << 8u);
            pio_sm_put_blocking(pio0, 3, lsb << 8u);
            break;
        case 4:
            pio_sm_put_blocking(pio1, 0, msb << 8u);
            pio_sm_put_blocking(pio1, 0, lsb << 8u);
            break;
        case 5:
            pio_sm_put_blocking(pio1, 1, msb << 8u);
            pio_sm_put_blocking(pio1, 1, lsb << 8u);
            break;
        case 6:
            pio_sm_put_blocking(pio1, 2, msb << 8u);
            pio_sm_put_blocking(pio1, 2, lsb << 8u);
            break;
        case 7:
            pio_sm_put_blocking(pio1, 3, msb << 8u);
            pio_sm_put_blocking(pio1, 3, lsb << 8u);
            break;
        default:
            printf("error!no such panel id: %d\n", panel_id);
            break;
    }
}



static inline void put_pixel_by_panel(uint8_t panel_id, uint32_t pixel_grb) {
    switch(panel_id){
        case 0:
            pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
            break;
        case 1:
            pio_sm_put_blocking(pio0, 1, pixel_grb << 8u);
            break;    
        case 2:
            pio_sm_put_blocking(pio0, 2, pixel_grb << 8u);
            break;    
        case 3:
            pio_sm_put_blocking(pio0, 3, pixel_grb << 8u);
            break;    
        case 4:
            pio_sm_put_blocking(pio1, 0, pixel_grb << 8u);
            break;    
        case 5:
            pio_sm_put_blocking(pio1, 1, pixel_grb << 8u);
            break;    
        case 6:
            pio_sm_put_blocking(pio1, 2, pixel_grb << 8u);
            break;    
        case 7:
            pio_sm_put_blocking(pio1, 3, pixel_grb << 8u);
            break;    
        default:
            printf("error!no such panel id: %d\n", panel_id);
            break;    
    }
}
static inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
    pio_sm_put_blocking(pio0, 1, pixel_grb << 8u);
    pio_sm_put_blocking(pio0, 2, pixel_grb << 8u);
    pio_sm_put_blocking(pio0, 3, pixel_grb << 8u);
    pio_sm_put_blocking(pio1, 0, pixel_grb << 8u);
    pio_sm_put_blocking(pio1, 1, pixel_grb << 8u);
    pio_sm_put_blocking(pio1, 2, pixel_grb << 8u);
    pio_sm_put_blocking(pio1, 3, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}

static int32_t pio_initial() {

    //pio0 4 sm port
    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, PIN_TX_0, 800000, false);
    ws2812_program_init(pio, sm+1, offset, PIN_TX_1, 800000, false);
    ws2812_program_init(pio, sm+2, offset, PIN_TX_2, 800000, false);
    ws2812_program_init(pio, sm+3, offset, PIN_TX_3, 800000, false);
    
    //pio0 4 sm port
    PIO pio_1 = pio1;
    int sm_1 = 0;
    uint offset_1 = pio_add_program(pio1, &ws2812_program);
    ws2812_program_init(pio_1, sm_1, offset_1, PIN_TX_4, 800000, false);
    ws2812_program_init(pio_1, sm_1+1, offset_1, PIN_TX_5, 800000, false);
    ws2812_program_init(pio_1, sm_1+2, offset_1, PIN_TX_6, 800000, false);
    ws2812_program_init(pio_1, sm_1+3, offset_1, PIN_TX_7, 800000, false);
    return 0;
}

// Function prototypes for our device specific endpoint handlers defined
// later on
void ep0_in_handler(uint8_t *buf, uint16_t len);
void ep0_out_handler(uint8_t *buf, uint16_t len);
void ep1_out_handler(uint8_t *buf, uint16_t len);
void ep2_in_handler(uint8_t *buf, uint16_t len);

// Global device address
static bool should_set_address = false;
static uint8_t dev_addr = 0;
static volatile bool configured = false;

// Global data buffer for EP0
static uint8_t ep0_buf[64];

// Struct defining the device configuration
static struct usb_device_configuration dev_config = {
        .device_descriptor = &device_descriptor,
        .interface_descriptor = &interface_descriptor,
        .config_descriptor = &config_descriptor,
        .lang_descriptor = lang_descriptor,
        .descriptor_strings = descriptor_strings,
        .endpoints = {
                {
                        .descriptor = &ep0_out,
                        .handler = &ep0_out_handler,
                        .endpoint_control = NULL, // NA for EP0
                        .buffer_control = &usb_dpram->ep_buf_ctrl[0].out,
                        // EP0 in and out share a data buffer
                        .data_buffer = &usb_dpram->ep0_buf_a[0],
                },
                {
                        .descriptor = &ep0_in,
                        .handler = &ep0_in_handler,
                        .endpoint_control = NULL, // NA for EP0,
                        .buffer_control = &usb_dpram->ep_buf_ctrl[0].in,
                        // EP0 in and out share a data buffer
                        .data_buffer = &usb_dpram->ep0_buf_a[0],
                },
                {
                        .descriptor = &ep1_out,
                        .handler = &ep1_out_handler,
                        // EP1 starts at offset 0 for endpoint control
                        .endpoint_control = &usb_dpram->ep_ctrl[0].out,
                        .buffer_control = &usb_dpram->ep_buf_ctrl[1].out,
                        // First free EPX buffer
                        .data_buffer = &usb_dpram->epx_data[0 * 64],
                },
                {
                        .descriptor = &ep2_in,
                        .handler = &ep2_in_handler,
                        .endpoint_control = &usb_dpram->ep_ctrl[1].in,
                        .buffer_control = &usb_dpram->ep_buf_ctrl[2].in,
                        // Second free EPX buffer
                        .data_buffer = &usb_dpram->epx_data[1 * 64],
                }
        }
};

/**
 * @brief Given an endpoint address, return the usb_endpoint_configuration of that endpoint. Returns NULL
 * if an endpoint of that address is not found.
 *
 * @param addr
 * @return struct usb_endpoint_configuration*
 */
struct usb_endpoint_configuration *usb_get_endpoint_configuration(uint8_t addr) {
    struct usb_endpoint_configuration *endpoints = dev_config.endpoints;
    for (int i = 0; i < USB_NUM_ENDPOINTS; i++) {
        if (endpoints[i].descriptor && (endpoints[i].descriptor->bEndpointAddress == addr)) {
            return &endpoints[i];
        }
    }
    return NULL;
}

/**
 * @brief Given a C string, fill the EP0 data buf with a USB string descriptor for that string.
 *
 * @param C string you would like to send to the USB host
 * @return the length of the string descriptor in EP0 buf
 */
uint8_t usb_prepare_string_descriptor(const unsigned char *str) {
    // 2 for bLength + bDescriptorType + strlen * 2 because string is unicode. i.e. other byte will be 0
    uint8_t bLength = 2 + (strlen(str) * 2);
    static const uint8_t bDescriptorType = 0x03;

    volatile uint8_t *buf = &ep0_buf[0];
    *buf++ = bLength;
    *buf++ = bDescriptorType;

    uint8_t c;

    do {
        c = *str++;
        *buf++ = c;
        *buf++ = 0;
    } while (c != '\0');

    return bLength;
}

/**
 * @brief Take a buffer pointer located in the USB RAM and return as an offset of the RAM.
 *
 * @param buf
 * @return uint32_t
 */
static inline uint32_t usb_buffer_offset(volatile uint8_t *buf) {
    return (uint32_t) buf ^ (uint32_t) usb_dpram;
}

/**
 * @brief Set up the endpoint control register for an endpoint (if applicable. Not valid for EP0).
 *
 * @param ep
 */
void usb_setup_endpoint(const struct usb_endpoint_configuration *ep) {
    printf("Set up endpoint 0x%x with buffer address 0x%p\n", ep->descriptor->bEndpointAddress, ep->data_buffer);

    // EP0 doesn't have one so return if that is the case
    if (!ep->endpoint_control) {
        return;
    }

    // Get the data buffer as an offset of the USB controller's DPRAM
    uint32_t dpram_offset = usb_buffer_offset(ep->data_buffer);
    uint32_t reg = EP_CTRL_ENABLE_BITS
                   | EP_CTRL_INTERRUPT_PER_BUFFER
                   | (ep->descriptor->bmAttributes << EP_CTRL_BUFFER_TYPE_LSB)
                   | dpram_offset;
    *ep->endpoint_control = reg;
}

/**
 * @brief Set up the endpoint control register for each endpoint.
 *
 */
void usb_setup_endpoints() {
    const struct usb_endpoint_configuration *endpoints = dev_config.endpoints;
    for (int i = 0; i < USB_NUM_ENDPOINTS; i++) {
        if (endpoints[i].descriptor && endpoints[i].handler) {
            usb_setup_endpoint(&endpoints[i]);
        }
    }
}

/**
 * @brief Set up the USB controller in device mode, clearing any previous state.
 *
 */
void usb_device_init() {
    // Reset usb controller
    reset_block(RESETS_RESET_USBCTRL_BITS);
    unreset_block_wait(RESETS_RESET_USBCTRL_BITS);

    // Clear any previous state in dpram just in case
    memset(usb_dpram, 0, sizeof(*usb_dpram)); // <1>

    // Enable USB interrupt at processor
    irq_set_enabled(USBCTRL_IRQ, true);

    // Mux the controller to the onboard usb phy
    usb_hw->muxing = USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS;

    // Force VBUS detect so the device thinks it is plugged into a host
    usb_hw->pwr = USB_USB_PWR_VBUS_DETECT_BITS | USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;

    // Enable the USB controller in device mode.
    usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS;

    // Enable an interrupt per EP0 transaction
    usb_hw->sie_ctrl = USB_SIE_CTRL_EP0_INT_1BUF_BITS; // <2>

    // Enable interrupts for when a buffer is done, when the bus is reset,
    // and when a setup packet is received
    usb_hw->inte = USB_INTS_BUFF_STATUS_BITS |
                   USB_INTS_BUS_RESET_BITS |
                   USB_INTS_SETUP_REQ_BITS;

    // Set up endpoints (endpoint control registers)
    // described by device configuration
    usb_setup_endpoints();

    // Present full speed device by enabling pull up on DP
    usb_hw_set->sie_ctrl = USB_SIE_CTRL_PULLUP_EN_BITS;
}

/**
 * @brief Given an endpoint configuration, returns true if the endpoint
 * is transmitting data to the host (i.e. is an IN endpoint)
 *
 * @param ep, the endpoint configuration
 * @return true
 * @return false
 */
static inline bool ep_is_tx(struct usb_endpoint_configuration *ep) {
    return ep->descriptor->bEndpointAddress & USB_DIR_IN;
}

/**
 * @brief Starts a transfer on a given endpoint.
 *
 * @param ep, the endpoint configuration.
 * @param buf, the data buffer to send. Only applicable if the endpoint is TX
 * @param len, the length of the data in buf (this example limits max len to one packet - 64 bytes)
 */
void usb_start_transfer(struct usb_endpoint_configuration *ep, uint8_t *buf, uint16_t len) {
    // We are asserting that the length is <= 64 bytes for simplicity of the example.
    // For multi packet transfers see the tinyusb port.
    assert(len <= 64);

    //printf("Start transfer of len %d on ep addr 0x%x\n", len, ep->descriptor->bEndpointAddress);

    // Prepare buffer control register value
    uint32_t val = len | USB_BUF_CTRL_AVAIL;

    if (ep_is_tx(ep)) {
        // Need to copy the data from the user buffer to the usb memory
        memcpy((void *) ep->data_buffer, (void *) buf, len);
        // Mark as full
        val |= USB_BUF_CTRL_FULL;
    }

    // Set pid and flip for next transfer
    val |= ep->next_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID;
    ep->next_pid ^= 1u;

    *ep->buffer_control = val;
}

/**
 * @brief Send device descriptor to host
 *
 */
void usb_handle_device_descriptor(void) {
    const struct usb_device_descriptor *d = dev_config.device_descriptor;
    // EP0 in
    struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP0_IN_ADDR);
    // Always respond with pid 1
    ep->next_pid = 1;
    usb_start_transfer(ep, (uint8_t *) d, sizeof(struct usb_device_descriptor));
}

/**
 * @brief Send the configuration descriptor (and potentially the configuration and endpoint descriptors) to the host.
 *
 * @param pkt, the setup packet received from the host.
 */
void usb_handle_config_descriptor(volatile struct usb_setup_packet *pkt) {
    uint8_t *buf = &ep0_buf[0];

    // First request will want just the config descriptor
    const struct usb_configuration_descriptor *d = dev_config.config_descriptor;
    memcpy((void *) buf, d, sizeof(struct usb_configuration_descriptor));
    buf += sizeof(struct usb_configuration_descriptor);

    // If we more than just the config descriptor copy it all
    if (pkt->wLength >= d->wTotalLength) {
        memcpy((void *) buf, dev_config.interface_descriptor, sizeof(struct usb_interface_descriptor));
        buf += sizeof(struct usb_interface_descriptor);
        const struct usb_endpoint_configuration *ep = dev_config.endpoints;

        // Copy all the endpoint descriptors starting from EP1
        for (uint i = 2; i < USB_NUM_ENDPOINTS; i++) {
            if (ep[i].descriptor) {
                memcpy((void *) buf, ep[i].descriptor, sizeof(struct usb_endpoint_descriptor));
                buf += sizeof(struct usb_endpoint_descriptor);
            }
        }

    }

    // Send data
    // Get len by working out end of buffer subtract start of buffer
    uint32_t len = (uint32_t) buf - (uint32_t) &ep0_buf[0];
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), &ep0_buf[0], len);
}

/**
 * @brief Handle a BUS RESET from the host by setting the device address back to 0.
 *
 */
void usb_bus_reset(void) {
    // Set address back to 0
    dev_addr = 0;
    should_set_address = false;
    usb_hw->dev_addr_ctrl = 0;
    configured = false;
}

/**
 * @brief Send the requested string descriptor to the host.
 *
 * @param pkt, the setup packet from the host.
 */
void usb_handle_string_descriptor(volatile struct usb_setup_packet *pkt) {
    uint8_t i = pkt->wValue & 0xff;
    uint8_t len = 0;

    if (i == 0) {
        len = 4;
        memcpy(&ep0_buf[0], dev_config.lang_descriptor, len);
    } else {
        // Prepare fills in ep0_buf
        len = usb_prepare_string_descriptor(dev_config.descriptor_strings[i - 1]);
    }

    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), &ep0_buf[0], len);
}

/**
 * @brief Handles a SET_ADDR request from the host. The actual setting of the device address in
 * hardware is done in ep0_in_handler. This is because we have to acknowledge the request first
 * as a device with address zero.
 *
 * @param pkt, the setup packet from the host.
 */
void usb_set_device_address(volatile struct usb_setup_packet *pkt) {
    // Set address is a bit of a strange case because we have to send a 0 length status packet first with
    // address 0
    dev_addr = (pkt->wValue & 0xff);
    printf("Set address %d\r\n", dev_addr);
    // Will set address in the callback phase
    should_set_address = true;
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0);
}

/**
 * @brief Handles a SET_CONFIGRUATION request from the host. Assumes one configuration so simply
 * sends a zero length status packet back to the host.
 *
 * @param pkt, the setup packet from the host.
 */
void usb_set_device_configuration(volatile struct usb_setup_packet *pkt) {
    // Only one configuration so just acknowledge the request
    printf("Device Enumerated\r\n");
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0);
    configured = true;
}

/**
 * @brief Respond to a setup packet from the host.
 *
 */
void usb_handle_setup_packet(void) {
    volatile struct usb_setup_packet *pkt = (volatile struct usb_setup_packet *) &usb_dpram->setup_packet;
    uint8_t req_direction = pkt->bmRequestType;
    uint8_t req = pkt->bRequest;

    // Reset PID to 1 for EP0 IN
    usb_get_endpoint_configuration(EP0_IN_ADDR)->next_pid = 1u;

    if (req_direction == USB_DIR_OUT) {
        if (req == USB_REQUEST_SET_ADDRESS) {
            usb_set_device_address(pkt);
        } else if (req == USB_REQUEST_SET_CONFIGURATION) {
            usb_set_device_configuration(pkt);
        } else {
            printf("Other OUT request (0x%x)\r\n", pkt->bRequest);
        }
    } else if (req_direction == USB_DIR_IN) {
        if (req == USB_REQUEST_GET_DESCRIPTOR) {
            uint16_t descriptor_type = pkt->wValue >> 8;

            switch (descriptor_type) {
                case USB_DT_DEVICE:
                    usb_handle_device_descriptor();
                    printf("GET DEVICE DESCRIPTOR\r\n");
                    break;

                case USB_DT_CONFIG:
                    usb_handle_config_descriptor(pkt);
                    printf("GET CONFIG DESCRIPTOR\r\n");
                    break;

                case USB_DT_STRING:
                    usb_handle_string_descriptor(pkt);
                    printf("GET STRING DESCRIPTOR\r\n");
                    break;

                default:
                    printf("Unhandled GET_DESCRIPTOR type 0x%x\r\n", descriptor_type);
            }
        } else {
            printf("Other IN request (0x%x)\r\n", pkt->bRequest);
        }
    }
}

/**
 * @brief Notify an endpoint that a transfer has completed.
 *
 * @param ep, the endpoint to notify.
 */
static void usb_handle_ep_buff_done(struct usb_endpoint_configuration *ep) {
    uint32_t buffer_control = *ep->buffer_control;
    // Get the transfer length for this endpoint
    uint16_t len = buffer_control & USB_BUF_CTRL_LEN_MASK;

    // Call that endpoints buffer done handler
    ep->handler((uint8_t *) ep->data_buffer, len);
}

/**
 * @brief Find the endpoint configuration for a specified endpoint number and
 * direction and notify it that a transfer has completed.
 *
 * @param ep_num
 * @param in
 */
static void usb_handle_buff_done(uint ep_num, bool in) {
    uint8_t ep_addr = ep_num | (in ? USB_DIR_IN : 0);
    //printf("EP %d (in = %d) done\n", ep_num, in);
    for (uint i = 0; i < USB_NUM_ENDPOINTS; i++) {
        struct usb_endpoint_configuration *ep = &dev_config.endpoints[i];
        if (ep->descriptor && ep->handler) {
            if (ep->descriptor->bEndpointAddress == ep_addr) {
                usb_handle_ep_buff_done(ep);
                return;
            }
        }
    }
}

/**
 * @brief Handle a "buffer status" irq. This means that one or more
 * buffers have been sent / received. Notify each endpoint where this
 * is the case.
 */
static void usb_handle_buff_status() {
    uint32_t buffers = usb_hw->buf_status;
    uint32_t remaining_buffers = buffers;

    uint bit = 1u;
    for (uint i = 0; remaining_buffers && i < USB_NUM_ENDPOINTS * 2; i++) {
        if (remaining_buffers & bit) {
            // clear this in advance
            usb_hw_clear->buf_status = bit;
            // IN transfer for even i, OUT transfer for odd i
            usb_handle_buff_done(i >> 1u, !(i & 1u));
            remaining_buffers &= ~bit;
        }
        bit <<= 1u;
    }
}

/**
 * @brief USB interrupt handler
 *
 */
/// \tag::isr_setup_packet[]
void isr_usbctrl(void) {
    // USB interrupt handler
    uint32_t status = usb_hw->ints;
    uint32_t handled = 0;

    // Setup packet received
    if (status & USB_INTS_SETUP_REQ_BITS) {
        handled |= USB_INTS_SETUP_REQ_BITS;
        usb_hw_clear->sie_status = USB_SIE_STATUS_SETUP_REC_BITS;
        usb_handle_setup_packet();
    }
/// \end::isr_setup_packet[]

    // Buffer status, one or more buffers have completed
    if (status & USB_INTS_BUFF_STATUS_BITS) {
        handled |= USB_INTS_BUFF_STATUS_BITS;
        usb_handle_buff_status();
    }

    // Bus is reset
    if (status & USB_INTS_BUS_RESET_BITS) {
        printf("BUS RESET\n");
        handled |= USB_INTS_BUS_RESET_BITS;
        usb_hw_clear->sie_status = USB_SIE_STATUS_BUS_RESET_BITS;
        usb_bus_reset();
    }

    if (status ^ handled) {
        panic("Unhandled IRQ 0x%x\n", (uint) (status ^ handled));
    }
}

/**
 * @brief EP0 in transfer complete. Either finish the SET_ADDRESS process, or receive a zero
 * length status packet from the host.
 *
 * @param buf the data that was sent
 * @param len the length that was sent
 */
void ep0_in_handler(uint8_t *buf, uint16_t len) {
    if (should_set_address) {
        // Set actual device address in hardware
        usb_hw->dev_addr_ctrl = dev_addr;
        should_set_address = false;
    } else {
        // Receive a zero length status packet from the host on EP0 OUT
        struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP0_OUT_ADDR);
        usb_start_transfer(ep, NULL, 0);
    }
}

void ep0_out_handler(uint8_t *buf, uint16_t len) {
    ;
}

uint8_t tmp_buf[64];
//uint8_t id = 0;

int8_t get_id_num(uint8_t *buf){
    
    int8_t id_tmp = -1;
    if(strncmp("cmd:", buf, 4) == 0){
        printf("got cmd!\n");
        return -2;
    }else if(strncmp("Hello", buf, 5) == 0){
        printf("got Hello!\n");
		return -3;
    }else{
        if(strncmp("id0:", buf, 4) == 0){
            id_tmp = 0;
        }else if(strncmp("id1:", buf, 4) == 0){
            id_tmp = 1;
        }else if(strncmp("id2:", buf, 4) == 0){
            id_tmp = 2;
        }else if(strncmp("id3:", buf, 4) == 0){
            id_tmp = 3;
        }else if(strncmp("id4:", buf, 4) == 0){
            id_tmp = 4;
        }else if(strncmp("id5:", buf, 4) == 0){
            id_tmp = 5;
        }else if(strncmp("id6:", buf, 4) == 0){
            id_tmp = 6;
        }else if(strncmp("id7:", buf, 4) == 0){
            id_tmp = 7;
        }else{
            id_tmp = -1;
            return id_tmp;//follow ori id
        }
    }
    if(panel_id != id_tmp){
        data_offset = 0;
        panel_id = id_tmp;
    }
    return id_tmp; 
}

char* parser_cmd(char *cmd){
    char *reply = malloc(64);
    char *tmp0 = malloc(64);
    char *tmp1 = malloc(64);
    memset(tmp0, 0, 64);
    memset(tmp1, 0, 64);
    if(strstr(cmd, "set_port_res")){
        sscanf(cmd, "cmd:set_port_res,%d,%d", LED_WIDTH, LED_HEIGHT);
        memset(reply, 0, 64);
        //LED_WIDTH=atoi(tmp0);
        //LED_HEIGHT=atoi(tmp1);
        sprintf(reply, "OK!%d,%d", LED_WIDTH, LED_HEIGHT); 
    }else if(strstr(cmd, "set_test_color_stop")){
        b_color_test_mode = false; 
    }else if(strstr(cmd, "set_test_color_start")){
        printf("cmd:%s\n", cmd); 
        unsigned int test_color;
        sscanf(cmd, "cmd:set_test_color_start,%s", tmp0);
        b_color_test_mode = true;
        printf("color:0x%x\n", atoi(tmp0));
        i_led_test_color = atoi(tmp0);
        unsigned int i_red = (i_led_test_color >> 16) & 0xff;
        unsigned int i_green = (i_led_test_color >> 8) & 0xff;
        unsigned int i_blue = (i_led_test_color) & 0xff;
        printf("r : %d, g : %d, b: %d\n", i_red, i_green, i_blue);
        for(int l = 0; l < 3; l ++ ){
            for(int n = 0; n < LED_PANEL_COUNT; n++){
		        for(int m = 0; m < (LED_WIDTH*LED_HEIGHT*COLOR_CHANNEL); m++ ){
			        if((m%3) == 0){               //0x400000 400000 400000  => red
				        led_rgb_buf[l][n][m] = i_red;
			        }if((m%3) == 2){                 //0x000040 000040 000040  => blue
				        led_rgb_buf[l][n][m] = i_green;
			        }if((m%3) == 1){                 //0x004000 004000 004000  => green 
				        led_rgb_buf[l][n][m] = i_blue;
			        }
		        }
	        }
        }
          
        sprintf(reply, "OK!%s", tmp0);
    }else if(strstr(cmd, "set_pixel_interval")){
        sscanf(cmd, "cmd:set_pixel_interval,%s", tmp0);
        if((atoi(tmp0) < 0)||(atoi(tmp0) > 10)){
            sprintf(reply, "NG!%s", tmp0); 
        }else{    
            if(atoi(tmp0) != 0){
                width_interval = atoi(tmp0) + 1;
                height_interval = atoi(tmp0) + 1;    
                sprintf(reply, "OK!%s", tmp0);
            }else{
                width_interval = 0;
                height_interval = 0;
                sprintf(reply, "OK!%d", atoi(tmp0));
            }
        }
    }else{
        sprintf(reply, "unknown cmd\n");
    }
    return reply;
}

// Device specific functions
void ep1_out_handler(uint8_t *buf, uint16_t len) {
    unsigned int pattern = 0;
	int n,m;
    int8_t res = get_id_num(buf);
    if(res == -2){//got cmd!
        printf("got cmd!%s\n", buf);
        char *reply = parser_cmd(buf);
        memset(buf, 0, 64);
        sprintf(buf, "%s", reply);
        struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP2_IN_ADDR);
        usb_start_transfer(ep, buf, 64);
        return;
    }else if(res == -3){
        printf("got hello!\n");
        sprintf(buf, "%s", VERSION);
        struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP2_IN_ADDR);
        usb_start_transfer(ep, buf, 15);
        return;
    }else if(res == -1){
        memcpy((led_rgb_buf[rgb_buf_write_idx][panel_id] + data_offset), buf, len);
        data_offset += len; 
    }else if((res >= 0)&&(res <=7)){
        if( res == 0){
            //sem_acquire_blocking(&led_frame_sem);
            //critical_section_enter_blocking(&c_s);
            if(rgb_buf_write_idx == 2){
                rgb_buf_write_idx = 0; 
            }else{
                rgb_buf_write_idx ++;
            }
            if(rgb_buf_write_idx == 0){
                rgb_buf_read_idx = 2;
            }else{
                rgb_buf_read_idx = rgb_buf_write_idx - 1;
            }
            force_refresh = 1;
            //sem_release(&led_frame_sem);
            //critical_section_exit(&c_s);
        } 
        //printf("got color buf\n");
        memcpy((led_rgb_buf[rgb_buf_write_idx][panel_id]), buf + 4, len - 4 );
        data_offset += len-4; 
    }else{
        //printf("buf : %s!\n", buf);
    }

    usb_start_transfer(usb_get_endpoint_configuration(EP1_OUT_ADDR), NULL, 64);
}

void ep2_in_handler(uint8_t *buf, uint16_t len) {
    printf("Sent %d bytes to host\n", len);
    // Get ready to rx again from host
    usb_start_transfer(usb_get_endpoint_configuration(EP1_OUT_ADDR), NULL, 64);
}

int main(void) {
    unsigned int pattern = 0;
	int n, m, l;
    //int force_refresh = 1;
    int pre_rgb_buf_idx = -1;
    int color_upscale_factor = 256;
    uint16_t r_value, g_value, b_value;
    stdio_init_all();
    printf("USB Device Low-Level hardware example\n");
    usb_device_init();
    pio_initial();

    //test over clocking
    //vreg_set_voltage(VREG_VOLTAGE_1_20);
    //sleep_ms(1000);
    //set_sys_clock_khz(2500000, false);

    //sem_init(&led_frame_sem, 1, 1);
    //critical_section_init(&c_s);



#if 0 //marked gpio irq function
    int ret = gpio_get_dir(15);
    printf("gpio 15 dir is %d\n", ret); //default is in
    gpio_pull_up(15);
    gpio_irq_enable(15, &gpio_callback, GPIO_IRQ_EDGE_RISE );
#endif	
    // Wait until configured
    while (!configured) {
        tight_loop_contents();
    }

	//fill initial color
    for(l = 0; l < 3; l ++ ){
        for(n = 0; n < LED_PANEL_COUNT; n++){
		    for(m = 0; m < (LED_WIDTH*LED_HEIGHT*COLOR_CHANNEL); m++ ){
			    if((m%3) == 0){               //red
				    led_rgb_buf[l][n][m] = 0x00;
			    }if((m%3) == 2){                 //blue
				    led_rgb_buf[l][n][m] = 0x00;
			    }if((m%3) == 1){                 //green
				    led_rgb_buf[l][n][m] = 0x40;
			    }
		    }
	    }
    }
    
	for(int j = 0; j < 1; j++){
	    for(int k = 0; k < 1; k++){
            for(n = 0; n < LED_PANEL_COUNT; n ++){
                        
                r_value = led_rgb_buf[rgb_buf_read_idx][n][(((j*LED_WIDTH) + k)*COLOR_CHANNEL) + (0)];
                g_value = (led_rgb_buf[rgb_buf_read_idx][n][(((j*LED_WIDTH) + k)*COLOR_CHANNEL) + 1]);
                b_value = ((led_rgb_buf[rgb_buf_read_idx][n][(((j*LED_WIDTH) + k)*COLOR_CHANNEL) + 2]));
                r_value = r_value * color_upscale_factor;
                if(r_value > 0xffff){
                    r_value = 0xffff;
                }
                g_value = g_value * color_upscale_factor;
                if(g_value > 0xffff){
                    g_value = 0xffff;
                }
                b_value = b_value * color_upscale_factor;
                if(b_value > 0xffff){
                    b_value = 0xffff;
                }
                        
                        
                color_msb = ((g_value & 0x0000ffff) << 8) | ((b_value >> 8) & 0x0000ffff);
                color_lsb = ((b_value & 0x000000ff) << 16) | r_value;
                put_pixel_by_panel_64bits(n, color_msb, color_lsb);
           }
	   }
    }
    for(l = 0; l < 3; l ++ ){
        for(n = 0; n < LED_PANEL_COUNT; n++){
		    for(m = 0; m < (LED_WIDTH*LED_HEIGHT*COLOR_CHANNEL); m++ ){
			    if((m%3) == 0){               //red
				    led_rgb_buf[l][n][m] = 0x00;
			    }if((m%3) == 2){                 //blue
				    led_rgb_buf[l][n][m] = 0x00;
			    }if((m%3) == 1){                 //green
				    led_rgb_buf[l][n][m] = 0x00;
			    }
		    }
	    }
    }
    

    // Get ready to rx from host
    usb_start_transfer(usb_get_endpoint_configuration(EP1_OUT_ADDR), NULL, 64);

    // Everything is interrupt driven so just loop here
    while (1) {
        //tight_loop_contents(); //marked this busy loop
	    //test pattern
        int output = 0;
        //critical_section_enter_blocking(&c_s);
        //b_color_test_mode = true;
        if(b_color_test_mode == true){
            output = 1;
        }else{
            output = force_refresh;
        }
        //critical_section_exit(&c_s);
        if(output == 0){
	        sleep_ms(3);
            //critical_section_exit(&c_s);
            continue;
        }else{
        
        }
	    for(int j = 0; j < LED_HEIGHT; j++){
	        for(int k = 0; k < LED_WIDTH; k++){
                    for(n = 0; n < LED_PANEL_COUNT; n ++){
                        
                        r_value = led_rgb_buf[rgb_buf_read_idx][n][(((j*LED_WIDTH) + k)*COLOR_CHANNEL) + (0)];
                        g_value = (led_rgb_buf[rgb_buf_read_idx][n][(((j*LED_WIDTH) + k)*COLOR_CHANNEL) + 1]);
                        b_value = ((led_rgb_buf[rgb_buf_read_idx][n][(((j*LED_WIDTH) + k)*COLOR_CHANNEL) + 2]));
                        r_value = r_value * color_upscale_factor;
                        if(r_value > 0xffff){
                            r_value = 0xffff;
                        }
                        g_value = g_value * color_upscale_factor;
                        if(g_value > 0xffff){
                            g_value = 0xffff;
                        }
                        b_value = b_value * color_upscale_factor;
                        if(b_value > 0xffff){
                            b_value = 0xffff;
                        }
                        
                        
                        color_msb = ((g_value & 0x0000ffff) << 8) | ((b_value >> 8) & 0x0000ffff);
                        color_lsb = ((b_value & 0x000000ff) << 16) | r_value;
                        put_pixel_by_panel_64bits(n, color_msb, color_lsb);
                    }
	        }
	    }
        //sem_release(&led_frame_sem);
        //critical_section_enter_blocking(&c_s);
        force_refresh = 0;
        //critical_section_exit(&c_s);
        //printf("Apattern : 0x%x\n", pattern);             
	    sleep_ms(1);
    }

    return 0;
}
