/*
 * MIT License
 *
 * Copyright (c) 2020 Michael Volk
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice this permission notice, and the disclaimer below
 * shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef ST7789_H
#define ST7789_H

#include <freertos/FreeRTOS.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_err.h>


// Commands, excluding some read commands
#define ST7789_CMD_NOP             0x00
#define ST7789_CMD_SWRESET         0x01
#define ST7789_CMD_RDDID           0x04
#define ST7789_CMD_RDDST           0x09
#define ST7789_CMD_SLPIN           0x10
#define ST7789_CMD_SLPOUT          0x11
#define ST7789_CMD_PTLON           0x12
#define ST7789_CMD_NORON           0x13
#define ST7789_CMD_INVOFF          0x20
#define ST7789_CMD_INVON           0x21
#define ST7789_CMD_GAMSET          0x26
#define ST7789_CMD_DISPOFF         0x28
#define ST7789_CMD_DISPON          0x29
#define ST7789_CMD_CASET           0x2A
#define ST7789_CMD_RASET           0x2B
#define ST7789_CMD_RAMWR           0x2C
#define ST7789_CMD_RAMRD           0x2E
#define ST7789_CMD_PTLAR           0x30
#define ST7789_CMD_VSCRDEF         0x33
#define ST7789_CMD_TEOFF           0x34
#define ST7789_CMD_TEON            0x35
#define ST7789_CMD_MADCTL          0x36
#define ST7789_CMD_VSCRSADD        0x37
#define ST7789_CMD_IDMOFF          0x38
#define ST7789_CMD_IDMON           0x39
#define ST7789_CMD_COLMOD          0x3A
#define ST7789_CMD_RAMWRC          0x3C
#define ST7789_CMD_TESCAN          0x44
#define ST7789_CMD_WRDISBV         0x51
#define ST7789_CMD_WRCTRLD         0x53
#define ST7789_CMD_WRCACE          0x55
#define ST7789_CMD_WRCABCMB        0x5E
#define ST7789_CMD_RAMCTRL         0xB0
#define ST7789_CMD_RBGCTRL         0xB1
#define ST7789_CMD_PORCTRL         0xB2
#define ST7789_CMD_FRCTRL1         0xB3
#define ST7789_CMD_PARCTRL         0xB5
#define ST7789_CMD_GCTRL           0xB7
#define ST7789_CMD_GTADJ           0xB8
#define ST7789_CMD_DGMEN           0xBA
#define ST7789_CMD_VCOMS           0xBB
#define ST7789_CMD_LCMCTRL         0xC0
#define ST7789_CMD_IDSET           0xC1
#define ST7789_CMD_VDVVRHEN        0xC2
#define ST7789_CMD_VRHS            0xC3
#define ST7789_CMD_VDVSET          0xC4
#define ST7789_CMD_VCMOFSET        0xC5
#define ST7789_CMD_FRCTR2          0xC6
#define ST7789_CMD_CABCCTRL        0xC7
#define ST7789_CMD_REGSEL1         0xC8
#define ST7789_CMD_REGSEL2         0xCA
#define ST7789_CMD_PWMFRSEL        0xCC
#define ST7789_CMD_PWCTRL1         0xD0
#define ST7789_CMD_VAVANEN         0xD2
#define ST7789_CMD_CMD2EN          0xD5
#define ST7789_CMD_RDID1           0xDA
#define ST7789_CMD_RDID2           0xDB
#define ST7789_CMD_RDID3           0xDC
#define ST7789_CMD_PVGAMCTRL       0xE0
#define ST7789_CMD_NVGAMCTRL       0xE1
#define ST7789_CMD_DGMLUTR         0xE2
#define ST7789_CMD_DGMLUTB         0xE3
#define ST7789_CMD_GATECTRL        0xE4
#define ST7789_CMD_SPI2EN          0xE7
#define ST7789_CMD_PWCTRL2         0xE8
#define ST7789_CMD_EQCTRL          0xE9
#define ST7789_CMD_PROMCTRL        0xEC
#define ST7789_CMD_PROMEN          0xFA
#define ST7789_CMD_NVMSET          0xFC
#define ST7789_CMD_PROMACT         0xFE

// Gamma Curves
#define ST7789_CFG_GC0_G22_1       0x01
#define ST7789_CFG_GC1_G18_2       0x02
#define ST7789_CFG_GC3_G25_3       0x04
#define ST7789_CFG_GC4_G10_4       0x08

// MADCTRL
#define ST7789_CFG_MIRROR_X        0x40
#define ST7789_CFG_MIRROR_Y        0x80
#define ST7789_CFG_EXCHANGE_XY     0x20
#define ST7789_CFG_REFRESH_RTL     0x04
#define ST7789_CFG_REFRESH_BTT     0x10
#define ST7789_CFG_BGR             0x08

#define ST7789_CFG_12_BIT_COLOR    0x03
#define ST7789_CFG_16_BIT_COLOR    0x05
#define ST7789_CFG_18_BIT_COLOR    0x06

#define ST7789_CFG_BCKL_OFF        0x00
#define ST7789_CFG_BCKL_ON         0x01

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Initialized and used internally
 *
 * Supplies data/command line instructions in
 * SPI transactions.
 */
typedef struct {
    gpio_num_t gpio_dc;
    uint8_t setting;
} st7789_dc_setting_t;


/**
 * @brief Device descriptor
 *
 * GPIO pins and SPI host must be configured by
 * caller. All other fields are configured by
 * st7789_init(...).
 */
typedef struct {
    /** @brief SPI device handle */
    spi_device_handle_t spi_device;
    /** @brief Internal use only */
    st7789_dc_setting_t dc_cmd;
    /** @brief Internal use only */
    st7789_dc_setting_t dc_data;
    /** @brief Optional Reset GPIO pin */
    gpio_num_t gpio_rst;
    /** @brief Optional Backlight GPIO pin */
    gpio_num_t gpio_bckl;
} st7789_device_t;


/**
 * @brief Configuration for initalization
 */
typedef struct {
    /** @brief SPI Host (HSPI or VSPI) */
    spi_host_device_t host;
    /** @brief SPI Chip Select GPIO pin */
    gpio_num_t gpio_cs;
    /** @brief Data/Command Select GPIO pin */
    gpio_num_t gpio_dc;
    /** @brief Optional Reset GPIO pin */
    gpio_num_t gpio_rst;
    /** @brief Optional Backlight GPIO pin */
    gpio_num_t gpio_bckl;
} st7789_params_t;


/**
 * @brief Convenient way to reference the device
 */
typedef st7789_device_t* st7789_device_handle_t;


/**
 * @brief Initialize an SPI command transaction
 *
 * Clears the transaction and configures it to
 * transmit the provided command, but does not send
 * the transaction.
 *
 * This is a low-level function that most callers will
 * not need to use.
 *
 * @param device the ST7789 device handle
 * @param transaction the SPI transaction to configure
 * @param cmd the ST7789 command to be transmitted
 */
void st7789_cmd_init(
    st7789_device_handle_t device,
    spi_transaction_t *transaction,
    uint8_t cmd
);


/**
 * @brief Initialize an SPI command transaction
 *
 * Clears the transaction and configures it to
 * transmit the provided command, but does not send
 * the transaction.
 *
 * This is a low-level function that most callers will
 * not need to use.
 *
 * NOTE 1: If using DMA, the provided data must be in
 * RAM and not ROM since DMA cannot access ROM.
 *
 * NOTE 2: The number of bytes that can be sent is limited
 * by the `max_transfer_sz` value supplied via `bus_config_t`
 * when initializing the SPI bus. The default value of this
 * parameter is 4096 bytes.
 *
 * NOTE 3: This function uses the transaction's `tx_buffer`
 * rather than using the 64-byte `tx_data`. For this reason,
 * the provided data must remain accessible in memory until
 * the transaction is complete.
 *
 * @param device the ST7789 device handle
 * @param transaction the SPI transaction to configure
 * @param data the data, in RAM, to be transmitted
 * @param len_bytes the byte length of the data
 */
void st7789_data_init(
    st7789_device_handle_t device,
    spi_transaction_t *transaction,
    const uint8_t * data,
    size_t len_bytes
);


/**
 * @brief Asynchronously execute SPI transactions
 *
 * Executes transactions using interrupts rather than
 * polling. Returns immediately. Use `st7789_await(...)`
 * to block until the transactions are complete.
 *
 * This is a low-level function that most callers will
 * not need to use.
 *
 * @param device the ST7789 device handle
 * @param transactions the SPI transactions to execute
 * @param num_transactions the count of transactions to
 *        execute
 * @return ESP_OK or an error code
 */
esp_err_t st7789_enqueue(
    st7789_device_handle_t device,
    spi_transaction_t *transactions,
    size_t num_transactions
);


/**
 * @brief Block until SPI transactions are complete
 *
 * Use this method after calling `st7735_enqueue(...)`
 * to block until all of the enqueued transactions are
 * complete.
 *
 * This is a low-level function that most callers will
 * not need to use.
 *
 * NOTE 1: Because ticks_to_wait applies for each
 * transaction this method serially waits upon, the
 * total time before return could in the worse case
 * be num_transactions * ticks_to_wait.
 *
 * @param device the ST7789 device handle
 * @param num_transactions the count of transactions to
 *        wait for
 * @param ticks_to_wait how long to wait for any one
 *        transaction to complete
 * @return ESP_OK or an error code
 */
esp_err_t st7789_await(
    st7789_device_handle_t device,
    size_t num_transactions,
    TickType_t ticks_to_wait
);


/**
 * @brief Synchronously execute SPI transactions
 *
 * This method conviently wraps serial calls to
 * st7789_enqueue(...) and st7789_await(...) with
 * a 10-second ticks_to_wait value.
 *
 * This method will block the task (but not the CPU)
 * until either all of the transactions complete, or
 * until the timeout is reached while waiting for any
 * one of the transactions to complete.
 *
 * This is a low-level function that most callers will
 * not need to use.
 *
 * @param device the ST7789 device handle
 * @param transactions the SPI transactions to execute
 * @param num_transactions the count of transactions to
 *        wait for
 * @return ESP_OK or an error code
 */
esp_err_t st7789_exec(
    st7789_device_handle_t device,
    spi_transaction_t *transactions,
    size_t num_transactions
);


/**
 * @brief Send a command to the LCD.
 *
 * Uses spi_device_polling_transmit, which waits
 * until the transfer is complete and blocks
 * both the thread and the CPU but is more
 * efficient than interrupt-based transfers for
 * small transmissions.
 *
 * This is a low-level function that most callers will
 * not need to use.
 *
 * @param device the ST7789 device handle
 * @param cmd the ST7789 command to send
 * @return ESP_OK or an error code
 */
esp_err_t st7789_send_cmd(
    st7789_device_handle_t device,
    const uint8_t cmd
);


/**
 * @brief Send data to the LCD.
 *
 * Uses spi_device_polling_transmit, which waits
 * until the transfer is complete and blocks
 * both the thread and the CPU but is more
 * efficient than interrupt-based transfers for
 * small transmissions.
 *
 * This is a low-level function that most callers will
 * not need to use.
 *
 * NOTE 1: This function is inappropriate for transfers
 * of large payloads, such as pixel data. Use the
 * interrupt-based methods instead (_enqueue, _await,
 * _exec).
 *
 * NOTE 2: If using DMA, the provided data must be in
 * RAM and not ROM since DMA cannot access ROM.
 *
 * NOTE 3: The number of bytes that can be sent is limited
 * by the `max_transfer_sz` value supplied via `bus_config_t`
 * when initializing the SPI bus. The default value of this
 * parameter is 4096 bytes.
 *
 * NOTE 4: This function uses the transaction's `tx_buffer`
 * rather than using the 64-byte `tx_data`. For this reason,
 * the provided data must remain accessible in memory until
 * the transaction is complete.
 *
 * @param device the ST7789 device handle
 * @param cmd the ST7789 command to send
 * @return ESP_OK or an error code
 */
esp_err_t st7789_send_data(
    st7789_device_handle_t device,
    const uint8_t *data,
    size_t len_bytes
);


/**
 * @brief Add the device to the SPI bus
 *
 * Adds the device to the SPI bus and
 * completes initialization of the device
 * descriptor using memory allocated on the
 * heap by this method.
 *
 * For further documentation, see
 * st7789_init_static(...).
 *
 * @param params configuration parameters
 */
st7789_device_handle_t st7789_init(
    const st7789_params_t *params
);


/**
 * @brief Add the device to the SPI bus
 *
 * Adds the device to the SPI bus and
 * completes initialization of the device
 * descriptor using memory allocated by the
 * caller.
 *
 * This method will add the device to the SPI
 * bus with a 6-transaction queue, SPI mode 0,
 * and a 10MHz clock speed. Future enhancements
 * may make these hard-coded parameters
 * customizable.
 *
 * NOTE 1: HSPI or VSPI are supported SPI hosts.
 * SPI1 is not recommended.
 *
 * Note 2: GPIO pins must be set in the
 * params before invoking this method. The rst
 * and bckl pins are optional and their absence
 * is gracefully handled in all of this library's
 * functions.
 *
 * Note 3: The SPI bus itself must already
 * be initialized before calling this
 * method. This library does not configure or
 * initialize the SPI bus. This library was tested
 * using the following bus config:
 *
 * SPI host: HSPI
 * mosi_io_num: GPIO 12
 * miso_io_num: GPIO 13 (not used by this library)
 * sclk_io_num: GPIO 14
 * quadwp_io_num: -1
 * quadhd_io_num: -1
 * max_transfer_sz: 4096 bytes
 * DMA Channel: 2
 *
 * @param params configuration parameters
 * @param device pointer to the allocated memory
 *        to populate with the st7789_device_t.
 */
void st7789_init_static(
    const st7789_params_t *params,
    st7789_device_handle_t device
);


/**
 * @brief Performs a hardware reset
 *
 * Sends hardware reset signal over the RST GPIO pin (if connected).
 * The entire sequence involves three separate delays totaling 500ms.
 * This method does not return until that sequence is complete.
 *
 * Returns immediately without performing any work if the RST GPIO
 * pin is GPIO_NUM_NC.
 *
 * @param device the ST7789 device handle
 */
void st7789_hwreset(st7789_device_handle_t device);


/**
 * @brief Performs a software reset
 *
 * Sends SWRESET (01h) in a single SPI transaction, followed by
 * a conservative 150ms delay to ensure stabilization prior to
 * subsequent commands. Does not return until this sequence is
 * complete.
 *
 * The datasheet calls for a 5ms delay before subsequent commands,
 * and a 120 ms delay before sending a SLPOUT command.
 *
 * @param device the ST7789 device handle
 * @return ESP_OK or an error code
 */
esp_err_t st7789_swreset(st7789_device_handle_t device);


/**
 * @brief Enters sleep mode
 *
 * Sends a SLPIN (10h) command in a single SPI transaction,
 * followed by a conservative 150 ms delay to ensure
 * stabilization prior to subsequent commands. Does not
 * return until this sequence is complete.
 *
 * The datasheet calls for a 120ms delay before calling
 * SLPOUT.
 *
 * @param device the ST7789 device handle
 * @return ESP_OK or an error code
 */
esp_err_t st7789_slpin(st7789_device_handle_t device);


/**
 * @brief Exits sleep mode
 *
 * Sends a SLPOUT (11h) command in a single SPI transaction,
 * followed by a conservative 150 ms delay to ensure
 * stabilization prior to subsequent commands. Does not
 * return until this sequence is complete.
 *
 * The datasheet calls for a 120ms delay before calling
 * SLPIN.
 *
 * @param device the ST7789 device handle
 * @return ESP_OK or an error code
 */
esp_err_t st7789_slpout(st7789_device_handle_t device);


/**
 * @brief Turns on partial mode.
 *
 * Sends a PTLON (12h) command in a single SPI transaction,
 * followed by a brief 10ms delay for stablization.
 *
 * @param device the ST7789 device handle
 * @return ESP_OK or an error code
 */
esp_err_t st7789_ptlon(st7789_device_handle_t device);


/**
 * @brief Turns off partial mode.
 *
 * Sends a NORON (13h) command in a single SPI transaction,
 * followed by a brief 10ms delay for stablization.
 *
 * @param device the ST7789 device handle
 * @return ESP_OK or an error code
 */
esp_err_t st7789_noron(st7789_device_handle_t device);


/**
 * @brief Disables color inversion
 *
 * Sends the INVOFF (21h) command in a single SPI transaction.
 *
 * @param device the ST7789 device handle
 * @return ESP_OK or an error code
 */
esp_err_t st7789_invoff(st7789_device_handle_t device);


/**
 * @brief Enables color inversion
 *
 * Sends the INVON (20h) command in a single SPI transaction.
 *
 * Each pixel's color value will be inverted bitwise.
 *
 * @param device the ST7789 device handle
 * @return ESP_OK or an error code
 */
esp_err_t st7789_invon(st7789_device_handle_t device);


/**
 * @brief Sets the gamma curve
 *
 * Sends a GAMSET command (26h) and 8 bytes of data.
 *
 * @param device the ST7789 device handle
 * @param config 8 bytes representing gamma curve, built
 *        up by bitwise OR of any combination of the
 *        ST7789_CFG_GC_* values or none at all (0x00)
 * @return ESP_OK or an error code
 */
esp_err_t st7789_gamset(
    st7789_device_handle_t device,
    uint8_t config
);


/**
 * @brief Turns the display off
 *
 * Sends a DSPOFF (28h) command in a single SPI transaction,
 * followed by a conservative 100ms delay for stabilization.
 *
 * @param device the ST7789 device handle
 * @return ESP_OK or an error code
 */
esp_err_t st7789_dispoff(st7789_device_handle_t device);


/**
 * @brief Turns the display on
 *
 * Sends a DISPON (29h) command in a single SPI transaction,
 * followed by a conservative 100ms delay for stabilization.
 *
 * @param device the ST7789 device handle
 * @return ESP_OK or an error code
 */
esp_err_t st7789_dispon(st7789_device_handle_t device);


/**
 * @brief Sets the column (x-axis) address range
 *
 * Sends a CASET (2Ah) command in one SPI transaction
 * and four configuration bytes in a second SPI
 * transaction.
 *
 * @param device the ST7789 device handle
 * @param x_min the lowest-valued x-axis address
 * @param x_max the highest-valued x-axis address,
 *        where x_max >= x_min
 * @return ESP_OK or an error code
 */
esp_err_t st7789_caset(
    st7789_device_handle_t device,
    uint8_t x_min,
    uint8_t x_max
);


/**
 * @brief Sets the row (y-axis) address range
 *
 * Sends a RASET (3Bh) command in one SPI transaction
 * and four configuration bytes in a second SPI
 * transaction.
 *
 * @param device the ST7789 device handle
 * @param y_min the lowest-valued y-axis address
 * @param y_max the highest-valued y-axis address,
 *        where y_max >= y_min
 * @return ESP_OK or an error code
 */
esp_err_t st7789_raset(
    st7789_device_handle_t device,
    uint8_t y_min,
    uint8_t y_max
);


/**
 * @brief Writes pixel data to device RAM
 *
 * Sends a RAMWR (2Ch) command in one SPI transaction
 * and the buffer in a second SPI transaction.
 *
 * The location where the pixels will be written
 * must first have been set with st7789_caset(...)
 * and st7789_raset(...).
 *
 * For large amounts of data, the data transaction
 * will be sent using interrupts, allowing the CPU
 * to do other work while the task is blocked
 * waiting for a completion signal. Polling is used
 * for small amounts of data.
 *
 * The data in the buffer needs to be arranged such
 * that the first two bytes read represent the color
 * of the lower-left most pixel in the space, with
 * subsequent reads moving right in the space before
 * wrapping to the left-most position of the next
 * line.
 *
 * NOTE 1: Beware that the ESP32 stores uint16_t
 * values most significant byte first. Thus, a
 * RGB/565 uint16_t value is arranged in memory
 * (and sent byte-by-byte to the TFT controller) as
 * G[2:0]BRG[5:3]. Obviously, this will not produce
 * the desired effect. Pre-inverting the bytes
 * when building color values is one of several
 * possible solutions to this problem.
 *
 * NOTE 2: If using DMA, the provided data must be in
 * RAM and not ROM since DMA cannot access ROM.
 *
 * NOTE 3: The number of bytes that can be sent is limited
 * by the `max_transfer_sz` value supplied via `bus_config_t`
 * when initializing the SPI bus. The default value of this
 * parameter is 4096 bytes.
 *
 * NOTE 4: This function uses the transaction's `tx_buffer`
 * rather than using the 64-byte `tx_data`. For this reason,
 * the provided data must remain accessible in memory until
 * the transaction is complete.
 *
 * @param device the ST7789 device handle
 * @param buffer the pixel data to write
 * @param num_pixels the number of pixels to write
 * @return ESP_OK or an error code
 */
esp_err_t st7789_ramwr(
    st7789_device_handle_t device,
    uint16_t * buffer,
    size_t num_pixels
);


/**
 * @brief Sets memory data access configuration
 *
 * Sends a MADCTL (36h) command in one SPI transaction and
 * one configuration byte in a second SPI transaction.
 *
 * The config value 0x00 represents:
 * - No mirroring of x or y axis
 * - No exchange of x and y axis
 * - Data transmitted from IC to LCD top-to-bottom and
 *   left-to-right
 * - LCD panel pixels are arranged in R-G-B order
 *
 * Bitwise or (|) 0x00 with the following to set up the
 * configuration:
 * - ST7789_CFG_MIRROR_X to mirror the x axis ("MX")
 * - ST7789_CFG_MIRROR_Y to mirror the y axis ("MY")
 * - ST7789_CFG_EXCHANGE_XY to exchange the x and y axis
 *   ("MV")
 * - ST7789_CFG_REFRESH_RTL to transmit data from IC to LCD
 *   right-to-left ("MH")
 * - ST7789_CFG_REFRESH_BTT to transmit data from IC to LCD
 *   bottom-to-top ("ML")
 * - ST7789_CFG_BGR for LCD panel pixels arranged in B-G-R
 *   order ("RGB")
 *
 * Note that names above quoted in parenthesis are the
 * associated bitfield names as described in the datasheet.
 *
 * @param device the ST7789 device handle
 * @param config see description above
 * @return ESP_OK or an error code
 */
esp_err_t st7789_madctl(
    st7789_device_handle_t device,
    uint8_t config
);


/**
 * @brief Sets the color mode configuration
 *
 * Sends a COLMOD (3Ah) command in one SPI transaction and
 * one configuration byte in a second SPI transaction.
 *
 * @param device the ST7789 device handle
 * @param config ST7789_CFG_12_BIT_COLOR,
 *        ST7789_CFG_16_BIT_COLOR or ST7789_CFG_18_BIT_COLOR
 * @return ESP_OK or an error code
 */
esp_err_t st7789_colmod(
    st7789_device_handle_t device,
    uint8_t config
);


/**
 * @brief Sets the backlight illumination level full on or full off
 *
 * Returns without taking any action if the backlight
 * pin is not connected (GPIO_NUM_NC).
 *
 * @param device the ST7789 device handle
 * @param level st7789_CFG_BCKL_ON, st7789_CFG_BCKL_OFF
 */
void st7789_backlight(
    st7789_device_handle_t device,
    uint8_t level
);


/**
 * @brief Writes pixel data to device RAM
 *
 * This higher-level function sends the command and
 * data sequence necessary to write pixel data to a
 * defines space within the frame memory buffer.
 *
 * The data in the buffer needs to be arranged such
 * that the first two bytes read represent the color
 * of the lower-left most pixel in the space, with
 * subsequent reads moving right in the space before
 * wrapping to the left-most position of the next
 * line.
 *
 * NOTE 1: Beware that the ESP32 stores uint16_t
 * values most significant byte first. Thus, a
 * RGB/565 uint16_t value is arranged in memory
 * (and sent byte-by-byte to the TFT controller) as
 * G[2:0]BRG[5:3]. Obviously, this will not produce
 * the desired effect. Pre-inverting the bytes
 * when building color values is one of several
 * possible solutions to this problem.
 *
 * NOTE 2: If using DMA, the provided data must be in
 * RAM and not ROM since DMA cannot access ROM.
 *
 * NOTE 3: The number of bytes that can be sent is limited
 * by the `max_transfer_sz` value supplied via `bus_config_t`
 * when initializing the SPI bus. The default value of this
 * parameter is 4096 bytes.
 *
 * NOTE 4: This function uses the transaction's `tx_buffer`
 * rather than using the 64-byte `tx_data`. For this reason,
 * the provided data must remain accessible in memory until
 * the transaction is complete.
 *
 * @param device the ST7789 device handle
 * @param buffer the pixel data to write
 * @param num_pixes the number of pixels to write
 * @return ESP_OK or an error code
 */
esp_err_t st7789_paint(
    st7789_device_handle_t device,
    uint16_t * buffer,
    uint8_t x_min,
    uint8_t x_max,
    uint8_t y_min,
    uint8_t y_max
);


#ifdef __cplusplus
}
#endif

#endif // ST7789_H
