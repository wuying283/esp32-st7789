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

#include "st7789.h"
#include <string.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>


static const char * ST7789_TAG = "st7789";


void st7789_cmd_init(
    st7789_device_handle_t device,
    spi_transaction_t *transaction,
    uint8_t cmd)
{
    memset(transaction, 0, sizeof(spi_transaction_t));
    transaction->user = &(device->dc_cmd);
    transaction->flags = SPI_TRANS_USE_TXDATA;
    transaction->tx_data[0] = cmd;
    transaction->length = 8;
}


void st7789_data_init(
    st7789_device_handle_t device,
    spi_transaction_t *transaction,
    const uint8_t *data,
    size_t len_bytes)
{
    memset(transaction, 0, sizeof(spi_transaction_t));
    transaction->user = &(device->dc_data);
    transaction->tx_buffer = data;
    transaction->length = len_bytes * 8;
}


esp_err_t st7789_enqueue(
    st7789_device_handle_t device,
    spi_transaction_t *transactions,
    size_t num_transactions)
{
    uint8_t i;
    esp_err_t ret = ESP_OK;
    for (i = 0; i < num_transactions; i++) {
        ret = spi_device_queue_trans(
            device->spi_device,
            &transactions[i],
            portMAX_DELAY
        );
        if (ret != ESP_OK) {
            ESP_LOGE(
                ST7789_TAG,
                "str7789_enqueue(...) failed in spi_device_queue_trans(...): %s",
                esp_err_to_name(ret)
            );
            break;
        }
    }
    return ret;
}


esp_err_t st7789_await(
    st7789_device_handle_t device,
    size_t num_transactions,
    TickType_t ticks_to_wait)
{
    spi_transaction_t *transaction;
    esp_err_t ret = ESP_OK;
    esp_err_t inner_ret;
    for (size_t i = 0; i < num_transactions; i++) {
        inner_ret = spi_device_get_trans_result(
            device->spi_device, &transaction,
            ticks_to_wait
        );
        if (inner_ret != ESP_OK) {
            ESP_LOGE(
                ST7789_TAG,
                "st7789_await(...) failed in spi_device_get_trans_result(...): %s",
                esp_err_to_name(inner_ret)
            );
            if (ret == ESP_OK) ret = inner_ret;
        }
    }
    return ret;
}


esp_err_t st7789_exec(
    st7789_device_handle_t device,
    spi_transaction_t *transactions,
    size_t num_transactions)
{
    esp_err_t ret;
    ret = st7789_enqueue(device, transactions, num_transactions);
    if (ret != ESP_OK) {
        ESP_LOGE(
            ST7789_TAG,
            "st7789_exec(...) failed in st7789_enqueue(...): %s",
            esp_err_to_name(ret)
        );
    } else {
        ret = st7789_await(device, num_transactions, 10000.0 / portTICK_PERIOD_MS);
        if (ret != ESP_OK) {
            ESP_LOGE(
                ST7789_TAG,
                "st7789_exec(...) failed in st7789_await(...): %s",
                esp_err_to_name(ret)
            );
        }
    }
    return ret;
}


esp_err_t st7789_send_cmd(
    st7789_device_handle_t device,
    const uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t;
    st7789_cmd_init(device, &t, cmd);
    ret = spi_device_polling_transmit(device->spi_device, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(
            ST7789_TAG,
            "st7789_send_cmd(...) failed in spi_device_polling_transmit(...): %s",
            esp_err_to_name(ret)
        );
    }
    return ret;
}


esp_err_t st7789_send_data(
    st7789_device_handle_t device,
    const uint8_t *data,
    size_t len_bytes)
{
    esp_err_t ret = ESP_OK;
    spi_transaction_t t;
    if (len_bytes > 0)  {
        st7789_data_init(device, &t, data, len_bytes);
        ret = spi_device_polling_transmit(device->spi_device, &t);
        if (ret != ESP_OK) {
            ESP_LOGE(
                ST7789_TAG,
                "st7789_send_data(...) failed in spi_device_polling_transmit(...): %s",
                esp_err_to_name(ret)
            );
        }
    }
    return ret;
}


void st7789_spi_pre_transfer_callback_isr(spi_transaction_t *t)
{
    st7789_dc_setting_t * dc = (st7789_dc_setting_t *) t->user;
    gpio_set_level(dc->gpio_dc, dc->setting);
}


st7789_device_handle_t st7789_init(
    const st7789_params_t *params)
{
    st7789_device_handle_t device;
    device = (st7789_device_handle_t) malloc(sizeof(st7789_device_t));
    if (!device) {
        ESP_LOGE(
            ST7789_TAG,
            "ST7789 failed to allocate memory for device descriptor"
        );
        esp_restart();
    }
    st7789_init_static(params, device);
    return device;
}


void st7789_init_static(
    const st7789_params_t *params,
    st7789_device_handle_t device)
{
    if (params->host != HSPI_HOST && params->host != VSPI_HOST) {
        ESP_LOGE(ST7789_TAG, "ST7789 requires either HSPI or VSPI host");
        esp_restart();
    }
    if (params->gpio_dc == GPIO_NUM_NC) {
        ESP_LOGE(ST7789_TAG, "ST7789 requires a connected D/C pin");
        esp_restart();
    }

    spi_device_interface_config_t dev_cfg = {
        .mode = 0,
        .clock_speed_hz = SPI_MASTER_FREQ_10M,
        .spics_io_num = params->gpio_cs,
        .queue_size = 6,
        .pre_cb = st7789_spi_pre_transfer_callback_isr,
    };
    esp_err_t ret = spi_bus_add_device(
        params->host,
        &dev_cfg,
        &(device->spi_device)
    );
    ESP_ERROR_CHECK(ret);

    // Initialize non-SPI GPIOs
    gpio_set_direction(params->gpio_dc, GPIO_MODE_OUTPUT);
    device->dc_cmd.gpio_dc = params->gpio_dc;
    device->dc_cmd.setting = 0;
    device->dc_data.gpio_dc = params->gpio_dc;
    device->dc_data.setting = 1;

    if (params->gpio_rst != GPIO_NUM_NC)
        gpio_set_direction(params->gpio_rst, GPIO_MODE_OUTPUT);
    device->gpio_rst = params->gpio_rst;

    if (params->gpio_bckl != GPIO_NUM_NC)
        gpio_set_direction(params->gpio_bckl, GPIO_MODE_OUTPUT);
    device->gpio_bckl = params->gpio_bckl;
}


void st7789_hwreset(st7789_device_handle_t device)
{
    if (device->gpio_rst != GPIO_NUM_NC) {
        gpio_set_level(device->gpio_rst, 1);
        vTaskDelay(100 / portTICK_RATE_MS);
        gpio_set_level(device->gpio_rst, 0);
        vTaskDelay(100 / portTICK_RATE_MS);
        gpio_set_level(device->gpio_rst, 1);
        vTaskDelay(300 / portTICK_RATE_MS);
    }
}


static esp_err_t st7789_with_data(
    st7789_device_handle_t device,
    const char *fn_name,
    uint8_t cmd,
    void *data,
    size_t len_bytes)
{
    esp_err_t ret = st7789_send_cmd(device, cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(
            ST7789_TAG,
            "%s(...) failed on st7789_send_cmd(...): %s",
            fn_name,
            esp_err_to_name(ret)
        );
    } else if (len_bytes > 0) {
        ret = st7789_send_data(device, data, len_bytes);
        if (ret != ESP_OK) {
            ESP_LOGE(
                ST7789_TAG,
                "%s(...) failed on st7789_send_data(...): %s",
                fn_name,
                esp_err_to_name(ret)
            );
        }
    }
    return ret;
}


static esp_err_t st7789_set(
    st7789_device_handle_t device,
    const char *fn_name,
    uint8_t cmd)
{
    return st7789_with_data(
        device,
        fn_name,
        cmd,
        (void *) 0,
        0
    );
}


esp_err_t st7789_swreset(st7789_device_handle_t device)
{
    esp_err_t ret = st7789_set(
        device,
        "st7789_swreset",
        ST7789_CMD_SWRESET
    );
    vTaskDelay(150.0 / portTICK_RATE_MS);
    return ret;
}


esp_err_t st7789_slpin(st7789_device_handle_t device)
{
    esp_err_t ret = st7789_set(
        device,
        "st7789_slpin",
        ST7789_CMD_SLPIN
    );
    vTaskDelay(150.0 / portTICK_RATE_MS);
    return ret;
}


esp_err_t st7789_slpout(st7789_device_handle_t device) {
    esp_err_t ret = st7789_set(
        device,
        "st7789_slpout",
        ST7789_CMD_SLPOUT
    );
    vTaskDelay(150.0 / portTICK_RATE_MS);
    return ret;
}


esp_err_t st7789_noron(st7789_device_handle_t device)
{
    esp_err_t ret = st7789_set(
        device,
        "st7789_noron",
        ST7789_CMD_NORON
    );
    vTaskDelay(10.0 / portTICK_RATE_MS);
    return ret;
}


esp_err_t st7789_ptlon(st7789_device_handle_t device)
{
    esp_err_t ret = st7789_set(
        device,
        "st7789_ptlon",
        ST7789_CMD_PTLON
    );
    vTaskDelay(10.0 / portTICK_RATE_MS);
    return ret;
}


esp_err_t st7789_invoff(st7789_device_handle_t device)
{
    return st7789_set(
        device,
        "st7789_invoff",
        ST7789_CMD_INVOFF
    );
}


esp_err_t st7789_invon(st7789_device_handle_t device)
{
    return st7789_set(
        device,
        "st7789_invon",
        ST7789_CMD_INVON
    );
}


esp_err_t st7789_gamset(
    st7789_device_handle_t device,
    uint8_t *config)
{
    uint8_t param = config & 0x0F;
    return st7789_with_data(
        device,
        "st7789_gamset",
        ST7789_CMD_GAMSET,
        &param,
        1
    );
}


esp_err_t st7789_dispoff(st7789_device_handle_t device)
{
    esp_err_t ret = st7789_set(
        device,
        "st7789_dspoff",
        ST7789_CMD_DISPOFF
    );
    vTaskDelay(10.0 / portTICK_RATE_MS);
    return ret;
}


esp_err_t st7789_dispon(st7789_device_handle_t device)
{
    esp_err_t ret = st7789_set(
        device,
        "st7789_dispon",
        ST7789_CMD_DISPON
    );
    vTaskDelay(10.0 / portTICK_RATE_MS);
    return ret;
}


esp_err_t st7789_caset(
    st7789_device_handle_t device,
    uint8_t x_min,
    uint8_t x_max
) {
    uint8_t buffer[4] = { 0x00, x_min, 0x00, x_max };
    return st7789_with_data(
        device,
        "st7789_caset",
        ST7789_CMD_CASET,
        buffer,
        4
    );
}


esp_err_t st7789_raset(
    st7789_device_handle_t device,
    uint8_t y_min,
    uint8_t y_max)
{
    uint8_t buffer[4] = { 0x00, y_min, 0x00, y_max };
    return st7789_with_data(
        device,
        "st7789_raset",
        ST7789_CMD_RASET,
        buffer,
        4
    );
}


esp_err_t st7789_ramwr(
    st7789_device_handle_t device,
    uint16_t * buffer,
    size_t num_pixels)
{
    if (num_pixels == 0) return ESP_OK;
    esp_err_t ret = st7789_send_cmd(device, ST7789_CMD_RAMWR);
    if (ret != ESP_OK) {
        ESP_LOGE(
            ST7789_TAG,
            "st7789_ramwr(...) failed on st7789_send_cmd(...): %s",
            esp_err_to_name(ret)
        );
    } else if (num_pixels < 32) {
        // For small amounts of data, use polling
        ret = st7789_send_data(device, (void *) buffer, num_pixels * 2);
        if (ret != ESP_OK) {
            ESP_LOGE(
                ST7789_TAG,
                "st7789_ramwr(...) failed on st7789_send_data(...): %s",
                esp_err_to_name(ret)
            );
        }
    } else {
        // For large amounts of data, use interrupts
        spi_transaction_t trans;
        st7789_data_init(device, &trans, (void *) buffer, num_pixels * 2);
        esp_err_t ret = st7789_exec(device, &trans, 1);
        if (ret != ESP_OK) {
            ESP_LOGE(
                ST7789_TAG,
                "st7789_ramwr(...) failed in st7789_exec: %s",
                esp_err_to_name(ret)
            );
        }
    }
    return ret;
}


esp_err_t st7789_madctl(
    st7789_device_handle_t device,
    uint8_t config)
{
    return st7789_with_data(
        device,
        "st7789_madctl",
        ST7789_CMD_MADCTL,
        &config,
        1
    );
}


esp_err_t st7789_colmod(
    st7789_device_handle_t device,
    uint8_t config)
{
    return st7789_with_data(
        device,
        "st7789_colmod",
        ST7789_CMD_COLMOD,
        &config,
        1
    );
}


void st7789_backlight(
    st7789_device_handle_t device,
    uint8_t level)
{
    if (device->gpio_bckl == GPIO_NUM_NC)
        return;
    if (level == 0 || level == 1)
        gpio_set_level(device->gpio_bckl, level);
}


esp_err_t st7789_paint(
    st7789_device_handle_t device,
    uint16_t * buffer,
    uint8_t x_min,
    uint8_t x_max,
    uint8_t y_min,
    uint8_t y_max)
{
    spi_transaction_t trans[6];

    st7789_cmd_init(device, &trans[0], ST7789_CMD_CASET);
    uint8_t col_range[4] = {
        0x00, x_min,
        0x00, x_max
    };
    st7789_data_init(device, &trans[1], col_range, 4);

    st7789_cmd_init(device, &trans[2], ST7789_CMD_RASET);
    uint8_t row_range[4] = {
        0x00, y_min,
        0x00, y_max
    };
    st7789_data_init(device, &trans[3], row_range, 4);

    st7789_cmd_init(device, &trans[4], ST7789_CMD_RAMWR);
    size_t num_bytes =
        (((x_max - x_min) + 1) * ((y_max - y_min) + 1)) * 2;
    st7789_data_init(device, &trans[5], (uint8_t *) buffer, num_bytes);

    esp_err_t ret = st7789_exec(device, trans, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(
            ST7789_TAG,
            "st7789_paint(...) failed in st7789_exec: %s",
            esp_err_to_name(ret)
        );
    }
    return ret;
}
