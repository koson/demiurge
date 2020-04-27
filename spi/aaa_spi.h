/*
  Copyright 2019, Awesome Audio Apparatus.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

      https://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
      limitations under the License.
*/

#ifndef _DEMIURGE_MY_SPI_H_
#define _DEMIURGE_MY_SPI_H_

#include <driver/spi_common.h>
#include <driver/spi_master.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DEMIURGE_ESP_ERR(base, x)      ((esp_err_t) ((base) + (x)))
#define DEMIURGE_ESP_ERR_USER_BASE    (0x40000000)
#define DEMIURGE_ESP_ERR_SPI_BASE     (DEMIURGE_ESP_ERR_USER_BASE + 0x1000 * 1)
#define DEMIURGE_ESP_ERR_SPI_SPI1_IS_NOT_SUPPORTED    DEMIURGE_ESP_ERR_SPI(1)
#define DEMIURGE_ESP_ERR_SPI_INVALID_HOST_NUMBER      DEMIURGE_ESP_ERR_SPI(2)
#define DEMIURGE_ESP_ERR_SPI_INVALID_DMA_CHANNEL      DEMIURGE_ESP_ERR_SPI(3)

#define DEMIURGE_ESP_ERR_SPI(x)        DEMIURGE_ESP_ERR(DEMIURGE_ESP_ERR_SPI_BASE, (x))
#define DEMIURGE_ESP_ERR_SPI_HOST_ALREADY_IN_USE      DEMIURGE_ESP_ERR_SPI(4)
#define DEMIURGE_ESP_ERR_SPI_DMA_ALREADY_IN_USE       DEMIURGE_ESP_ERR_SPI(5)

esp_err_t
aaa_spi_prepare_circular(const spi_host_device_t spiHostDevice, const int dma_chan,
                         const lldesc_t *lldescs_out, const lldesc_t *lldescs_in,
                         const long dmaClockSpeedInHz, const gpio_num_t mosi_gpio_num,
                         const gpio_num_t miso_gpio_num, const gpio_num_t sclk_gpio_num, const int waitCycle);

esp_err_t aaa_spi_release_circular_buffer(spi_host_device_t host, int dma_chan);

spi_dev_t *aaa_spi_get_hw_for_host(spi_host_device_t host);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
