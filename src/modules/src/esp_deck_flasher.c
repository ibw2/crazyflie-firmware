/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @file esp_deck_flasher.c
 * Handles flashing of binaries on the ESP32
 *  
 */

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#define DEBUG_MODULE "ESPFL"
#include "debug.h"

#include "FreeRTOS.h"
#include "aideck.h"
#include "deck.h"
#include "esp_deck_flasher.h"
#include "esp_rom_bootloader.h"
#include "uart2.h"

static bool inBootloaderMode = true;
static bool hasStarted = false;

bool espDeckFlasherCheckVersionAndBoot()
{
  hasStarted = true;
  return true;
}

static uint32_t sequenceNumber;
static uint32_t numberOfDataPackets;
static uint8_t sendBuffer[ESP_MTU + 10 + 16];
static uint8_t overshoot;
static uint32_t sendBufferIndex;

bool espDeckFlasherWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t *buffer)
{
  if (memAddr == 0)
  {
    uart2Init(115200);
    espRomBootloaderInit();
    if (!espRomBootloaderSync(&sendBuffer[0]))
    {
      DEBUG_PRINT("Sync failed\n");
      return false;
    }
    if (!espRomBootloaderSpiAttach(&sendBuffer[0]))
    {
      DEBUG_PRINT("SPI attach failed\n");
      return false;
    }

    numberOfDataPackets = (((ESP_BITSTREAM_SIZE - 1) / ESP_MTU) + ((ESP_BITSTREAM_SIZE / ESP_MTU) < 0 ? 0 : 1)) >> 0;
    DEBUG_PRINT("Will send %lu data packets\n", numberOfDataPackets);

    if (!espRomBootloaderFlashBegin(&sendBuffer[0], numberOfDataPackets, ESP_BITSTREAM_SIZE, ESP_FW_ADDRESS)) // placeholder erase size
    {
      DEBUG_PRINT("Failed to start flashing\n");
      return 0;
    }
    sequenceNumber = 0;
    sendBufferIndex = 0;
  }

  // assemble buffer until full
  if (sendBufferIndex + writeLen >= ESP_MTU)
  {
    overshoot = sendBufferIndex + writeLen - ESP_MTU;
    memcpy(&sendBuffer[9 + 16 + sendBufferIndex], buffer, writeLen - overshoot);
    sendBufferIndex += writeLen - overshoot;
  }
  else
  {
    memcpy(&sendBuffer[9 + 16 + sendBufferIndex], buffer, writeLen);
    sendBufferIndex += writeLen;
  }

  // send buffer if full
  if (sendBufferIndex == ESP_MTU || ((sequenceNumber == numberOfDataPackets - 1) && (sendBufferIndex == ESP_BITSTREAM_SIZE % ESP_MTU)))
  {
    if (!espRomBootloaderFlashData(&sendBuffer[0], sendBufferIndex, sequenceNumber))
    {
      DEBUG_PRINT("Flash write failed\n");
      return false;
    }
    else
    {
      DEBUG_PRINT("Flash write successful\n");
    }

    // put overshoot into send buffer for next send & update sendBufferIndex
    if (overshoot)
    {
      memcpy(&sendBuffer[9 + 16 + 0], &buffer[writeLen - overshoot], overshoot);
      sendBufferIndex = overshoot;
      overshoot = 0;
    }
    else
    {
      sendBufferIndex = 0;
    }

    // increment sequence number
    sequenceNumber++;

    // if very last radio packet triggered overshoot, send padded carry buffer
    if (((sequenceNumber == numberOfDataPackets - 1) && (sendBufferIndex == ESP_BITSTREAM_SIZE % ESP_MTU)))
    {

      if (!espRomBootloaderFlashData(&sendBuffer[0], sendBufferIndex, sequenceNumber))
      {
        DEBUG_PRINT("Flash write failed\n");
        return false;
      }
    }
  }

  return true;
}

uint8_t espDeckFlasherPropertiesQuery()
{
  uint8_t result = 0;

  if (hasStarted)
  {
    result |= DECK_MEMORY_MASK_STARTED;
  }

  if (inBootloaderMode)
  {
    result |= DECK_MEMORY_MASK_BOOT_LOADER_ACTIVE | DECK_MEMORY_MASK_UPGRADE_REQUIRED;
  }

  return result;
}
