/*
 * Adafruit PN532 library adapted to use in NRF51 and NRF52
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Adafruit Industries
 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
//#include "sdk_config.h"
#ifndef ADAFRUIT_PN532_ENABLED
#define ADAFRUIT_PN532_ENABLED 1
#endif

#ifndef TAG_DETECT_TIMEOUT
#define TAG_DETECT_TIMEOUT  4000
#define SHORT_TIMEOUT       500
#define VERY_SHORT_TIMEOUT  10
#endif

#ifndef PN532_IRQ
#define PN532_IRQ               0
#endif

#ifndef PN532_RESET
#define PN532_RESET              0
#endif


#ifndef PN532_CONFIG_TWI_INSTANCE
#define PN532_CONFIG_TWI_INSTANCE 0
#endif

#ifndef ADAFRUIT_PN532_LOG_ENABLED
#define ADAFRUIT_PN532_LOG_ENABLED 0
#endif

#ifndef ADAFRUIT_PN532_LOG_LEVEL
#define ADAFRUIT_PN532_LOG_LEVEL 3
#endif

#ifndef ADAFRUIT_PN532_INFO_COLOR
#define ADAFRUIT_PN532_INFO_COLOR 0
#endif

#ifndef PN532_PACKBUFF_SIZE
#define PN532_PACKBUFF_SIZE 256
#endif

#if ADAFRUIT_PN532_ENABLED
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "adafruit_pn532.h"
#include "board.h"
#include "app_error.h"

#define STM_LOG_MODULE_NAME adafruit_pn532
#if ADAFRUIT_PN532_LOG_ENABLED
#define STM_LOG_LEVEL       ADAFRUIT_PN532_LOG_LEVEL
#define STM_LOG_INFO_COLOR  ADAFRUIT_PN532_INFO_COLOR
#else // ADAFRUIT_PN532_LOG_ENABLED
#define STM_LOG_LEVEL       0
#endif // ADAFRUIT_PN532_LOG_ENABLED



/**@brief Function for decoding a uint32 value in big-endian format.
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value.
 */
static __INLINE uint32_t uint32_big_decode(const uint8_t * p_encoded_data)
{
    return ( (((uint32_t)((uint8_t *)p_encoded_data)[0]) << 24) |
             (((uint32_t)((uint8_t *)p_encoded_data)[1]) << 16) |
             (((uint32_t)((uint8_t *)p_encoded_data)[2]) << 8)  |
             (((uint32_t)((uint8_t *)p_encoded_data)[3]) << 0) );
}

// Type 2 Tag page/block read/write restrictions.
#define T2T_MAX_READ_PAGE_NUMBER                          255
#define T2T_MIN_WRITE_PAGE_NUMBER                         4
#define T2T_MAX_WRITE_PAGE_NUMBER                         255

// Lengths and offsets for specific commands and responses.
#define COMMAND_GETFIRMWAREVERSION_LENGTH                     1
#define REPLY_GETFIRMWAREVERSION_LENGTH                       (5 + PN532_FRAME_OVERHEAD)

#define COMMAND_SAMCONFIGURATION_LENGTH                       4
#define REPLY_SAMCONFIGURATION_LENGTH                         (1 + PN532_FRAME_OVERHEAD)

#define COMMAND_POWERDOWN_BASE_LENGTH                         2  // No GenerateIRQ parameter.
#define REPLY_POWERDOWN_LENGTH                                (2 + PN532_FRAME_OVERHEAD)

#define COMMAND_RFCONFIGURATION_MAXRETRIES_LENGTH             5
#define COMMAND_RFCONFIGURATION_RFFIELD_LENGTH                3
#define REPLY_RFCONFIGURATION_LENGTH                          (1 + PN532_FRAME_OVERHEAD)

#define COMMAND_INLISTPASSIVETARGET_BASE_LENGTH               3
#define REPLY_INLISTPASSIVETARGET_106A_TARGET_LENGTH          (17 + PN532_FRAME_OVERHEAD)
#define REPLY_INLISTPASSIVETARGET_106A_NBTG_OFFSET            7
#define REPLY_INLISTPASSIVETARGET_106A_TG_OFFSET              8
#define REPLY_INLISTPASSIVETARGET_106A_SENS_RES_BYTE_1_OFFSET 10
#define REPLY_INLISTPASSIVETARGET_106A_SENS_RES_BYTE_2_OFFSET 9
#define REPLY_INLISTPASSIVETARGET_106A_SEL_RES_OFFSET         11
#define REPLY_INLISTPASSIVETARGET_106A_UID_LEN_OFFSET         12
#define REPLY_INLISTPASSIVETARGET_106A_UID_OFFSET             13

#define COMMAND_INDATAEXCHANGE_BASE_LENGTH                    2
#define REPLY_INDATAEXCHANGE_BASE_LENGTH                      (2 + PN532_FRAME_OVERHEAD)

// Configuration parameters for SAMCONFIGURATION command.
#define SAMCONFIGURATION_MODE_NORMAL                          0x01
#define SAMCONFIGURATION_MODE_VIRTUAL_CARD                    0x02
#define SAMCONFIGURATION_MODE_WIRED_CARD                      0x03
#define SAMCONFIGURATION_MODE_DUAL_CARD                       0x04

#define SAMCONFIGURATION_IRQ_ENABLED                          0x01
#define SAMCONFIGURATION_IRQ_DISABLED                         0x00

// Configuration parameters for POWERDOWN command.
#define POWERDOWN_WAKEUP_IRQ                                  0x80
#define POWERDOWN_WAKEUP_SPI                                  0x20

// Configuration parameters for RFCONFIGURATION command.
#define RFCONFIGURATION_CFGITEM_RFFIELD                       0x01
#define RFCONFIGURATION_CFGITEM_MAXRETRIES                    0x05
#define RFCONFIGURATION_RFFIELD_ON                            0x01
#define RFCONFIGURATION_RFFIELD_OFF                           0x00

// Error mask for the status mask in INDATAEXCHANGE frame.
#define PN532_STATUS_ERROR_MASK                               0x3F

// Size of the PN532 size packet.
#define PN532_ACK_PACKET_SIZE                                 6

// Default time-out for read_passive_target_id (time required for field scan).
#define PN532_DEFAULT_WAIT_FOR_READY_TIMEOUT                  25

/**
 * @brief Information about the communication between the host and the Adafruit PN532 Shield.
 */
typedef struct
{
    uint8_t ss;            // !< Slave select signal for SPI.
    uint8_t clk;           // !< Clock signal for SPI.
    uint8_t mosi;          // !< Master output, slave input signal for SPI.
    uint8_t miso;          // !< Master input, slave output signal for SPI.
    uint8_t irq;           // !< Interrupt pin for Adafruit.
    uint8_t reset;         // !< Reset pin for Adafruit.
    uint8_t in_listed_tag; // !< Tag number of in listed tags.
    bool    using_spi;     // !< True if using SPI, false if using I2C.
    bool    hardware_spi;  // !< True if using hardware SPI, false if using software SPI.
} adafruit_pn532;


// ACK frame format.
static const uint8_t m_pn532_ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
// Firmware version reply frame format (preamble to command byte).
static const uint8_t m_pn532_rsp_firmware_ver[] = {0x00, 0xFF, 0x06, 0xFA, 0xD5, 0x03};

static adafruit_pn532 m_pn532_object = {
    .clk          = 0,
    .miso         = 0,
    .mosi         = 0,
    .ss           = 0,
    .irq          = PN532_IRQ,
    .reset        = PN532_RESET,
    .using_spi    = false,
    .hardware_spi = false
};


static uint8_t m_pn532_packet_buf[PN532_PACKBUFF_SIZE];

static uint8_t m_pn532_rxtx_buffer[PN532_PACKBUFF_SIZE]; /// Buffer for low level communication.

static bool m_lib_initialized = false;


/**
 * @brief Function to configure pins in host chip.
 *
 * This function configures specific pins to interact with the PN532 module.
 */
static void adafruit_pn532_pin_setup(void)
{

}


/**
 * @brief Function to calculate the checksum byte.
 *
 * This function calculates the checksum byte, so that the sum of all verified bytes
 * and the checksum byte is equal to 0.
 *
 * @param  current_sum[in]  Sum of all bytes used to calculate checksum.
 *
 * @retval Value of the checksum byte.
 */
static uint8_t adafruit_pn532_cs_complement_calc(uint8_t current_sum)
{
    return ~current_sum + 1;
}


/**
 * @brief Function to check correctness of PN532 Normal information frame header.
 *
 * @param  p_buffer[in]  Pointer to the buffer containing frame header.
 * @param  p_length[out] Pointer to the variable where the data length will be stored.
 *
 * @retval STM_SUCCESS             If the header was correct.
 * @retval nfc_error_codes  Otherwise.
 */
static ret_code_t adafruit_pn532_header_check(uint8_t const * p_buffer, uint8_t * p_length)
{
    // Preamble
    if ( (p_buffer[PN532_PREAMBLE_OFFSET] != PN532_PREAMBLE) ||
         (p_buffer[PN532_STARTCODE1_OFFSET] != PN532_STARTCODE1) ||
         (p_buffer[PN532_STARTCODE2_OFFSET] != PN532_STARTCODE2) )
    {
        return NFC_INVALID_PREAMBLE;
    }
    // Data length
    if (p_buffer[PN532_LENGTH_CS_OFFSET] !=
        adafruit_pn532_cs_complement_calc(p_buffer[PN532_LENGTH_OFFSET]))
    {
        return NFC_LENGTH_CS_ERROR;
    }
    // Direction byte
    if ( (p_buffer[PN532_TFI_OFFSET] != PN532_PN532TOHOST) &&
         (p_buffer[PN532_TFI_OFFSET] != PN532_HOSTTOPN532) )
    {
        return NFC_EXCHANGE_DIR_ERROR;
    }

    *p_length = p_buffer[PN532_LENGTH_OFFSET];

    return STM_SUCCESS;
}


ret_code_t adafruit_pn532_init(bool force)
{
    uint32_t ver_data; // Variable to store firmware version read from PN532.

    if (m_lib_initialized && !(force))
    {
        return STM_SUCCESS;
    }

    if (force)
    {
    }

    if (m_pn532_object.using_spi)
    {
        return NFC_INTERNAL_ERROR;
    }

    ret_code_t err_code = adafruit_pn532_i2c_create();
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    adafruit_pn532_pin_setup();

    BSP_DelayMs(100);


    err_code = adafruit_pn532_firmware_version_get(&ver_data);
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }


    err_code = adafruit_pn532_sam_config(SAMCONFIGURATION_MODE_NORMAL);
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    err_code = adafruit_pn532_passive_activation_retries_set(0xFF);
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }


    m_lib_initialized = true;

    return STM_SUCCESS;
}


ret_code_t adafruit_pn532_i2c_create(void)
{

    return STM_SUCCESS;
}


void adafruit_pn532_tag_info_printout(nfc_a_tag_info const * const p_tag_info)
{
}


ret_code_t adafruit_pn532_firmware_version_get(uint32_t * p_response)
{

    m_pn532_packet_buf[0] = PN532_COMMAND_GETFIRMWAREVERSION;

    ret_code_t err_code = adafruit_pn532_cmd_send(m_pn532_packet_buf,
                                                  COMMAND_GETFIRMWAREVERSION_LENGTH,
                                                  1000);

    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    // Read data packet.
    err_code = adafruit_pn532_data_read(m_pn532_packet_buf, REPLY_GETFIRMWAREVERSION_LENGTH);

    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    if (memcmp(m_pn532_packet_buf + 1, m_pn532_rsp_firmware_ver, sizeof(m_pn532_rsp_firmware_ver)) != 0)
    {
        return NFC_RSP_VERSION_ERROR;
    }

    // Extract firmware version from the frame.
    *p_response = uint32_big_decode(m_pn532_packet_buf + PN532_DATA_OFFSET + 1);

    return STM_SUCCESS;
}


ret_code_t adafruit_pn532_cmd_send(uint8_t * p_cmd, uint8_t cmd_len, uint16_t timeout)
{

    ret_code_t err_code = adafruit_pn532_command_write(p_cmd, cmd_len);

    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    // Wait for ACK
    if (!adafruit_pn532_waitready_ms(timeout))
    {
        return NFC_TIME_OUT;
    }

    return adafruit_pn532_ack_read();
}


ret_code_t adafruit_pn532_sam_config(uint8_t mode)
{

    ret_code_t err_code;

    if ( (mode != SAMCONFIGURATION_MODE_NORMAL) &&
         (mode != SAMCONFIGURATION_MODE_VIRTUAL_CARD) &&
         (mode != SAMCONFIGURATION_MODE_WIRED_CARD) &&
         (mode != SAMCONFIGURATION_MODE_DUAL_CARD) )
    {
        return NFC_INVALID_PARAM;
    }

    m_pn532_packet_buf[0] = PN532_COMMAND_SAMCONFIGURATION;
    m_pn532_packet_buf[1] = mode;
    m_pn532_packet_buf[2] = 0x14; // Time-out value
    m_pn532_packet_buf[3] = SAMCONFIGURATION_IRQ_ENABLED;

    err_code = adafruit_pn532_cmd_send(m_pn532_packet_buf, COMMAND_SAMCONFIGURATION_LENGTH, 1000);
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    err_code = adafruit_pn532_data_read(m_pn532_packet_buf, REPLY_SAMCONFIGURATION_LENGTH);
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    if (!(m_pn532_packet_buf[PN532_DATA_OFFSET] == PN532_COMMAND_SAMCONFIGURATION + 1))
    {
        return NFC_INVALID_RESPONSE;
    }

    return STM_SUCCESS;
}


ret_code_t adafruit_pn532_power_down(void)
{

    m_pn532_packet_buf[0] = PN532_COMMAND_POWERDOWN;
    m_pn532_packet_buf[1] = POWERDOWN_WAKEUP_IRQ;

    ret_code_t err_code = adafruit_pn532_cmd_send(m_pn532_packet_buf,
                                                  COMMAND_POWERDOWN_BASE_LENGTH,
                                                  1000);
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    err_code = adafruit_pn532_data_read(m_pn532_packet_buf, REPLY_POWERDOWN_LENGTH);
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    if (!(m_pn532_packet_buf[PN532_DATA_OFFSET] == PN532_COMMAND_POWERDOWN + 1))
    {
        return STM_ERROR_NOT_FOUND;
    }

    // From PN532 user manual: "The PN532 needs approximately 1 ms to get into Power Down mode,
    // after the command response." (Rev. 02, p. 7.2.11, page 98)
    BSP_DelayMs(1);

    return STM_SUCCESS;
}


ret_code_t adafruit_pn532_wake_up(void)
{
    ret_code_t err_code;

    if (m_pn532_object.using_spi)
    {
        return NFC_INTERNAL_ERROR;
    }

    // Wakeup procedure as specified in PN532 User Manual Rev. 02, p. 7.2.11, page 99.
    uint8_t dummy_byte = 0x55;
    err_code = BSP_I2C1_IO_Write(PN532_I2C_ADDRESS, &dummy_byte, 1);
    if (err_code != STM_SUCCESS)
    {
        return NFC_IO_ERROR_BASE + err_code;
    }
    // Wait specified time to ensure that the PN532 shield is fully operational
    // (PN532 data sheet, Rev. 3.2, page 209).
    BSP_DelayMs(2);

    return STM_SUCCESS;
}


ret_code_t adafruit_pn532_passive_activation_retries_set(uint8_t max_retries)
{
    ret_code_t err_code;

    m_pn532_packet_buf[0] = PN532_COMMAND_RFCONFIGURATION;
    m_pn532_packet_buf[1] = RFCONFIGURATION_CFGITEM_MAXRETRIES;
    m_pn532_packet_buf[2] = 0xFF;        // MxRtyATR retries (default value)
    m_pn532_packet_buf[3] = 0x01;        // MxRtyPSL retries (default value)
    m_pn532_packet_buf[4] = max_retries; // MxRtyPassiveActivation retries (user value)


    err_code = adafruit_pn532_cmd_send(m_pn532_packet_buf,
                                       COMMAND_RFCONFIGURATION_MAXRETRIES_LENGTH,
                                       1000);

    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    return STM_SUCCESS;
}

ret_code_t pn532_set_nfca_target_init_command(void){
    m_pn532_packet_buf[0] = PN532_COMMAND_INLISTPASSIVETARGET;
    m_pn532_packet_buf[1] = 1; // Maximum number of targets.
    m_pn532_packet_buf[2] = PN532_MIFARE_ISO14443A_BAUD;

    ret_code_t err_code = adafruit_pn532_cmd_send(m_pn532_packet_buf,
                                                  COMMAND_INLISTPASSIVETARGET_BASE_LENGTH,
                                                  PN532_DEFAULT_WAIT_FOR_READY_TIMEOUT);

    return err_code;
}

ret_code_t pn532_read_nfca_target_init_resp(nfc_a_tag_info * p_tag_info){
    if (p_tag_info == NULL)
    {
        return NFC_INVALID_PARAM;
    }
    if (!adafruit_pn532_is_ready())
    {
        return NFC_RESP_NOT_READY;
    }

    ret_code_t err_code = adafruit_pn532_data_read(m_pn532_packet_buf,
                                        REPLY_INLISTPASSIVETARGET_106A_TARGET_LENGTH);
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    if (m_pn532_packet_buf[REPLY_INLISTPASSIVETARGET_106A_NBTG_OFFSET] != 1)
    {
        return NFC_INVALID_RESPONSE;
    }

    if (MAX_NFC_A_ID_LEN < m_pn532_packet_buf[REPLY_INLISTPASSIVETARGET_106A_UID_LEN_OFFSET])
    {
        return NFC_INVALID_LENGTH;
    }

    p_tag_info->sens_res[SENS_RES_ANTICOLLISION_INFO_BYTE] =
        m_pn532_packet_buf[REPLY_INLISTPASSIVETARGET_106A_SENS_RES_BYTE_1_OFFSET];
    p_tag_info->sens_res[SENS_RES_PLATFORM_INFO_BYTE] =
        m_pn532_packet_buf[REPLY_INLISTPASSIVETARGET_106A_SENS_RES_BYTE_2_OFFSET];

    p_tag_info->sel_res    = m_pn532_packet_buf[REPLY_INLISTPASSIVETARGET_106A_SEL_RES_OFFSET];
    p_tag_info->nfc_id_len = m_pn532_packet_buf[REPLY_INLISTPASSIVETARGET_106A_UID_LEN_OFFSET];
    memcpy(p_tag_info->nfc_id,
           m_pn532_packet_buf + REPLY_INLISTPASSIVETARGET_106A_UID_OFFSET,
           p_tag_info->nfc_id_len);

    m_pn532_object.in_listed_tag = m_pn532_packet_buf[REPLY_INLISTPASSIVETARGET_106A_TG_OFFSET];

    return STM_SUCCESS;
}

ret_code_t adafruit_pn532_nfc_a_target_init(nfc_a_tag_info * p_tag_info,
                                            uint16_t         timeout)
{

    if (p_tag_info == NULL)
    {
        return NFC_INVALID_PARAM;
    }

    m_pn532_packet_buf[0] = PN532_COMMAND_INLISTPASSIVETARGET;
    m_pn532_packet_buf[1] = 1; // Maximum number of targets.
    m_pn532_packet_buf[2] = PN532_MIFARE_ISO14443A_BAUD;

    ret_code_t err_code = adafruit_pn532_cmd_send(m_pn532_packet_buf,
                                                  COMMAND_INLISTPASSIVETARGET_BASE_LENGTH,
                                                  PN532_DEFAULT_WAIT_FOR_READY_TIMEOUT);
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }


    // Give PN532 a little time to scan in case time-out is very small.
    if (timeout < PN532_DEFAULT_WAIT_FOR_READY_TIMEOUT)
    {
        timeout = PN532_DEFAULT_WAIT_FOR_READY_TIMEOUT;
    }

    if (!adafruit_pn532_waitready_ms(timeout))
    {
        return NFC_TIME_OUT;
    }

    err_code = adafruit_pn532_data_read(m_pn532_packet_buf,
                                        REPLY_INLISTPASSIVETARGET_106A_TARGET_LENGTH);
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    if (m_pn532_packet_buf[REPLY_INLISTPASSIVETARGET_106A_NBTG_OFFSET] != 1)
    {
        return NFC_INVALID_RESPONSE;
    }

    if (MAX_NFC_A_ID_LEN < m_pn532_packet_buf[REPLY_INLISTPASSIVETARGET_106A_UID_LEN_OFFSET])
    {
        return NFC_INVALID_LENGTH;
    }

    p_tag_info->sens_res[SENS_RES_ANTICOLLISION_INFO_BYTE] =
        m_pn532_packet_buf[REPLY_INLISTPASSIVETARGET_106A_SENS_RES_BYTE_1_OFFSET];
    p_tag_info->sens_res[SENS_RES_PLATFORM_INFO_BYTE] =
        m_pn532_packet_buf[REPLY_INLISTPASSIVETARGET_106A_SENS_RES_BYTE_2_OFFSET];

    p_tag_info->sel_res    = m_pn532_packet_buf[REPLY_INLISTPASSIVETARGET_106A_SEL_RES_OFFSET];
    p_tag_info->nfc_id_len = m_pn532_packet_buf[REPLY_INLISTPASSIVETARGET_106A_UID_LEN_OFFSET];
    memcpy(p_tag_info->nfc_id,
           m_pn532_packet_buf + REPLY_INLISTPASSIVETARGET_106A_UID_OFFSET,
           p_tag_info->nfc_id_len);

    m_pn532_object.in_listed_tag = m_pn532_packet_buf[REPLY_INLISTPASSIVETARGET_106A_TG_OFFSET];

    return STM_SUCCESS;
}


ret_code_t adafruit_pn532_in_data_exchange(uint8_t * p_send,
                                           uint8_t   send_len,
                                           uint8_t * p_response,
                                           uint8_t * p_response_len)
{

    if ((uint16_t) send_len + 2 > PN532_PACKBUFF_SIZE)
    {
        return NFC_DATA_LENGTH_EXCEED;
    }

    if ((uint16_t) (*p_response_len) + REPLY_INDATAEXCHANGE_BASE_LENGTH > PN532_PACKBUFF_SIZE)
    {
        return NFC_DATA_LENGTH_EXCEED;
    }

    // Prepare command.
    m_pn532_packet_buf[0] = PN532_COMMAND_INDATAEXCHANGE;
    m_pn532_packet_buf[1] = m_pn532_object.in_listed_tag;
    memcpy(m_pn532_packet_buf + 2, p_send, send_len);

    ret_code_t err_code = adafruit_pn532_cmd_send(m_pn532_packet_buf,
                                                  send_len + 2,
                                                  TAG_DETECT_TIMEOUT);
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    // Note : The wait time was increased from 1 sec to 10 sec as some APDU in card upgrade take longer than 1 sec
    if (!adafruit_pn532_waitready_ms(TAG_DETECT_TIMEOUT))
    {
        return NFC_TIME_OUT;
    }

    err_code = adafruit_pn532_data_read(m_pn532_packet_buf,
                                        *p_response_len + REPLY_INDATAEXCHANGE_BASE_LENGTH);
                                                        // + 2 for command and status byte
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    uint8_t length = 0;
    err_code = adafruit_pn532_header_check(m_pn532_packet_buf, &length);
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    if ( (m_pn532_packet_buf[PN532_TFI_OFFSET] != PN532_PN532TOHOST) ||
         (m_pn532_packet_buf[PN532_DATA_OFFSET] != PN532_COMMAND_INDATAEXCHANGE + 1) )
    {
        return NFC_EXCHANGE_DIR_ERROR;
    }

    // Check InDataExchange Status byte.
    if ((m_pn532_packet_buf[PN532_DATA_OFFSET + 1] & PN532_STATUS_ERROR_MASK) != 0x00)
    {
        return PN532_ERROR_BASE + (m_pn532_packet_buf[PN532_DATA_OFFSET + 1] & PN532_STATUS_ERROR_MASK);
    }

    length -= 3; // Calculate the actual data length

    // Silently truncate response to fit into reply desired data size.
    if (length > *p_response_len)
    {
        length = *p_response_len;
    }

    memcpy(p_response, m_pn532_packet_buf + PN532_DATA_OFFSET + 2, length);
    *p_response_len = length;

    return STM_SUCCESS;
}


ret_code_t adafruit_pn532_tag2_read(uint8_t start_page, uint8_t * p_buffer)
{

    ret_code_t err_code;

    uint8_t cmd_buf[2];
    uint8_t response_len = T2T_MAX_DATA_EXCHANGE;

    cmd_buf[0] = MIFARE_CMD_READ;
    cmd_buf[1] = start_page;

    err_code = adafruit_pn532_in_data_exchange(cmd_buf, 2, p_buffer, &response_len);
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }


    return STM_SUCCESS;
}


ret_code_t adafruit_pn532_tag2_page_write(uint8_t page, uint8_t * p_data)
{
    if (page < T2T_MIN_WRITE_PAGE_NUMBER)
    {
        return NFC_INVALID_PARAM;
    }


    uint8_t write_buf[T2T_MAX_DATA_EXCHANGE];
    uint8_t response_len = T2T_MAX_DATA_EXCHANGE;

    write_buf[0] = MIFARE_ULTRALIGHT_CMD_WRITE;
    write_buf[1] = page;
    memcpy(write_buf + 2, p_data, T2T_PAGE_SIZE);

    ret_code_t err_code = adafruit_pn532_in_data_exchange(write_buf,
                                                          2 + T2T_PAGE_SIZE,
                                                          write_buf,
                                                          &response_len);
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    return STM_SUCCESS;
}


ret_code_t adafruit_pn532_ndef_uri_tag2_write(uint8_t uri_id, char * p_url, uint8_t data_len)
{
    uint8_t page_buf[4]   = {0};
    uint8_t uri_len       = strlen(p_url);
    uint8_t page_header[] =
    {
        0x00, 0x03, uri_len + 5, 0xD1,
        0x01, uri_len + 1, 0x55, uri_id
    };
    uint8_t page_header_len = sizeof(page_header);

    if ( (uri_len < 1) || (uri_len + 1 > (data_len - page_header_len)))
    {
        return NFC_INVALID_PARAM;
    }

    ret_code_t err_code;
    int32_t    i;
    uint8_t    current_page = 4;

    for (i = 0; i < 2; i++)
    {
        memcpy(page_buf, page_header + 4 * i, T2T_PAGE_SIZE);
        err_code = adafruit_pn532_tag2_page_write(current_page, page_buf);
        if (err_code != STM_SUCCESS)
        {
            return err_code;
        }
        current_page++;
    }

    char  * url_ptr    = p_url;
    uint8_t len_to_cpy = 0;

    while (uri_len > 0)
    {
        // Prepare length of the chunk to copy.
        if (uri_len < T2T_PAGE_SIZE)
        {
            len_to_cpy = uri_len;
            // If do not copy a full page, prepare the buffer.
            memset(page_buf, 0x00, T2T_PAGE_SIZE);
            page_buf[len_to_cpy] = 0xFE; // Terminator block.
        }
        else
        {
            len_to_cpy = T2T_PAGE_SIZE;
        }

        memcpy(page_buf, url_ptr, len_to_cpy);

        err_code = adafruit_pn532_tag2_page_write(current_page, page_buf);
        if (err_code != STM_SUCCESS)
        {
            return err_code;
        }

        current_page++;

        // If the last page was sent, and there was no chance to insert TLV Terminator block,
        // send another page with Terminator block in it.
        if (uri_len == T2T_PAGE_SIZE)
        {
            memset(page_buf, 0x00, T2T_PAGE_SIZE);
            page_buf[0] = 0xFE;
            err_code    = adafruit_pn532_tag2_page_write(current_page, page_buf);
            if (err_code != STM_SUCCESS)
            {
                return err_code;
            }
            current_page++;
        }

        uri_len -= len_to_cpy;
        url_ptr += len_to_cpy;
    }

    return STM_SUCCESS;
}


ret_code_t adafruit_pn532_ack_read(void)
{

    uint8_t    ack_buf[PN532_ACK_PACKET_SIZE];
    ret_code_t err_code;

    err_code = adafruit_pn532_data_read(ack_buf, PN532_ACK_PACKET_SIZE);
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    // Wait for irq to be taken off.
    for (uint16_t i = 0; i < SHORT_TIMEOUT; i++)
    {
        if (!adafruit_pn532_is_ready())
        {
            break;
        }
    }

    if (memcmp(ack_buf, m_pn532_ack, PN532_ACK_PACKET_SIZE) != 0)
    {
        return NFC_INTERNAL_ERROR;
    }

    return STM_SUCCESS;
}


bool adafruit_pn532_is_ready(void)
{
    return BSP_GET_IRQ_STATUS() == 0;
}


bool adafruit_pn532_waitready_ms(uint16_t timeout)
{
    uint16_t timer  = 0;
    bool     result = false;

    result = adafruit_pn532_is_ready();
    while ((!result) && (timer < timeout))
    {
        timer += 1;
        BSP_DelayMs(1);
        result = adafruit_pn532_is_ready();
    }

    return result;
}


ret_code_t adafruit_pn532_data_read(uint8_t * p_buff, uint8_t n)
{
    if (!adafruit_pn532_waitready_ms(PN532_DEFAULT_WAIT_FOR_READY_TIMEOUT))
    {
        return NFC_TIME_OUT;
    }

    if (m_pn532_object.using_spi)
    {
        return NFC_INTERNAL_ERROR;
    }

    if ((uint16_t) n + 1 > PN532_PACKBUFF_SIZE)
    {
        return NFC_DATA_LENGTH_EXCEED;
    }

    if (n == UINT8_MAX)
    {
        return NFC_DATA_LENGTH_EXCEED;
    }

    ret_code_t err_code;
    // In case of I2C, read the additional status byte.

    err_code = BSP_I2C1_IO_Read(PN532_I2C_ADDRESS, m_pn532_rxtx_buffer, n + 1);
    if (err_code != STM_SUCCESS)
    {
        return NFC_IO_ERROR_BASE + err_code;
    }
    memcpy(p_buff, m_pn532_rxtx_buffer + 1, n);


    return STM_SUCCESS;
}


ret_code_t adafruit_pn532_command_write(uint8_t * p_cmd, uint8_t cmd_len)
{
    ret_code_t err_code;
    uint8_t    checksum;

    if (m_pn532_object.using_spi)
    {
        return NFC_INTERNAL_ERROR;
    }

    if ((uint16_t) cmd_len + PN532_FRAME_OVERHEAD > PN532_PACKBUFF_SIZE)
    {
        return NFC_INVALID_PARAM;
    }

    // Compose header part of the command frame.
    m_pn532_rxtx_buffer[0] = PN532_PREAMBLE;
    m_pn532_rxtx_buffer[1] = PN532_STARTCODE1;
    m_pn532_rxtx_buffer[2] = PN532_STARTCODE2;
    m_pn532_rxtx_buffer[3] = cmd_len + 1; // Data length + TFI byte.
    m_pn532_rxtx_buffer[4] = adafruit_pn532_cs_complement_calc(cmd_len + 1);
    m_pn532_rxtx_buffer[5] = PN532_HOSTTOPN532;

    // Copy the payload data.
    memcpy(m_pn532_rxtx_buffer + HEADER_SEQUENCE_LENGTH, p_cmd, cmd_len);

    // Calculate checksum.
    checksum = PN532_HOSTTOPN532;
    for (uint8_t i = 0; i < cmd_len; i++)
    {
        checksum += p_cmd[i];
    }
    checksum = adafruit_pn532_cs_complement_calc(checksum);

    // Compose checksum part of the command frame.
    m_pn532_rxtx_buffer[HEADER_SEQUENCE_LENGTH + cmd_len]     = checksum;
    m_pn532_rxtx_buffer[HEADER_SEQUENCE_LENGTH + cmd_len + 1] = PN532_POSTAMBLE;


    err_code = BSP_I2C1_IO_Write(PN532_I2C_ADDRESS,
                              m_pn532_rxtx_buffer,
                              cmd_len + PN532_FRAME_OVERHEAD);
    if (err_code != STM_SUCCESS)
    {
        return NFC_IO_ERROR_BASE + err_code;
    }

    return STM_SUCCESS;
}


/** Function for enabling or disabling the PN532 RF field.
 *
 *  This function sends a configuration command to the PN532, which enables or disables the RF field.
 *
 *  @param     field_conf   A value indicating whether the RF field should be turned on or off.
 *                          Valid values are 1 (field on) and 0 (field off).
 *
 *  @retval    STM_SUCCESS              If the RF field was enabled successfully.
 *  @retval    NFC_INVALID_PARAM        If the value in field_conf was invalid.
 *  @retval    Other                    Otherwise.
 */
static ret_code_t adafruit_pn532_field_switch(uint8_t field_conf)
{
    ret_code_t err_code;

    if ( (field_conf != RFCONFIGURATION_RFFIELD_ON) && (field_conf != RFCONFIGURATION_RFFIELD_OFF) )
    {
        return NFC_INVALID_PARAM;
    }

    m_pn532_packet_buf[0] = PN532_COMMAND_RFCONFIGURATION;
    m_pn532_packet_buf[1] = RFCONFIGURATION_CFGITEM_RFFIELD;
    m_pn532_packet_buf[2] = field_conf;

    err_code = adafruit_pn532_cmd_send(m_pn532_packet_buf,
                                       COMMAND_RFCONFIGURATION_RFFIELD_LENGTH,
                                       SHORT_TIMEOUT);
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    if (!adafruit_pn532_waitready_ms(PN532_DEFAULT_WAIT_FOR_READY_TIMEOUT))
    {
        return NFC_TIME_OUT;
    }

    return STM_SUCCESS;
}

ret_code_t adafruit_pn532_deselect() {
    m_pn532_packet_buf[0] = PN532_COMMAND_INDESELECT;
    m_pn532_packet_buf[1] = 0x00;

    ret_code_t err_code = adafruit_pn532_cmd_send(m_pn532_packet_buf,
                                                  2,
                                                  SHORT_TIMEOUT);
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    if (!adafruit_pn532_waitready_ms(SHORT_TIMEOUT))
    {
        return NFC_TIME_OUT;
    }

    err_code = adafruit_pn532_data_read(m_pn532_packet_buf,
                                        2 + REPLY_INDATAEXCHANGE_BASE_LENGTH);
    // + 2 for command and status byte
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    uint8_t length = 0;
    err_code = adafruit_pn532_header_check(m_pn532_packet_buf, &length);
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    if ( (m_pn532_packet_buf[PN532_TFI_OFFSET] != PN532_PN532TOHOST) ||
         (m_pn532_packet_buf[PN532_DATA_OFFSET] != PN532_COMMAND_INDESELECT + 1) )
    {
        return NFC_EXCHANGE_DIR_ERROR;
    }

    // Check InDataExchange Status byte.
    if ((m_pn532_packet_buf[PN532_DATA_OFFSET + 1] & PN532_STATUS_ERROR_MASK) != 0x00)
    {
        return PN532_ERROR_BASE + (m_pn532_packet_buf[PN532_DATA_OFFSET + 1] & PN532_STATUS_ERROR_MASK);
    }
    return STM_SUCCESS;
}


ret_code_t adafruit_pn532_release() {
    m_pn532_packet_buf[0] = PN532_COMMAND_INRELEASE;
    m_pn532_packet_buf[1] = 0x00;

    ret_code_t err_code = adafruit_pn532_cmd_send(m_pn532_packet_buf,
                                                  2,
                                                  SHORT_TIMEOUT);
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    if (!adafruit_pn532_waitready_ms(SHORT_TIMEOUT))
    {
        return NFC_TIME_OUT;
    }

    err_code = adafruit_pn532_data_read(m_pn532_packet_buf,
                                        2 + REPLY_INDATAEXCHANGE_BASE_LENGTH);
    // + 2 for command and status byte
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    uint8_t length = 0;
    err_code = adafruit_pn532_header_check(m_pn532_packet_buf, &length);
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    if ( (m_pn532_packet_buf[PN532_TFI_OFFSET] != PN532_PN532TOHOST) ||
         (m_pn532_packet_buf[PN532_DATA_OFFSET] != PN532_COMMAND_INRELEASE + 1) )
    {
        return NFC_EXCHANGE_DIR_ERROR;
    }

    // Check InDataExchange Status byte.
    if ((m_pn532_packet_buf[PN532_DATA_OFFSET + 1] & PN532_STATUS_ERROR_MASK) != 0x00)
    {
        return PN532_ERROR_BASE + (m_pn532_packet_buf[PN532_DATA_OFFSET + 1] & PN532_STATUS_ERROR_MASK);
    }
    return STM_SUCCESS;
}


ret_code_t adafruit_pn532_field_on(void)
{
    return adafruit_pn532_field_switch(RFCONFIGURATION_RFFIELD_ON);
}


ret_code_t adafruit_pn532_field_off(void)
{
    return adafruit_pn532_field_switch(RFCONFIGURATION_RFFIELD_OFF);
}

void adafruit_pn532_clear_buffers(void)
{
    memset(m_pn532_packet_buf, 0, sizeof(m_pn532_packet_buf));
    memset(m_pn532_rxtx_buffer, 0, sizeof(m_pn532_rxtx_buffer));
}

ret_code_t adafruit_diagnose_comm_line(uint8_t * p_send, uint8_t send_len)
{
    uint8_t p_response_len = send_len + 1;
    if ((uint16_t) send_len + 2 > PN532_PACKBUFF_SIZE)
    {
        return STM_ERROR_INTERNAL;
    }

    // Prepare command.
    m_pn532_packet_buf[0] = PN532_COMMAND_DIAGNOSE;
    m_pn532_packet_buf[1] = 0x00;       ///< 0x00 NumTst value for communication line test between host controller and the PN532
    memcpy(m_pn532_packet_buf + 2, p_send, send_len);

    ret_code_t err_code = adafruit_pn532_cmd_send(m_pn532_packet_buf,
                                                  send_len + 2,
                                                  SHORT_TIMEOUT);
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    // Note : The wait time was increased from 1 sec to 10 sec as some APDU in card upgrade take longer than 1 sec
    if (!adafruit_pn532_waitready_ms(SHORT_TIMEOUT))
    {
        return STM_ERROR_INTERNAL;
    }

    err_code = adafruit_pn532_data_read(m_pn532_packet_buf,
                                        p_response_len + REPLY_INDATAEXCHANGE_BASE_LENGTH);
    // + 2 for command and status byte
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    uint8_t length = 0;
    err_code = adafruit_pn532_header_check(m_pn532_packet_buf, &length);
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    if ( (m_pn532_packet_buf[PN532_TFI_OFFSET] != PN532_PN532TOHOST) ||
         (m_pn532_packet_buf[PN532_DATA_OFFSET] != PN532_COMMAND_DIAGNOSE + 1) )
    {
        return STM_ERROR_INTERNAL;
    }

    // Compare response data with original data.
    if (memcmp(m_pn532_packet_buf + PN532_DATA_OFFSET + 2, p_send, send_len) != 0x00)
    {
        return 11;
    }

    return STM_SUCCESS;
}

ret_code_t adafruit_diagnose_card_presence(void)
{
    uint8_t p_response_len = 1;

    // Prepare command.
    m_pn532_packet_buf[0] = PN532_COMMAND_DIAGNOSE;
    m_pn532_packet_buf[1] = 0x06;       ///< 0x06 NumTst value for Card presence detection check

    ret_code_t err_code = adafruit_pn532_cmd_send(m_pn532_packet_buf,
                                                  2,
                                                  SHORT_TIMEOUT);
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    if (!adafruit_pn532_waitready_ms(VERY_SHORT_TIMEOUT))
    {
        return STM_ERROR_TIMEOUT;
    }

    err_code = adafruit_pn532_data_read(m_pn532_packet_buf,
                                        p_response_len + REPLY_INDATAEXCHANGE_BASE_LENGTH);
    // + 2 for command and status byte
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    uint8_t length = 0;
    err_code = adafruit_pn532_header_check(m_pn532_packet_buf, &length);
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    if ( (m_pn532_packet_buf[PN532_TFI_OFFSET] != PN532_PN532TOHOST) ||
         (m_pn532_packet_buf[PN532_DATA_OFFSET] != PN532_COMMAND_DIAGNOSE + 1) )
    {
        return STM_ERROR_INTERNAL;
    }

    // Check status byte.
    if ((m_pn532_packet_buf[PN532_DATA_OFFSET + 1] & PN532_STATUS_ERROR_MASK) != 0x00)
    {
        return m_pn532_packet_buf[PN532_DATA_OFFSET + 1];
    }

    // Return the status byte received from PN532 response.
    return m_pn532_packet_buf[PN532_DATA_OFFSET + 1];
}

ret_code_t adafruit_diagnose_self_antenna(uint8_t threshold)
{
    uint8_t p_response_len = 1;

    // Prepare command.
    m_pn532_packet_buf[0] = PN532_COMMAND_DIAGNOSE;
    m_pn532_packet_buf[1] = 0x07;       ///< 0x07 NumTst value for Self antenna detection
    m_pn532_packet_buf[2] = threshold;

    ret_code_t err_code = adafruit_pn532_cmd_send(m_pn532_packet_buf,
                                                  3,
                                                  SHORT_TIMEOUT);
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    // Note : The wait time was increased from 1 sec to 10 sec as some APDU in card upgrade take longer than 1 sec
    if (!adafruit_pn532_waitready_ms(SHORT_TIMEOUT))
    {
        return STM_ERROR_INTERNAL;
    }

    err_code = adafruit_pn532_data_read(m_pn532_packet_buf,
                                        p_response_len + REPLY_INDATAEXCHANGE_BASE_LENGTH);
    // + 2 for command and status byte
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    uint8_t length = 0;
    err_code = adafruit_pn532_header_check(m_pn532_packet_buf, &length);
    if (err_code != STM_SUCCESS)
    {
        return err_code;
    }

    if ( (m_pn532_packet_buf[PN532_TFI_OFFSET] != PN532_PN532TOHOST) ||
         (m_pn532_packet_buf[PN532_DATA_OFFSET] != PN532_COMMAND_DIAGNOSE + 1) )
    {
        return STM_ERROR_INTERNAL;
    }

    // Check status byte
    if ((m_pn532_packet_buf[PN532_DATA_OFFSET + 1] & PN532_STATUS_ERROR_MASK) != 0x00)
    {
        return m_pn532_packet_buf[PN532_DATA_OFFSET + 1];
    }

    return m_pn532_packet_buf[PN532_DATA_OFFSET + 1];
}

#endif // ADAFRUIT_PN532_ENABLED
