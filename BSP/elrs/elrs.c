#include "elrs.h"
#include "daemon.h"
#include "ti_msp_dl_config.h"
#include "string.h"

#define ELRS_CRSF_MAX_PACKET_SIZE      64U
#define ELRS_CRSF_MAX_PAYLOAD_LEN      (ELRS_CRSF_MAX_PACKET_SIZE - 4U)
#define ELRS_CRSF_CRC_POLY             0xD5U

#define ELRS_CRSF_ADDRESS_FC           0xC8U
#define ELRS_CRSF_TYPE_LINK_STATS      0x14U
#define ELRS_CRSF_TYPE_RC_CHANNELS     0x16U

#define ELRS_CRSF_RC_PAYLOAD_SIZE      22U
#define ELRS_CRSF_LINK_PAYLOAD_SIZE    10U
#define ELRS_DAEMON_RELOAD_COUNT       50U

#define ELRS_UART_ERROR_INTERRUPTS     (DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR | \
                                        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR | \
                                        DL_UART_MAIN_INTERRUPT_PARITY_ERROR | \
                                        DL_UART_MAIN_INTERRUPT_BREAK_ERROR | \
                                        DL_UART_MAIN_INTERRUPT_NOISE_ERROR)
#define ELRS_UART_RX_INTERRUPTS        (DL_UART_MAIN_INTERRUPT_RX | \
                                        DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR)
#define ELRS_UART_HANDLED_INTERRUPTS   (ELRS_UART_RX_INTERRUPTS | ELRS_UART_ERROR_INTERRUPTS)

static uint8_t elrs_rx_buf[ELRS_CRSF_MAX_PACKET_SIZE];
static uint8_t elrs_rx_index;
static volatile ELRS_Data_s elrs_data;
static DaemonInstance *elrs_daemon;

static uint8_t ELRS_Crc8(const uint8_t *data, uint8_t len);
static void ELRS_ShiftRxBuffer(uint8_t count);
static void ELRS_ProcessBuffer(void);
static void ELRS_ProcessPacket(void);
static void ELRS_ParseChannels(const uint8_t *payload);
static void ELRS_ParseLinkStatistics(const uint8_t *payload);
static void ELRS_ResetData(void);
static void ELRS_SetOfflineState(void);
static void ELRS_ClearUartFifo(void);
static void ELRS_LostCallback(void *ptr);

void ELRS_Init(void)
{
    ELRS_ResetData();
    ELRS_ClearUartFifo();

        DL_UART_Main_enableInterrupt(ELRS_INST, ELRS_UART_HANDLED_INTERRUPTS);
    DL_UART_clearInterruptStatus(ELRS_INST, ELRS_UART_HANDLED_INTERRUPTS);

    Daemon_Init_Config_s daemon_config = {
        .callback = ELRS_LostCallback,
        .owner_id = NULL,
        .reload_count = ELRS_DAEMON_RELOAD_COUNT,
    };
    elrs_daemon = DaemonRegister(&daemon_config);

    NVIC_ClearPendingIRQ(ELRS_INST_INT_IRQN);
    NVIC_EnableIRQ(ELRS_INST_INT_IRQN);
}

void ELRS_ReceiveByte(uint8_t data)
{
    if (elrs_rx_index >= ELRS_CRSF_MAX_PACKET_SIZE)
    {
        elrs_rx_index = 0U;
    }

    elrs_rx_buf[elrs_rx_index++] = data;
    ELRS_ProcessBuffer();
}

const volatile ELRS_Data_s *ELRS_GetData(void)
{
    return &elrs_data;
}

uint16_t ELRS_GetChannel(uint8_t index)
{
    if (index >= ELRS_CHANNEL_NUM)
    {
        return 0U;
    }

    return elrs_data.channel[index];
}

uint8_t ELRS_IsOnline(void)
{
    return (elrs_daemon != NULL) &&
           (elrs_data.online != 0U) &&
           (DaemonIsOnline(elrs_daemon) != 0U);
}

uint8_t ELRS_IsFrameUpdated(void)
{
    return elrs_data.frame_updated;
}

void ELRS_ClearFrameUpdated(void)
{
    elrs_data.frame_updated = 0U;
}

uint32_t ELRS_GetUartErrorCount(void)
{
    return elrs_data.uart_error_count;
}

static uint8_t ELRS_Crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0U;

    while (len-- != 0U)
    {
        crc ^= *data++;
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = (uint8_t)((crc << 1U) ^ ELRS_CRSF_CRC_POLY);
            }
            else
            {
                crc <<= 1U;
            }
        }
    }

    return crc;
}

static void ELRS_ShiftRxBuffer(uint8_t count)
{
    if (count >= elrs_rx_index)
    {
        elrs_rx_index = 0U;
        return;
    }

    elrs_rx_index -= count;
    memmove(elrs_rx_buf, &elrs_rx_buf[count], elrs_rx_index);
}

static void ELRS_ProcessBuffer(void)
{
    uint8_t reprocess;

    do
    {
        reprocess = 0U;

        if (elrs_rx_index > 1U)
        {
            uint8_t len = elrs_rx_buf[1];

            if ((len < 3U) || (len > (ELRS_CRSF_MAX_PAYLOAD_LEN + 2U)))
            {
                ELRS_ShiftRxBuffer(1U);
                reprocess = 1U;
            }
            else if (elrs_rx_index >= (uint8_t)(len + 2U))
            {
                uint8_t rx_crc = elrs_rx_buf[2U + len - 1U];
                uint8_t calc_crc = ELRS_Crc8(&elrs_rx_buf[2], (uint8_t)(len - 1U));

                if (calc_crc == rx_crc)
                {
                    ELRS_ProcessPacket();
                    ELRS_ShiftRxBuffer((uint8_t)(len + 2U));
                }
                else
                {
                    ELRS_ShiftRxBuffer(1U);
                }

                reprocess = 1U;
            }
        }
    } while (reprocess != 0U);
}

static void ELRS_ProcessPacket(void)
{
    uint8_t len = elrs_rx_buf[1];
    uint8_t type = elrs_rx_buf[2];
    const uint8_t *payload = &elrs_rx_buf[3];
    uint8_t payload_len = (uint8_t)(len - 2U);

    if (elrs_rx_buf[0] != ELRS_CRSF_ADDRESS_FC)
    {
        return;
    }

    switch (type)
    {
        case ELRS_CRSF_TYPE_RC_CHANNELS:
            if (payload_len == ELRS_CRSF_RC_PAYLOAD_SIZE)
            {
                ELRS_ParseChannels(payload);
            }
            break;
        case ELRS_CRSF_TYPE_LINK_STATS:
            if (payload_len == ELRS_CRSF_LINK_PAYLOAD_SIZE)
            {
                ELRS_ParseLinkStatistics(payload);
            }
            break;
        default:
            break;
    }
}

static void ELRS_ParseChannels(const uint8_t *payload)
{
    for (uint8_t ch = 0U; ch < ELRS_CHANNEL_NUM; ch++)
    {
        uint16_t bit_index = (uint16_t)ch * 11U;
        uint8_t byte_index = (uint8_t)(bit_index >> 3U);
        uint8_t bit_offset = (uint8_t)(bit_index & 0x07U);
        uint32_t value = (uint32_t)payload[byte_index] |
                         ((uint32_t)payload[byte_index + 1U] << 8U);

        if ((byte_index + 2U) < ELRS_CRSF_RC_PAYLOAD_SIZE)
        {
            value |= (uint32_t)payload[byte_index + 2U] << 16U;
        }

        elrs_data.channel[ch] = (uint16_t)((value >> bit_offset) & 0x07FFU);
    }

    elrs_data.rc_frame_count++;
    elrs_data.frame_updated = 1U;
    elrs_data.online = 1U;
    if (elrs_daemon != NULL)
    {
        DaemonReload(elrs_daemon);
    }
}

static void ELRS_ParseLinkStatistics(const uint8_t *payload)
{
    elrs_data.link.uplink_rssi_1 = payload[0];
    elrs_data.link.uplink_rssi_2 = payload[1];
    elrs_data.link.uplink_link_quality = payload[2];
    elrs_data.link.uplink_snr = (int8_t)payload[3];
    elrs_data.link.active_antenna = payload[4];
    elrs_data.link.rf_mode = payload[5];
    elrs_data.link.uplink_tx_power = payload[6];
    elrs_data.link.downlink_rssi = payload[7];
    elrs_data.link.downlink_link_quality = payload[8];
    elrs_data.link.downlink_snr = (int8_t)payload[9];
    elrs_data.link_frame_count++;
    elrs_data.link_updated = 1U;
}

static void ELRS_ResetData(void)
{
    memset((void *)&elrs_data, 0, sizeof(elrs_data));
    for (uint8_t i = 0U; i < ELRS_CHANNEL_NUM; i++)
    {
        elrs_data.channel[i] = ELRS_CHANNEL_VALUE_MID;
    }
    elrs_rx_index = 0U;
}

static void ELRS_SetOfflineState(void)
{
    for (uint8_t i = 0U; i < ELRS_CHANNEL_NUM; i++)
    {
        elrs_data.channel[i] = ELRS_CHANNEL_VALUE_MID;
    }

    // Force the mode switch channel low so remote walking exits on signal loss.
    elrs_data.channel[4] = ELRS_CHANNEL_VALUE_MIN;
    elrs_data.online = 0U;
    elrs_data.frame_updated = 1U;
    elrs_rx_index = 0U;
}

static void ELRS_ClearUartFifo(void)
{
    while (!DL_UART_isRXFIFOEmpty(ELRS_INST))
    {
        (void)DL_UART_receiveData(ELRS_INST);
    }
}

static void ELRS_LostCallback(void *ptr)
{
    (void)ptr;
    ELRS_SetOfflineState();
}

void ELRS_INST_IRQHandler(void)
{
    uint32_t status = DL_UART_getEnabledInterruptStatus(ELRS_INST, ELRS_UART_HANDLED_INTERRUPTS);

    if (status != 0U)
    {
        DL_UART_clearInterruptStatus(ELRS_INST, status);
    }

    if ((status & ELRS_UART_ERROR_INTERRUPTS) != 0U)
    {
        elrs_data.uart_error_count++;
        elrs_rx_index = 0U;
        ELRS_ClearUartFifo();
        return;
    }

    if ((status & ELRS_UART_RX_INTERRUPTS) != 0U)
    {
        while (!DL_UART_isRXFIFOEmpty(ELRS_INST))
        {
            ELRS_ReceiveByte(DL_UART_receiveData(ELRS_INST));
        }
    }
}
