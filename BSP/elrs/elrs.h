#ifndef BSP_ELRS_H
#define BSP_ELRS_H

#include "stdint.h"

#define ELRS_CHANNEL_NUM              16U
#define ELRS_CHANNEL_VALUE_MIN        172U
#define ELRS_CHANNEL_VALUE_MID        992U
#define ELRS_CHANNEL_VALUE_MAX        1811U
#define ELRS_CHANNEL_VALUE_1000       191U
#define ELRS_CHANNEL_VALUE_2000       1792U

typedef struct
{
    uint8_t uplink_rssi_1;
    uint8_t uplink_rssi_2;
    uint8_t uplink_link_quality;
    int8_t uplink_snr;
    uint8_t active_antenna;
    uint8_t rf_mode;
    uint8_t uplink_tx_power;
    uint8_t downlink_rssi;
    uint8_t downlink_link_quality;
    int8_t downlink_snr;
} ELRS_LinkStatistics_s;

typedef struct
{
    uint16_t channel[ELRS_CHANNEL_NUM];
    ELRS_LinkStatistics_s link;
    uint32_t rc_frame_count;
    uint32_t link_frame_count;
    uint8_t frame_updated;
    uint8_t link_updated;
} ELRS_Data_s;

void ELRS_Init(void);
void ELRS_ReceiveByte(uint8_t data);
const volatile ELRS_Data_s *ELRS_GetData(void);
uint16_t ELRS_GetChannel(uint8_t index);
uint8_t ELRS_IsFrameUpdated(void);
void ELRS_ClearFrameUpdated(void);

#endif
