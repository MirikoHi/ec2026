#include "K230.h"
#include "daemon.h"
//#include <ti/devices/msp/msp.h>
//#include <ti/driverlib/dl_uart.h>
#define DATA_PACKET_LENGTH 8

static volatile uint8_t RxBuffer[DATA_PACKET_LENGTH];
volatile uint8_t k230_data_valid;

typedef union {
    uint16_t word;
    uint8_t byte[2];
} x_pos_union_typedef;

typedef union {
    float dt;
    uint8_t byte[4];
} dt_union_typedef;

static x_pos_union_typedef x_pos_union;
static dt_union_typedef    dt_union;
static steel_ball_movement_typedef current_steel_ball_movement;

void Clear_UART_FIFO(void) {
    while (!DL_UART_isRXFIFOEmpty(K230_INST)) {
        (void)DL_UART_receiveData(K230_INST); // ????????
    }
}
void K230_Init(void) {
    Clear_UART_FIFO(); // ???????????????FIFO
    NVIC_ClearPendingIRQ(K230_INST_INT_IRQN);
    NVIC_EnableIRQ(K230_INST_INT_IRQN);
    DL_UART_enableInterrupt(K230_INST, DL_UART_INTERRUPT_RX);
    DL_UART_clearInterruptStatus(K230_INST, DL_UART_INTERRUPT_RX); // ????ж???λ
}

uint8_t get_CRC8(const uint8_t *data, uint32_t len) {
    uint8_t crc = 0x00U;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 0x80U) {
                crc = (uint8_t)((crc << 1) ^ 0x07U);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void K230_ReceiveData(const uint8_t RxData) {
    static volatile uint8_t RxState = 0;
    static uint8_t RxIndex = 0;
    switch (RxState) {
        case 0: {
            k230_data_valid = 0;
            RxIndex = 0;
            RxBuffer[RxIndex] = RxData;
            RxIndex = (RxIndex + 1) % DATA_PACKET_LENGTH;
            RxState = 1;
            break;
        }
        case 1: {
            RxBuffer[RxIndex] = RxData;
            RxIndex = (RxIndex + 1) % DATA_PACKET_LENGTH;
            RxState = 2;
            break;
        }
        case 2: {
            RxBuffer[RxIndex] = RxData;
            RxIndex = (RxIndex + 1) % DATA_PACKET_LENGTH;
            RxState = 3;
            break;
        }
        case 3: {
            RxBuffer[RxIndex] = RxData;
            RxIndex = (RxIndex + 1) % DATA_PACKET_LENGTH;
            RxState = 4;
            break;
        }
        case 4: {
            RxBuffer[RxIndex] = RxData;
            RxIndex = (RxIndex + 1) % DATA_PACKET_LENGTH;
            RxState = 5;
            break;
        }
        case 5: {
            RxBuffer[RxIndex] = RxData;
            RxIndex = (RxIndex + 1) % DATA_PACKET_LENGTH;
            RxState = 6;
            break;
        }
        case 6: {
            RxBuffer[RxIndex] = RxData;
            RxIndex = (RxIndex + 1) % DATA_PACKET_LENGTH;
            RxState = 7;
            break;
        }
        case 7: {
            RxBuffer[RxIndex] = RxData;
            RxIndex = 0;
            RxState = 8;
            break;
        }
        case 8: {
            const uint8_t crc = get_CRC8(RxBuffer, DATA_PACKET_LENGTH - 1);
            if (crc == RxBuffer[DATA_PACKET_LENGTH - 1]) {
                x_pos_union.byte[0] = RxBuffer[1];
                x_pos_union.byte[1] = RxBuffer[2];
                dt_union.byte[0] = RxBuffer[3];
                dt_union.byte[1] = RxBuffer[4];
                dt_union.byte[2] = RxBuffer[5];
                dt_union.byte[3] = RxBuffer[6];
                current_steel_ball_movement.dt = dt_union.dt;
                current_steel_ball_movement.x_position = x_pos_union.word;
                k230_data_valid = 1;
            } else {
                k230_data_valid = 0;
            }

        }
    }
}

void K230_INST_IRQHandler(void) {
    uint8_t RxData = DL_UART_receiveData(K230_INST);
    K230_ReceiveData(RxData);
    DL_UART_clearInterruptStatus(K230_INST, DL_UART_INTERRUPT_RX);
}

uint8_t K230_Read(steel_ball_movement_typedef *steel_ball_movement) {
    if (k230_data_valid) {
        steel_ball_movement->dt = current_steel_ball_movement.dt;
        steel_ball_movement->x_position = x_pos_union.word;
        k230_data_valid = 0;
        return 1;
    }
    return 0;
}