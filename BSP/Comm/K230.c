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
    /* 限定最多清 16 次，防止 DL_UART_isRXFIFOEmpty 不可靠导致死循环 */
    for (int _i = 0; _i < 16; _i++) {
        if (DL_UART_isRXFIFOEmpty(K230_INST))
            break;
        (void)DL_UART_receiveData(K230_INST);
    }
}
void K230_Init(void) {
    Clear_UART_FIFO(); // ???????????????FIFO
    NVIC_ClearPendingIRQ(K230_INST_INT_IRQN);
    NVIC_EnableIRQ(K230_INST_INT_IRQN);
    DL_UART_enableInterrupt(K230_INST, DL_UART_INTERRUPT_RX);
    DL_UART_clearInterruptStatus(K230_INST, DL_UART_INTERRUPT_RX); // ????ж???λ
}

uint8_t get_CRC8(uint8_t *data, uint32_t len) {
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
// 状态枚举
typedef enum {
    STATE_IDLE,      // 等待包头
    STATE_RECEIVING  // 正在接收数据
} RxState_t;

void K230_ReceiveData(const uint8_t RxData) {
    static RxState_t state = STATE_IDLE;
    static uint8_t index = 0;

    if (state == STATE_IDLE) {
        if (RxData == 0xA5) {
            RxBuffer[0] = RxData;
            index = 1;
            state = STATE_RECEIVING;
        }
    } else { // STATE_RECEIVING
        // 如果在收包过程中收到新的包头，则重新同步
        if (RxData == 0xA5) {
            RxBuffer[0] = RxData;
            index = 1;
            // 状态保持为 RECEIVING，但重新开始计数
            return;
        }

        RxBuffer[index] = RxData;
        index++;

        if (index == DATA_PACKET_LENGTH) {
            // 完整包已接收，进行校验
            const uint8_t crc = get_CRC8(RxBuffer, DATA_PACKET_LENGTH - 1);
            if (crc == RxBuffer[DATA_PACKET_LENGTH - 1]) {
                x_pos_union.byte[0] = RxBuffer[1];
                x_pos_union.byte[1] = RxBuffer[2];
                dt_union.byte[0]   = RxBuffer[3];
                dt_union.byte[1]   = RxBuffer[4];
                dt_union.byte[2]   = RxBuffer[5];
                dt_union.byte[3]   = RxBuffer[6];
                current_steel_ball_movement.dt        = dt_union.dt;
                current_steel_ball_movement.x_position = x_pos_union.word;
                k230_data_valid = 1;
            } else {
                k230_data_valid = 0;
            }
            // 复位状态，准备接收下一包
            state = STATE_IDLE;
            index = 0;
        }
    }
}

void K230_INST_IRQHandler(void) {
    static uint16_t cntr = 0;
    cntr ++;
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