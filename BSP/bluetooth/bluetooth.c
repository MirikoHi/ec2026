#include "bluetooth.h"
#include "ti_msp_dl_config.h"

/*
 * 手机控制命令数量很少，16字节接收队列足够。
 */
#define BLUETOOTH_RX_QUEUE_SIZE         16U

/*
 * 速度回传一帧约17~19字节。
 * 128字节能够缓存多帧数据。
 */
#define BLUETOOTH_TX_QUEUE_SIZE         128U

#define BLUETOOTH_UART_ERROR_INTERRUPTS                         \
    (DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |                     \
     DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |                     \
     DL_UART_MAIN_INTERRUPT_PARITY_ERROR |                      \
     DL_UART_MAIN_INTERRUPT_BREAK_ERROR |                       \
     DL_UART_MAIN_INTERRUPT_NOISE_ERROR)

#define BLUETOOTH_UART_RX_INTERRUPTS                            \
    (DL_UART_MAIN_INTERRUPT_RX |                                \
     DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR)

#define BLUETOOTH_UART_HANDLED_INTERRUPTS                       \
    (BLUETOOTH_UART_RX_INTERRUPTS |                             \
     BLUETOOTH_UART_ERROR_INTERRUPTS)

/* ================= 接收队列 ================= */

static volatile uint8_t bluetooth_rx_queue[BLUETOOTH_RX_QUEUE_SIZE];
static volatile uint8_t bluetooth_rx_head;
static volatile uint8_t bluetooth_rx_tail;
static volatile uint32_t bluetooth_rx_overflow_count;

/* ================= 发送队列 ================= */

/*
 * 发送队列只在Robot_Cmd任务中写入和读出，
 * UART中断不操作该队列，因此不需要互斥锁。
 */
static uint8_t bluetooth_tx_queue[BLUETOOTH_TX_QUEUE_SIZE];
static uint16_t bluetooth_tx_head;
static uint16_t bluetooth_tx_tail;

static volatile uint32_t bluetooth_tx_drop_count;
static volatile uint32_t bluetooth_uart_error_count;

/* ================= 内部函数 ================= */

static uint8_t Bluetooth_IsKnownCommand(uint8_t data);
static void Bluetooth_ClearUartRxFifo(void);

static uint16_t Bluetooth_GetTxFreeSize(void);
static uint8_t Bluetooth_QueueTxData(const uint8_t *data,
                                     uint16_t length);

static uint16_t Bluetooth_AppendUnsigned(uint8_t *buffer,
                                         uint16_t index,
                                         uint32_t value);

static uint16_t Bluetooth_AppendFixed3(uint8_t *buffer,
                                       uint16_t index,
                                       float value);

/* ================= 初始化 ================= */

void Bluetooth_Init(void)
{
    bluetooth_rx_head = 0U;
    bluetooth_rx_tail = 0U;

    bluetooth_tx_head = 0U;
    bluetooth_tx_tail = 0U;

    bluetooth_rx_overflow_count = 0U;
    bluetooth_tx_drop_count = 0U;
    bluetooth_uart_error_count = 0U;

    Bluetooth_ClearUartRxFifo();

    /*
     * SysConfig已经初始化UART_0硬件。
     * 这里补充开启接收和UART错误中断。
     */
    DL_UART_Main_enableInterrupt(
        UART_0_INST,
        BLUETOOTH_UART_HANDLED_INTERRUPTS);

    DL_UART_clearInterruptStatus(
        UART_0_INST,
        BLUETOOTH_UART_HANDLED_INTERRUPTS);

    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

/* ================= 接收处理 ================= */

void Bluetooth_ReceiveByte(uint8_t data)
{
    uint8_t next_head;

    /*
     * 不属于当前协议的字节直接丢弃。
     * 上位机回显、换行符等不会影响小车。
     */
    if (Bluetooth_IsKnownCommand(data) == 0U)
    {
        return;
    }

    next_head = (uint8_t)(
        (bluetooth_rx_head + 1U) % BLUETOOTH_RX_QUEUE_SIZE);

    /*
     * 队列满时丢弃本次新命令。
     */
    if (next_head == bluetooth_rx_tail)
    {
        bluetooth_rx_overflow_count++;
        return;
    }

    bluetooth_rx_queue[bluetooth_rx_head] = data;
    bluetooth_rx_head = next_head;
}

uint8_t Bluetooth_ReadCommand(uint8_t *command)
{
    if (command == 0)
    {
        return 0U;
    }

    if (bluetooth_rx_head == bluetooth_rx_tail)
    {
        return 0U;
    }

    *command = bluetooth_rx_queue[bluetooth_rx_tail];

    bluetooth_rx_tail = (uint8_t)(
        (bluetooth_rx_tail + 1U) % BLUETOOTH_RX_QUEUE_SIZE);

    return 1U;
}

static uint8_t Bluetooth_IsKnownCommand(uint8_t data)
{
    /*
BLUETOOTH_CMD_MANUAL_MODE	0xA0	切换为手动遥控模式
BLUETOOTH_CMD_TRACE_MODE	0xA1	切换为自动寻迹模式
BLUETOOTH_CMD_FORWARD	    0xAA	小车前进
BLUETOOTH_CMD_BACKWARD	    0xBB	小车后退
BLUETOOTH_CMD_FORWARD_LEFT	0xCC	小车左转/左前
BLUETOOTH_CMD_FORWARD_RIGHT	0xDD	小车右转/右前
BLUETOOTH_CMD_STOP	        0xEE	小车急停
BLUETOOTH_CMD_DISABLE	    0xF0	电机失能（自由滑行）
BLUETOOTH_CMD_ENABLE	    0xF1	电机使能
*/
    switch (data)
    {
        case BLUETOOTH_CMD_MANUAL_MODE:
        case BLUETOOTH_CMD_TRACE_MODE:

        case BLUETOOTH_CMD_FORWARD:
        case BLUETOOTH_CMD_BACKWARD:
        case BLUETOOTH_CMD_FORWARD_LEFT:
        case BLUETOOTH_CMD_FORWARD_RIGHT:
        case BLUETOOTH_CMD_STOP:

        case BLUETOOTH_CMD_DISABLE:
        case BLUETOOTH_CMD_ENABLE:
            return 1U;

        default:
            return 0U;
    }
}

/* ================= 速度回传 ================= */

uint8_t Bluetooth_SendMotorSpeed(float left_speed,
                                 float right_speed)
{
    uint8_t frame[40];
    uint16_t index = 0U;

    frame[index++] = 'L';
    frame[index++] = ':';

    index = Bluetooth_AppendFixed3(
        frame,
        index,
        left_speed);

    frame[index++] = ',';
    frame[index++] = 'R';
    frame[index++] = ':';

    index = Bluetooth_AppendFixed3(
        frame,
        index,
        right_speed);

    frame[index++] = '\r';
    frame[index++] = '\n';

    return Bluetooth_QueueTxData(frame, index);
}

/**
 * @brief 将浮点数格式化为三位小数
 *
 * 不使用sprintf浮点格式化，避免增加大量库代码。
 *
 * 示例：
 *  0.125 -> "0.125"
 * -0.082 -> "-0.082"
 */
static uint16_t Bluetooth_AppendFixed3(uint8_t *buffer,
                                       uint16_t index,
                                       float value)
{
    int32_t value_mmps;
    uint32_t absolute_value;
    uint32_t integer_part;
    uint32_t decimal_part;

    /*
     * m/s放大1000倍，转为mm/s。
     */
    if (value >= 0.0f)
    {
        value_mmps = (int32_t)(value * 1000.0f + 0.5f);
    }
    else
    {
        value_mmps = (int32_t)(value * 1000.0f - 0.5f);
    }

    if (value_mmps < 0)
    {
        buffer[index++] = '-';
        absolute_value = (uint32_t)(-value_mmps);
    }
    else
    {
        absolute_value = (uint32_t)value_mmps;
    }

    integer_part = absolute_value / 1000U;
    decimal_part = absolute_value % 1000U;

    index = Bluetooth_AppendUnsigned(
        buffer,
        index,
        integer_part);

    buffer[index++] = '.';

    buffer[index++] =
        (uint8_t)('0' + (decimal_part / 100U));

    buffer[index++] =
        (uint8_t)('0' + ((decimal_part / 10U) % 10U));

    buffer[index++] =
        (uint8_t)('0' + (decimal_part % 10U));

    return index;
}

static uint16_t Bluetooth_AppendUnsigned(uint8_t *buffer,
                                         uint16_t index,
                                         uint32_t value)
{
    uint8_t temp[10];
    uint8_t length = 0U;

    do
    {
        temp[length++] =
            (uint8_t)('0' + (value % 10U));

        value /= 10U;
    }
    while ((value != 0U) && (length < sizeof(temp)));

    while (length > 0U)
    {
        length--;
        buffer[index++] = temp[length];
    }

    return index;
}

/* ================= 非阻塞发送 ================= */

static uint8_t Bluetooth_QueueTxData(const uint8_t *data,
                                     uint16_t length)
{
    uint16_t i;

    if ((data == 0) || (length == 0U))
    {
        return 0U;
    }

    /*
     * 不允许只放入半帧数据。
     */
    if (Bluetooth_GetTxFreeSize() < length)
    {
        bluetooth_tx_drop_count++;
        return 0U;
    }

    for (i = 0U; i < length; i++)
    {
        bluetooth_tx_queue[bluetooth_tx_head] = data[i];

        bluetooth_tx_head =
            (bluetooth_tx_head + 1U) %
            BLUETOOTH_TX_QUEUE_SIZE;
    }

    return 1U;
}

static uint16_t Bluetooth_GetTxFreeSize(void)
{
    if (bluetooth_tx_head >= bluetooth_tx_tail)
    {
        return (uint16_t)(
            BLUETOOTH_TX_QUEUE_SIZE -
            (bluetooth_tx_head - bluetooth_tx_tail) -
            1U);
    }

    return (uint16_t)(
        bluetooth_tx_tail -
        bluetooth_tx_head -
        1U);
}

void Bluetooth_PollTx(void)
{
    /*
     * 只要UART发送寄存器或FIFO还有空间，
     * 就尽可能多地放入待发送字节。
     *
     * 该函数不会等待某个字节发送完成。
     */
    while ((bluetooth_tx_tail != bluetooth_tx_head) &&
           (!DL_UART_Main_isTXFIFOFull(UART_0_INST)))
    {
        DL_UART_Main_transmitData(
            UART_0_INST,
            bluetooth_tx_queue[bluetooth_tx_tail]);

        bluetooth_tx_tail =
            (bluetooth_tx_tail + 1U) %
            BLUETOOTH_TX_QUEUE_SIZE;
    }
}

uint32_t Bluetooth_GetTxDropCount(void)
{
    return bluetooth_tx_drop_count;
}

uint32_t Bluetooth_GetUartErrorCount(void)
{
    return bluetooth_uart_error_count;
}

/* ================= UART中断 ================= */

static void Bluetooth_ClearUartRxFifo(void)
{
    while (!DL_UART_isRXFIFOEmpty(UART_0_INST))
    {
        (void)DL_UART_receiveData(UART_0_INST);
    }
}

void UART_0_INST_IRQHandler(void)//这个是蓝牙的串口
{
    uint32_t status;

    status = DL_UART_getEnabledInterruptStatus(
        UART_0_INST,
        BLUETOOTH_UART_HANDLED_INTERRUPTS);

    if (status != 0U)
    {
        DL_UART_clearInterruptStatus(
            UART_0_INST,
            status);
    }

    if ((status & BLUETOOTH_UART_ERROR_INTERRUPTS) != 0U)
    {
        bluetooth_uart_error_count++;

        bluetooth_rx_head = 0U;
        bluetooth_rx_tail = 0U;

        Bluetooth_ClearUartRxFifo();
        return;
    }

    if ((status & BLUETOOTH_UART_RX_INTERRUPTS) != 0U)
    {
        while (!DL_UART_isRXFIFOEmpty(UART_0_INST))
        {
            Bluetooth_ReceiveByte(
                DL_UART_receiveData(UART_0_INST));
        }
    }
}