#include "flash_param_store.h"

#include <string.h>
#include "bsp_log.h"
#include "Chassis.h"
#include "trace.h"

#define FLASH_PARAM_MAGIC               0x4350524DU
#define FLASH_PARAM_VERSION             1U
#define FLASH_PARAM_CRC_INIT            0xFFFFFFFFU
#define FLASH_PARAM_CRC_POLY            0xEDB88320U
#define FLASH_PARAM_TEST_SEQ0           10U
#define FLASH_PARAM_TEST_SEQ1           11U

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t payload_size;
    uint32_t sequence;
    uint32_t default_revision;
    uint32_t crc32;
} FlashParam_RecordHeader_s;

typedef struct
{
    FlashParam_RecordHeader_s header;
    FlashParam_Data_s data;
} FlashParam_Record_s;

static FlashParam_State_s flash_param_state = {0};

/**
  * @brief 组装默认参数
  * @param params 输出默认参数结构体指针
  * @note Flash 库不保存业务默认值，默认值由各个使用模块填充
  */
static void FlashParam_BuildDefault(FlashParam_Data_s *params)
{
    if (params == NULL)
    {
        return;
    }

    memset(params, 0, sizeof(*params));
    Chassis_FillDefaultParams(params);
    Trace_FillDefaultParams(params);
}

/**
  * @brief 根据槽号换算 Flash 物理地址
  * @param slot 槽号，范围 0~1
  * @return 槽对应的 Flash 起始地址
  * @note 用法：内部读写槽时使用
  */
static uint32_t FlashParam_SlotAddress(uint8_t slot)
{
    return (slot == 0U) ? FLASH_PARAM_SLOT0_ADDR : FLASH_PARAM_SLOT1_ADDR;
}

/**
  * @brief 将槽号转换为参数来源枚举
  * @param slot 槽号，范围 0~1
  * @return 槽对应的参数来源枚举
  * @note 用法：内部选择活动槽后，用于更新 flash_param_state.source
  */
static FlashParam_Source_e FlashParam_SlotToSource(uint8_t slot)
{
    return (slot == 0U) ? FLASH_PARAM_SOURCE_SLOT0 : FLASH_PARAM_SOURCE_SLOT1;
}

/**
  * @brief 将参数来源转换为槽号
  * @param source 当前参数来源
  * @return 来源对应的槽号，默认来源按 slot0 处理
  * @note 用法：保存新参数时，用当前槽异或 1 得到备用槽
  */
static uint8_t FlashParam_SourceToSlot(FlashParam_Source_e source)
{
    if (source == FLASH_PARAM_SOURCE_SLOT1)
    {
        return 1U;
    }

    return 0U;
}

/**
  * @brief 计算 CRC32
  * @param data 待计算数据指针
  * @param len 待计算数据长度，单位字节
  * @return CRC32 结果
  * @note 用法：保存参数时生成校验值，读取参数时重新计算并比较
  */
static uint32_t FlashParam_Crc32(const void *data, uint32_t len)
{
    uint32_t crc = FLASH_PARAM_CRC_INIT;
    const uint8_t *bytes = (const uint8_t *)data;

    while (len-- > 0U)
    {
        crc ^= *bytes++;
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            uint32_t mask = 0U - (crc & 1U);
            crc = (crc >> 1U) ^ (FLASH_PARAM_CRC_POLY & mask);
        }
    }

    return ~crc;
}

/**
  * @brief 校验完整参数记录
  * @param record 待读取记录指针
  * @return FLASH_PARAM_OK 表示记录有效，其他值表示空指针或校验失败
  * @note 用法：每次从槽读取记录后调用，用于判断该槽是否可用
  */
static FlashParam_Status_e FlashParam_ValidateRecord(const FlashParam_Record_s *record)
{
    uint32_t crc = 0U;

    if (record == NULL)
    {
        return FLASH_PARAM_ERR_PARAM;
    }

    if ((record->header.magic != FLASH_PARAM_MAGIC) ||
        (record->header.version != FLASH_PARAM_VERSION) ||
        (record->header.payload_size != sizeof(FlashParam_Data_s)))
    {
        return FLASH_PARAM_ERR_INVALID;
    }

    crc = FlashParam_Crc32(&record->data, sizeof(record->data));
    if (crc != record->header.crc32)
    {
        return FLASH_PARAM_ERR_INVALID;
    }

    return FLASH_PARAM_OK;
}

/**
  * @brief 设置内存默认状态
  * @note 用法：加载前或格式化后，把内存中的活动参数恢复为默认值
  * @note 本函数只改内存状态，不直接写 Flash
  */
static void FlashParam_SetDefaultState(void)
{
    FlashParam_BuildDefault(&flash_param_state.params);
    flash_param_state.source = FLASH_PARAM_SOURCE_DEFAULT;
    flash_param_state.sequence = 0U;
    flash_param_state.default_revision = FLASH_PARAM_DEFAULT_REVISION;
}

/**
  * @brief 读取并校验指定槽记录
  * @param slot 槽号，范围 0~1
  * @param record 输出完整记录指针
  * @return FLASH_PARAM_OK 表示读取并校验成功
  * @note 用法：内部加载、读槽和写后回读校验时使用
  */
static FlashParam_Status_e FlashParam_ReadRecord(uint8_t slot, FlashParam_Record_s *record)
{
    W25Q128JV_Status_e flash_status;

    if ((record == NULL) || (slot >= FLASH_PARAM_SLOT_COUNT))
    {
        return FLASH_PARAM_ERR_PARAM;
    }

    flash_status = W25Q128JV_ReadData(FlashParam_SlotAddress(slot),
                                      (uint8_t *)record,
                                      sizeof(*record));
    if (flash_status != W25Q128JV_OK)
    {
        return FLASH_PARAM_ERR_FLASH;
    }

    return FlashParam_ValidateRecord(record);
}

/**
  * @brief 检查 Flash JEDEC ID
  * @return FLASH_PARAM_OK 表示 Flash 通信和型号正确
  * @note 用法：加载参数前调用，避免把通信异常误判为参数损坏
  */
static FlashParam_Status_e FlashParam_CheckFlashId(void)
{
    W25Q128JV_JedecId_s id = {0};
    W25Q128JV_Status_e status;

    W25Q128JV_SpiInit();
    status = W25Q128JV_ReadJedecId(&id);
    if (status != W25Q128JV_OK)
    {
        LOGERROR("[param] flash JEDEC read fail, err=%d", status);
        return FLASH_PARAM_ERR_FLASH;
    }

    LOGINFO("[param] flash JEDEC mf=0x%02X, dev=0x%04X",
            id.manufacturer_id,
            id.device_id);
    return FLASH_PARAM_OK;
}

/**
  * @brief 比较两份参数是否一致
  * @param lhs 第一份参数指针
  * @param rhs 第二份参数指针
  * @return 1 表示一致，0 表示不一致
  * @note 用法：双槽自测中用于确认读取内容符合预期
  */
static uint8_t FlashParam_IsSameData(const FlashParam_Data_s *lhs,
                                     const FlashParam_Data_s *rhs)
{
    if ((lhs == NULL) || (rhs == NULL))
    {
        return 0U;
    }

    return (memcmp(lhs, rhs, sizeof(FlashParam_Data_s)) == 0) ? 1U : 0U;
}

/**
  * @brief 生成自测参数
  * @param params 输出自测参数指针
  * @param index 自测参数编号，0 或 1
  * @note 用法：双槽自测中生成两份明显不同的参数，确认槽切换是否生效
  */
static void FlashParam_MakeTestData(FlashParam_Data_s *params, uint8_t index)
{
    FlashParam_GetDefault(params);
    if (params == NULL)
    {
        return;
    }

    /* 用明显不同的标记值区分两个槽，避免只测到默认值。 */
    params->trace_pid.Kp = (index == 0U) ? 0.101f : 0.202f;
    params->line_yaw_pid.Kp = (index == 0U) ? 0.303f : 0.404f;
    params->turn_pid.Kp = (index == 0U) ? 0.505f : 0.606f;
    params->line_distance_m[0] = (index == 0U) ? 1.111f : 2.222f;
    params->line_accel_m = (index == 0U) ? 0.333f : 0.444f;
    params->action_speed_mps = (index == 0U) ? 0.155f : 0.255f;
}

/**
  * @brief 检查自测步骤结果
  * @param step 当前自测步骤编号
  * @param expected 期望读取到的参数
  * @param expected_source 期望参数来源
  * @param expected_sequence 期望参数序号
  * @return FLASH_PARAM_OK 表示本步骤结果符合预期
  * @note 用法：双槽自测每一步写入或破坏后调用，统一输出检查结果 log
  */
static FlashParam_Status_e FlashParam_TestExpect(uint8_t step,
                                                 const FlashParam_Data_s *expected,
                                                 FlashParam_Source_e expected_source,
                                                 uint32_t expected_sequence)
{
    FlashParam_Data_s loaded;
    FlashParam_Source_e source = FLASH_PARAM_SOURCE_DEFAULT;
    uint32_t sequence = 0U;
    FlashParam_Status_e status;

    status = FlashParam_Load(&loaded, &source, &sequence);
    if (status != FLASH_PARAM_OK)
    {
        LOGERROR("[param_test] step%u load fail, err=%d", step, status);
        return status;
    }

    if ((source != expected_source) || (sequence != expected_sequence) ||
        (FlashParam_IsSameData(&loaded, expected) == 0U))
    {
        LOGERROR("[param_test] step%u mismatch, source=%d expect=%d, seq=%u expect=%u",
                 step,
                 source,
                 expected_source,
                 sequence,
                 expected_sequence);
        return FLASH_PARAM_ERR_INVALID;
    }

    LOGINFO("[param_test] step%u OK, source=%d, seq=%u",
            step,
            source,
            sequence);
    return FLASH_PARAM_OK;
}

void FlashParam_GetDefault(FlashParam_Data_s *params)
{
    if (params != NULL)
    {
        FlashParam_BuildDefault(params);
    }
}

FlashParam_Status_e FlashParam_ReadSlot(uint8_t slot,
                                        FlashParam_Data_s *params,
                                        uint32_t *sequence)
{
    FlashParam_Record_s record;
    FlashParam_Status_e status;

    status = FlashParam_ReadRecord(slot, &record);
    if (status != FLASH_PARAM_OK)
    {
        return status;
    }

    if (params != NULL)
    {
        *params = record.data;
    }

    if (sequence != NULL)
    {
        *sequence = record.header.sequence;
    }

    return FLASH_PARAM_OK;
}

FlashParam_Status_e FlashParam_WriteSlot(uint8_t slot,
                                         const FlashParam_Data_s *params,
                                         uint32_t sequence)
{
    FlashParam_Record_s record;
    FlashParam_Record_s verify_record;
    W25Q128JV_Status_e flash_status;

    if ((params == NULL) || (slot >= FLASH_PARAM_SLOT_COUNT))
    {
        return FLASH_PARAM_ERR_PARAM;
    }

    memset(&record, 0xFF, sizeof(record));
    record.header.magic = FLASH_PARAM_MAGIC;
    record.header.version = FLASH_PARAM_VERSION;
    record.header.payload_size = sizeof(FlashParam_Data_s);
    record.header.sequence = sequence;
    record.header.default_revision = FLASH_PARAM_DEFAULT_REVISION;
    record.data = *params;
    record.header.crc32 = FlashParam_Crc32(&record.data, sizeof(record.data));

    flash_status = W25Q128JV_Erase4K(FlashParam_SlotAddress(slot));
    if (flash_status != W25Q128JV_OK)
    {
        LOGERROR("[param] erase slot%u fail, err=%d", slot, flash_status);
        return FLASH_PARAM_ERR_FLASH;
    }

    flash_status = W25Q128JV_WriteBuffer(FlashParam_SlotAddress(slot),
                                         (const uint8_t *)&record,
                                         sizeof(record));
    if (flash_status != W25Q128JV_OK)
    {
        LOGERROR("[param] write slot%u fail, err=%d", slot, flash_status);
        return FLASH_PARAM_ERR_FLASH;
    }

    if (FlashParam_ReadRecord(slot, &verify_record) != FLASH_PARAM_OK)
    {
        LOGERROR("[param] verify slot%u fail", slot);
        return FLASH_PARAM_ERR_INVALID;
    }

    if (memcmp(&record, &verify_record, sizeof(record)) != 0)
    {
        LOGERROR("[param] verify slot%u mismatch", slot);
        return FLASH_PARAM_ERR_INVALID;
    }

    LOGINFO("[param] save slot%u rev=%u seq=%u OK",
            slot,
            FLASH_PARAM_DEFAULT_REVISION,
            sequence);
    return FLASH_PARAM_OK;
}

FlashParam_Status_e FlashParam_Load(FlashParam_Data_s *params,
                                    FlashParam_Source_e *source,
                                    uint32_t *sequence)
{
    FlashParam_Record_s slot_record[FLASH_PARAM_SLOT_COUNT];
    FlashParam_Status_e slot_status[FLASH_PARAM_SLOT_COUNT];
    FlashParam_Status_e status;
    uint8_t selected_slot = 0U;
    uint8_t repair_slot = FLASH_PARAM_SLOT_COUNT;
    uint8_t need_repair = 0U;
    uint32_t repair_sequence = 1U;

    FlashParam_SetDefaultState();
    memset(flash_param_state.slot_valid, 0, sizeof(flash_param_state.slot_valid));

    if (FlashParam_CheckFlashId() != FLASH_PARAM_OK)
    {
        if (params != NULL)
        {
            *params = flash_param_state.params;
        }
        if (source != NULL)
        {
            *source = flash_param_state.source;
        }
        if (sequence != NULL)
        {
            *sequence = flash_param_state.sequence;
        }
        return FLASH_PARAM_ERR_FLASH;
    }

    for (uint8_t slot = 0U; slot < FLASH_PARAM_SLOT_COUNT; slot++)
    {
        slot_status[slot] = FlashParam_ReadRecord(slot, &slot_record[slot]);
        if (slot_status[slot] == FLASH_PARAM_OK)
        {
            flash_param_state.slot_valid[slot] = 1U;
            LOGINFO("[param] slot%u valid, rev=%u, seq=%u",
                    slot,
                    slot_record[slot].header.default_revision,
                    slot_record[slot].header.sequence);
        }
        else
        {
            LOGWARNING("[param] slot%u invalid, err=%d", slot, slot_status[slot]);
        }
    }

    selected_slot = FLASH_PARAM_SLOT_COUNT;
    for (uint8_t slot = 0U; slot < FLASH_PARAM_SLOT_COUNT; slot++)
    {
        if (flash_param_state.slot_valid[slot] == 0U)
        {
            continue;
        }

        if ((selected_slot >= FLASH_PARAM_SLOT_COUNT) ||
            (slot_record[slot].header.default_revision > slot_record[selected_slot].header.default_revision) ||
            ((slot_record[slot].header.default_revision == slot_record[selected_slot].header.default_revision) &&
             (slot_record[slot].header.sequence > slot_record[selected_slot].header.sequence)))
        {
            selected_slot = slot;
        }
    }

    if (selected_slot >= FLASH_PARAM_SLOT_COUNT)
    {
        LOGWARNING("[param] no valid slot, use defaults");
        status = FlashParam_WriteSlot(0U, &flash_param_state.params, 1U);
        if (status == FLASH_PARAM_OK)
        {
            flash_param_state.source = FLASH_PARAM_SOURCE_SLOT0;
            flash_param_state.sequence = 1U;
            flash_param_state.default_revision = FLASH_PARAM_DEFAULT_REVISION;
            flash_param_state.slot_valid[0] = 1U;
            LOGINFO("[param] repair empty slots with defaults OK");
        }
        else
        {
            LOGERROR("[param] repair empty slots fail, err=%d", status);
        }
        if (params != NULL)
        {
            *params = flash_param_state.params;
        }
        if (source != NULL)
        {
            *source = flash_param_state.source;
        }
        if (sequence != NULL)
        {
            *sequence = flash_param_state.sequence;
        }
        return FLASH_PARAM_OK;
    }

    if (slot_record[selected_slot].header.default_revision < FLASH_PARAM_DEFAULT_REVISION)
    {
        LOGWARNING("[param] code defaults newer, flash rev=%u, code rev=%u",
                   slot_record[selected_slot].header.default_revision,
                   FLASH_PARAM_DEFAULT_REVISION);
        status = FlashParam_WriteSlot(0U, &flash_param_state.params, slot_record[selected_slot].header.sequence + 1U);
        if (status == FLASH_PARAM_OK)
        {
            flash_param_state.source = FLASH_PARAM_SOURCE_SLOT0;
            flash_param_state.sequence = slot_record[selected_slot].header.sequence + 1U;
            flash_param_state.default_revision = FLASH_PARAM_DEFAULT_REVISION;
            flash_param_state.slot_valid[0] = 1U;
            LOGINFO("[param] update flash with code defaults OK, rev=%u, seq=%u",
                    flash_param_state.default_revision,
                    flash_param_state.sequence);
        }
        else
        {
            LOGERROR("[param] update flash with code defaults fail, err=%d", status);
        }

        if (params != NULL)
        {
            *params = flash_param_state.params;
        }
        if (source != NULL)
        {
            *source = flash_param_state.source;
        }
        if (sequence != NULL)
        {
            *sequence = flash_param_state.sequence;
        }
        return FLASH_PARAM_OK;
    }

    flash_param_state.params = slot_record[selected_slot].data;
    flash_param_state.source = FlashParam_SlotToSource(selected_slot);
    flash_param_state.sequence = slot_record[selected_slot].header.sequence;
    flash_param_state.default_revision = slot_record[selected_slot].header.default_revision;

    LOGINFO("[param] load slot%u, rev=%u, seq=%u",
            selected_slot,
            flash_param_state.default_revision,
            flash_param_state.sequence);

    if (flash_param_state.slot_valid[0] != flash_param_state.slot_valid[1])
    {
        repair_slot = selected_slot ^ 1U;
        repair_sequence = flash_param_state.sequence + 1U;
        if (repair_sequence == 0U)
        {
            repair_sequence = 1U;
        }
        need_repair = 1U;
    }

    if (need_repair)
    {
        status = FlashParam_WriteSlot(repair_slot, &flash_param_state.params, repair_sequence);
        if (status == FLASH_PARAM_OK)
        {
            flash_param_state.source = FlashParam_SlotToSource(repair_slot);
            flash_param_state.sequence = repair_sequence;
            flash_param_state.default_revision = FLASH_PARAM_DEFAULT_REVISION;
            flash_param_state.slot_valid[repair_slot] = 1U;
            LOGINFO("[param] repair slot%u OK, rev=%u, seq=%u",
                    repair_slot,
                    flash_param_state.default_revision,
                    repair_sequence);
        }
        else
        {
            LOGERROR("[param] repair slot%u fail, err=%d", repair_slot, status);
        }
    }

    if (params != NULL)
    {
        *params = flash_param_state.params;
    }
    if (source != NULL)
    {
        *source = flash_param_state.source;
    }
    if (sequence != NULL)
    {
        *sequence = flash_param_state.sequence;
    }

    return FLASH_PARAM_OK;
}

FlashParam_Status_e FlashParam_Save(const FlashParam_Data_s *params)
{
    uint8_t target_slot;
    uint32_t next_sequence;
    FlashParam_Status_e status;

    if (params == NULL)
    {
        return FLASH_PARAM_ERR_PARAM;
    }

    if (flash_param_state.source == FLASH_PARAM_SOURCE_DEFAULT)
    {
        target_slot = 0U;
        next_sequence = 1U;
    }
    else
    {
        target_slot = FlashParam_SourceToSlot(flash_param_state.source) ^ 1U;
        next_sequence = flash_param_state.sequence + 1U;
        if (next_sequence == 0U)
        {
            next_sequence = 1U;
        }
    }

    status = FlashParam_WriteSlot(target_slot, params, next_sequence);
    if (status != FLASH_PARAM_OK)
    {
        return status;
    }

    flash_param_state.params = *params;
    flash_param_state.source = FlashParam_SlotToSource(target_slot);
    flash_param_state.sequence = next_sequence;
    flash_param_state.default_revision = FLASH_PARAM_DEFAULT_REVISION;
    flash_param_state.slot_valid[target_slot] = 1U;

    return FLASH_PARAM_OK;
}

FlashParam_Status_e FlashParam_Format(void)
{
    W25Q128JV_Status_e status0;
    W25Q128JV_Status_e status1;

    status0 = W25Q128JV_Erase4K(FLASH_PARAM_SLOT0_ADDR);
    status1 = W25Q128JV_Erase4K(FLASH_PARAM_SLOT1_ADDR);
    FlashParam_SetDefaultState();
    memset(flash_param_state.slot_valid, 0, sizeof(flash_param_state.slot_valid));

    if ((status0 != W25Q128JV_OK) || (status1 != W25Q128JV_OK))
    {
        LOGERROR("[param] format fail, slot0=%d, slot1=%d", status0, status1);
        return FLASH_PARAM_ERR_FLASH;
    }

    LOGINFO("[param] format OK");
    return FLASH_PARAM_OK;
}

FlashParam_Status_e FlashParam_RunSelfTest(void)
{
    FlashParam_Data_s backup_params;
    FlashParam_Data_s default_params;
    FlashParam_Data_s test_slot0;
    FlashParam_Data_s test_slot1;
    FlashParam_Source_e backup_source = FLASH_PARAM_SOURCE_DEFAULT;
    uint32_t backup_sequence = 0U;
    FlashParam_Status_e result = FLASH_PARAM_OK;
    FlashParam_Status_e status;
    W25Q128JV_Status_e flash_status;

    LOGWARNING("[param_test] self test start, slots 0x%06X/0x%06X will be erased",
               FLASH_PARAM_SLOT0_ADDR,
               FLASH_PARAM_SLOT1_ADDR);

    status = FlashParam_Load(&backup_params, &backup_source, &backup_sequence);
    if (status != FLASH_PARAM_OK)
    {
        LOGERROR("[param_test] backup current params fail, err=%d", status);
        return status;
    }
    LOGINFO("[param_test] backup source=%d, seq=%u", backup_source, backup_sequence);

    FlashParam_GetDefault(&default_params);
    FlashParam_MakeTestData(&test_slot0, 0U);
    FlashParam_MakeTestData(&test_slot1, 1U);

    status = FlashParam_Format();
    if (status != FLASH_PARAM_OK)
    {
        result = status;
        goto restore;
    }

    status = FlashParam_TestExpect(1U,
                                   &default_params,
                                   FLASH_PARAM_SOURCE_SLOT0,
                                   1U);
    if (status != FLASH_PARAM_OK)
    {
        result = status;
        goto restore;
    }

    status = FlashParam_WriteSlot(0U, &test_slot0, FLASH_PARAM_TEST_SEQ0);
    if (status != FLASH_PARAM_OK)
    {
        result = status;
        goto restore;
    }

    status = FlashParam_TestExpect(2U,
                                   &test_slot0,
                                   FLASH_PARAM_SOURCE_SLOT1,
                                   FLASH_PARAM_TEST_SEQ1);
    if (status != FLASH_PARAM_OK)
    {
        result = status;
        goto restore;
    }

    status = FlashParam_WriteSlot(1U, &test_slot1, FLASH_PARAM_TEST_SEQ1);
    if (status != FLASH_PARAM_OK)
    {
        result = status;
        goto restore;
    }

    status = FlashParam_TestExpect(3U,
                                   &test_slot1,
                                   FLASH_PARAM_SOURCE_SLOT1,
                                   FLASH_PARAM_TEST_SEQ1);
    if (status != FLASH_PARAM_OK)
    {
        result = status;
        goto restore;
    }

    /* 擦除最新槽来模拟新槽损坏，加载时应自动回退到旧槽。 */
    flash_status = W25Q128JV_Erase4K(FLASH_PARAM_SLOT1_ADDR);
    if (flash_status != W25Q128JV_OK)
    {
        LOGERROR("[param_test] corrupt slot1 fail, err=%d", flash_status);
        result = FLASH_PARAM_ERR_FLASH;
        goto restore;
    }
    LOGINFO("[param_test] corrupt slot1 OK");

    status = FlashParam_TestExpect(4U,
                                   &test_slot0,
                                   FLASH_PARAM_SOURCE_SLOT1,
                                   FLASH_PARAM_TEST_SEQ1);
    if (status != FLASH_PARAM_OK)
    {
        result = status;
        goto restore;
    }

restore:
    status = FlashParam_Save(&backup_params);
    if (status != FLASH_PARAM_OK)
    {
        LOGERROR("[param_test] restore params fail, err=%d", status);
        return status;
    }
    LOGINFO("[param_test] restore params OK, source=%d, seq=%u",
            FlashParam_GetSource(),
            FlashParam_GetSequence());

    if (result == FLASH_PARAM_OK)
    {
        LOGINFO("[param_test] self test PASS");
    }
    else
    {
        LOGERROR("[param_test] self test FAIL, err=%d", result);
    }

    return result;
}

FlashParam_Status_e FlashParam_Init(void)
{
    return FlashParam_Load(NULL, NULL, NULL);
}

const FlashParam_Data_s *FlashParam_GetActive(void)
{
    return &flash_param_state.params;
}

const FlashParam_State_s *FlashParam_GetState(void)
{
    return &flash_param_state;
}

FlashParam_Source_e FlashParam_GetSource(void)
{
    return flash_param_state.source;
}

uint32_t FlashParam_GetSequence(void)
{
    return flash_param_state.sequence;
}

uint32_t FlashParam_GetDefaultRevision(void)
{
    return flash_param_state.default_revision;
}
