/*
* Flash-based Non-Volatile Memory (NVM)
* 
* This file supports storing and loading persistent configuration based on
* the STM32 builtin flash memory.
*
* The STM32F405xx has 12 flash sectors of heterogeneous size. We use the last
* two sectors for configuration data. These pages have a size of 128kB each.
* Setting any bit in these sectors to 0 is always possible, but setting them
* to 1 requires erasing the whole sector.
*
* We consider each sector as an array of 64-bit fields except the first N bytes, which we
* instead use as an allocation block. The allocation block is a compact bit-field (2 bit per entry)
* that keeps track of the state of each field (erased, invalid, valid).
*
* One sector is always considered the valid (read) sector and the other one is the
* target for the next write access: they can be considered to be ping-pong or double buffred.
*
* When writing a block of data, instead of always erasing the whole writable sector the
* new data is appended in the erased area. This presumably increases flash life span.
* The writable sector is only erased if there is not enough space for the new data.
*
* On startup, if there is exactly one sector
* whose last non-erased value has the state "valid" that sector is considered
* the valid sector. In any other case the selection is undefined.
*
*
* To write a new block of data atomically we first mark all associated fields
* as "invalid" (in the allocation table) then write the data and then mark the
* fields as "valid" (in the direction of increasing address).
*/

/*
* 基于闪存的非易失性存储器 (NVM)
*
* 该文件支持基于STM32内置闪存的持久化配置的存储和加载。
*
* STM32F405xx 有 12 个不同大小的闪存扇区。我们将最后两个扇区用于配置数据。这些页面每个都有 128kB 的大小。
* 始终可以将这些扇区中的任何位设置为 0，但将它们设置为 1 需要擦除整个扇区。
*
* 我们将每个扇区视为一个 64 位字段数组，除了前 N 个字节，我们将其用作分配块。
* 分配块是一个紧凑的位字段（每个条目 2 位），用于跟踪每个字段的状态（erased, invalid, valid）。
*
* 一个扇区始终被视为有效（read）扇区，另一个扇区是下一次写入访问的目标：它们可以被视为乒乓或双缓冲。
*
* 当写入一个数据块时，不是总是擦除整个可写扇区，而是将新数据附加到擦除区域中。这可能会增加闪存寿命。
* 只有在没有足够空间容纳新数据时，才会擦除可写扇区。
*
* 在启动时，如果恰好有一个扇区的最后一个未擦除值具有“valid”状态，则该扇区被视为有效扇区。
* 在任何其他情况下，选择是未定义的。
*
* 为了原子地写入一个新的数据块，我们首先将所有关联的字段标记为“invalid”（在分配表中），
* 然后写入数据，然后将字段标记为“valid”（在增加地址的方向）。
*/
#include "stm32_nvm.h"

#include <string.h>

#if defined(STM32F405xx)

#include <stm32f405xx.h>
#include <stm32f4xx_hal.h>

// refer to page 75 of datasheet:
// http://www.st.com/content/ccc/resource/technical/document/reference_manual/3d/6d/5a/66/b4/99/40/d4/DM00031020.pdf/files/DM00031020.pdf/jcr:content/translations/en.DM00031020.pdf
#define FLASH_SECTOR_A FLASH_SECTOR_10
#define FLASH_SECTOR_A_BASE (const volatile uint8_t*)0x80C0000UL
#define FLASH_SECTOR_A_SIZE 0x20000UL
#define FLASH_SECTOR_B FLASH_SECTOR_11
#define FLASH_SECTOR_B_BASE (const volatile uint8_t*)0x80E0000UL
#define FLASH_SECTOR_B_SIZE 0x20000UL

#elif defined(STM32F722xx)

#include <stm32f722xx.h>
#include <stm32f7xx_hal.h>

// refer to page 68 of datasheet:
// https://www.st.com/resource/en/reference_manual/dm00305990-stm32f72xxx-and-stm32f73xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
#define FLASH_SECTOR_A FLASH_SECTOR_1
#define FLASH_SECTOR_A_BASE (const volatile uint8_t*)0x8004000UL
#define FLASH_SECTOR_A_SIZE 0x4000UL
#define FLASH_SECTOR_B FLASH_SECTOR_2
#define FLASH_SECTOR_B_BASE (const volatile uint8_t*)0x8008000UL
#define FLASH_SECTOR_B_SIZE 0x4000UL

#else
#error "unknown flash sector size"
#endif

typedef enum {
    VALID = 0,
    INVALID = 1,
    ERASED = 3
} field_state_t;

typedef struct {
    size_t index;               //!< next field to be written to (can be equal to n_data)
    const uint32_t sector_id;   //!< HAL ID of this sector
    const size_t n_data;        //!< number of 64-bit fields in this sector
    const size_t n_reserved;    //!< number of 64-bit fields in this sector that are reserved for the allocation table
    const volatile uint8_t* const alloc_table;
    const volatile uint64_t* const data;
} sector_t;

sector_t sectors[] = { {
    .sector_id = FLASH_SECTOR_A,
    .n_data = FLASH_SECTOR_A_SIZE >> 3,
    .n_reserved = (FLASH_SECTOR_A_SIZE >> 3) >> 5,
    .alloc_table = FLASH_SECTOR_A_BASE,
    .data = (uint64_t *)FLASH_SECTOR_A_BASE
}, {
    .sector_id = FLASH_SECTOR_B,
    .n_data = FLASH_SECTOR_B_SIZE >> 3,
    .n_reserved = (FLASH_SECTOR_B_SIZE >> 3) >> 5,
    .alloc_table = FLASH_SECTOR_B_BASE,
    .data = (uint64_t *)FLASH_SECTOR_B_BASE
}};

uint8_t read_sector_; // 0 or 1 to indicate which sector to read from and which to write to
size_t n_staging_area_; // number of 64-bit values that were reserved using NVM_start_write
size_t n_valid_; // number of 64-bit fields that can be read

static const uint32_t FLASH_ERR_FLAGS =
#if defined(FLASH_FLAG_EOP)
        FLASH_FLAG_EOP |
#endif
#if defined(FLASH_FLAG_OPERR)
        FLASH_FLAG_OPERR |
#endif
#if defined(FLASH_FLAG_WRPERR)
        FLASH_FLAG_WRPERR |
#endif
#if defined(FLASH_FLAG_PGAERR)
        FLASH_FLAG_PGAERR |
#endif
#if defined(FLASH_FLAG_PGSERR)
        FLASH_FLAG_PGSERR |
#endif
#if defined(FLASH_FLAG_PGPERR)
        FLASH_FLAG_PGPERR |
#endif
        0;

static void HAL_FLASH_ClearError() {
    __HAL_FLASH_CLEAR_FLAG(FLASH_ERR_FLAGS);
}


// @brief Erases a flash sector. This sets all bits in the sector to 1.
// The sector's current index is reset to the minimum value (n_reserved).
// @returns 0 on success or a non-zero error code otherwise

// @brief 擦除闪存扇区。 这会将扇区中的所有位设置为 1。
// 扇区的当前索引被重置为最小值（n_reserved）。
// @returns 0 成功或非零错误代码否则
int erase(sector_t *sector) {
    FLASH_EraseInitTypeDef erase_struct = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
#if defined(FLASH_OPTCR_nDBANK)
        .Banks = 0, // only used for mass erase
#endif
        .Sector = sector->sector_id,
        .NbSectors = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3
    };
    HAL_FLASH_Unlock();
    HAL_FLASH_ClearError();
    uint32_t sector_error;
    if (HAL_FLASHEx_Erase(&erase_struct, &sector_error) != HAL_OK)
        goto fail;
    sector->index = sector->n_reserved;

    HAL_FLASH_Lock();
    return 0;
fail:
    HAL_FLASH_Lock();
    //printf("erase failed: %u \r\n", HAL_FLASH_GetError());
    return HAL_FLASH_GetError(); // non-zero
}


// @brief Writes states into the allocation table.
// The write operation goes in the direction of increasing indices.
// @param state: 11: erased, 10: writing, 00: valid data
// @returns 0 on success or a non-zero error code otherwise

// @brief 将状态写入分配表。
// 写操作朝着索引递增的方向进行。
// @param state: 11: erased, 10: writing, 00: valid data
// @returns 0 成功或非零错误代码
int set_allocation_state(sector_t *sector, size_t index, size_t count, field_state_t state) {
    if (index < sector->n_reserved)
        return -1;
    if (index + count >= sector->n_data)
        return -1;

    // expand state to state for 4 values
    const uint8_t states = (state << 0) | (state << 2) | (state << 4) | (state << 6);
    
    // handle unaligned start
    uint8_t mask = ~(0xff << ((index & 0x3) << 1));
    count += index & 0x3;
    index -= index & 0x3;

    HAL_FLASH_Unlock();
    HAL_FLASH_ClearError();

    // write states
    for (; count >= 4; count -= 4, index += 4) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uintptr_t)&sector->alloc_table[index >> 2], states | mask) != HAL_OK)
            goto fail;
        mask = 0;
    }

    // handle unaligned end
    if (count) {
        mask |= ~(0xff >> ((4 - count) << 1));
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uintptr_t)&sector->alloc_table[index >> 2], states | mask) != HAL_OK)
            goto fail;
    }
    
    HAL_FLASH_Lock();
    return 0;
fail:
    HAL_FLASH_Lock();
    return HAL_FLASH_GetError(); // non-zero
}

// @brief Reads the allocation table from behind to determine how many fields match the
// reference state.
// @param sector: The sector on which to perform the search
// @param max_index: The maximum index that should be considered
// @param ref_state: The reference state
// @param state: Set to the first encountered state that is unequal to ref_state.
//               Set to ref_state if all encountered states are equal to ref_state.
// @returns The smallest index that points to a field with ref_state.
//          This value is at least sector->n_reserved and at most max_index.

// @brief 从后面读取分配表以确定有多少字段匹配引用状态。
// @param sector: 执行搜索的扇区
// @param max_index: 应该考虑的最大索引
// @param ref_state: 参考状态
// @param state: 设置为第一个遇到的不等于 ref_state 的状态。
// 如果所有遇到的状态都等于 ref_state，则设置为 ref_state。
// @returns 指向具有 ref_state 的字段的最小索引。
// 这个值至少是sector->n_reserved，至多是max_index。
size_t scan_allocation_table(sector_t *sector, size_t max_index, field_state_t ref_state, field_state_t *state) {
    const uint8_t ref_states = (ref_state << 0) | (ref_state << 2) | (ref_state << 4) | (ref_state << 6);
    size_t index = (((max_index + 3) >> 2) << 2); // start at the max index but round up to a multiple of 4
    size_t ignore = index - max_index;
    uint8_t states = ref_states;

    //printf("scan from %08x to %08x for %02x\r\n", index, sector->n_reserved, ref_states); osDelay(5);

    // read 4 states at a time
    for (; index >= (sector->n_reserved + 4); index -= 4) {
        states = sector->alloc_table[(index - 1) >> 2];
        if (ignore) { // ignore the upper 1, 2 or 3 states if max_index was unaligned
            uint8_t ignore_mask = ~(0xff >> (ignore << 1));
            states = (states & ~ignore_mask) | (ref_states & ignore_mask);
            ignore = 0;
        }
        if (states != ref_states)
            break;
    }

    // once we encounterd a byte with any state mismatch determine which of the 4 states it is
    for (; ((states >> 6) == (ref_states & 0x3)) && (index > sector->n_reserved); index--) {
        states <<= 2;
    }
    
    *state = states >> 6;
    //printf("(it's %02x)\r\n", index); osDelay(5);
    return index;
}

// Loads the head of the NVM data.
// If this function fails subsequent calls to NVM functions (other than NVM_init or NVM_erase)
// cause undefined behavior.
// @returns 0 on success or a non-zero error code otherwise

// 加载 NVM 数据的头部。
// 如果此函数在后续调用 NVM 函数（除 NVM_init 或 NVM_erase 之外）失败，则会导致未定义的行为。
// @returns 0 成功或非零错误代码
int NVM_init(void) {
    field_state_t sector0_state, sector1_state;
    // 全扇区反向扫描未被 ERASED 的第一个 block 的 index 和它的状态 state
    sectors[0].index = scan_allocation_table(&sectors[0], sectors[0].n_data,
                ERASED, &sector0_state);
    sectors[1].index = scan_allocation_table(&sectors[1], sectors[1].n_data,
                ERASED, &sector1_state);
    //printf("sector states: %02x, %02x\r\n", sector0_state, sector1_state); osDelay(5);

    // Select valid sector on a best effort basis
    // (in unfortunate cases valid_sector might actually point
    // to an invalid or erased sector)
    // 尽最大努力选择有效扇区（在不幸的情况下，valid_sector 实际上可能指向无效或已擦除的扇区）
    read_sector_ = 0; // 默认从 sector0 读取
    if (sector1_state == VALID) // 如果 sector1 的最新状态是 VALID 有效的就从 sector1 读
        read_sector_ = 1;
    
    // count the number of valid fields
    // 统计有效域的 block 数量
    sector_t *read_sector = &sectors[read_sector_];
    uint8_t first_nonvalid_state;
    // 从当前 index 反向(减小的方向)查找第一个不是 VALID 的 block 索引
    size_t min_valid_index = scan_allocation_table(read_sector, read_sector->index,
        VALID, &first_nonvalid_state);
    n_valid_ = read_sector->index - min_valid_index;
    
    n_staging_area_ = 0;

    int status = 0;
    /*// bring non-valid sectors into a known state
    this is not absolutely required
    if (sector0_state != VALID)
        status |= erase(&sectors[0]);
    if (sector1_state != VALID)
        status |= erase(&sectors[1]);
    */
    return status;
}

// @brief Erases all data in the NVM.
//
// If this function fails subsequent calls to NVM functions (other than NVM_init or NVM_erase)
// cause undefined behavior.
// Caution: this function may take a long time (like 1 second)
//
// @returns 0 on success or a non-zero error code otherwise

// @brief 擦除 NVM 中的所有数据。
//
// 如果此函数在后续调用 NVM 函数（除 NVM_init 或 NVM_erase 之外）失败，则会导致未定义的行为。
// 注意：这个函数可能需要很长时间（比如 1 秒）
//
// @returns 0 成功或非零错误代码否则
int NVM_erase(void) {
    read_sector_ = 0;
    sectors[0].index = sectors[0].n_reserved; // 索引指向保留域
    sectors[1].index = sectors[1].n_reserved;

    int state = 0;
    state |= erase(&sectors[0]);
    state |= erase(&sectors[1]);
    return state;
}

// @brief Returns the maximum number of bytes that can be read using NVM_read.
// This holds until NVM_commit is called.

// @brief 返回可以使用 NVM_read 读取的最大字节数。
// 这一直保持到 NVM_commit 被调用。
size_t NVM_get_max_read_length(void) {
    return n_valid_ << 3; // 每个可用的 block 是 64-bit 的数据，所以 n_valid_ * 8 就是剩余字节数
}

// @brief Returns the maximum length (in bytes) that can passed to NVM_start_write.
// This holds until NVM_commit is called.

// @brief 返回可以传递给 NVM_start_write 的最大长度（以字节为单位）。
// 这一直保持到 NVM_commit 被调用。 
size_t NVM_get_max_write_length(void) {
    sector_t *target = &sectors[1 - read_sector_]; // 向另一个 sector 写
    return (target->n_data - target->n_reserved) << 3; // 最多写满数据域
}

// @brief Reads from the latest committed block in the non-volatile memory.
// The function either succeeds or leaves the provided buffer unmodified.
// @param offset: offset in bytes (0 meaning the beginning of the valid area)
// @param data: buffer to write to
// @param length: length in bytes (if (offset + length) is out of range, the function fails)
// @returns 0 on success or a non-zero error code otherwise

// @brief 从非易失性存储器中最新提交的块中读取。
// 该函数要么成功，要么不修改提供的缓冲区。
// @param offset: 以字节为单位的偏移量（0 表示有效区域的开始）
// @param data: 要写入的缓冲区
// @param length: 以字节为单位的长度（如果（偏移量 + 长度）超出范围，则函数失败）
// @returns 0 成功或非零错误代码否则
int NVM_read(size_t offset, uint8_t *data, size_t length) {
    if (offset + length > (n_valid_ << 3)) 
        return -1; // 超出索引区间
    sector_t *read_sector = &sectors[read_sector_];
    const uint8_t *src_ptr = ((const uint8_t *)&read_sector->data[read_sector->index - n_valid_]) + offset;
    memcpy(data, src_ptr, length);
    return 0;
}

// @brief Starts an atomic write operation.
//
// The most recent valid NVM data is not modified or invalidated until NVM_commit is called.
// The length must be at most equal to the size indicated by NVM_get_max_write_length().
//
// @param length: Length of the staging block that should be created

// @brief 开始一个原子写操作。
//
// 在调用 NVM_commit 之前，不会修改或使最近的有效 NVM 数据无效。
// 长度必须最多等于 NVM_get_max_write_length() 指示的大小，即最多写满。
//
// @param length: 应创建的暂存块的长度
int NVM_start_write(size_t length) {
    int status = 0;
    sector_t *target = &sectors[1 - read_sector_];

    length = (length + 7) >> 3; // round to multiple of 64 bit 对齐
    if (length > target->n_data - target->n_reserved)
        return -1; // 写入数据超限

    // make room for the new data 现存 ERASED 区域不够则擦除整个扇区
    if (length > target->n_data - target->index)
        if ((status = erase(target)))
            return status;

    // invalidate the fields we're about to write
    // 现存 ERASED 区域足够则将要写入的区域标记为 INVALID
    status = set_allocation_state(target, target->index, length, INVALID);
    if (status)
        return status;

    n_staging_area_ = length;
    return 0;
}

// @brief Writes to the current data block that was opened with NVM_start_write.
//
// The operation fails if (offset + length) is larger than the length passed to NVM_start_write.
// The most recent valid NVM data is not modified or invalidated until NVM_commit is called.
// Warning: Writing different data to the same area multiple times during a single transaction
// will cause data corruption.
//
// @param offset: The offset in bytes, 0 being the beginning of the staging block.
// @param data: Pointer to the data that should be written
// @param length: Data length in bytes

// @brief 写入使用 NVM_start_write 打开的当前数据块。
//
// 如果 (offset + length) 大于传递给 NVM_start_write 的长度，则操作失败。
// 在调用 NVM_commit 之前，不会修改或使最近的有效 NVM 数据无效。
// 警告：在单个事务期间多次将不同的数据写入同一区域会导致数据损坏。
//
// @param offset：以字节为单位的偏移量，0 是暂存块的开始。
// @param data: 指向应写入数据的指针
// @param length: 以字节为单位的数据长度
int NVM_write(size_t offset, uint8_t *data, size_t length) {
    if (offset + length > (n_staging_area_ << 3))
        return -1;
    sector_t *target = &sectors[1 - read_sector_];

    HAL_FLASH_Unlock();
    HAL_FLASH_ClearError();

    // handle unaligned start
    for (; (offset & 0x3) && length; ++data, ++offset, --length)
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,
                ((uintptr_t)&target->data[target->index]) + offset, *data) != HAL_OK)
            goto fail;

    // write 32-bit values (64-bit doesn't work)
    for (; length >= 4; data += 4, offset += 4, length -=4)
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                ((uintptr_t)&target->data[target->index]) + offset, *(uint32_t*)data) != HAL_OK)
            goto fail;

    // handle unaligned end
    for (; length; ++data, ++offset, --length)
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,
                ((uintptr_t)&target->data[target->index]) + offset, *data) != HAL_OK)
            goto fail;

    HAL_FLASH_Lock();
    return 0;
fail:
    HAL_FLASH_Lock();
    return HAL_FLASH_GetError(); // non-zero
}

// @brief Commits the new data to NVM atomically.
int NVM_commit(void) {
    sector_t *read_sector = &sectors[read_sector_];
    sector_t *write_sector = &sectors[1 - read_sector_];

    // mark the newly-written fields as valid
    int status = set_allocation_state(write_sector, write_sector->index, n_staging_area_, VALID);
    if (status)
        return status;

    write_sector->index += n_staging_area_;
    n_valid_ = n_staging_area_;
    n_staging_area_ = 0;
    read_sector_ = 1 - read_sector_;

    // invalidate the other sector
    if (read_sector->index < read_sector->n_data) {
        status = set_allocation_state(read_sector, read_sector->index, 1, INVALID);
        read_sector->index += 1;
    } else {
        status = erase(read_sector);
    }

    return status;
}


#include <cmsis_os.h>
#include <stdio.h>
/** @brief Call this at startup to test/demo the NVM driver

 Expected output when starting with a fully erased NVM

    [1st boot]
    === NVM TEST ===
    NVM is empty
    write 0x00, ..., 0x25 to NVM
    new data committed to NVM
    
    [2nd boot]
    === NVM TEST ===
    NVM contains 40 valid bytes:
    00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f
    10 11 12 13 14 15 16 17 18 19 1a 1b 1c 1d 1e 1f
    20 21 22 23 24 25 ff ff
    write 0xbd, ..., 0xe2 to NVM
    new data committed to NVM

    [3rd boot]
    === NVM TEST ===
    NVM contains 40 valid bytes:
    bd be bf c0 c1 c2 c3 c4 c5 c6 c7 c8 c9 ca cb cc
    cd ce cf d0 d1 d2 d3 d4 d5 d6 d7 d8 d9 da db dc
    dd de df e0 e1 e2 ff ff
    write 0xcb, ..., 0xf0 to NVM
    new data committed to NVM
*/
void NVM_demo(void) {
    const size_t len = 38;
    uint8_t data[len];
    int progress = 0;
    uint8_t seed = 0;

    osDelay(100);
    printf("=== NVM TEST ===\r\n"); osDelay(5);
    //NVM_erase();
    if (progress++, NVM_init() != 0)
        goto fail;
    
    // load bytes from NVM and print them
    size_t available = NVM_get_max_read_length();
    if (available) {
        printf("NVM contains %d valid bytes:\r\n", available); osDelay(5);
        uint8_t buf[available];
        if (progress++, NVM_read(0, buf, available) != 0)
            goto fail;
        for (size_t pos = 0; pos < available; ++pos) {
            seed += buf[pos];
            printf(" %02x", buf[pos]);
            if ((((pos + 1) % 16) == 0) || ((pos + 1) == available))
                printf("\r\n");
            osDelay(2);
        }
    } else {
        printf("NVM is empty\r\n"); osDelay(5);
    }

    // store new bytes in NVM (data based on seed)
    printf("write 0x%02x, ..., 0x%02x to NVM\r\n", seed, seed + len - 1); osDelay(5);
    for (size_t i = 0; i < len; i++)
        data[i] = seed++;
    if (progress++, NVM_start_write(len) != 0)
        goto fail;
    if (progress++, NVM_write(0, data, len / 2))
        goto fail;
    if (progress++, NVM_write(len / 2, &data[len / 2], len - (len / 2)))
        goto fail;
    if (progress++, NVM_commit())
        goto fail;
    printf("new data committed to NVM\r\n"); osDelay(5);

    return;

fail:
    printf("NVM test failed at %d!\r\n", progress);
}
