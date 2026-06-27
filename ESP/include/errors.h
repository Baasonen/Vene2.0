#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "errors_generated.h"
#include "state.h"

// Live errors: always calculated in error task
// Latched errors: set by individual faults, require manual clearing

inline void setError(ErrorBit bit)
{
    __atomic_fetch_or(&globalState.status.errorCode, (uint32_t)1UL << bit, __ATOMIC_SEQ_CST);
}

inline void clearError(ErrorBit bit)
{
    __atomic_fetch_and(&globalState.status.errorCode, ~((uint32_t)1UL << bit), __ATOMIC_SEQ_CST);
}

inline void setErrorIf(ErrorBit bit, bool condition)
{
    condition ? setError(bit) : clearError(bit);
}

inline bool hasError(ErrorBit bit)
{
    return __atomic_load_n(&globalState.status.errorCode, __ATOMIC_SEQ_CST) & ((uint32_t)1UL << bit);
}

inline uint32_t getErrorCode()
{
    return __atomic_load_n(&globalState.status.errorCode, __ATOMIC_SEQ_CST);
}

inline void clearAllErrors()
{
    uint32_t current = __atomic_load_n(&globalState.status.errorCode, __ATOMIC_SEQ_CST);
    __atomic_store_n(&globalState.status.errorCode, current & 0x1u, __ATOMIC_SEQ_CST); // Preserve the init bit
}

inline void printActiveErrors()
{
    uint32_t code = getErrorCode();
    if (code == 0)
    {
        Serial.println("  (none)");
        return;
    }
    for (size_t i = 0; i < ERROR_BIT_COUNT; i++)
    {
        if (code & ((uint32_t)1UL << ERROR_BIT_TABLE[i].bit))
        {
            Serial.printf("  [%s] %s\n", ERROR_BIT_TABLE[i].name, ERROR_BIT_TABLE[i].description);
        }
    }
}

void errorTask(void* pv);