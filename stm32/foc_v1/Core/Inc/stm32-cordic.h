#ifndef STM32_CORDIC_H
#define STM32_CORDIC_H

#include "main.h"
#include "math.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal_cordic.h"
#include <stdint.h>

extern CORDIC_HandleTypeDef hcordic;

/**
 * @brief computes cosine with cordic coprocessor in q31 format without conversion.
 * 
 * @param theta_q31 input angle in q31 format, scaled by 1/pi.
 * @return int32_t
 */
static inline int32_t cordic_q31_cos(int32_t theta_q31) {
    CORDIC_ConfigTypeDef sConfig;
    int32_t cos_q31;

    sConfig.Function = CORDIC_FUNCTION_COSINE;
    sConfig.Precision = CORDIC_PRECISION_6CYCLES;
    sConfig.Scale = CORDIC_SCALE_0;
    sConfig.NbWrite = CORDIC_NBWRITE_1;
    sConfig.NbRead = CORDIC_NBREAD_1;
    sConfig.InSize = CORDIC_INSIZE_32BITS;
    sConfig.OutSize = CORDIC_OUTSIZE_32BITS;

    HAL_CORDIC_Configure(&hcordic, &sConfig);
    HAL_CORDIC_CalculateZO(&hcordic, &theta_q31, &cos_q31, 1, 0);

    return cos_q31;
}

/**
 * @brief computes sine with cordic coprocessor in q31 format without conversion.
 * 
 * @param theta_q31 input angle in q31 format, scaled by 1/pi.
 * @return int32_t
 */
static inline int32_t cordic_q31_sin(int32_t theta_q31) {
    CORDIC_ConfigTypeDef sConfig;
    int32_t sin_q31;

    sConfig.Function = CORDIC_FUNCTION_SINE;
    sConfig.Precision = CORDIC_PRECISION_6CYCLES;
    sConfig.Scale = CORDIC_SCALE_0;
    sConfig.NbWrite = CORDIC_NBWRITE_1;
    sConfig.NbRead = CORDIC_NBREAD_1;
    sConfig.InSize = CORDIC_INSIZE_32BITS;
    sConfig.OutSize = CORDIC_OUTSIZE_32BITS;

    HAL_CORDIC_Configure(&hcordic, &sConfig);
    HAL_CORDIC_CalculateZO(&hcordic, &theta_q31, &sin_q31, 1, 0);

    return sin_q31;
}

static inline HAL_StatusTypeDef cordic_q31_cos_sin(int32_t theta_q31, int32_t *result_ptr) {
    CORDIC_ConfigTypeDef sConfig;

    sConfig.Function = CORDIC_FUNCTION_COSINE;
    sConfig.Precision = CORDIC_PRECISION_6CYCLES;
    sConfig.Scale = CORDIC_SCALE_0;
    sConfig.NbWrite = CORDIC_NBWRITE_1;
    sConfig.NbRead = CORDIC_NBREAD_2;
    sConfig.InSize = CORDIC_INSIZE_32BITS;
    sConfig.OutSize = CORDIC_OUTSIZE_32BITS;

    HAL_CORDIC_Configure(&hcordic, &sConfig);
    return HAL_CORDIC_CalculateZO(&hcordic, &theta_q31, result_ptr, 1, 0);
}

#endif  /* STM32_CORDIC_H */