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

/**
 * @brief computes cosine with cordic coprocessor in q15 format without conversion.
 * 
 * @param theta_q15 input angle in q15 format, scaled by 1/pi.
 * @return int16_t
 */
// static inline int16_t cordic_q15_cos(int16_t theta_q15) {
//     CORDIC_ConfigTypeDef sConfig;
//     int32_t input_packed = ((int32_t)theta_q15) & 0xffff;
//     int32_t output_packed;
//     int16_t cos_q15;

//     sConfig.Function = CORDIC_FUNCTION_COSINE;
//     sConfig.Precision = CORDIC_PRECISION_6CYCLES;
//     sConfig.Scale = CORDIC_SCALE_0;
//     sConfig.NbWrite = CORDIC_NBWRITE_1;
//     sConfig.NbRead = CORDIC_NBREAD_1;
//     sConfig.InSize = CORDIC_INSIZE_16BITS;
//     sConfig.OutSize = CORDIC_OUTSIZE_16BITS;

//     HAL_CORDIC_Configure(&hcordic, &sConfig);
//     HAL_CORDIC_CalculateZO(&hcordic, &input_packed, &output_packed, 1, 0);
//     // cos_q15 = (int16_t)(output_packed & 0xffff);
//     cos_q15 = (int16_t)output_packed;

//     return cos_q15;
// }

static inline int16_t cordic_q15_cos(int16_t theta_q15) {
    CORDIC_ConfigTypeDef sConfig;
    // Fix: Cast to uint16_t before int32_t to prevent sign extension
    int32_t input_packed = (uint16_t)theta_q15; 
    int32_t output_packed = 0;

    sConfig.Function = CORDIC_FUNCTION_COSINE;
    sConfig.Precision = CORDIC_PRECISION_6CYCLES;
    sConfig.Scale = CORDIC_SCALE_0;
    sConfig.NbWrite = CORDIC_NBWRITE_1;
    sConfig.NbRead = CORDIC_NBREAD_1;
    sConfig.InSize = CORDIC_INSIZE_16BITS;
    sConfig.OutSize = CORDIC_OUTSIZE_16BITS;

    HAL_CORDIC_Configure(&hcordic, &sConfig);
    
    // Pass the address of our 32-bit container
    HAL_CORDIC_CalculateZO(&hcordic, &input_packed, &output_packed, 1, 0);

    // Result is in the lower 16 bits
    return (int16_t)(output_packed & 0xFFFF);
}

#endif  /* STM32_CORDIC_H */