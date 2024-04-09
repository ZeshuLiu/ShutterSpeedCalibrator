#ifndef _SMU_H_
#define _SMU_H_
#include "main.h"
#include "stdio.h"
#include "tim.h"
#include "adc.h"

#define __DEBUG_SMU__

#define DART_READ_CNT 100                                       // 使用 DART_READ_CNT 个读数来确定快门关闭下的暗电压
#define ADC_LOG_MAX 1024                                      // 保存的ADC读数个数
#define ADC_DARK_COFF 1.5f                                      // 认为是暗电流的阈值比例

#define VREFINT_CAL ((uint16_t) *(VREFINT_CAL_ADDR))            // ADC 校准通道校准值
#define Z_GET_VDDA(VrefData) ((3.0f * VREFINT_CAL) / VrefData)  // 获取经过校准通道校准的ADC读数(float)

HAL_StatusTypeDef InitSMU(void);
float SMU_OPEN_CALC(uint64_t * open_span, double * open_sum_f);

#endif // _SMU_H_
