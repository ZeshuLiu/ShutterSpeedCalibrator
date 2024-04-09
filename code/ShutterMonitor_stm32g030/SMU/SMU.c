/**
  ******************************************************************************
  * @file SMU
  * @author 刘泽书 (liuzeshu10@163.com) 
  * @brief 宝丽来&富士拍立得快门速度测试器
  * @date 2024-02-27
  * @version 0.0.1
  ******************************************************************************
*/

#include "SMU.h"
uint16_t ADC_Data[2] = {0};
uint16_t ADC_DARK_u16 = 0;
uint16_t ADC_DARK_THRASH = 0;
float ADC_DARK_f = 0.0f;
uint16_t ADC_LOG[ADC_LOG_MAX] = {0};  // 保存的快门打开状态下的ADC读数
uint16_t ADC_LOG_COUNT = 0;           // 快门ADC读数的个数


/**
 * @brief 进行ADC校准，开启ADC连续转换（计时器驱动），读取暗电压的大小
 *
 * @return HAL_StatusTypeDef 是否成功
 */
HAL_StatusTypeDef InitSMU(void)
{
  HAL_StatusTypeDef hst = HAL_OK;
  extern _Bool NewADC;

  printf("[INFO]初始化SMU配置!\r\n");

  // 校准ADC
  printf("[INFO] ADC校准开始!\r\n");
  hst |= HAL_ADCEx_Calibration_Start(&hadc1);
  if (hst != HAL_OK){
    printf("[ERROR] ADC校准失败!\r\n");
    return hst;
  }
  printf("[OK] ADC校准完成!\r\n");

  // 开启ADC连续采样
  hst |= HAL_TIM_Base_Start_IT(&htim3);
  if (hst != HAL_OK){
    printf("[ERROR] 定时器启动失败!\r\n");
    return hst;
  }
  printf("[OK] 定时器启动!\r\n");
  hst |= HAL_ADC_Start_DMA(&hadc1, (uint32_t *) ADC_Data, 2);
  if (hst != HAL_OK){
    printf("[ERROR] ADC连续转换开始失败!\r\n");
    return hst;
  }
  printf("[OK] ADC连续转换开始!\r\n");
  HAL_Delay(1000);

  // 获取暗光下的电压大小
  printf("[INFO] 开始读取暗光下的电压!\r\n");
  while (!NewADC);  // 等待新数据出现 //! 可能会卡死在这
  printf("[INFO] ADC 参考电压校准值为0x%X, VDDA:%04f \r\n", VREFINT_CAL, Z_GET_VDDA(ADC_Data[1]));
  for (uint8_t i = 0; i < DART_READ_CNT; i++)
  {
    while (!NewADC);  // 等待新数据出现 //! 可能会卡死在这
    ADC_DARK_u16 += ADC_Data[0];
    ADC_DARK_f += ADC_Data[0]*Z_GET_VDDA(ADC_Data[1])/4096;
  }
  ADC_DARK_u16 /= DART_READ_CNT;
  ADC_DARK_f /= DART_READ_CNT;
  ADC_DARK_THRASH = (uint16_t) (ADC_DARK_COFF * ADC_DARK_u16);

  printf("[OK] 读取暗光下的电压完成，暗光下的电压为:%.4fV | 0x%X!\r\n", ADC_DARK_f, ADC_DARK_u16);

  return HAL_OK;
}

/**
 * @brief 计算快门开启时间，当收到新数据后进行调用，调用后如果判断快门打开则会阻塞在该函数内直至快门关闭
 *
 * @param open_span 指向存放打开时间计数的地址
 * @param open_sum_f 指向存放打开光强积分的地址
 *
 * @return float 快门实际的开启时间。0表示未开启
 */
float SMU_OPEN_CALC(uint64_t * open_span, double * open_sum_f)
{
  uint8_t debounce_cnt = 0;
  *open_span = 0;   // 对传入指针处的值进行初始化
  *open_sum_f = 0;  // 对传入指针处的值进行初始化
  extern _Bool NewADC;

  // 如果此时快门未开，需要5次以上消抖(20k下最多支持1/4000的快门速度)
  for ( ADC_LOG_COUNT = 0; ADC_LOG_COUNT < 5; ADC_LOG_COUNT++)
  {
    while(!NewADC) __nop();
    NewADC = 0;

    if (ADC_Data[0] < ADC_DARK_THRASH){
      return 0.0f;
    }
    ADC_LOG[ADC_LOG_COUNT] = ADC_Data[0];
  }
  // 快门打开，开始计数

  // 打开状态一直循环
  debounce_cnt = 0;
  while (1)
  {
    while(!NewADC) __nop();
    NewADC = 0;

    // 如果关闭了则退出循环
    if(ADC_Data[0] < ADC_DARK_THRASH){
      debounce_cnt ++;
    }
    if (debounce_cnt == 5)
    {
#ifdef __DEBUG_SMU__
      printf("快门关闭\r\n");
#endif
      break;
    }

    // 此时还在开着
    //- *open_span += 1;
    *open_sum_f += ADC_Data[0]*Z_GET_VDDA(ADC_Data[1])/4096;
    if(ADC_LOG_COUNT < ADC_LOG_MAX) ADC_LOG[ADC_LOG_COUNT] = ADC_Data[0];   //! 防止越界
		ADC_LOG_COUNT++;
  }

  // 此时已经关闭
  *open_span = ADC_LOG_COUNT;
  //- *open_sum_f /= *open_span;
#ifdef __DEBUG_SMU__
  printf("快门打开光强积分: %g，打开时间计数为: %llu，打开时间: %llums\r\n", *open_sum_f, *open_span, (uint64_t) ( (*open_span) / 20));
#endif

  return (float) (*open_sum_f);
}
