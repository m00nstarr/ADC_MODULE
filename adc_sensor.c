#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes
#include "adc_sensor.h"
#include <math.h>
#include "relay_control.h"

/*
 * ADC MODULE (LV BAT+1, +2, +3
 * volatile ADC_LV_B# is ADC value, 
 * (ADC_LV_B * ADC_REF_Voltage(1.2V) / ADC_MAX(4095) ) = board voltage value 
 * board_ voltage_value * 118 / 18 is real_value
 */

//#define DEBUG 1

volatile float INP_Voltage_B1, INP_Voltage_B2, INP_Voltage_B3, INP_Voltage_3V3, INP_Voltage_5V;
volatile uint16_t ADC_LV_B1, ADC_LV_B2, ADC_LV_B3;
volatile uint16_t TEMP_MP, TEMP_MN, TEMP_BOARD;
volatile uint16_t ADC_3V3, ADC_5V;

extern volatile P_state pra_state;

float RT_table[RT_TABLE_MAX_IDX] = {
    188.5 ,178.5 ,169.0 ,160.2 ,151.9,
    144.1 ,136.7 ,129.8, 123.3 ,117.1,
    111.3 ,105.7 ,100.5 ,95.52 ,90.84,
    86.43 ,82.26 ,78.33 ,74.61 ,71.10,
    67.77 ,64.57 ,61.54 ,58.68 ,55.97,
    53.41 ,50.98 ,48.68 ,46.50 ,44.43,
    42.47 ,40.57 ,38.77 ,37.06 ,35.44,
    33.90 ,32.44 ,31.05 ,29.73 ,28.48,
    27.28 ,26.13 ,25.03 ,23.99 ,23.00,
    22.05 ,21.15 ,20.30 ,19.48 ,18.70,
    17.96 ,17.24 ,16.56 ,15.9  ,15.28,
    14.69 ,14.12 ,13.58 ,13.06 ,12.56,
    12.09 ,11.63 ,11.2  ,10.78 ,10.38,
    10,    9.632 ,9.281 ,8.944 ,8.622,
    8.313, 8.014, 7.728 ,7.454 ,7.192,
    6.94  ,6.699 ,6.467 ,6.245 ,6.032,
    5.827 ,5.629 ,5.438 ,5.255, 5.08,
    4.911 ,4.749 ,4.593 ,4.443 ,4.299,
    4.16  ,4.026 ,3.896 ,3.771 ,3.651,
    3.536 ,3.425 ,3.318 ,3.215 ,3.116,
    3.02, 2.927 ,2.838 ,2.751 ,2.668,
    2.588 ,2.511 ,2.436 ,2.364 ,2.295,
    2.228 ,2.163 ,2.1 ,2.039 ,1.98,
    1.924 ,1.869 ,1.816 ,1.765 ,1.716,
    1.668 ,1.621 ,1.577 ,1.533 ,1.491,
    1.451 ,1.411 ,1.373 ,1.336 ,1.3,
    1.266 ,1.232 ,1.2 ,1.168 ,1.137,
    1.108 ,1.079 ,1.051 ,1.024 ,0.9984,
    0.9731 ,0.9484 ,0.9246 ,0.9014 ,0.8789,
    0.8572 ,0.836 ,0.8155 ,0.7956 ,0.7763,
    0.7576 ,0.7393 ,0.7215 ,0.7043 ,0.6876,
    0.6713 ,0.6555 ,0.6402 ,0.6253 ,0.6108,
    0.5967 ,0.5829 ,0.5695 ,0.5565 ,0.5438,
    0.5315 ,0.5195 ,0.5078 ,0.4965 ,0.4855,
    0.4747 ,0.4642 ,0.454 ,0.4441 ,0.4344,
    0.425 ,0.4158 ,0.4068 ,0.3981 ,0.3897,
    0.3814 ,0.3733 ,0.3655 ,0.3578 ,0.3503,
    0.3431 ,0.336 ,0.329 ,0.3223 ,0.3157,
    0.3093 ,0.303 ,0.2969 ,0.291 ,0.2851,
    0.2794 ,0.2739 ,0.2685 ,0.2632 ,0.2581,
    0.253 ,0.2481 ,0.2433 ,0.2386 ,0.234,
    0.2296 ,0.2252 ,0.2209 ,0.2168 ,0.2127,
    0.2087 ,0.2048 ,0.201 ,0.1973 ,0.1937,
    0.1901 ,0.1867 ,0.1833 ,0.1799 ,0.1767,
    0.1735 ,0.1704 ,0.1674 ,0.1644 ,0.1615,
    0.1587 ,0.1559 ,0.1532 ,0.1505 ,0.1479,
    0.1453 ,0.1428 ,0.1404 ,0.138 ,0.1356,
    0.1333 ,0.1311 ,0.1289 ,0.1267 ,0.1246, 0.1226
}; 

float ratio_table[TABLE_MAX_IDX] = 
{
  44.605,
  33.281,
  25.044,
  19.003,
  14.536,

  11.206,
  8.7041,
  6.8104,
  5.3665,
  4.2576,

  3.4001,
  2.7326,
  2.2096,
  1.7973,
  1.4703,

  1.2093,
  1.0,
  0.83113,
  0.69418,
  0.58255,

  0.49112,
  0.41587,
  0.35365,
  0.30197,
  0.25888,

  0.22278,
  0.19243,
  0.16681,
  0.1451,
  0.12663,

  0.11088,
  0.097381,
  0.085788,
  0.075795,
  0.067155,

  0.059663,
  0.053146,
  0.047463,
  0.042493,
  0.038134,

  0.034302,
  0.030925
};

float alpha_table[TABLE_MAX_IDX] =
{
  5.9,
  5.8,
  5.6,
  5.4,
  5.3,

  5.1,
  5.0,
  4.8,
  4.7,
  4.6,

  4.4,
  4.3,
  4.2,
  4.1,
  4.0,

  3.9,
  3.7,
  3.6,
  3.6,
  3.5,

  3.4,
  3.3,
  3.2,
  3.1,
  3.0,

  3.0,
  2.9,
  2.8,
  2.8,
  2.7,
  
  2.6,
  2.6,
  2.5,
  2.4,
  2.4,

  2.3,
  2.3,
  2.2,
  2.2,
  2.1,

  2.1,
  2.1
};
int RT_LUT(float RNTC){
    
    RNTC /= 1000;
    
    int idx = 0;
    while(RT_table[idx++] >= RNTC){
        
    }
    //printf("idx : %d , R value : %f , RNTC value : %f\r\n", idx, RT_table[idx-1], RNTC);
    
    return (-40 + (idx -1));
}


double TEMP_equation(float RNTC){

  float ratio;
  int idx;
  double TEMP;

  ratio = RNTC / 10000;
  for(idx= 0; idx < TABLE_MAX_IDX; idx++){

  	if( ratio > ratio_table[idx] ){
  		break;
  	}
    	else if( ratio == ratio_table[idx]){
     		break;
    	}
  }  

  if((idx == 0 && ratio > ratio_table[idx]) || idx >= TABLE_MAX_IDX ){
    return -100.0;
  }
  else if( ratio == ratio_table[idx]){
  
    TEMP = -55 + 5 * idx;
    //printf("TEMP : %lf'C\n", TEMP);
    
    return TEMP;  
  }

  int temp1 , temp2;
  temp1 = -55 +( 5 * (idx-1));
  temp2 = -55 +( 5 * (idx));
  
  float alpha;
  alpha = alpha_table[idx-1];
  //printf("TEMP between %d'C and %d'c\n", temp1, temp2 );
  //printf("uses alpha %f\n", alpha);


  double p, q;
  p = (temp1 + 273.15); 
  q = ((double)alpha / 100.0) * pow(p, 2.0) ;


  //printf("%lf\n", q); 

  double RT_RTX, log_value;
  RT_RTX = (double)RNTC / (ratio_table[idx-1]*10000);
  log_value = log(RT_RTX);

  //printf("%lf , %lf\n", RT_RTX, log_value);

  TEMP = q * ( p - 273.15 ) - (273.15 * p * log_value);
  TEMP = TEMP/ ( p * log_value + q );

  //printf("TEMP : %lf'C\n\n", TEMP);

  return TEMP;
}



int read_ADC() {
    
    ADCHS_ChannelConversionStart(ADCHS_CH3);
    while(!ADCHS_ChannelResultIsReady(ADCHS_CH3))
    {

    };
    ADC_LV_B1 = ADCHS_ChannelResultGet(ADCHS_CH3);
        
    ADCHS_ChannelConversionStart(ADCHS_CH4);
    while(!ADCHS_ChannelResultIsReady(ADCHS_CH4))
    {

    };
    ADC_LV_B2 = ADCHS_ChannelResultGet(ADCHS_CH4);

    //polling LVB3
    ADCHS_ChannelConversionStart(ADCHS_CH5);
    while(!ADCHS_ChannelResultIsReady(ADCHS_CH5))
    {

    };
    ADC_LV_B3 = ADCHS_ChannelResultGet(ADCHS_CH5);

    
    ADCHS_ChannelConversionStart(ADCHS_CH25);
    while(!ADCHS_ChannelResultIsReady(ADCHS_CH25))
    {
    };
    TEMP_MN = ADCHS_ChannelResultGet(ADCHS_CH25);

    ADCHS_ChannelConversionStart(ADCHS_CH26);
    while(!ADCHS_ChannelResultIsReady(ADCHS_CH26))
    {
    };
    TEMP_MP = ADCHS_ChannelResultGet(ADCHS_CH26);

    ADCHS_ChannelConversionStart(ADCHS_CH28);
    while(!ADCHS_ChannelResultIsReady(ADCHS_CH28))
    {
    };
    TEMP_BOARD = ADCHS_ChannelResultGet(ADCHS_CH28);
    
    
    ADCHS_ChannelConversionStart(ADCHS_CH1);
    while(!ADCHS_ChannelResultIsReady(ADCHS_CH1))
    {
    };
    ADC_3V3 = ADCHS_ChannelResultGet(ADCHS_CH1);

    ADCHS_ChannelConversionStart(ADCHS_CH2);
    while(!ADCHS_ChannelResultIsReady(ADCHS_CH2))
    {
    };
    ADC_5V = ADCHS_ChannelResultGet(ADCHS_CH2);

    INP_Voltage_B1 = ((float)ADC_LV_B1 * ADC_REF / ADC_MAX) * (R1 + R2) / R2; 
    INP_Voltage_B2 = ((float)ADC_LV_B2 * ADC_REF / ADC_MAX) * (R1 + R2) / R2; 
    INP_Voltage_B3 = ((float)ADC_LV_B3 * ADC_REF / ADC_MAX) * (R1 + R2) / R2;
    INP_Voltage_3V3 = ((float)ADC_3V3 * ADC_REF / ADC_MAX) * 2;
    INP_Voltage_5V = ((float)ADC_5V * ADC_REF / ADC_MAX) * 2;
    
    uint16_t LVB1, LVB2, LVB3;
    uint8_t V_3V3, V_5V;
    LVB1 = (uint16_t)(INP_Voltage_B1 * LVB_FACTOR);
    LVB2 = (uint16_t)(INP_Voltage_B2 * LVB_FACTOR);
    LVB3 = (uint16_t)(INP_Voltage_B3 * LVB_FACTOR);
    V_3V3 = (uint8_t)(INP_Voltage_3V3 * VOL_FACTOR);
    V_5V = (uint8_t)(INP_Voltage_5V * VOL_FACTOR);
  
    bit_conversion(LVB1, low_voltage.b_1, 2);
    bit_conversion(LVB2, low_voltage.b_2, 2);
    bit_conversion(LVB3, low_voltage.b_3, 2);
    ref_voltage.ref_3v = V_3V3;
    ref_voltage.ref_5v = V_5V;

#ifdef DEBUG
    printf("LVBAT +1: %d, +2: %d, +3: %d\r\n", LVB1, LVB2, LVB3);
    printf("3V3 : %d, 5V : %d \r\n", V_3V3, V_5V);
#endif
    
    float INP_Temp_MP, INP_Temp_MN, INP_Temp_BOARD;
    INP_Temp_MP = ((float)TEMP_MP * ADC_REF / ADC_MAX);
    INP_Temp_MN = ((float)TEMP_MN * ADC_REF / ADC_MAX);
    INP_Temp_BOARD = ((float)TEMP_BOARD * ADC_REF / ADC_MAX);
    float RNTC_MP, RNTC_MN, RNTC_BOARD;
   
    RNTC_MP = (10000 * INP_Temp_MP ) / (ADC_REF - INP_Temp_MP);
    RNTC_MN = (10000 * INP_Temp_MN ) / (ADC_REF - INP_Temp_MN);
    RNTC_BOARD = (10000 * INP_Temp_BOARD ) / (ADC_REF - INP_Temp_BOARD);
    
    double temp_mp, temp_mn;
    int temp_board;
    temp_mp = TEMP_equation(RNTC_MP);
    temp_mn = TEMP_equation(RNTC_MN);
    temp_board = RT_LUT(RNTC_BOARD);
    
    if(temp_mp == -100 || temp_mn == -100){
        //TODO
        //out of range temperature ( return value : -100 )
        //need to implement error correction code
    }
    
    temp.temperature1 = (int)temp_mp + 40;
    temp.temperature2 = (int)temp_mn + 40;
    temp.temperature3 = temp_board + 40;
#ifdef DEBUG
    printf("RNTC_MP : %d , RNTC_MN : %d \r\n", (int)RNTC_MP, (int)RNTC_MN);
    printf("TEMP_MP : %d , TEMP_MN : %d \r\n\r\n", (int)temp_mp, (int)temp_mn);
#endif
    
    return 0;
}


/* *****************************************************************************
 End of File
 */
