/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _ADC_SENSOR_H    /* Guard against multiple inclusion */
#define _ADC_SENSOR_H

#include "structure.h"

#ifdef __cplusplus
extern "C" {
#endif

#define R1 100
#define R2 18
#define ADC_REF 3.3f
#define ADC_MAX 4095
#define LVB_FACTOR 1000
#define VOL_FACTOR 10
#define TEMP_OFFSET 40
#define TABLE_MAX_IDX 42
#define RT_TABLE_MAX_IDX 241
    

extern TEMPERATURE temp;
extern LOW_VOLTAGE low_voltage;
extern REF_VOLTAGE ref_voltage;
    
int read_ADC();
double TEMP_equation(float);

#endif