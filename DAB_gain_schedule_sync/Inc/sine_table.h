#ifndef __SINE_TABLE_H
#define __SINE_TABLE_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "main.h"
   
#define TABLE_LEN 120
   
extern short DtTable [200];
extern short sineTable [TABLE_LEN];
extern short K11Table  [TABLE_LEN];
extern short K22Table  [TABLE_LEN];
// synchronization
extern float currScale;
extern uint8_t sync;

#ifdef __cplusplus
}
#endif
#endif /*__ sine_table_H */