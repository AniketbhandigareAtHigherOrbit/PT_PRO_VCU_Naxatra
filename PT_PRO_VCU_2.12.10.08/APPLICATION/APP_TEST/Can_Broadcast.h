#ifndef CAN_BROADCAST_H
#define CAN_BROADCAST_H

#include <stdint.h>
#include "AppTest.h"
#include "pal_can_conf.h"

void Broadcast_Init(void);
void Broadcast_Task_100ms(void);

#endif /* CAN_BROADCAST_H */