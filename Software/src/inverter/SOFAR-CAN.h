#ifndef SOFAR_CAN_H
#define SOFAR_CAN_H
#include "../include.h"

#define CAN_INVERTER_SELECTED

void transmit_can(CAN_frame_t* tx_frame, int interface);

#endif
