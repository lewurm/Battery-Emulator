#ifndef _DATALAYER_CANLOGGING_H_
#define _DATALAYER_CANLOGGING_H_

#include "../include.h"

typedef struct {
    uint16_t buffer = 0;
} DATALAYER_CANLOGGING_BUFFER;


class DataLayerExtended {
 public:
  DATALAYER_CANLOGGING_BUFFER canlogger;
};

extern DataLayerCanlogging datalayer_canlogging;

#endif
