#define UTILS_H
#include "contiki.h"
#include <stdlib.h>

#define SIZE_CHILDREN 10

typedef struct child {
  linkaddr_t addr;
} child_t;

typedef struct parent {
  linkaddr_t addr;
  uint8_t id;
  int16_t RSSI;
} parent_t;

typedef struct child_array {
  child_t** children;
  int size;
  int free_spot;
} children_t;
