// Wokwi Custom Chip - For docs and examples see:
// https://docs.wokwi.com/chips-api/getting-started
//
// SPDX-License-Identifier: MIT
// Copyright 2023 Ailou

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>

typedef struct {
  pin_t pin_out;
  uint8_t pin_state;
  uint32_t flow_rate_attr;
  timer_t timer_id;
} chip_state_t;


void chip_timer_callback(void *user_data) {
  chip_state_t *data = (chip_state_t*)user_data;

  float flowRate = attr_read_float(data->flow_rate_attr);

  float frequency = ((flowRate * 1000)/60.0f)/2.25f;
  frequency = frequency * 2;  // Both a HIGH and LOW per pulse
  uint32_t interval = 1000000*(1.0f/frequency);

  // printf("Interval: %d\n", interval);

  pin_write(data->pin_out, data->pin_state);
  data->pin_state = !data->pin_state;

  timer_start(data->timer_id, interval, false);
}


void chip_init() {
  setvbuf(stdout, NULL, _IOLBF, 1024); // Limit output buffering to a single line
  printf("\nchip_init() begin\n");

  chip_state_t *chip = malloc(sizeof(chip_state_t));

  chip->flow_rate_attr = attr_init_float("flowRate", 5.5);
  
  chip->pin_out = pin_init("OUT", OUTPUT);
  chip->pin_state = LOW;

  const timer_config_t config = {
    .callback = chip_timer_callback,
    .user_data = chip,
  };
  chip->timer_id = timer_init(&config);
  timer_start(chip->timer_id, 1, false);
}














