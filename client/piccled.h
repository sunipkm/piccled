/**
 * @file piccled.h
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief 
 * @version 0.0.1
 * @date 2025-06-12
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef PICCLED_H
#define PICCLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

int piccled_open(const char *port);
int piccled_set(int fd, uint8_t value);

#ifdef __cplusplus
}
#endif
#endif