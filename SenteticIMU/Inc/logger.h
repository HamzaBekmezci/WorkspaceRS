#ifndef LOGGER_H
#define LOGGER_H

#include "stdio.h"

FILE* logger_init(const char *filename, const char *header);
void logger_write_row(FILE *file, float time, float roll, float pitch, float yaw, 
                      float ax, float ay, float az, float gx, float gy, float gz);
void logger_close(FILE *file);

#endif // LOGGER_H