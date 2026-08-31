#include "logger.h"

FILE* logger_init(const char *filename, const char *header) {
    FILE *file = fopen(filename, "w");
    if (file == NULL) {
        return NULL;
    }
    fprintf(file, "%s\n", header);
    fflush(file);
    return file;
}

void logger_write_row(FILE *file, float time, float roll, float pitch, float yaw, 
                      float ax, float ay, float az, float gx, float gy, float gz) {
    if (file != NULL) {
        fprintf(file, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                time, roll, pitch, yaw, ax, ay, az, gx, gy, gz);
        fflush(file);
    }
}

void logger_close(FILE *file) {
    if (file != NULL) {
        fclose(file);
    }
}