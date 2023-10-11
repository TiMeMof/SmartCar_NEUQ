#ifndef __QIN_FLASH_H__
#define __QIN_FLASH_H__
#include "zf_common_headfile.h"

typedef struct {
    uint32 integer;
    uint32 decimal;
}my_double_s;

uint8 read_write_gps_flash(int page);

uint8 write_doubles_in(double *p,int num, int which_page);
double read_one_double_out(int begin, int which_page);
double* read_doubles_out(int begin, int which_page);

void read_gps_out(int which_page);
extern uint8 flash_state;
extern uint32 read_write_flash_flag;

extern double latitude_flash[30];
extern double longitude_flash[30];
extern int only_read_once,only_write_once;
#endif
