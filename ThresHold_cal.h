#ifndef __THRESHOLD_CAL_H__
#define __THRESHOLD_CAL_H__

#include "zf_common_headfile.h"




typedef enum{

    X_way,
    Y_way,
    All_way

}SobelDirection;

void adaptiveThreshold      (uint8 *img, uint8 *fixed_img, int M_H, int M_W);

void OTSU                   (uint8 *img, uint8 *fixed_img, int M_H, int M_W);

uint8 Sobel_cal_threshold    (uint8 *img, uint8 *fixed_img, int M_H, int M_W, SobelDirection core);

#endif
