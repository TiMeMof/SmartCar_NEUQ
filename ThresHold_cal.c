#include "ThresHold_cal.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     自适应阈值（计算量很大）
// 参数说明     img             传入的图像数组
// 参数说明     fixed_img       传出的图像数组
// 参数说明     M_H             传入图像的高
// 参数说明     M_W             传入图像的宽
// 返回参数     void
// 使用示例     adaptiveThreshold_f(mt9v03x_image[0],image[0], MT9V03X_H, MT9V03X_W);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void adaptiveThreshold(uint8 *img, uint8 *fixed_img,int M_H,int M_W){

    int i,j;
    //局部阈值
    int threadhold;
    //核大小
    int block_size = 9;

    //核大小的一半
    int half = block_size / 2;

    //每次在核上的横、竖步长
    int delta_h,delta_w;


    int fix_num = 10;
    for(i=1;i<M_W-1;i++){
       for(j=1;j<M_H-1;j++){
           threadhold = 0;
           //求区域阈值
           for (delta_w = -half; delta_w <= half; delta_w++) {
               for (delta_h = -half; delta_h <= half; delta_h++) {
                   threadhold += *(img + (i+delta_h) + (j+delta_w)* M_W);
               }
           }

           threadhold /= block_size*block_size;
           threadhold -= fix_num;


           if(*(img+j*M_W+i)>threadhold){
               *(fixed_img+j*M_W+i) = 255;
           }
           else{
               *(fixed_img+j*M_W+i) = 0;
           }

       }
   }


}

int MAX_F (int a , int b)/*自定义函数*/
{
    if(a>b)
    {
        return a;
    }
    else
    {
        return b;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     大津法阈值（计算量一般）
// 参数说明     img             传入的图像数组
// 参数说明     fixed_img       传出的图像数组
// 参数说明     M_H             传入图像的高
// 参数说明     M_W             传入图像的宽
// 返回参数     void
// 使用示例     OTSU(mt9v03x_image[0],image[0], MT9V03X_H, MT9V03X_W);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void OTSU(uint8 *img, uint8 *fixed_img,int M_H,int M_W){

    uint16 gray[256]={0};

    //得到灰度图
    for(int i=0;i<M_H*M_W;i++){
        gray[*(img+i)]++;
    }

    int Sum_pixels;
        Sum_pixels = M_W * M_H;   //总像素点

//    //峰1
//    int H1;
//    for(int j=0;j<254;j++){
//        if(*(gray+j+1) >= *(gray+j)){
//            H1 = *(gray+j+1);
//        }
//    }
//
//    //峰2
//    int H2_1,H2_2,H2;
//    for(int k=H1;k<254;k++){
//        if(*(gray+k+1) >= *(gray+k)){
//            H2_1 = *(gray+k+1);
//        }
//    }
//    for(int k=H1;k>0;k--){
//        if(*(gray+k-1) >= *(gray+k)){
//            H2_2 = *(gray+k-1);
//        }
//    }

//    H2 = MAX_F(H2_1,H2_2);

    //阈值
    int threshold;
    int i,j;
    float pixelPro[256];
    float u;//图像平均总灰度

    //计算灰度分布，以及平均灰度
    for (int k = 0; k < 256; ++k)
    {
     pixelPro[k] = (float)gray[k] / Sum_pixels;
     u += k * pixelPro[k];
    }


    //计算合理阈值
    float maxVariance = 0.0;  //最大类间方差
    float w0 = 0, u0  = 0;  //w0 前景比例，u0 前景平均灰度
    //后景比例w1 = 1 - w0
    float variance;
    for (int k = 0; k < 256; k++) {
        w0 += pixelPro[k];  //假设当前灰度i为阈值, 0~i 灰度像素所占整幅图像的比例即前景比例
        u0  += k * pixelPro[k];
        variance = pow((1+w0/(1-w0))*u0 - u/(1-w0), 2) * pow(w0,0.8) * pow(1 - w0,0.8);    //类间方差
        if(variance > maxVariance)
        {
            maxVariance = variance;
            threshold = k;
        }
    }

    for(i=0;i<M_W;i++){
        for(j=0;j<M_H;j++){
            if(*(img+j*M_W+i)>threshold){
               *(fixed_img+j*M_W+i) = 255;
            }
            else{
               *(fixed_img+j*M_W+i) = 0;
            }

        }
    }

}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     sobel边缘检测（计算量一般）
// 参数说明     img             传入的图像数组
// 参数说明     fixed_img       传出的图像数组
// 参数说明     M_H             传入图像的高
// 参数说明     M_W             传入图像的宽
// 参数说明     core            x还是y方向的边缘检测
// 返回参数     void
// 使用示例     Sobel_cal_threshold(mt9v03x_image[0],image[0], MT9V03X_H, MT9V03X_W, X_way);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 Sobel_cal_threshold(uint8 *img, uint8 *fixed_img,int M_H,int M_W, SobelDirection core){
    if(X_way == core){
        int core_x[3][3];
        core_x[0][0] =-1; core_x[1][0] =-2; core_x[2][0] =-1; core_x[0][2] = 1; core_x[1][2] = 2; core_x[2][2] = 1;

        for(int h=0;h<M_H;h++){
            for(int w=0;w<M_W;w++){
                if(h==0||w==0){
                    *(fixed_img + h*M_W + w) = 0;
                }
                else{
                    *(fixed_img + h*M_W + w) = *(img + (h-1)*M_W + w-1)*core_x[0][0] + *(img + (h)*M_W + w-1)  *core_x[1][0] +
                                               *(img + (h+1)*M_W + w-1)*core_x[2][0] +*(img + (h-1)*M_W + w+1)*core_x[0][2] +
                                               *(img + (h)*M_W + w+1)  *core_x[1][2] + *(img + (h+1)*M_W + w+1)*core_x[2][2];
                }
            }
        }

    }
    else if(Y_way == core){
        int core_y[3][3];
        core_y[0][0] =-1; core_y[0][1] =-2; core_y[0][2] =-1; core_y[2][0] = 1; core_y[2][1] = 2; core_y[2][2] = 1;

        for(int h=0;h<M_H;h++){
            for(int w=0;w<M_W;w++){
                if(h==0||w==0){
                    *(fixed_img + h*M_W + w) = 0;
                }
                else{
                    *(fixed_img + h*M_W + w) = *(img + (h-1)*M_W + w-1)*core_y[0][0] + *(img + (h-1)*M_W + w)  *core_y[0][1] +
                                               *(img + (h-1)*M_W + w+1)*core_y[0][2] + *(img + (h+1)*M_W + w-1)*core_y[2][0] +
                                               *(img + (h+1)*M_W + w)  *core_y[2][1] + *(img + (h+1)*M_W + w+1)*core_y[2][2];
                }
            }
        }
    }
    else if(All_way == core){
        int core_x[3][3];
        core_x[0][0] =-1; core_x[1][0] =-2; core_x[2][0] =-1; core_x[0][2] = 1; core_x[1][2] = 2; core_x[2][2] = 1;
        int core_y[3][3];
        core_y[0][0] =-1; core_y[0][1] =-2; core_y[0][2] =-1; core_y[2][0] = 1; core_y[2][1] = 2; core_y[2][2] = 1;

        for(int h=0;h<M_H;h++){
            for(int w=0;w<M_W;w++){
                if(h==0||w==0){
                    *(fixed_img + h*M_W + w) = 0;
                }
                else{
                    int a  =  *(img + (h-1)*M_W + w-1)*core_x[0][0] + *(img + (h)*M_W + w-1)  *core_x[1][0] +
                              *(img + (h+1)*M_W + w-1)*core_x[2][0] + *(img + (h-1)*M_W + w+1)*core_x[0][2] +
                              *(img + (h)*M_W + w+1)  *core_x[1][2] + *(img + (h+1)*M_W + w+1)*core_x[2][2] +
                              *(img + (h-1)*M_W + w-1)*core_y[0][0] + *(img + (h-1)*M_W + w)  *core_y[0][1] +
                              *(img + (h-1)*M_W + w+1)*core_y[0][2] + *(img + (h+1)*M_W + w-1)*core_y[2][0] +
                              *(img + (h+1)*M_W + w)  *core_y[2][1] + *(img + (h+1)*M_W + w+1)*core_y[2][2];

                    if(a<=200)
                        *(fixed_img + h*M_W + w) = (uint8)a;
                    else if(a<2000)
                        *(fixed_img + h*M_W + w) = (uint8)(0.030555*(a-200))+200;
                    else
                        *(fixed_img + h*M_W + w) = 255;
                }
            }
        }
    }
    return 0;

}
