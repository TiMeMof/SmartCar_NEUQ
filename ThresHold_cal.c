#include "ThresHold_cal.h"

//-------------------------------------------------------------------------------------------------------------------
// �������     ����Ӧ��ֵ���������ܴ�
// ����˵��     img             �����ͼ������
// ����˵��     fixed_img       ������ͼ������
// ����˵��     M_H             ����ͼ��ĸ�
// ����˵��     M_W             ����ͼ��Ŀ�
// ���ز���     void
// ʹ��ʾ��     adaptiveThreshold_f(mt9v03x_image[0],image[0], MT9V03X_H, MT9V03X_W);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void adaptiveThreshold(uint8 *img, uint8 *fixed_img,int M_H,int M_W){

    int i,j;
    //�ֲ���ֵ
    int threadhold;
    //�˴�С
    int block_size = 9;

    //�˴�С��һ��
    int half = block_size / 2;

    //ÿ���ں��ϵĺᡢ������
    int delta_h,delta_w;


    int fix_num = 10;
    for(i=1;i<M_W-1;i++){
       for(j=1;j<M_H-1;j++){
           threadhold = 0;
           //��������ֵ
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

int MAX_F (int a , int b)/*�Զ��庯��*/
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
// �������     �����ֵ��������һ�㣩
// ����˵��     img             �����ͼ������
// ����˵��     fixed_img       ������ͼ������
// ����˵��     M_H             ����ͼ��ĸ�
// ����˵��     M_W             ����ͼ��Ŀ�
// ���ز���     void
// ʹ��ʾ��     OTSU(mt9v03x_image[0],image[0], MT9V03X_H, MT9V03X_W);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void OTSU(uint8 *img, uint8 *fixed_img,int M_H,int M_W){

    uint16 gray[256]={0};

    //�õ��Ҷ�ͼ
    for(int i=0;i<M_H*M_W;i++){
        gray[*(img+i)]++;
    }

    int Sum_pixels;
        Sum_pixels = M_W * M_H;   //�����ص�

//    //��1
//    int H1;
//    for(int j=0;j<254;j++){
//        if(*(gray+j+1) >= *(gray+j)){
//            H1 = *(gray+j+1);
//        }
//    }
//
//    //��2
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

    //��ֵ
    int threshold;
    int i,j;
    float pixelPro[256];
    float u;//ͼ��ƽ���ܻҶ�

    //����Ҷȷֲ����Լ�ƽ���Ҷ�
    for (int k = 0; k < 256; ++k)
    {
     pixelPro[k] = (float)gray[k] / Sum_pixels;
     u += k * pixelPro[k];
    }


    //���������ֵ
    float maxVariance = 0.0;  //�����䷽��
    float w0 = 0, u0  = 0;  //w0 ǰ��������u0 ǰ��ƽ���Ҷ�
    //�󾰱���w1 = 1 - w0
    float variance;
    for (int k = 0; k < 256; k++) {
        w0 += pixelPro[k];  //���赱ǰ�Ҷ�iΪ��ֵ, 0~i �Ҷ�������ռ����ͼ��ı�����ǰ������
        u0  += k * pixelPro[k];
        variance = pow((1+w0/(1-w0))*u0 - u/(1-w0), 2) * pow(w0,0.8) * pow(1 - w0,0.8);    //��䷽��
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
// �������     sobel��Ե��⣨������һ�㣩
// ����˵��     img             �����ͼ������
// ����˵��     fixed_img       ������ͼ������
// ����˵��     M_H             ����ͼ��ĸ�
// ����˵��     M_W             ����ͼ��Ŀ�
// ����˵��     core            x����y����ı�Ե���
// ���ز���     void
// ʹ��ʾ��     Sobel_cal_threshold(mt9v03x_image[0],image[0], MT9V03X_H, MT9V03X_W, X_way);
// ��ע��Ϣ
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
