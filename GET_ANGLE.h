#ifndef __GET_ANGLE_H__
#define __GET_ANGLE_H__

#include "zf_common_headfile.h"

#define    Gyro_R    0.0010653f    //���ݳ��Ըò���

typedef enum{
    IMU963RA,
    IMU660RA,
    ICM20948
}IMU_MODE_SELECT;

typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}Butter_BufferData;

typedef struct
{
  float a[3];
  float b[3];
}Butter_Parameter;

typedef struct {
    float acce_x;
    float acce_y;
    float acce_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
}IMU_Offset_s;

typedef struct {
    float gx_a,gy_a,gz_a;  // �����ǲ���ֵ����λ��angle/s��
    float gx_r,gy_r,gz_r;  // �����ǲ���ֵ����λ��rad/s��
    float gx_r_old,gy_r_old,gz_r_old;
    float ax, ay, az;      // ���ٶȼƲ���ֵ����λ��m/s^2��
    float ax_old, ay_old, az_old;
    float mx, my, mz;      // �شżƲ���ֵ����λ��uT��
    float mx_f,my_f,mz_f;  // �شż�����ֵ
    IMU_Offset_s offset;    // �����
} imu_data_s;

//  roll          ����ǣ�Χ��x��ת��������λ�ǡ�
//  pitch         �����ǣ�Χ��y��ת��������λ�ǡ�
//  yaw           ƫ���ǣ�Χ��z��ת��������λ�ǡ�
typedef struct {
    float roll_a;      //�Ƕ���
    float roll_a_old;
    float pitch_a;     //�Ƕ���
    float pitch_a_old;
    float yaw_a;       //�Ƕ���
    float yaw_a_old;
    float yaw_s;     //�Ƕ���
    float roll_r;      //������
    float pitch_r;     //������
    float yaw_r;       //������

}EulerAngle_s;

typedef struct {
    imu_data_s data;
    EulerAngle_s angle;
}imu_Data_All_s;

//��Ԫ���ṹ��
typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
    float q0_old;
    float q1_old;
    float q2_old;
    float q3_old;
} Quaternion;


//IMU��������ܽṹ��
extern imu_Data_All_s IMU;
extern Quaternion q;

void IMU_Offset_Init(IMU_MODE_SELECT IMU_MODE);

void Madgwick_9_DOF_Get_Angle(IMU_MODE_SELECT IMU_MODE,uint8 delta_t);

void Madgwick_AHRS_6_DOF_Get_Angle(double beta, uint8 betaIn_is_true, IMU_MODE_SELECT IMU_MODE, uint8 delta_t);
extern double BETA;

void AHRS_6_DOF_Get_Angle(IMU_MODE_SELECT IMU_MODE,float delta_T);
#endif
