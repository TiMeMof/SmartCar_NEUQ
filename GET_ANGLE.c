#include "GET_ANGLE.h"
#include <math.h>
void Euler_ANGLE_To_REG(imu_Data_All_s *imu_in);
void Euler_REG_To_ANGLE(void);
void imu963ra_get_value(void);
void imu660ra_get_value(void);
/***********************************************************/
//巴特沃夫滤波算法
Butter_BufferData gyro_filter_buf[3],accel_filter_buf[3];
Butter_Parameter Accel_Parameter,Gyro_Parameter;
//IMU相关数据总结构体
imu_Data_All_s IMU;
/***********************************************************/
typedef struct {
    double a0, a1, a2; // Coefficients of numerator
    double b0, b1, b2; // Coefficients of denominator
    double x1, x2;     // Input delay elements
    double y1, y2;     // Output delay elements
} Filter;
Filter filter;
// Initialize filter coefficients
void initFilter(Filter* filter, double fs, double f0, double Q) {
    double w0 = 2 * PI * f0 / fs;
    double alpha = sin(w0) / (2 * Q);

    filter->b0 = Q * alpha;
    filter->b1 = 0;
    filter->b2 = -Q * alpha;
    filter->a0 = 1 + alpha;
    filter->a1 = -2 * cos(w0);
    filter->a2 = 1 - alpha;
    filter->x1 = filter->x2 = filter->y1 = filter->y2 = 0;
}
// Apply the filter to a sample
double applyFilter(Filter* filter, double input) {
    double output = (filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2
                    - filter->a1 * filter->y1 - filter->a2 * filter->y2) / filter->a0;

    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;

    return output;
}

//巴特沃夫滤波器初始化
void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent,Butter_Parameter *LPF)
{
  float fr = sample_frequent / cutoff_frequent;
  float ohm = tanf(M_PI / fr);
  float c = 1.0f + 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm;
  if (cutoff_frequent <= 0.0f) {
    // no filtering
    return;
  }
  LPF->b[0] = ohm * ohm / c;
  LPF->b[1] = 2.0f * LPF->b[0];
  LPF->b[2] = LPF->b[0];
  LPF->a[0]=1.0f;
  LPF->a[1] = 2.0f * (ohm * ohm - 1.0f) / c;
  LPF->a[2] = (1.0f - 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm) / c;
}

//二阶低通巴特沃夫滤波器
float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
    if(Buffer->Output_Butter[0]==0&&
         Buffer->Output_Butter[1]==0&&
         Buffer->Output_Butter[2]==0&&
         Buffer->Input_Butter[0]==0&&
         Buffer->Input_Butter[1]==0&&
         Buffer->Input_Butter[2]==0)
    {
        Buffer->Output_Butter[0]=curr_input;
        Buffer->Output_Butter[1]=curr_input;
        Buffer->Output_Butter[2]=curr_input;
        Buffer->Input_Butter[0]=curr_input;
        Buffer->Input_Butter[1]=curr_input;
        Buffer->Input_Butter[2]=curr_input;
        return curr_input;
    }

  /* 加速度计Butterworth滤波 */
  /* 获取最新x(n) */
  Buffer->Input_Butter[2]=curr_input;
  /* Butterworth滤波 */
  Buffer->Output_Butter[2]=Parameter->b[0] * Buffer->Input_Butter[2]
                                                    +Parameter->b[1] * Buffer->Input_Butter[1]
                                                    +Parameter->b[2] * Buffer->Input_Butter[0]
                                                    -Parameter->a[1] * Buffer->Output_Butter[1]
                                                    -Parameter->a[2] * Buffer->Output_Butter[0];
  /* x(n) 序列保存 */
  Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
  Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
  /* y(n) 序列保存 */
  Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
  Buffer->Output_Butter[1]=Buffer->Output_Butter[2];

    for(uint16_t i=0;i<3;i++)
    {
      if(isnan(Buffer->Output_Butter[i])==1
            ||isnan(Buffer->Input_Butter[i])==1)
            {
                Buffer->Output_Butter[0]=curr_input;
                Buffer->Output_Butter[1]=curr_input;
                Buffer->Output_Butter[2]=curr_input;
                Buffer->Input_Butter[0]=curr_input;
                Buffer->Input_Butter[1]=curr_input;
                Buffer->Input_Butter[2]=curr_input;
                return curr_input;
            }
    }
  return Buffer->Output_Butter[2];
}
/***********************************************************/
/*
 * ********************** 九轴IMU解算 **********************
  ___    ____   ___  _____   ___ __  __ _   _
 / _ \  |  _ \ / _ \|  ___| |_ _|  \/  | | | |
| (_) | | | | | | | | |_     | || |\/| | | | |
 \__, | | |_| | |_| |  _|    | || |  | | |_| |
   /_/  |____/ \___/|_|     |___|_|  |_|\___/

 * ********************** 九轴IMU解算 **********************
 */
#define Kp 17.0f        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f       // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.005f   // half the sample period采样周期的一半

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;   // scaled integral error

void IMU_Offset_Init(IMU_MODE_SELECT IMU_MODE)
{
    Set_Cutoff_Frequency(200,35,&Accel_Parameter);
    Set_Cutoff_Frequency(200,7.5,&Gyro_Parameter);

//    double fs = 200;  // Sample rate (Hz)
//    double f0 = 40;   // Center frequency (Hz)
//    double Q = .707;    // Quality factor
//    initFilter(&filter, fs, f0, Q);

    float a_x,a_y,a_z,gx_a,gy_a,gz_a;
    float gx_a_t,gy_a_t,gz_a_t;
    int n=200;

    if(IMU_MODE == IMU963RA){
        for(int i=0;i<n;i++){
            imu963ra_get_acc();
            imu963ra_get_gyro();
            a_x += LPButterworth(imu963ra_acc_transition(imu963ra_acc_x),&accel_filter_buf[0],&Accel_Parameter);
            a_y += LPButterworth(imu963ra_acc_transition(imu963ra_acc_y),&accel_filter_buf[1],&Accel_Parameter);
            a_z += LPButterworth(imu963ra_acc_transition(imu963ra_acc_z),&accel_filter_buf[2],&Accel_Parameter);


            gx_a_t= imu963ra_gyro_transition(imu963ra_gyro_x);
            gx_a_t/=100.;gx_a_t*=100.;
            gx_a_t =LPButterworth(gx_a_t,&gyro_filter_buf[0],&Gyro_Parameter);
            gx_a+=gx_a_t;

            gy_a_t= imu963ra_gyro_transition(imu963ra_gyro_y);
            gy_a_t/=100.;gy_a_t*=100.;
            gy_a_t= LPButterworth(gy_a_t,&gyro_filter_buf[1],&Gyro_Parameter);
            gy_a+=gy_a_t;

            gz_a_t= imu963ra_gyro_transition(imu963ra_gyro_z);
            gz_a_t/=100.;gz_a_t*=100.;
            gz_a_t= LPButterworth(gz_a_t,&gyro_filter_buf[2],&Gyro_Parameter);
            gz_a+=gz_a_t;

            system_delay_ms(5);
        }
    }
    else{
        for(int i=0;i<n;i++){
            imu660ra_get_acc();
            imu660ra_get_gyro();
            a_x += LPButterworth(imu660ra_acc_transition(imu660ra_acc_x),&accel_filter_buf[0],&Accel_Parameter);
            a_y += LPButterworth(imu660ra_acc_transition(imu660ra_acc_y),&accel_filter_buf[1],&Accel_Parameter);
            a_z += LPButterworth(imu660ra_acc_transition(imu660ra_acc_z),&accel_filter_buf[2],&Accel_Parameter);

            gx_a_t= imu660ra_gyro_transition(imu660ra_gyro_x);
            gx_a_t/=100.;gx_a_t*=100.;
            gx_a_t =LPButterworth(gx_a_t,&gyro_filter_buf[0],&Gyro_Parameter);
            gx_a+=gx_a_t;

            gy_a_t= imu660ra_gyro_transition(imu660ra_gyro_y);
            gy_a_t/=100.;gy_a_t*=100.;
            gy_a_t= LPButterworth(gy_a_t,&gyro_filter_buf[1],&Gyro_Parameter);
            gy_a+=gy_a_t;

            gz_a_t= imu660ra_gyro_transition(imu660ra_gyro_z);
            gz_a_t/=100.;gz_a_t*=100.;
            gz_a_t= LPButterworth(gz_a_t,&gyro_filter_buf[2],&Gyro_Parameter);
            gz_a+=gz_a_t;
            system_delay_ms(5);
        }
    }
    IMU.data.offset.acce_x = a_x/n;
    IMU.data.offset.acce_y = a_y/n;
    IMU.data.offset.acce_z = a_z/n;

    IMU.data.offset.gyro_x = gx_a/n;
    IMU.data.offset.gyro_y = gy_a/n;
    IMU.data.offset.gyro_z = gz_a/n;
}


//-------------------------------------------------------------------------------------------------------------------
//  函数简介      得到基于地球坐标系三个角度函数
//  参数说明      IMU_MODE      选择对应的IMU
//  参数说明      delta_t       delta_t是采样频率，单位是ms
//-------------------------------------------------------------------------------------------------------------------
void Madgwick_9_DOF_Get_Angle(IMU_MODE_SELECT IMU_MODE,uint8 delta_t)
{

    //将delta_t换算成s为单位
    delta_t = delta_t/1000.0f;

    //***********************************以下是获取原始数据***********************************

    if(IMU_MODE == IMU963RA){
        imu963ra_get_value();
    }
    else if(IMU_MODE == IMU660RA){
        imu660ra_get_value();
    }

    float mx = IMU.data.mx_f;
    float my = IMU.data.my_f;
    float mz = IMU.data.mz_f;

    float ax = IMU.data.ax;
    float ay = IMU.data.ay;
    float az = IMU.data.az;

    float gx = IMU.data.gx_a*Gyro_R;
    float gy = IMU.data.gy_a*Gyro_R;
    float gz = IMU.data.gz_a*Gyro_R;

    //原文链接：http://t.csdn.cn/mq5wp
    //椭圆校正原文链接：http://t.csdn.cn/FQ4nZ
    float norm;

    float hx, hy, hz, bx, bz;

    float wx, wy, wz;

    float vx, vy, vz;

    float ex, ey, ez;

    // 先把这些用得到的值算好
    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;

    norm = sqrtf(ax*ax + ay*ay + az*az);       //acc数据归一化
    static float norm_0;
    if(norm!=0)
    norm_0=norm;
    else
    norm=norm_0;


    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;
    norm = sqrtf(mx*mx + my*my + mz*mz);       //mag数据归一化
    static float norm_1;
    if(norm!=0)
    norm_1=norm;
    else
    norm=norm_1;


    mx = mx / norm;
    my = my / norm;
    mz = mz / norm;
    //  mx = 0;
    //  my = 0;
    //  mz = 0;

    hx = 2 * mx * (0.5 - q2q2 - q3q3) + 2 * my * (q1q2 - q0q3) + 2 * mz * (q1q3 + q0q2);
    hy = 2 * mx * (q1q2 + q0q3) + 2 * my * (0.5 - q1q1 - q3q3) + 2 * mz * (q2q3 - q0q1);
    hz = 2 * mx * (q1q3 - q0q2) + 2 * my * (q2q3 + q0q1) + 2 * mz * (0.5 - q1q1 -q2q2);
    bx = sqrtf((hx*hx) + (hy*hy));
    bz = hz;

    // estimated direction of gravity and flux (v and w)  估计重力方向和流量/变迁
    vx = 2*(q1q3 - q0q2);                                     //四元素中xyz的表示
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3 ;

    wx = 2 * bx * (0.5 - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2);
    wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3);
    wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5 - q1q1 - q2q2);

    // error is sum of cross product between reference direction of fields and direction measured by sensors
    //  ex = (ay*vz - az*vy) ;                                   //向量外积在相减得到差分就是误差
    //  ey = (az*vx - ax*vz) ;
    //  ez = (ax*vy - ay*vx) ;

    ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

    exInt = exInt + ex * Ki;         //对误差进行积分
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    // adjusted gyroscope measurements
    gx = gx + Kp*ex + exInt;        //将误差PI后补偿到陀螺仪，即补偿零点漂移
    gy = gy + Kp*ey + eyInt;
    gz = gz + Kp*ez + ezInt;        //这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减

    // integrate quaternion rate and normalise     //四元素的分方程
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    // normalise quaternion
    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;

    IMU.angle.yaw_r     = atan2f(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1); // yaw
    IMU.angle.pitch_r   = asinf(-2 * q1 * q3 + 2 * q0* q2); // pitch
    IMU.angle.roll_r    = atan2f(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1); // roll
    Euler_REG_To_ANGLE();

    static float yaw_last=0;
    if (isnan(IMU.angle.yaw_a)) {
        IMU.angle.yaw_a=yaw_last;
    } else {
        yaw_last=IMU.angle.yaw_a;// 结果不为 NaN，进行后续计算
    }

    while(IMU.angle.yaw_a < 0){
        IMU.angle.yaw_a += 360.f;
    }
    while(IMU.angle.yaw_a > 360.f){
        IMU.angle.yaw_a-=360.f;
    }
    ///////////////////////////////////////////////////////////////////////////
    IMU.angle.yaw_s=IMU.angle.yaw_a;
    IMU.angle.yaw_s-=test_data[2][3];
    while(IMU.angle.yaw_s <0){
        IMU.angle.yaw_s+=360.f;
    }
    while(IMU.angle.yaw_s > 360.f){
        IMU.angle.yaw_s-=360.f;
    }
    ///////////////////////////////////////////////////////////////////////////
    //以下更改是为了将yaw轴与gps角度统一处理，只在车的前方的点才有效

    IMU.angle.yaw_a-=test_data[2][2];
    while(IMU.angle.yaw_a > 180.f){
        IMU.angle.yaw_a-=360.f;
    }
}


/*
 * ********************** 六轴IMU解算 **********************
  __     ____   ___  _____   ___ __  __ _   _
 / /_   |  _ \ / _ \|  ___| |_ _|  \/  | | | |
| '_ \  | | | | | | | |_     | || |\/| | | | |
| (_) | | |_| | |_| |  _|    | || |  | | |_| |
 \___/  |____/ \___/|_|     |___|_|  |_|\___/

 * ********************** 六轴IMU解算 **********************
 */
/*
利用梯度下降滤波算法，对六轴imu数据进行滤波
大概步骤：由加速度计计算出一组姿态四元数q_delta，再和陀螺仪计算出来的姿态四元数q_omiga进行融合
*/



//四元数归一化
void QuaternionNormalize(Quaternion *q)
{
    float norm = sqrtf(q->q0 * q->q0 + q->q1 * q->q1 + q->q2 * q->q2 + q->q3 * q->q3);
    q->q0 /= norm;
    q->q1 /= norm;
    q->q2 /= norm;
    q->q3 /= norm;
}

//四元数转换为欧拉角
void QuaternionToEuler(Quaternion *q, float *roll, float *pitch, float *yaw)
{
    *roll = atan2f(2 * (q->q0 * q->q1 + q->q2 * q->q3), 1 - 2 * (q->q1 * q->q1 + q->q2 * q->q2));
    *pitch = asinf(2 * (q->q0 * q->q2 - q->q3 * q->q1));
    *yaw = atan2f(2 * (q->q0 * q->q3 + q->q1 * q->q2), 1 - 2 * (q->q2 * q->q2 + q->q3 * q->q3));
}

void Euler_ANGLE_To_REG(imu_Data_All_s *imu_in){
    imu_in->angle.pitch_r   =   imu_in->angle.pitch_a*PI/180.;
    imu_in->angle.roll_r    =   imu_in->angle.roll_a*PI/180.;
    imu_in->angle.yaw_r     =   imu_in->angle.yaw_a*PI/180.;
}

void Euler_REG_To_ANGLE(void){
    IMU.angle.pitch_a   =   IMU.angle.pitch_r*180./PI;
    IMU.angle.roll_a    =   IMU.angle.roll_r*180./PI;
    IMU.angle.yaw_a     =   IMU.angle.yaw_r*180./PI;
}

float invSqrt(float a){
    return 1./sqrtf(a);
}

double restrict_the_BETA(double b){
    if(b>0.0001){
        b=0.0001;
    }
    else if(b<0.00001){
        b=0.00001;
    }
    return b;
}

/* beta就是四元数微分方程求解的姿态算法的收敛速度，
 * 这个值就是陀螺仪的测量误差，
 * 通过查询IMU器件的PDF手册就可以知道，
 * 一般是一个很小的值。
 * */
Quaternion q={1,0,0,0,1,0,0,0};
Quaternion Q_info={1,0,0,0,1,0,0,0};
double BETA=0.00001;
//-------------------------------------------------------------------------------------------------------------------
//  gyro                是弧度制
//  beta                是收敛速度
//  betaIn_is_true      是beta传入是否为有效值，0为有效值，否则beta为默认值
//  delta_t             为采样时间，单位是ms
//-------------------------------------------------------------------------------------------------------------------
void Madgwick_AHRS_6_DOF_Get_Angle(double beta, uint8 beta_in_is_true, IMU_MODE_SELECT IMU_MODE, uint8 delta_t){

    float sampleFreq = 1000./delta_t;
    if(beta_in_is_true!=0){
        beta = 0.00001;
    }

    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    IMU.data.gx_r_old = IMU.data.gx_r;
    IMU.data.gy_r_old = IMU.data.gy_r;
    IMU.data.gz_r_old = IMU.data.gz_r;
    IMU.data.ax_old = IMU.data.ax;
    IMU.data.ay_old = IMU.data.ay;
    IMU.data.az_old = IMU.data.az;
    if(IMU_MODE == IMU963RA){
        imu963ra_get_value();
    }
    else if(IMU_MODE == IMU660RA){
        imu660ra_get_value();
    }
    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q.q1 * IMU.data.gx_r_old - q.q2 * IMU.data.gy_r_old - q.q3 * IMU.data.gz_r_old);
    qDot2 = 0.5f * (q.q0 * IMU.data.gx_r_old + q.q2 * IMU.data.gz_r_old - q.q3 * IMU.data.gy_r_old);
    qDot3 = 0.5f * (q.q0 * IMU.data.gy_r_old - q.q1 * IMU.data.gz_r_old + q.q3 * IMU.data.gx_r_old);
    qDot4 = 0.5f * (q.q0 * IMU.data.gz_r_old + q.q1 * IMU.data.gy_r_old - q.q2 * IMU.data.gx_r_old);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((IMU.data.ax_old == 0.0f) && (IMU.data.ay_old == 0.0f) && (IMU.data.az_old == 0.0f))) {
        float ax=IMU.data.ax_old,
               ay=IMU.data.ay_old,
               az=IMU.data.az_old;

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q.q0;
        _2q1 = 2.0f * q.q1;
        _2q2 = 2.0f * q.q2;
        _2q3 = 2.0f * q.q3;
        _4q0 = 4.0f * q.q0;
        _4q1 = 4.0f * q.q1;
        _4q2 = 4.0f * q.q2;
        _8q1 = 8.0f * q.q1;
        _8q2 = 8.0f * q.q2;
        q0q0 = q.q0 * q.q0;
        q1q1 = q.q1 * q.q1;
        q2q2 = q.q2 * q.q2;
        q3q3 = q.q3 * q.q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q.q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q.q3 - _2q1 * ax + 4.0f * q2q2 * q.q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }
    q.q0_old=q.q0;
    q.q1_old=q.q1;
    q.q2_old=q.q2;
    q.q3_old=q.q3;
    // Integrate rate of change of quaternion to yield quaternion
    q.q0 += qDot1 * (1.0f / sampleFreq);
    q.q1 += qDot2 * (1.0f / sampleFreq);
    q.q2 += qDot3 * (1.0f / sampleFreq);
    q.q3 += qDot4 * (1.0f / sampleFreq);

    // Normalise quaternion
    recipNorm = invSqrt(q0q0 + q1q1 + q2q2 + q3q3);
    q.q0 *= recipNorm;
    q.q1 *= recipNorm;
    q.q2 *= recipNorm;
    q.q3 *= recipNorm;
    QuaternionToEuler(&q, &IMU.angle.roll_r, &IMU.angle.pitch_r, &IMU.angle.yaw_r);

    IMU.angle.yaw_a_old=IMU.angle.yaw_a;
    IMU.angle.pitch_a_old=IMU.angle.pitch_a;
    IMU.angle.roll_a_old=IMU.angle.roll_a;

    IMU.angle.pitch_a   =   RAD_TO_ANGLE(IMU.angle.pitch_r);
    IMU.angle.roll_a    =   RAD_TO_ANGLE(IMU.angle.roll_r);
    IMU.angle.yaw_a     =   RAD_TO_ANGLE(IMU.angle.yaw_r);

    //动态变化BETA
//    double d_pitch = abs((IMU.angle.pitch_a-IMU.angle.pitch_a_old)/(float)delta_t);
//    double d_roll  = abs((IMU.angle.roll_a-IMU.angle.roll_a_old)/(float)delta_t);
//    double d_yaw   = abs((IMU.angle.yaw_a-IMU.angle.yaw_a_old)/(float)delta_t);
    double d_gx_r = abs((IMU.data.ax-IMU.data.ax_old)/(float)delta_t);
    double d_gy_r = abs((IMU.data.ay-IMU.data.ay_old)/(float)delta_t);
    double d_gz_r = abs((IMU.data.az-IMU.data.az_old)/(float)delta_t);
    //d_yaw大约在0~1之间
    BETA=restrict_the_BETA(d_gx_r*d_gx_r*d_gx_r*0.000006+d_gy_r*d_gy_r*d_gy_r*0.000002+d_gz_r*d_gz_r*d_gz_r*0.000002);
//    static uint32 ihare=0;
//    ihare++;
//    if(ihare%50==0)
//    printf("y%f p%f r%f\r\n",d_yaw*1000,d_pitch*1000,d_roll*1000);
//        printf("y%1.9f\r\n",d_yaw*1000);

//    if((IMU.angle.yaw_a>0&&IMU.angle.yaw_a_old<0)||(IMU.angle.yaw_a<0&&IMU.angle.yaw_a_old>0)){
//        if(abs(IMU.angle.yaw_a+IMU.angle.yaw_a_old)>2){
//            q.q0=q.q0_old;
//            q.q1=q.q1_old;
//            q.q2=q.q2_old;
//            q.q3=q.q3_old;
//        }
//    }
//    else{
//        if(abs(IMU.angle.yaw_a-IMU.angle.yaw_a_old)>2){
//            q.q0=q.q0_old;
//            q.q1=q.q1_old;
//            q.q2=q.q2_old;
//            q.q3=q.q3_old;
//        }
//    }
}


float param_Kp = 1.0;   // 加速度计(磁力计)的收敛速率比例增益50
float param_Ki = 0.20;   //陀螺仪收敛速率的积分增益 0.2
float I_ex, I_ey, I_ez;  // 误差积分
/**
  * brief IMU_AHRSupdate_noMagnetic  姿态解算融合，是Crazepony和核心算法
  * 使用的是互补滤波算法，没有使用Kalman滤波算法
  */
//-------------------------------------------------------------------------------------------------------------------
//  IMU_MODE            选择对应的IMU
//  delta_t             为采样时间，单位是ms
//-------------------------------------------------------------------------------------------------------------------
void AHRS_6_DOF_Get_Angle(IMU_MODE_SELECT IMU_MODE,float delta_T)
{
    delta_T = delta_T/1000.;
    if(IMU_MODE == IMU963RA){
        imu963ra_get_value();
    }
    else if(IMU_MODE == IMU660RA){
        imu660ra_get_value();
    }

    float gx=IMU.data.gx_a;
    float gy=IMU.data.gy_a;
    float gz=IMU.data.gz_a;
    float ax=IMU.data.ax;
    float ay=IMU.data.ay;
    float az=IMU.data.az;

//    float halfT = 0.5 * delta_T;
    float vx, vy, vz;    //当前的机体坐标系上的重力单位向量
    float ex, ey, ez;    //四元数计算值与加速度计测量值的误差
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
//    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
//    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    // float delta_2 = 0;

    //对加速度数据进行归一化 得到单位加速度
    float norm = invSqrt(ax*ax + ay*ay + az*az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    vx = 2*(q1q3 - q0q2);
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    //用叉乘误差来做PI修正陀螺零偏，
    //通过调节 param_Kp，param_Ki 两个参数，
    //可以控制加速度计修正陀螺仪积分姿态的速度。
    I_ex += delta_T * ex;   // integral error scaled by Ki
    I_ey += delta_T * ey;
    I_ez += delta_T * ez;

    gx = gx+ param_Kp*ex + param_Ki*I_ex;
    gy = gy+ param_Kp*ey + param_Ki*I_ey;
    gz = gz+ param_Kp*ez + param_Ki*I_ez;


    /*数据修正完成，下面是四元数微分方程*/


    //四元数微分方程，其中halfT为测量周期的1/2，gx gy gz为陀螺仪角速度，以下都是已知量，这里使用了一阶龙哥库塔求解四元数微分方程
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + ( q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + ( q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + ( q0*gz + q1*gy - q2*gx)*halfT;
    //    delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);
    // 整合四元数率    四元数微分方程  四元数更新算法，二阶毕卡法
    //    q0 = (1-delta_2/8)*q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    //    q1 = (1-delta_2/8)*q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    //    q2 = (1-delta_2/8)*q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    //    q3 = (1-delta_2/8)*q3 + (q0*gz + q1*gy - q2*gx)*halfT;
    // normalise quaternion
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    Q_info.q0 = q0 * norm;
    Q_info.q1 = q1 * norm;
    Q_info.q2 = q2 * norm;
    Q_info.q3 = q3 * norm;
}

void imu963ra_get_value(void){

    //读取寄存器
    imu963ra_get_acc();
    imu963ra_get_gyro();
    imu963ra_get_mag();

    //单位转换
    IMU.data.ax = imu963ra_acc_transition(imu963ra_acc_x);
    IMU.data.ay = imu963ra_acc_transition(imu963ra_acc_y);
    IMU.data.az = imu963ra_acc_transition(imu963ra_acc_z);
    IMU.data.gx_a = imu963ra_gyro_transition(imu963ra_gyro_x);
    IMU.data.gy_a = imu963ra_gyro_transition(imu963ra_gyro_y);
    IMU.data.gz_a = imu963ra_gyro_transition(imu963ra_gyro_z);
    IMU.data.mx = imu963ra_mag_transition(imu963ra_mag_x);
    IMU.data.my = imu963ra_mag_transition(imu963ra_mag_y);
    IMU.data.mz = imu963ra_mag_transition(imu963ra_mag_z);

    //校正误差
        //加速度计
        IMU.data.ax = IMU.data.ax - IMU.data.offset.acce_x;
        IMU.data.ax = LPButterworth(IMU.data.ax - IMU.data.offset.acce_x,&accel_filter_buf[0],&Accel_Parameter);
        IMU.data.ax = IMU.data.ax/10.;
        IMU.data.ax = IMU.data.ax*10.;

        IMU.data.ay = IMU.data.ay - IMU.data.offset.acce_y;
        IMU.data.ay = LPButterworth(IMU.data.ay - IMU.data.offset.acce_y,&accel_filter_buf[1],&Accel_Parameter);
        IMU.data.ay = IMU.data.ay/10.;
        IMU.data.ay = IMU.data.ay*10.;

        IMU.data.az = IMU.data.az - IMU.data.offset.acce_z;
        IMU.data.az = LPButterworth(IMU.data.az - IMU.data.offset.acce_z,&accel_filter_buf[2],&Accel_Parameter);
        IMU.data.az = IMU.data.az/10.;
        IMU.data.az = IMU.data.az*10.;
        //陀螺仪
        IMU.data.gx_a = IMU.data.gx_a - IMU.data.offset.gyro_x;
        IMU.data.gx_a = LPButterworth(IMU.data.gx_a,&gyro_filter_buf[0],&Gyro_Parameter);
//        IMU.data.gx_a = applyFilter(&filter,IMU.data.gx_a);
        IMU.data.gx_a = IMU.data.gx_a/100.;
        IMU.data.gx_a = IMU.data.gx_a*100.;

        IMU.data.gy_a = IMU.data.gy_a - IMU.data.offset.gyro_y;
        IMU.data.gy_a = LPButterworth(IMU.data.gy_a,&gyro_filter_buf[1],&Gyro_Parameter);
//        IMU.data.gy_a = applyFilter(&filter,IMU.data.gy_a);
        IMU.data.gy_a = IMU.data.gy_a/100.;
        IMU.data.gy_a = IMU.data.gy_a*100.;

        IMU.data.gz_a = IMU.data.gz_a - IMU.data.offset.gyro_z;
        IMU.data.gz_a = LPButterworth(IMU.data.gz_a,&gyro_filter_buf[2],&Gyro_Parameter);
//        IMU.data.gz_a = applyFilter(&filter,IMU.data.gz_a);
        IMU.data.gz_a = IMU.data.gz_a/100.;
        IMU.data.gz_a = IMU.data.gz_a*100.;
        //磁力计
        IMU.data.mx_f = (IMU.data.mx-0.00092002)/(0.56112);
        IMU.data.my_f = (IMU.data.my-0.10417)/(0.55129);
        IMU.data.mz_f = (IMU.data.mz+0.0067333)/(0.54547);

    //陀螺仪弧度转换
    IMU.data.gx_r = IMU.data.gx_a*PI/180.;
    IMU.data.gy_r = IMU.data.gy_a*PI/180.;
    IMU.data.gz_r = IMU.data.gz_a*PI/180.;
}

void imu660ra_get_value(void){
    imu660ra_get_acc();
    imu660ra_get_gyro();
    IMU.data.ax = imu660ra_acc_transition(imu660ra_acc_x);
    IMU.data.ay = imu660ra_acc_transition(imu660ra_acc_y);
    IMU.data.az = imu660ra_acc_transition(imu660ra_acc_z);
    IMU.data.gx_a = imu660ra_gyro_transition(imu660ra_gyro_x);
    IMU.data.gy_a = imu660ra_gyro_transition(imu660ra_gyro_y);
    IMU.data.gz_a = imu660ra_gyro_transition(imu660ra_gyro_z);

    //校正误差
        //加速度计
        IMU.data.ax = IMU.data.ax - IMU.data.offset.acce_x;
        IMU.data.ax = LPButterworth(IMU.data.ax - IMU.data.offset.acce_x,&accel_filter_buf[0],&Accel_Parameter);
        IMU.data.ax = IMU.data.ax/10.;
        IMU.data.ax = IMU.data.ax*10.;

        IMU.data.ay = IMU.data.ay - IMU.data.offset.acce_y;
        IMU.data.ay = LPButterworth(IMU.data.ay - IMU.data.offset.acce_y,&accel_filter_buf[1],&Accel_Parameter);
        IMU.data.ay = IMU.data.ay/10.;
        IMU.data.ay = IMU.data.ay*10.;

        IMU.data.az = IMU.data.az - IMU.data.offset.acce_z;
        IMU.data.az = LPButterworth(IMU.data.az - IMU.data.offset.acce_z,&accel_filter_buf[2],&Accel_Parameter);
        IMU.data.az = IMU.data.az/10.;
        IMU.data.az = IMU.data.az*10.;
//        //加速度计
//        IMU.data.ax = LPButterworth(IMU.data.ax - IMU.data.offset.acce_x,&accel_filter_buf[0],&Accel_Parameter);
//        IMU.data.ay = LPButterworth(IMU.data.ay - IMU.data.offset.acce_y,&accel_filter_buf[1],&Accel_Parameter);
//        IMU.data.az = LPButterworth(IMU.data.az - IMU.data.offset.acce_z,&accel_filter_buf[2],&Accel_Parameter);

        //陀螺仪
        IMU.data.gx_a = IMU.data.gx_a - IMU.data.offset.gyro_x;
        IMU.data.gx_a = LPButterworth(IMU.data.gx_a,&gyro_filter_buf[0],&Gyro_Parameter);
//        IMU.data.gx_a = applyFilter(&filter,IMU.data.gx_a);
        IMU.data.gx_a = IMU.data.gx_a/100.;
        IMU.data.gx_a = IMU.data.gx_a*100.;

        IMU.data.gy_a = IMU.data.gy_a - IMU.data.offset.gyro_y;
        IMU.data.gy_a = LPButterworth(IMU.data.gy_a,&gyro_filter_buf[1],&Gyro_Parameter);
//        IMU.data.gy_a = applyFilter(&filter,IMU.data.gy_a);
        IMU.data.gy_a = IMU.data.gy_a/100.;
        IMU.data.gy_a = IMU.data.gy_a*100.;

        IMU.data.gz_a = IMU.data.gz_a - IMU.data.offset.gyro_z;
        IMU.data.gz_a = LPButterworth(IMU.data.gz_a,&gyro_filter_buf[2],&Gyro_Parameter);
//        IMU.data.gz_a = applyFilter(&filter,IMU.data.gz_a);
        IMU.data.gz_a = IMU.data.gz_a/100.;
        IMU.data.gz_a = IMU.data.gz_a*100.;

    //陀螺仪弧度转换
    IMU.data.gx_r = IMU.data.gx_a*PI/180.;
    IMU.data.gy_r = IMU.data.gy_a*PI/180.;
    IMU.data.gz_r = IMU.data.gz_a*PI/180.;
}
