#include "qin_flash.h"
#include "zf_device_gps_tau1201.h"
uint8 flash_state;
uint32 read_write_flash_flag=0;
int only_read_once=0,only_write_once=0;
/*
 * GPS数据写入
 * 示例：write_gps_in(GPS_point,point_max,8);
 * GPS_point[point_max][2]即为GPS数据数组
 * */
uint8 read_write_gps_flash(int page){
    uint8 rt_v=0;

    if(read_write_flash_flag%2==0){
        if(only_write_once==0){
            flash_buffer_clear();
            only_write_once++;
            only_read_once=0;

//            GPS_point[0][0]=39.123;
//            GPS_point[0][1]=119.456;
//            for(int i=0;i<30;i++){
//                xy_point[i][0]=i+0.123456;
//                xy_point[i][1]=i+0.654321;
//            }
            int index=0;
            uint32 zhenshu, xiaoshu, fuhao1,fuhao2;//符号是1表示是负数，0表示是正数
            //放入gps第一个点
            zhenshu = (uint32)GPS_point[0][0];
            xiaoshu = (uint32)((GPS_point[0][0]-(uint32)GPS_point[0][0])*1000000);
            flash_union_buffer[index].uint32_type=zhenshu;index++;
            flash_union_buffer[index].uint32_type=xiaoshu;index++;
            zhenshu = (uint32)GPS_point[0][1];
            xiaoshu = (uint32)((GPS_point[0][1]-(uint32)GPS_point[0][1])*1000000);
            flash_union_buffer[index].uint32_type=zhenshu;index++;
            flash_union_buffer[index].uint32_type=xiaoshu;index++;
            for(int i=0;i<5;i++){
                float xy1 = base_point[i][0];if(xy1<0){xy1=-xy1;fuhao1=1;}else fuhao1=0;
                float xy2 = base_point[i][1];if(xy2<0){xy2=-xy2;fuhao2=1;}else fuhao2=0;
                zhenshu = (uint32)xy1;
                xiaoshu = (uint32)((xy1-(uint32)xy1)*1000000);
                flash_union_buffer[index].uint32_type=fuhao1;
                index++;
                flash_union_buffer[index].uint32_type=zhenshu;
                index++;
                flash_union_buffer[index].uint32_type=xiaoshu;
                index++;

                zhenshu = (uint32)xy2;
                xiaoshu = (uint32)((xy2-(uint32)xy2)*1000000);
                flash_union_buffer[index].uint32_type=fuhao2;
                index++;
                flash_union_buffer[index].uint32_type=zhenshu;
                index++;
                flash_union_buffer[index].uint32_type=xiaoshu;
                index++;
            }


            //放入xy的30个点
            for(int i=0;i<30;i++){
                float xy1 = xy_point[i][0];if(xy1<0){xy1=-xy1;fuhao1=1;}else fuhao1=0;
                float xy2 = xy_point[i][1];if(xy2<0){xy2=-xy2;fuhao2=1;}else fuhao2=0;
                zhenshu = (uint32)xy1;
                xiaoshu = (uint32)((xy1-(uint32)xy1)*1000000);
                flash_union_buffer[index].uint32_type=fuhao1;
                index++;
                flash_union_buffer[index].uint32_type=zhenshu;
                index++;
                flash_union_buffer[index].uint32_type=xiaoshu;
                index++;

                zhenshu = (uint32)xy2;
                xiaoshu = (uint32)((xy2-(uint32)xy2)*1000000);
                flash_union_buffer[index].uint32_type=fuhao2;
                index++;
                flash_union_buffer[index].uint32_type=zhenshu;
                index++;
                flash_union_buffer[index].uint32_type=xiaoshu;
                index++;
            }

            //point max
            uint32 pointmax_tmp=(uint32)point_max;
            flash_union_buffer[index].uint32_type=pointmax_tmp;
            index++;

            for(int i=0;i<30;i++){
                uint32 state_tmp = (uint32)point_state[i];
                flash_union_buffer[index].uint32_type=state_tmp;
                index++;
            }

            flash_write_page_from_buffer(0, page);
            flash_state=1;
            ips200_show_uint(30,16*5,flash_union_buffer[0].uint32_type,10);
            ips200_show_uint(30,16*6,flash_union_buffer[1].uint32_type,10);
            ips200_show_uint(30,16*7,flash_union_buffer[2].uint32_type,10);
            ips200_show_uint(30,16*8,flash_union_buffer[3].uint32_type,10);
            ips200_show_uint(30,16*9,flash_union_buffer[4].uint32_type,10);
            ips200_show_uint(30,16*10,flash_union_buffer[5].uint32_type,10);
            ips200_show_uint(30,16*11,flash_union_buffer[6].uint32_type,10);
            ips200_show_uint(30,16*12,flash_union_buffer[7].uint32_type,10);
            ips200_show_uint(30,16*13,flash_union_buffer[8].uint32_type,10);
            ips200_show_uint(30,16*14,flash_union_buffer[9].uint32_type,10);
            ips200_show_uint(30,16*15,flash_union_buffer[10].uint32_type,10);
            ips200_show_uint(30,16*16,flash_union_buffer[11].uint32_type,10);
            ips200_show_uint(30,16*17,flash_union_buffer[12].uint32_type,10);
            ips200_show_uint(30,16*18,flash_union_buffer[13].uint32_type,10);
            ips200_show_uint(30,16*19,flash_union_buffer[14].uint32_type,10);

            ips200_show_uint(30+100,16*5,flash_union_buffer[15].uint32_type,10);
            ips200_show_uint(30+100,16*6,flash_union_buffer[16].uint32_type,10);
            ips200_show_uint(30+100,16*7,flash_union_buffer[17].uint32_type,10);
            ips200_show_uint(30+100,16*8,flash_union_buffer[18].uint32_type,10);
            ips200_show_uint(30+100,16*9,flash_union_buffer[19].uint32_type,10);
            ips200_show_uint(30+100,16*10,flash_union_buffer[20].uint32_type,10);
            ips200_show_uint(30+100,16*11,flash_union_buffer[21].uint32_type,10);
            ips200_show_uint(30+100,16*12,flash_union_buffer[22].uint32_type,10);
            ips200_show_uint(30+100,16*13,flash_union_buffer[23].uint32_type,10);
            ips200_show_uint(30+100,16*14,flash_union_buffer[24].uint32_type,10);
            ips200_show_uint(30+100,16*15,flash_union_buffer[25].uint32_type,10);
            ips200_show_uint(30+100,16*16,flash_union_buffer[26].uint32_type,10);
            ips200_show_uint(30+100,16*17,flash_union_buffer[27].uint32_type,10);
            ips200_show_uint(30+100,16*18,flash_union_buffer[28].uint32_type,10);
            ips200_show_uint(30+100,16*19,flash_union_buffer[29].uint32_type,10);
        }
    }
    else{
        if(only_read_once==0){
            flash_buffer_clear();
            only_read_once++;
            only_write_once=0;
            for(int i=0;i<30;i++){
                xy_point[i][0]=0;
                xy_point[i][1]=0;
            }

            int index_addNum=2;
            flash_read_page_to_buffer(0, page);
            int index=0;
            uint32 zhenshu, xiaoshu,fuhao1,fuhao2;
            //取出gps第一个点
            zhenshu = flash_union_buffer[index].uint32_type;
            index+=index_addNum;
            xiaoshu = flash_union_buffer[index].uint32_type;
            index+=index_addNum;
            GPS_point[0][0]=(double)zhenshu + ((double)xiaoshu/(double)1000000.);;
            zhenshu = flash_union_buffer[index].uint32_type;
            index+=index_addNum;
            xiaoshu = flash_union_buffer[index].uint32_type;
            index+=index_addNum;
            GPS_point[0][1]=(double)zhenshu + ((double)xiaoshu/(double)1000000.);



            for(int i=0;i<5;i++){
                fuhao1=flash_union_buffer[index].uint32_type;
                index+=index_addNum;
                zhenshu=flash_union_buffer[index].uint32_type;
                index+=index_addNum;
                xiaoshu=flash_union_buffer[index].uint32_type;
                index+=index_addNum;
                if(fuhao1>0)base_point[i][0]=-((float)zhenshu + (float)xiaoshu/(float)1000000);
                else base_point[i][0]=((float)zhenshu + (float)xiaoshu/(float)1000000);

                fuhao2=flash_union_buffer[index].uint32_type;
                index+=index_addNum;
                zhenshu=flash_union_buffer[index].uint32_type;
                index+=index_addNum;
                xiaoshu=flash_union_buffer[index].uint32_type;
                index+=index_addNum;
                if(fuhao2>0)base_point[i][1]=-((float)zhenshu + (float)xiaoshu/(float)1000000);
                else base_point[i][1]=((float)zhenshu + (float)xiaoshu/(float)1000000);
            }
            //取出xy的30个点
            for(int i=0;i<30;i++){
                fuhao1=flash_union_buffer[index].uint32_type;
                index+=index_addNum;
                zhenshu=flash_union_buffer[index].uint32_type;
                index+=index_addNum;
                xiaoshu=flash_union_buffer[index].uint32_type;
                index+=index_addNum;
                if(fuhao1>0)xy_point[i][0]=-((float)zhenshu + (float)xiaoshu/(float)1000000);
                else xy_point[i][0]=((float)zhenshu + (float)xiaoshu/(float)1000000);

                fuhao2=flash_union_buffer[index].uint32_type;
                index+=index_addNum;
                zhenshu=flash_union_buffer[index].uint32_type;
                index+=index_addNum;
                xiaoshu=flash_union_buffer[index].uint32_type;
                index+=index_addNum;
                if(fuhao2>0)xy_point[i][1]=-((float)zhenshu + (float)xiaoshu/(float)1000000);
                else xy_point[i][1]=((float)zhenshu + (float)xiaoshu/(float)1000000);
            }
            point_max=(uint8)flash_union_buffer[index].uint32_type;
            index+=index_addNum;

            for(int i=0;i<30;i++){
                point_state[i]=(uint8)flash_union_buffer[index].uint32_type;
                index+=index_addNum;
            }

            ips200_show_float(30,16*5,xy_point[0][0],4,6);
            ips200_show_float(30,16*6,xy_point[0][1],4,6);
            ips200_show_float(30,16*7,xy_point[1][0],4,6);
            ips200_show_float(30,16*8,xy_point[1][1],4,6);
            ips200_show_float(30,16*9,xy_point[2][0],4,6);
            ips200_show_float(30,16*10,xy_point[2][1],4,6);
            ips200_show_float(30,16*11,xy_point[3][0],4,6);
            ips200_show_float(30,16*12,xy_point[3][1],4,6);
            ips200_show_float(30,16*13,xy_point[4][0],4,6);
            ips200_show_float(30,16*14,xy_point[4][1],4,6);
            ips200_show_float(30,16*15,xy_point[5][0],4,6);
            ips200_show_float(30,16*16,xy_point[5][1],4,6);
            ips200_show_float(30,16*17,xy_point[6][0],4,6);
            ips200_show_float(30,16*18,xy_point[6][1],4,6);
            ips200_show_float(30,16*19,xy_point[7][0],4,6);

            ips200_show_float(30+100,16*5,xy_point[7][1],4,6);
            ips200_show_float(30+100,16*6,xy_point[8][0],4,6);
            ips200_show_float(30+100,16*7,xy_point[8][1],4,6);
            ips200_show_float(30+100,16*8,xy_point[9][0],4,6);
            ips200_show_float(30+100,16*9,xy_point[9][1],4,6);
            ips200_show_float(30+100,16*10,xy_point[10][0],4,6);
            ips200_show_float(30+100,16*11,xy_point[10][1],4,6);
            ips200_show_float(30+100,16*12,xy_point[11][0],4,6);
            ips200_show_float(30+100,16*13,xy_point[11][1],4,6);
            ips200_show_float(30+100,16*14,xy_point[12][0],4,6);
            ips200_show_float(30+100,16*15,xy_point[12][1],4,6);
            ips200_show_float(30+100,16*16,xy_point[13][0],4,6);
            ips200_show_float(30+100,16*17,xy_point[13][1],4,6);
            ips200_show_float(30+100,16*18,GPS_point[0][0],4,6);
            ips200_show_float(30+100,16*19,GPS_point[0][1],4,6);
//            point_max=21;
            Z_line=fitLine(xy_point, 5);
//            Z_line.intercept=0;
            S_line=fitLine(base_point, 5);
            Round_line=calculateLine(Z_line.slope,xy_point[5][0], xy_point[5][1]) ;
            SZ_line=calculateLine(Z_line.slope,xy_point[11][0], xy_point[11][1]) ;
        }
    }

    return rt_v;
}



/*
 * 读取flash数据，返回一个double列表
 * 示例：double *a = read_gps_out(point_max, 8);
 * a[]的数据形式为：{latitude1,latitude2,latitude3,......,longitude1,longitude2,longitude3...}
 * a[]的长度就是2*point_max，因为一共有point_max个点
 * 所以要对a[]再进行处理才能得到GPS_point[][]
 * */
//
double latitude_flash[30];
double longitude_flash[30];

void read_gps_out(int which_page){
    flash_buffer_clear();
    flash_read_page_to_buffer(0, which_page);
//    point_max=flash_union_buffer[0].uint8_type;
    GPS_point[0][0]=flash_union_buffer[0].double_type;
    GPS_point[0][1]=flash_union_buffer[1].double_type;
    int index=2,index2=0;
    for(int i=0;i<30;i++){
        xy_point[index2][0]=flash_union_buffer[index].float_type;
        index+=2;index2+=2;
        xy_point[index2][1]=flash_union_buffer[index].float_type;
        index+=2;index2+=2;
    }

    ips200_show_uint(30,16*5,flash_union_buffer[0].double_type,10);
    ips200_show_uint(30,16*6,flash_union_buffer[1].double_type,10);
    ips200_show_uint(30,16*7,flash_union_buffer[2].float_type,10);
    ips200_show_uint(30,16*8,flash_union_buffer[3].float_type,10);
    ips200_show_uint(30,16*9,flash_union_buffer[4].float_type,10);
    ips200_show_uint(30,16*10,flash_union_buffer[5].float_type,10);
    ips200_show_uint(30,16*11,flash_union_buffer[6].float_type,10);
    ips200_show_uint(30,16*12,flash_union_buffer[7].float_type,10);
    ips200_show_uint(30,16*13,flash_union_buffer[8].float_type,10);
    ips200_show_uint(30,16*14,flash_union_buffer[9].float_type,10);
    ips200_show_uint(30,16*15,flash_union_buffer[10].float_type,10);
    ips200_show_uint(30,16*16,flash_union_buffer[11].float_type,10);
    ips200_show_uint(30,16*17,flash_union_buffer[12].float_type,10);
    ips200_show_uint(30,16*18,flash_union_buffer[13].float_type,10);
    ips200_show_uint(30,16*19,flash_union_buffer[14].float_type,10);

//    for(int i=0;i<30;i++){
//        latitude_flash[i]=0;
//        longitude_flash[i]=0;
//    }
//    uint32 zhenshu;
//    uint32 xiaoshu;
//    int index=2;
//    for(int i=0;i<point_max;i++){
//        zhenshu = flash_union_buffer[index].uint32_type;
//        index+=2;
//        xiaoshu = flash_union_buffer[index].uint32_type;
//        index+=2;
////        ips200_show_uint (30, 16*5, zhenshu, 10);
////        ips200_show_uint (30, 16*6, xiaoshu, 10);
//        latitude_flash[i] = (double)zhenshu + ((double)xiaoshu/(double)1000000.);
////        ips200_show_float(30, 16*7, latitude_flash[i], 3, 6);
//
//        zhenshu = flash_union_buffer[index].uint32_type;
//        index+=2;
//        xiaoshu = flash_union_buffer[index].uint32_type;
//        index+=2;
////        ips200_show_uint (30, 16*8, zhenshu, 10);
////        ips200_show_uint (30, 16*9, xiaoshu, 10);
//        longitude_flash[i]= (double)zhenshu + ((double)xiaoshu/(double)1000000.);
////        ips200_show_float(30, 16*10, longitude_flash[i], 3, 6);
//    }

//    ips200_show_float(0,16*5,latitude_flash[0],3,6);
//    ips200_show_float(0,16*6,latitude_flash[1],3,6);
//    ips200_show_float(0,16*7,latitude_flash[2],3,6);
//    ips200_show_float(0,16*8,latitude_flash[3],3,6);
//    ips200_show_float(0,16*9,latitude_flash[4],3,6);
//    ips200_show_float(0,16*10,latitude_flash[5],3,6);
//    ips200_show_float(0,16*11,latitude_flash[6],3,6);
//    ips200_show_float(0,16*12,latitude_flash[7],3,6);
//    ips200_show_float(0,16*13,latitude_flash[8],3,6);
//    ips200_show_float(0,16*14,latitude_flash[9],3,6);
//    ips200_show_float(0,16*15,latitude_flash[10],3,6);
//    ips200_show_float(0,16*16,latitude_flash[11],3,6);
//    ips200_show_float(0,16*17,latitude_flash[12],3,6);
//    ips200_show_float(0,16*18,latitude_flash[13],3,6);
//    ips200_show_float(0,16*19,latitude_flash[14],3,6);
//
//    ips200_show_float(120,16*5,longitude_flash[0],3,6);
//    ips200_show_float(120,16*6,longitude_flash[1],3,6);
//    ips200_show_float(120,16*7,longitude_flash[2],3,6);
//    ips200_show_float(120,16*8,longitude_flash[3],3,6);
//    ips200_show_float(120,16*9,longitude_flash[4],3,6);
//    ips200_show_float(120,16*10,longitude_flash[5],3,6);
//    ips200_show_float(120,16*11,longitude_flash[6],3,6);
//    ips200_show_float(120,16*12,longitude_flash[7],3,6);
//    ips200_show_float(120,16*13,longitude_flash[8],3,6);
//    ips200_show_float(120,16*14,longitude_flash[9],3,6);
//    ips200_show_float(120,16*15,longitude_flash[10],3,6);
//    ips200_show_float(120,16*16,longitude_flash[11],3,6);
//    ips200_show_float(120,16*17,longitude_flash[12],3,6);
//    ips200_show_float(120,16*18,longitude_flash[13],3,6);
//    ips200_show_float(120,16*19,longitude_flash[14],3,6);
    flash_state=2;
}

/*
 * 传入一个double数组
 * num          存入double的个数
 * which_page   存入的扇区
 * 返回值         0-正常，1-此扇区已经写过了，需要清理后再写，2-写入数据失败
 * */
uint8 write_doubles_in(double *p,int num, int which_page){
    uint8 return_val=0;
    flash_read_page_to_buffer(0, which_page);           // 将数据从 flash 读取到缓冲区
    if(flash_union_buffer[0].int32_type != 1234567){
        flash_union_buffer[0].int32_type = 1234567;
        for(int i=0;i<num;i++){
            double temp = *(p+i);
            my_double_s all;
            all.integer = (int32)temp;
//            all.decimal = (int32)((temp - all.integer)*1000000);
            flash_union_buffer[i*2+1].int32_type = all.integer;
            flash_union_buffer[i*2+2].int32_type = all.decimal;
        }
        int ifsucc = flash_write_page_from_buffer(0, 8);
        if(ifsucc==1)
            return_val=2;
        flash_buffer_clear();
    }
    else{
        return_val = 1;
        printf("\r\n This page has been stored nums!! \r\n");
    }
    return return_val;
}

/*
 * begin        起始读取地址
 * which_page   读取的扇区
 * */
double read_one_double_out(int begin, int which_page){
    flash_read_page_to_buffer(0, which_page);
    int zhenshu = flash_union_buffer[begin].int32_type;
    int xiaoshu = flash_union_buffer[begin+1].int32_type;
    double all = (double)zhenshu + (double)xiaoshu/1000000.;
    flash_buffer_clear();
    return all;
}


/*
 * 读取指定页数的任意个数double，返回一个double数组
 * */
double p_a;
double* read_doubles_out(int num, int which_page){
    flash_read_page_to_buffer(0, which_page);

    double *p=&p_a;

    if(flash_union_buffer[0].int32_type == 1234567){
        for(int i=0;i<num;i++){
            int zhenshu = flash_union_buffer[i*2+1].int32_type;
            int xiaoshu = flash_union_buffer[i*2+2].int32_type;
            *(p+i) = (double)zhenshu + (double)xiaoshu/1000000.;
        }
        flash_buffer_clear();
    }
    else{
        printf("\r\n ERROR! Dosn't store correct nums in this page!\r\n");
    }

    return p;
}
