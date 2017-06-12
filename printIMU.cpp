
#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "MPU9250.h"
#include <unistd.h>

#include <math.h>


#define DEBUG 0
#define USE_MYSQL 0

#if USE_MYSQL
#include <mysql.h>
#endif

// Register SPI addresses
#define DEVID 0x68//(0x68)
#define WHO_AM_I 0x75

int fd; // Device file handle of mpu9250
int md; // Device file handle of ak8963
int bd; // Device file handle of bmp280

typedef struct
{
    signed short x;
    signed short y;
    signed short z;
} pos_struct;

pos_struct Est_A, Est_G, Est_M;
//pos_struct Est_G;
//pos_struct Est_M;
short MPU_temp = 0;
int BMP_temp = 0;
int BMP_press = 0;

float magCorrect_ASA[3] = {1.0, 1.0, 1.0};
const float MagHardCorrect[3] = {0.0, -150.0, -200.0};
//float MPU_MAG_asay;
//float MPU_MAG_asaz;

void Get_acc(void)
{
	unsigned char d[6];

	d[1] = wiringPiI2CReadReg8(fd, ACCEL_XOUT_H);//0x3B);
	d[0] = wiringPiI2CReadReg8(fd, ACCEL_XOUT_L);//0x3C);
	d[3] = wiringPiI2CReadReg8(fd, ACCEL_YOUT_H);//0x3D);
	d[2] = wiringPiI2CReadReg8(fd, ACCEL_YOUT_L);//0x3E);
	d[5] = wiringPiI2CReadReg8(fd, ACCEL_ZOUT_H);//0x3F);
	d[4] = wiringPiI2CReadReg8(fd, ACCEL_ZOUT_L);//0x40);

	short x = *(short *)&d[0];
	short y = *(short *)&d[2];
	short z = *(short *)&d[4];

	if(DEBUG) printf("ACC x=%d, y=%d, z=%d\n\r", x, y, z);

	Est_A.x = x;
	Est_A.y = y;
	Est_A.z = z;
}


void Get_gyro(void)
{
	unsigned char d[6];

	d[1] = wiringPiI2CReadReg8(fd, GYRO_XOUT_H);//0x3B);
	d[0] = wiringPiI2CReadReg8(fd, GYRO_XOUT_L);//0x3C);
	d[3] = wiringPiI2CReadReg8(fd, GYRO_YOUT_H);//0x3D);
	d[2] = wiringPiI2CReadReg8(fd, GYRO_YOUT_L);//0x3E);
	d[5] = wiringPiI2CReadReg8(fd, GYRO_ZOUT_H);//0x3F);
	d[4] = wiringPiI2CReadReg8(fd, GYRO_ZOUT_L);//0x40);

	short x = *(short *)&d[0];
	short y = *(short *)&d[2];
	short z = *(short *)&d[4];

	if(DEBUG) printf("GYR x=%d, y=%d, z=%d\n\r", x, y, z);

	Est_G.x = x;
	Est_G.y = y;
	Est_G.z = z;
}

void Get_temp(void)
{
	unsigned char d[2];

	d[1] = wiringPiI2CReadReg8(fd, TEMP_OUT_H);
	d[0] = wiringPiI2CReadReg8(fd, TEMP_OUT_L);

	short tem = *(short *)&d[0];

	if(DEBUG) printf("TEM tem=%d\n\r", tem);

	MPU_temp = tem;
}


void Get_mag(void)
{
	unsigned char sr1 = 0;


	wiringPiI2CWriteReg8(md, AK8963_CNTL, 0x11);
	int timeout = 10000;
	while((sr1&0x01) == 0) {
		usleep(10);
		sr1 = wiringPiI2CReadReg8(md, AK8963_ST1);
		if (timeout--<=0)return; }
	//usleep(100*1000);

	unsigned char d[6];

	d[0] = wiringPiI2CReadReg8(md, AK8963_XOUT_L);
	d[1] = wiringPiI2CReadReg8(md, AK8963_XOUT_H);
	d[2] = wiringPiI2CReadReg8(md, AK8963_YOUT_L);
	d[3] = wiringPiI2CReadReg8(md, AK8963_YOUT_H);
	d[4] = wiringPiI2CReadReg8(md, AK8963_ZOUT_L);
	d[5] = wiringPiI2CReadReg8(md, AK8963_ZOUT_H);

	short x = *(short *)&d[0];
	short y = *(short *)&d[2];
	short z = *(short *)&d[4];

	x = x*magCorrect_ASA[0];
	y = y*magCorrect_ASA[1];
	z = z*magCorrect_ASA[2];

	unsigned char st2 = wiringPiI2CReadReg8(md, AK8963_ST2);
	if (!(st2>>4)) return; // return if no 16-bit mode

	//y-=150;
	if(DEBUG) printf("MAG x=%d, y=%d, z=%d\n\r", x, y, z);

	Est_M.x = x;
	Est_M.y = y;
	Est_M.z = z;
}




unsigned short dig_T1 = 0;
short dig_T2 = 0;
short dig_T3 = 0;
unsigned short dig_P1;
short dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

#define BMP280_S32_t int
#define BMP280_U32_t unsigned int
#define BMP280_S64_t long long

BMP280_S32_t t_fine;
BMP280_S32_t bmp280_compensate_T_int32(BMP280_S32_t adc_T)
{
	BMP280_S32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((BMP280_S32_t)dig_T1<<1))) * ((BMP280_S32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((BMP280_S32_t)dig_T1)) * ((adc_T>>4) - ((BMP280_S32_t)dig_T1))) >> 12) *((BMP280_S32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

BMP280_U32_t bmp280_compensate_P_int64(BMP280_S32_t adc_P)
{
	BMP280_S64_t var1, var2, p;
	var1 = ((BMP280_S64_t)t_fine) - 128000;var2 = var1 * var1 * (BMP280_S64_t)dig_P6;
	var2 = var2 + ((var1*(BMP280_S64_t)dig_P5)<<17);
	var2 = var2 + (((BMP280_S64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (BMP280_S64_t)dig_P3)>>8) + ((var1 * (BMP280_S64_t)dig_P2)<<12);
	var1 = (((((BMP280_S64_t)1)<<47)+var1))*((BMP280_S64_t)dig_P1)>>33;
	if (var1 == 0){return 0; // avoid exception caused by division by zero
	}
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((BMP280_S64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((BMP280_S64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((BMP280_S64_t)dig_P7)<<4);
	return (BMP280_U32_t)p;
}


void BMPInit(void)
{
	unsigned char c[18];
	int i;

	for (i=0; i<6; i++) c[0+i] = wiringPiI2CReadReg8(bd, (0x88+i));

	dig_T1 = (unsigned short)((c[1]<<8)+c[0]);
	dig_T2 = (signed short)((c[3]<<8)+c[2]);
	dig_T3 = (signed short)((c[5]<<8)+c[4]);

	for (i=0; i<18; i++) c[i] = wiringPiI2CReadReg8(bd, ((0x8E)+i));

	dig_P1 = (unsigned short)((c[1]<<8)+c[0]);
	dig_P2 = (signed short)((c[3]<<8)+c[2]);
	dig_P3 = (signed short)((c[5]<<8)+c[4]);
	dig_P4 = (signed short)((c[7]<<8)+c[6]);
	dig_P5 = (signed short)((c[9]<<8)+c[8]);
	dig_P6 = (signed short)((c[11]<<8)+c[10]);
	dig_P7 = (signed short)((c[13]<<8)+c[12]);
	dig_P8 = (signed short)((c[15]<<8)+c[14]);
	dig_P9 = (signed short)((c[17]<<8)+c[16]);

}


void GetBmpTemp(void)
{
	char sr = 0;
	wiringPiI2CWriteReg8(bd, 0xF4, (0x03+(0x7<<2)+(0x7<<5)));
	//usleep(1000*1000);
	int timeout = 1000;
	while((sr&0x01) == 0) {
		usleep(1000);
		sr = wiringPiI2CReadReg8(bd, 0xF3);
		if (timeout--<=0)break; }

	unsigned char c[3];
	c[0] = wiringPiI2CReadReg8(bd, 0xFC);
	c[1] = wiringPiI2CReadReg8(bd, 0xFB);
	c[2] = wiringPiI2CReadReg8(bd, 0xFA);
	int tt = (c[0]>>4) + (c[1]<<4) + (c[2]<<12);

	if(DEBUG) printf("T = %d\n\r", bmp280_compensate_T_int32(tt));

	BMP_temp = bmp280_compensate_T_int32(tt);

}


void GetBmpPress(void)
{
	char sr = 0;
	wiringPiI2CWriteReg8(bd, 0xF4, (0x03+(0x7<<2)+(0x7<<5)));
	//usleep(1000*1000);
	int timeout = 1000;
	while((sr&0x01) == 0) {
		usleep(1000);
		sr = wiringPiI2CReadReg8(bd, 0xF3);
		if (timeout--<=0)break; }

	unsigned char c[3];
	c[0] = wiringPiI2CReadReg8(bd, 0xF9);
	c[1] = wiringPiI2CReadReg8(bd, 0xF8);
	c[2] = wiringPiI2CReadReg8(bd, 0xF7);
	int pp = (c[0]>>4) + (c[1]<<4) + (c[2]<<12);

	if(DEBUG) printf("P = %d Pa", bmp280_compensate_P_int64(pp)/256);
	if(DEBUG) printf(" (%d mmHg) \n\r", (bmp280_compensate_P_int64(pp)/256/133));

	BMP_press = bmp280_compensate_P_int64(pp)/256;
}


#if USE_MYSQL

MYSQL mysql;
MYSQL_RES *res;
MYSQL_ROW row;

void MYSQL_error(void){
   printf("%s\n", mysql_error(&mysql));
}



void MYSQL_clear(void)
{

	printf("Clear data mysql database...");

	if (!mysql_init (&mysql)) return;//abort ();
	if (!(mysql_real_connect(&mysql,"localhost","root","root", "DATA", 3306 , NULL , 0))) {MYSQL_error();return;}
	if (mysql_select_db(&mysql,"DATA"))  {MYSQL_error();return;}

	if(mysql_query(&mysql,"DELETE FROM TenDof")) {MYSQL_error();return;}

	mysql_free_result(res);
	mysql_close(&mysql);

}


void MYSQL_reload(void)
{
	char BUF_CMD[10*1024];

	printf("Update data mysql database...");

	if (!mysql_init (&mysql)) return;//abort ();
	if (!(mysql_real_connect(&mysql,"localhost","root","root", "DATA", 3306 , NULL , 0))) {MYSQL_error();return;}
	if (mysql_select_db(&mysql,"DATA"))  {MYSQL_error();return;}

	//if(mysql_query(&mysql,"DELETE FROM TenDof")) {MYSQL_error();return;}

	//sprintf(BUF_CMD, "INSERT INTO MySensers (N1, N2, N3, N4) VALUES (%d, 1, 1, 1)", MPU_temp);

	sprintf(BUF_CMD, "INSERT INTO `DATA`.`TenDof` (`ACC_X`, `ACC_Y`, `ACC_Z`, `GYRO_X`, `GYRO_Y`, `GYRO_Z`, `MAG_X`, `MAG_Y`, `MAG_Z`, `MPU_T`, `BMP_PRESS`, `BMP_T`) VALUES ('%d', '%d', '%d', '%d', '%d', '%d', '%d', '%d', '%d', '%d', '%d', '%d')",
	Est_A.x, Est_A.y, Est_A.z, Est_G.x, Est_G.y, Est_G.z, Est_M.x, Est_M.y, Est_M.z, MPU_temp, BMP_press, BMP_temp );



	//printf(BUF_CMD);
	if(mysql_query(&mysql,BUF_CMD)) {MYSQL_error();return;}

	//Get list
	//unsigned int i = 0;
	//if(mysql_query(&mysql,"SELECT * FROM MySensers")) {MYSQL_error();return;}
	//if (!(res = mysql_store_result(&mysql))) {MYSQL_error();return;}
	//while((row = mysql_fetch_row(res))) {
	//for (i = 0 ; i < mysql_num_fields(res); i++) printf("%s\t",row[i]);
	//	printf ("\n");
	//}
	//if (!mysql_eof(res))  {MYSQL_error ();return;}
	

	printf("finish\n\r");
	mysql_free_result(res);
	mysql_close(&mysql);

}
#endif

using namespace std;

int main(int argc, char** argv) {
    if(DEBUG)cout << "Starting program..." << endl;
    
    if ((fd = wiringPiI2CSetup(DEVID)) < 0) {
        if(DEBUG)cout << "Device connection not successfull!" << endl;
        return -1;
    }
    // Read who am i register as a test

    if (DEBUG) cout << "ACC Init. I am: " << wiringPiI2CReadReg8(fd, WHO_AM_I) << ". Should be: " <<  0x71 << endl;

    //Set magnit
    wiringPiI2CWriteReg8(fd, 0x6B, 0);
    wiringPiI2CWriteReg8(fd, 0x6A, 0);
    wiringPiI2CWriteReg8(fd, 0x37, 0x02);

    if ((md = wiringPiI2CSetup(0x0C)) < 0) {
        if(DEBUG)cout << "Device connection not successfull!" << endl;
        return -1;
    }
    else
    {
        if(DEBUG)cout << "MAG Init. I am: " << wiringPiI2CReadReg8(md, 0x01) << ". Should be: " <<  154 << endl;

    	unsigned char varX = wiringPiI2CReadReg8(md, AK8963_ASAX);
    	unsigned char varY = wiringPiI2CReadReg8(md, AK8963_ASAY);
    	unsigned char varZ = wiringPiI2CReadReg8(md, AK8963_ASAZ);

    	magCorrect_ASA[0] = ((((varX-128)*0.5)/128)+1);
    	magCorrect_ASA[1] = ((((varY-128)*0.5)/128)+1);
    	magCorrect_ASA[2] = ((((varZ-128)*0.5)/128)+1);
    }

    if ((bd = wiringPiI2CSetup(0x76)) < 0) { //or 0x77
        if(DEBUG) cout << "Device connection not successfull!" << endl;

        return -1;
    }
    else
    {
        if(DEBUG) cout << "PRE Init. I am: " << wiringPiI2CReadReg8(bd, 0xD0) << ". Should be: " <<  0x58 << endl;

#if USE_MYSQL
        //MYSQL_clear();
#endif

//while(1){
	Get_temp();
	Get_acc();
	Get_gyro();
	Get_mag();

	BMPInit();
	GetBmpTemp();
	GetBmpPress();

#define PI (3.14159265)


	while(1){

		Get_acc();
		Get_gyro();
		Get_mag();



		signed short x = Est_M.x + MagHardCorrect[0];
		signed short y = Est_M.y + MagHardCorrect[1];
		signed short z = Est_M.z + MagHardCorrect[2];

		//float gyroRoll = atan2((double)Est_G.y, (double)Est_G.z);

		#define GYRO_COEFF 0.0 //Coefficient of influence

		double DataRoll_X = (double)(Est_A.y + (Est_G.y*GYRO_COEFF));
		double DataRoll_Y = (double)(Est_A.z + (Est_G.z*GYRO_COEFF));
		float roll = atan2(DataRoll_X, DataRoll_Y);

		double DataPitch_X = (double)(Est_A.x + (Est_G.x*GYRO_COEFF));
		double DataPitch_Y = (double)(Est_A.z + (Est_G.z*GYRO_COEFF));
		float pitch = atan2(DataPitch_X, DataPitch_Y);

		float angel_roll = roll*180/3.14159265;
		float angel_pitch = pitch*180/3.14159265;

		double dataX = (((double)x)*cos(roll)) + (((double)z)*sin(roll));
		double dataY = (((double)y)*cos(pitch)) + (((double)z)*sin(pitch));

		float yaw = atan2(dataX, dataY);
		float angel_yaw = yaw*180/3.14159265;

		if ((angel_pitch<(-90)) || (angel_pitch>90)) {
			if (angel_yaw>=0) angel_yaw = 180 - angel_yaw;
			else angel_yaw = (180 + angel_yaw) * (-1);
		}
		if(0) {
			angel_yaw += -20;
			if (angel_yaw>180)angel_yaw = -180 + (angel_yaw-180);
			else if (angel_yaw<(-180))angel_yaw = 180 + (180 + angel_yaw);

		}


		//float angel1 = atan2((double)x, (double)y)*180/3.14159265;// + 180;
		//float angel2 = atan2((double)x, (double)z)*180/3.14159265;// + 180;
		//float angel3 = atan2((double)z, (double)y)*180/3.14159265;// + 180;

		//printf("GYRO x=%d, y=%d, z=%d,\n\r angel_xy = %.0f, angel_xz = %.0f, angel_zy = %.0f\n\r", Est_G.x, Est_G.y, Est_G.z, angel1, angel2, angel3);

		//printf("roll = %.0f, pitch = %.0f, yaw = %.2f\n\r", angel_roll, angel_pitch, angel_yaw);
		
		char jsonData[1024];
		
		sprintf(jsonData, "{");
		sprintf(jsonData, "%s\"status\":\"success\",", jsonData);
		sprintf(jsonData, "%s\"roll\":\"%0.2f\",", jsonData, angel_roll);
		sprintf(jsonData, "%s\"pitch\":\"%0.2f\",", jsonData, angel_pitch);
		sprintf(jsonData, "%s\"yaw\":\"%0.2f\",", jsonData, angel_yaw);
		sprintf(jsonData, "%s\"press\":\"%d\",", jsonData, BMP_press);
		sprintf(jsonData, "%s\"MPU_temp\":\"%d\",", jsonData, MPU_temp);
		sprintf(jsonData, "%s\"BMP_temp\":\"%d\"", jsonData, BMP_temp);
		sprintf(jsonData, "%s}", jsonData);
		printf("%s", jsonData);
		
		
		//printf("{\"status\":\"success\",\"roll\":\"%0.2f\",\"pitch\":\"%0.2f\",\"yaw\":\"%0.2f\"}",angel_roll, angel_pitch, angel_yaw);
		
		//{"status":"success","result0":"-48","result1":"44","result2":"148.46"}
return 1;
	usleep(500*1000);
	}
	//while(1){
#if USE_MYSQL
	MYSQL_reload();
#endif

//usleep(5000*1000);
//}


	}



    cout << "Terminating..." << endl;
}

