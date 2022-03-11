// test.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include <iostream>
#include <cmath>
#include <cstdio>
#include <vector>
#include <string>

#ifndef PI
#define	PI 3.14159265358979
#endif

#ifndef D2R
#define D2R (PI/180.0)
#endif

#ifndef R2D
#define R2D (180.0/PI)
#endif

#ifndef TWOPI
#define	TWOPI (PI+PI)
#endif

#ifndef ae_WGS84
#define ae_WGS84 6378137.0
#endif

#ifndef finv_WGS84
#define finv_WGS84 298.257223563
#endif

#ifndef grav_WGS84
#define	grav_WGS84 9.7803267714e0
#endif

typedef struct
{
	float timeimu;
	float fxyz[3];
	float wxyz[3];
	float countodr;
	float timeodr;
	float timegps;
	float weeksec;
	double lat;
	double lon;
	float ht;
	float msl;
	float speed;
	float yaw;
	float hdop;
	float pdop;
	float accuracy;
	uint8_t fixtype;
	uint8_t nsat;
	uint8_t pvtupdate; /* set to 1 if there are new GPS, set to 0 after */
}pvt_imu_dat_t;

#define MAXFIELD 100

static int parse_fields(char* const buffer, char** val)
{
	char* p, *q;
	int n = 0;

	/* parse fields */
	for (p = buffer; *p && n < MAXFIELD; p = q + 1) {
		if ((q = strchr(p, ',')) || (q = strchr(p, '*')) || (q = strchr(p, '\n'))) {
			val[n++] = p; *q = '\0';
		}
		else break;
	}
	return n;
}

static int parse_fields_data(char* const buffer, double* data)
{
	char* val[MAXFIELD];
	int n = parse_fields(buffer, val);
	for (int i = 0; i < n; ++i)
		data[i] = atof(val[i]);
	return n;
}

static FILE* set_output_file(const char* fname, const char* key)
{
	char filename[255] = { 0 }, outfilename[255] = { 0 };
	strcpy(filename, fname);
	char* temp = strrchr(filename, '.');
	if (temp) temp[0] = '\0';
	sprintf(outfilename, "%s-%s", filename, key);
	return fopen(outfilename, "w");
}

static void print_log(char** val, int num)
{
	for (int i = 0; i < num; ++i)
		printf("%s%c", val[i], (i + 1) == num ? '\n' : (i + 2) == num ? '*' : ',');
}

static void deg2dms(double deg, double* dms)
{
	double sign = deg < 0.0 ? (-1.0) : (1.0), a = fabs(deg);
	dms[0] = floor(a); a = (a - dms[0]) * 60.0;
	dms[1] = floor(a); a = (a - dms[1]) * 60.0;
	dms[2] = a; dms[0] *= sign;
}
extern int outnmea_gga(unsigned char* buff, float time, int type, double* blh, int ns, float dop, float age)
{
	double h, ep[6], dms1[3], dms2[3];
	char* p = (char*)buff, * q, sum;

	if (type != 1 && type != 4 && type != 5) {
		p += sprintf(p, "$GPGGA,,,,,,,,,,,,,,");
		for (q = (char*)buff + 1, sum = 0; *q; q++) sum ^= *q;
		p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
		return (int)(p - (char*)buff);
	}
	time -= 18.0;
	ep[2] = floor(time / (24 * 3600));
	time -= (float)(ep[2] * 24 * 3600.0);
	ep[3] = floor(time / 3600);
	time -= (float)(ep[3] * 3600);
	ep[4] = floor(time / 60);
	time -= (float)(ep[4] * 60);
	ep[5] = time;
	h = 0.0;
	deg2dms(fabs(blh[0]) * 180 / PI, dms1);
	deg2dms(fabs(blh[1]) * 180 / PI, dms2);
	p += sprintf(p, "$GPGGA,%02.0f%02.0f%05.2f,%02.0f%010.7f,%s,%03.0f%010.7f,%s,%d,%02d,%.1f,%.3f,M,%.3f,M,%.1f,",
		ep[3], ep[4], ep[5], dms1[0], dms1[1] + dms1[2] / 60.0, blh[0] >= 0 ? "N" : "S",
		dms2[0], dms2[1] + dms2[2] / 60.0, blh[1] >= 0 ? "E" : "W", type,
		ns, dop, blh[2] - h, h, age);
	for (q = (char*)buff + 1, sum = 0; *q; q++) sum ^= *q; /* check-sum */
	p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
	return (int)(p - (char*)buff);
}

#ifndef MAX_BUF_LEN
#define MAX_BUF_LEN 1024
#endif

typedef struct
{
	uint8_t dat[MAX_BUF_LEN];
	uint32_t nbyte;
}nmea_buff_t;

static int add_buff(nmea_buff_t* buff, uint8_t data)
{
	int ret = 0;
	if (buff->nbyte == 0)
	{
		if (data == '$')
		{
			buff->dat[buff->nbyte++] = data;
		}
	}
	else
	{
		ret = (data == '\n' && buff->dat[buff->nbyte - 1] == '\r');
		buff->dat[buff->nbyte++] = data;
	}
	return ret;
}

int process_ini_LC79D_v1(const char* fname)
{
	FILE* fGGA = NULL; /* logged LC79 nmea data */
	FILE* fGPS = NULL; /* GPS raw data */
	FILE* fPVT = NULL; /* GPS raw data */
	FILE* fIMU = NULL; /* IMU raw data */
	FILE* fRTS = NULL; /* real time ODR solution */
	FILE* fRTS_CSV = NULL; /* real time ODR solution */

	printf("%s\n", fname);

	char buffer[1024] = { 0 };
	fGGA = fopen(fname, "r"); if (fGGA == NULL) return 0;

	char* val[MAXFIELD];

	pvt_imu_dat_t pvt_imu = { 0 };

	double rate = 0.0;

	double heading = 0.0, speed = 0.0, lat = 0.0, lon = 0.0, ht = 0.0, pdop = 0.0, time_GPS = 0.0, satnum = 0.0;

	double StartTime = 0;
	uint8_t data = 0;
	nmea_buff_t buff = { 0 };
	while (fGGA != NULL && !feof(fGGA))
	{
		if ((data = fgetc(fGGA)) == EOF) break;
		if (!add_buff(&buff, data)) continue;

		memset(buffer, 0, sizeof(buffer));
		memcpy(buffer, buff.dat, sizeof(uint8_t) * buff.nbyte);

		//printf("%s", buffer);
		buff.nbyte = 0;

		if (strlen(buffer) < 1) continue;
		if (buffer[0] != '$') continue;
		int num = parse_fields(buffer, val);

		if (strstr(val[0], "GPGPS") != NULL || strstr(val[0], "PQGPS") != NULL || strstr(val[0], "PQTMGPS") != NULL)
		{
			if (num < 14)
			{
				print_log(val, num);
				continue;
			}
			/* real-time GPS solution */
			//$GPGPS, 474050, 160417.000, 37.412612081, -121.941537261, -25.0000, 3.2031, 0.0000, 0.0000, 5.0000, 2.0156, 2.4488, 1, 12, *40
			pvt_imu.timegps = (float)(atof(val[1])/1000.0); /* IMU time */
			pvt_imu.weeksec = (float)atof(val[2]); /* GPS Time */
			pvt_imu.lat = atof(val[3]) * D2R; /* latitude Î³¶È*/
			pvt_imu.lon = atof(val[4]) * D2R; /* longitude¾­¶È */
			pvt_imu.ht = (float)atof(val[5]); /* ht */
			pvt_imu.msl = (float)atof(val[6]); /* MSL */
			pvt_imu.speed = (float)atof(val[7]); /* speed */
			pvt_imu.yaw = (float)(atof(val[8])*D2R); /* heading */
			pvt_imu.accuracy = (float)atof(val[9]); /* accuracy/sigma */
			if (pvt_imu.accuracy < 1.0f) pvt_imu.accuracy = 1.0f;

			pvt_imu.hdop = (float)atof(val[10]); /* HDOP */
			pvt_imu.pdop = (float)atof(val[11]); /* PDOP */
			pvt_imu.fixtype = (uint8_t)atof(val[12]); /* fixType */
			pvt_imu.nsat = (uint8_t)atof(val[13]); /* sat number */

			if (pvt_imu.yaw < 0.0) pvt_imu.yaw += (float)TWOPI; /* heading */
			pvt_imu.pvtupdate = 1;

			if (pvt_imu.fixtype > 0 && pvt_imu.nsat > 0 && (fabs(pvt_imu.lat) > 1.0e-10 || fabs(pvt_imu.lon) > 1.0e-10 || fabs(pvt_imu.ht) > 1.0e-10))
			{	
				//add_gps_data(gpsdata);
				if (!fGPS) fGPS = set_output_file(fname, "gps.csv");
				if (!fPVT) fPVT = set_output_file(fname, "gps.nmea");

				if (fGPS)
				{
					fprintf(fGPS, "%10.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%3i,%3i,%10.4f\n"
						, pvt_imu.timegps, pvt_imu.lat * 180.0 / PI, pvt_imu.lon * 180.0 / PI, pvt_imu.ht, pvt_imu.speed, rate, pvt_imu.yaw * 180.0 / PI
						, pvt_imu.accuracy, pvt_imu.hdop, pvt_imu.pdop
						, pvt_imu.fixtype, pvt_imu.nsat, pvt_imu.weeksec
					);
				}

				if (fPVT)
				{
					unsigned char ggaBuffer[255] = { 0 };
					double blh[3] = { pvt_imu.lat, pvt_imu.lon, pvt_imu.ht };
					int len = outnmea_gga(ggaBuffer, (float)pvt_imu.timegps, 1, blh, pvt_imu.nsat, (float)pvt_imu.pdop, 0.0);
					if (fPVT) fprintf(fPVT, "%s", ggaBuffer);
				}

			}
			continue;
		}
		//if (strstr(val[0], "GPIMU") != NULL || strstr(val[0], "PQTMIMU") != NULL || strstr(val[0], "GZIMU") != NULL)
		if (strstr(val[0], "PQTMIMUCAL") != NULL || strstr(val[0], "PQTMIMUTYPE") != NULL)
		{
			print_log(val, num);
			continue;
		}
		/*if (strstr(val[0], "GPIMU") != NULL || strstr(val[0], "PQTMIMU") != NULL || strstr(val[0], "GZIMU") != NULL || strstr(val[0], "PQIMU") != NULL )
		{*/
		if (strstr(val[0], "GPIMU") != NULL ||strstr(val[0], "PQTMIMU") != NULL || strstr(val[0], "PQIMU") != NULL)
		{
			if (num < 10)
			{
				print_log(val, num);
				continue;
			}
			/* real - time IMU raw data */
			//$GPIMU, 11977,    0.1747,   -0.0172,    1.0087,    0.4200,   -0.3850,   -0.2450,25.6862,*59
			//$GPIMU, 44188,   -0.0110,   -0.0554,   -1.0544,   -9.6250,    0.2275,   -6.2300,   118, 43982*77
			pvt_imu.timeimu = (float)(atof(val[1])/1000.0); /* IMU time */

			pvt_imu.fxyz[0] = (float)(atof(val[2]) * grav_WGS84); /* fx */
			pvt_imu.fxyz[1] = (float)(atof(val[3]) * grav_WGS84); /* fy */
			pvt_imu.fxyz[2] = (float)(atof(val[4]) * grav_WGS84); /* fz */
			pvt_imu.wxyz[0] = (float)(atof(val[5]) * PI / 180.0); /* wx */
			pvt_imu.wxyz[1] = (float)(atof(val[6]) * PI / 180.0); /* wy */
			pvt_imu.wxyz[2] = (float)(atof(val[7]) * PI / 180.0); /* wz */

			/* old format is temp, new format is ODR couter and timer */
			pvt_imu.countodr = (float)(atof(val[8])); /* ODR counter */
			pvt_imu.timeodr= (float)(atof(val[9])); /* ODR timer */

			/* 0: imu filter, Roll & Pitch only */
			if (fIMU) fprintf(
				fIMU, "%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f\n"
				, pvt_imu.timeimu, pvt_imu.fxyz[0], pvt_imu.fxyz[1], pvt_imu.fxyz[2], pvt_imu.wxyz[0], pvt_imu.wxyz[1], pvt_imu.wxyz[2]
			);

			continue;
		}
		if (strstr(val[0], "GPINS") != NULL || strstr(val[0], "PQTMINS") != NULL || strstr(val[0], "PQINS") != NULL)
		{
			if (num < 12)
			{
				print_log(val, num);
				continue;
			}
			/* real-time INS solution */
			// $GPINS,timeIMU,type,pos,vel,att
			float timeIMU = (float)(atof(val[1]) / 1000.0); /* IMU time */
			int type = atoi(val[2]);
			double pos[3] = { atof(val[3]) * PI / 180.0, atof(val[4]) * PI / 180.0, atof(val[5])}; /* deg, deg, m */
			double vel[3] = { atof(val[6]), atof(val[7]), atof(val[8]) }; /* m/s, m/s, m/s */
			double att[3] = { atof(val[9]), atof(val[10]), atof(val[11]) }; /* deg, deg, deg */

			if (type > 0 && fabs(pos[0])>1.0e-10 && fabs(pos[1]) > 1.0e-10)
			{
				if (!fRTS) fRTS = set_output_file(fname, "rts.nmea");
				if (!fRTS_CSV) fRTS_CSV = set_output_file(fname, "rts.csv");

				if (fRTS_CSV) fprintf(fRTS_CSV, "%10.3f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%i,%7.5f\n",
					timeIMU, pos[0], pos[1], pos[2], vel[0], vel[1], vel[2], att[0], att[1], att[2], type, timeIMU-pvt_imu.timeimu);

				unsigned char ggaBuffer[255] = { 0 };
				int len = outnmea_gga(ggaBuffer, timeIMU, 5, pos, 10, 1.0, 1.0);
				if (fRTS) fprintf(fRTS, "%s", ggaBuffer);
			}

			continue;
		}
	}

	if (fGGA) fclose(fGGA);
	if (fGPS) fclose(fGPS);
	if (fPVT) fclose(fPVT);
	if (fIMU) fclose(fIMU);
	if (fRTS) fclose(fRTS);
	if (fRTS_CSV) fclose(fRTS_CSV);

	return 1;
}
 
int main(int argc, char** argv)
{
	if (argc < 2)
	{
		char buffer[255] = { 0 }, dirname[255] = { 0 }, imufname[255] = { 0 };
		FILE* fINI = fopen("data.ini", "r");
		unsigned long line = 0;
		char axis[255] = { 0 };
		char dname[255] = { 0 };
		while (fINI != NULL && !feof(fINI))
		{
			fgets(buffer, sizeof(buffer), fINI);
			if (strlen(buffer) < 1) continue;
			if (buffer[0] == ';' || buffer[0] == '#') continue;
			char* temp = strchr(buffer, '\n');
			if (temp != NULL) temp[0] = '\0';

			int type = 0;
			char fname[255] = { 0 };
			int num = sscanf(buffer, "%i,%[^\,]", &type, fname);
			if (num < 2) continue;
			switch (type) {
			case 0: /* data directory */
			{
				if (strlen(fname) > 0)
				{
					memset(dname, 0, sizeof(dname));
					strcpy(dname, fname);
					if (dname[strlen(dname) - 1] != '\\' && dname[strlen(dname) - 1] != '/')
						dname[strlen(dname)] = '\\';
				}
			} break;
			case 1: /* GPS+IMU with LC79D data */
			{
				if (strlen(fname) > 0)
				{
					char filename[255] = { 0 };
					if (fname[0] == '\\' || fname[0] == '/')
						sprintf(filename, "%s%s", dname, fname + 1);
					else
						sprintf(filename, "%s%s", dname, fname);
					process_ini_LC79D_v1(filename);
				}
			} break;
			case 2: /* axis defintion */
			{
				strcpy(axis, fname);
			} break;
			default:
				break;
			}
			++line;
		}
		if (fINI) fclose(fINI);
	}
	else
	{
		process_ini_LC79D_v1(argv[1]);
	}
	return 0;
}