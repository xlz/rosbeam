#ifndef DRIVE_COMMAND_H
#define DRIVE_COMMAND_H
#include <stdint.h>
#include <pthread.h>
#include <time.h>

#define DRIVE_COMMAND_MAGIC 0x5555aaaa
#define DRIVE_STATUS_MAGIC 0xcccc3333

struct drive_command
{
	uint32_t magic;
	uint32_t mode; // 1
	int32_t lin_vel; // /65536.0
	int32_t ang_vel; // /65536.0
	uint32_t type; // 0
	uint64_t tag; // filled in
	uint32_t lin_lim; // 1.5 /65536.0
	uint32_t ang_lim; // 1.5 /65536.0
	uint32_t lin_acc; // 2.0 /65536.0
	uint32_t ang_acc; // 0.5 /65536.0
	uint32_t crc; // filled in
} __attribute__((packed));

struct drive_status
{
	uint32_t magic;
	uint32_t flags; // m_en=f&1; b_en=f&2; cing=(f>>8)&0x40; cdet=(f>>8)&0x80; fault=f&0xff000000;
	int32_t speed_1;
	int32_t speed_2;
	int32_t current_1; // /1258.2912
	int32_t current_2; // /1258.2912
	int32_t temperature;
	int32_t batteryVoltage; // /65536.0
	int32_t batteryCurrent; // /65536.0
	int32_t chargeVoltage; // /65536.0
	int32_t chargeCurrent; // /65536.0
	int32_t accel_1;
	int32_t accel_2;
	int32_t accel_3;
	int32_t rotation_1;
	int32_t rotation_2;
	int32_t capacity; // /65536.0
	uint32_t timeStamp; // /1e6
	int32_t encoder_1; //lin_enc = (e1 - e2)*0.0040578907f
	int32_t encoder_2; //ang_enc = -(e1 + e2)*0.019415744f
	int32_t capacityMax; // /65536.0
	uint64_t velocityLimiterTag; // (tag >> 32) / 2e3
	int32_t actualLinearVelocity; // /65536.0
	int32_t actualAngularVelocity; // /65336.0
	int32_t limitedLinearVelocity; // /65536.0
	int32_t limitedAngularVelocity; // /65536.0
	int32_t linearVelocityLimit; // /65536.0
	int32_t angularVelocityLimit; // /65536.0
	int16_t drivingCommandLatency; // /1000.0
	int16_t limiterLatency; // /1000.0
	int32_t integratedYaw;
	uint32_t crc;
} __attribute__((packed));

#define DRIVE_SHM_NAME "/st001threaddata"

struct drive_shm {
	pthread_mutex_t cmd_lock;
	struct timespec cmd_time;
	struct drive_command cmd;

	pthread_mutex_t stat_lock;
	pthread_cond_t stat_cond;
	struct timespec stat_time;
	struct drive_status stat;
};

#endif
