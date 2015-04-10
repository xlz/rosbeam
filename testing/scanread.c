#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include "drive_command.h"

void print_command()
{
	struct drive_command cmd;
	char *line = (char *)&cmd;
	int i;
	for (i = 0; i < 48; i++) {
		scanf("\\x%2hhx", &line[i]);
	}
	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	double time = now.tv_sec + now.tv_nsec / 1e9;
	const double d = 65536;
	double tag = (cmd.tag >> 32) / 2e3;
	printf("tag=%.3f mode/?=%u/%x vel=%.2f %.2f acc=%.2f %.2f lim=%.2f %.2f\n", tag, cmd.mode, cmd.type, cmd.lin_vel/d, cmd.ang_vel/d, cmd.lin_acc/d, cmd.ang_acc/d, cmd.lin_lim/d, cmd.ang_lim/d);
}

void print_status()
{
	int i;
	unsigned char line[128];
	for (i = 0; i < 128; i++) {
		scanf("\\x%2hhx", &line[i]);
	}
	struct drive_status *s = (struct drive_status *)line;
	unsigned m_en = s->flags & 1;
	unsigned b_en = !!(s->flags & 2);
	unsigned cing = !!((s->flags >> 8) & 0x40);
	unsigned cdet = !!((s->flags >> 8) & 0x80);
	unsigned fault = !!(s->flags & 0xff000000);
	int32_t s1 = s->speed_1;
	int32_t s2 = s->speed_2;
	double I_1 = s->current_1 / 1258.2912;
	double I_2 = s->current_2 / 1258.2912;
	int32_t T = s->temperature;
	double battV = s->batteryVoltage / 65536.0;
	double battI = s->batteryCurrent / 65536.0;
	double chV = s->chargeVoltage / 65536.0;
	double chI = s->chargeCurrent / 65536.0;
	int32_t a1 = s->accel_1;
	int32_t a2 = s->accel_2;
	int32_t a3 = s->accel_3;
	int32_t r1 = s->rotation_1;
	int32_t r2 = s->rotation_2;
	double cap = s->capacity / 65536.0;
	double ts = s->timeStamp / 1e6;
	int32_t e1 = s->encoder_1;
	int32_t e2 = s->encoder_2;
	float lin_enc = (s->encoder_1 - s->encoder_2) * 0.0040578907f;
	float ang_enc = -(s->encoder_2 + s->encoder_1) * 0.019415744f;
	double capmax = s->capacityMax / 65536.0;
	double tag = (s->velocityLimiterTag >> 32) / 2e3;
	double actualLinearVelocity = s->actualLinearVelocity / 65536.0;
	double actualAngularVelocity = s->actualAngularVelocity / 65536.0;
	double limitedLinearVelocity = s->limitedLinearVelocity / 65536.0;
	double limitedAngularVelocity = s->limitedAngularVelocity / 65536.0;
	double linearVelocityLimit = s->linearVelocityLimit / 65536.0;
	double angularVelocityLimit = s->angularVelocityLimit / 65536.0;

	double drivingCommandLatency = s->drivingCommandLatency / 1000.0;
	double limiterLatency = s->limiterLatency / 1000.0;
	int32_t yaw = s->integratedYaw;

	printf("tag=%.3f lin_enc=%.3f ang_enc=%.3f avel=%.2f %.2f lvel=%.2f %.2f lim=%.2f %.2f latency: %.3f %.3f\n",
		tag, lin_enc, ang_enc,
		actualLinearVelocity, actualAngularVelocity,
		limitedLinearVelocity, limitedAngularVelocity,
		linearVelocityLimit, angularVelocityLimit,
		drivingCommandLatency, limiterLatency);
}
double now() {
	struct timespec n;
	clock_gettime(CLOCK_MONOTONIC, &n);
	return n.tv_sec + n.tv_nsec/1e9;
}

int main() {
	double start = now();
	setvbuf(stdout, NULL, _IONBF, 0);
	while (!feof(stdin) && !ferror(stdin)) {
		char callname[16];
		scanf("%[^(](%*[^\"]\"", callname);
		printf("%8.3f ", now() - start);
		if (strcmp(callname, "read") == 0) {
			print_status();
		} else if (strcmp(callname, "write") == 0) {
			print_command();
		}
		scanf("%*[^\n]\n");
	}
	return 0;
}
