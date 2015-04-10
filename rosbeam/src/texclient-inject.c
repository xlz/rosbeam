#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <dlfcn.h>
#include <sys/prctl.h>
#include <pthread.h>

#include "drive_command.h"

#define PROG "inject.so: "
#define DEVICE_OPEN_FLAGS 0x902
#define DEVICE_PATH "/dev/st001"
#define WATCHDOG_PATH "/proc/net/ipt_hashlimit/watchdog"

#define docrc _ZN5bacon5st00112Communicator3crcEPKcm
#define docrc_str "_ZN5bacon5st00112Communicator3crcEPKcm"
#define readPacket _ZN5bacon5st00112Communicator10readPacketEPcmjRKNS0_7TimeoutEb
#define readPacket_str "_ZN5bacon5st00112Communicator10readPacketEPcmjRKNS0_7TimeoutEb"

static uint32_t (*_docrc)(const void *buf, size_t len);

static int device_fd = -1;
static int watchdog_fd = -1;
static struct drive_shm *shm;

static uint64_t limiter_tag;

static int timespec_ge(const struct timespec *a, const struct timespec *b)
{
	return a->tv_sec > b->tv_sec || (a->tv_sec == b->tv_sec && a->tv_nsec > b->tv_nsec);
}
static int timespec_eq(const struct timespec *a, const struct timespec *b)
{
	return a->tv_sec == b->tv_sec && a->tv_nsec == b->tv_nsec;
}
//static uint64_t timespec_diff_msec(const struct timespec *a, const struct timespec *b)
//{
//	return (b->tv_sec - a->tv_sec) * 1000UL + b->tv_nsec / 1000000UL - a->tv_nsec / 1000000UL;
//}

static int watchdog_active()
{
	return 1;

	if (watchdog_fd < 0)
		return 0;

	ssize_t n;
	char buf[1024];
	n = pread(watchdog_fd, buf, sizeof(buf), 0);
	return n > 0;
}

static const void *inject_drive_command(const void *buf)
{
	const int RESET_TIMES = 10;
	//static struct timespec latest_tag_time;
	//static uint64_t latest_tag;
	static int reset_counter;
	struct timespec now;
	//uint64_t tag;

	if (!shm)
		return buf;

	if (!watchdog_active())
		return buf;

	clock_gettime(CLOCK_MONOTONIC, &now);

	//tag = ((const struct drive_command *)buf)->tag;
	//if (latest_tag < tag) {
	//	latest_tag = tag;
	//	latest_tag_time = now;
	//	return buf;
	//}

	static struct drive_command cmd;
	static struct timespec last_deadline;
	struct timespec deadline;

	pthread_mutex_lock(&shm->cmd_lock);

	deadline = shm->cmd_time;
	if (!timespec_eq(&deadline, &last_deadline)) {
		last_deadline = deadline;
		cmd = shm->cmd;
		reset_counter = RESET_TIMES;
	}

	pthread_mutex_unlock(&shm->cmd_lock);

	if (timespec_ge(&now, &deadline)) {
		if (reset_counter <= 0)
			return buf;
		cmd.lin_vel = 0;
		cmd.ang_vel = 0;
		cmd.lin_acc = 0;
		cmd.ang_acc = 0;
		cmd.lin_lim = 0;
		cmd.ang_lim = 0;
		reset_counter--;
	}

	//uint64_t msecs = timespec_diff_msec(&latest_tag_time, &now);
	//cmd.tag = ((latest_tag >> 33) + msecs) << 33;
	cmd.tag = limiter_tag;
	cmd.crc = _docrc(&cmd, sizeof(cmd) - sizeof(cmd.crc));
	return &cmd;
}

static void post_drive_status(const struct drive_status *stat)
{
	if (!shm)
		return;

	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	limiter_tag = stat->velocityLimiterTag;

	pthread_mutex_lock(&shm->stat_lock);
	shm->stat_time = now;
	shm->stat = *stat;
	pthread_cond_signal(&shm->stat_cond);
	pthread_mutex_unlock(&shm->stat_lock);
}

static int is_drive_command(int fd, const void *buf, size_t count)
{
	const struct drive_command *cmd = buf;
	return device_fd >= 0 && fd == device_fd &&
		count == sizeof(struct drive_command) &&
		cmd->magic == DRIVE_COMMAND_MAGIC && cmd->mode == 1;
}

static int is_drive_status(const void *buf, size_t count)
{
	const struct drive_status *stat = buf;
	return device_fd >= 0 &&
		count == sizeof(struct drive_status) &&
		stat->magic == DRIVE_STATUS_MAGIC;
}

#define WRAP(type,name,list) \
	static type (*_##name)list; \
	type name list __attribute__ ((visibility ("default"))); \
	type name list

#define WRAPINIT(name) \
	if (!_##name) _##name = dlsym(RTLD_NEXT, #name); \
	if (!_##name) { perror(PROG "dlsym"); exit(1); }

#define WRAPINIT_NAME(name, str) \
	if (!_##name) _##name = dlsym(RTLD_NEXT, str); \
	if (!_##name) { perror(PROG "dlsym"); exit(1); }

WRAP(ssize_t, write, (int fd, const void *buf, size_t count))
{
	WRAPINIT(write)

	if (is_drive_command(fd, buf, count))
		buf = inject_drive_command(buf);

	return _write(fd, buf, count);
}

WRAP(int64_t, readPacket, (void *_this, void *buf, size_t count, unsigned magic, void *timeout, char checkcrc))
{
	WRAPINIT_NAME(readPacket, readPacket_str)

	int64_t ret = _readPacket(_this, buf, count, magic, timeout, checkcrc);

	if (ret && is_drive_status(buf, count))
		post_drive_status(buf);

	return ret;
}

static void setup();

WRAP(int, open, (const char *pathname, int flags, ...))
{
	WRAPINIT(open)

	va_list ap;
	va_start(ap, flags);
	mode_t mode = va_arg(ap, mode_t);
	va_end(ap);

	int fd = _open(pathname, flags, mode);
	if (flags == DEVICE_OPEN_FLAGS && fd >= 0 && strcmp(pathname, DEVICE_PATH) == 0 && device_fd < 0) {
		device_fd = fd;
		setup();
	}

	return fd;
}

static void setup()
{
	WRAPINIT_NAME(docrc, docrc_str)

	int fd = shm_open(DRIVE_SHM_NAME, O_RDWR | O_CREAT | O_TRUNC | O_NOCTTY, 0666);
	if (fd < 0) {
		perror(PROG "shm_open");
		return;
	}

	if (ftruncate(fd, sizeof(struct drive_shm)) < 0) {
		perror(PROG "ftruncate");
		goto closefd;
	}

	shm = mmap(NULL, sizeof(struct drive_shm), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (shm == MAP_FAILED) {
		shm = NULL;
		perror(PROG "mmap");
		goto closefd;
	}

	pthread_mutexattr_t mutex_attr;
	pthread_mutexattr_init(&mutex_attr);
	pthread_mutexattr_setpshared(&mutex_attr, PTHREAD_PROCESS_SHARED);
	pthread_mutex_init(&shm->cmd_lock, &mutex_attr);
	pthread_mutex_init(&shm->stat_lock, &mutex_attr);

	pthread_condattr_t cond_attr;
	pthread_condattr_init(&cond_attr);
	pthread_condattr_setpshared(&cond_attr, PTHREAD_PROCESS_SHARED);
	pthread_cond_init(&shm->stat_cond, &cond_attr);

closefd:
	if (close(fd) < 0)
		perror(PROG "close");

	if (prctl(PR_SET_NAME, (unsigned long)"st001thread", 0, 0, 0) < 0)
		perror(PROG "PR_SET_NAME");

	watchdog_fd = _open(WATCHDOG_PATH, O_RDONLY);
        if (watchdog_fd < 0) {
		perror("open watchdog");
	}
}

__asm__(".symver open, open@GLIBC_2.2.5");
__asm__(".symver write, write@GLIBC_2.2.5");
