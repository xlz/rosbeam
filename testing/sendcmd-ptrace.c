#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
//#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <errno.h>

#include <sys/ptrace.h>
#include <sys/user.h>
#include <sys/wait.h>
#include <sys/syscall.h>
#include <sys/uio.h>

#include "drive_command.h"

#define docrc _ZN5bacon5st00112Communicator3crcEPKcm
extern uint32_t _ZN5bacon5st00112Communicator3crcEPKcm(const void *buf, unsigned long len);

static int st001_fd;
static struct drive_command cmdbuf;
static struct timespec deadline;

static int timespec_leq(const struct timespec *a, const struct timespec *b)
{
	return a->tv_sec < b->tv_sec || (a->tv_sec == b->tv_sec && a->tv_nsec <= b->tv_nsec);
}

static uint64_t timespec_diff_msec(const struct timespec *a, const struct timespec *b)
{
	return (b->tv_sec - a->tv_sec) * 1000UL + b->tv_nsec / 1000000UL - a->tv_nsec / 1000000UL;
}

static ssize_t process_vm_read(pid_t pid, void *dest, void *src, size_t n)
{
	struct iovec local = {dest, n};
	struct iovec remote = {src, n};
	return process_vm_readv(pid, &local, 1, &remote, 1, 0);
}

static ssize_t process_vm_write(pid_t pid, void *dest, void *src, size_t n)
{
	struct iovec local = {src, n};
	struct iovec remote = {dest, n};
	return process_vm_writev(pid, &local, 1, &remote, 1, 0);
}

static struct drive_command *prepare_drive_command(const struct drive_command *cmd)
{
	static struct timespec last_tag_time;
	static uint64_t last_tag;

	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);

	if (last_tag != cmd->tag) {
		last_tag = cmd->tag;
		last_tag_time = now;
		return NULL;
	}

	uint64_t msecs = timespec_diff_msec(&last_tag_time, &now);

	cmdbuf.tag = ((last_tag >> 33) + msecs) << 33;
	cmdbuf.crc = docrc(&cmdbuf, sizeof(cmdbuf) - sizeof(cmdbuf.crc));
	last_tag = cmdbuf.tag;

	return &cmdbuf;
}

static void fprint_drive_command(FILE *f, const char *msg, const struct drive_command *cmd)
{
	fprintf(stderr, "%s tag=%.3f vel=%.3f,%.3f acc=%.1f,%.1f lim=%.1f,%.1f\n",
		msg, (cmd->tag >> 33) / 1e3, cmd->lin_vel / 65536., cmd->ang_vel / 65536.,
		cmd->lin_acc / 65536., cmd->ang_acc / 65536., cmd->lin_lim / 65536., cmd->ang_lim / 65536.);
}

static void on_write(pid_t pid, int fd, void *buf, size_t count)
{
	if (fd != st001_fd || count != sizeof(struct drive_command))
		return;

	struct drive_command cmd;

	if (process_vm_read(pid, &cmd, buf, sizeof(cmd)) < 0) {
		perror("process_vm_readv");
		return;
	}

	if (cmd.mode != 1 || cmd.header != DRIVE_COMMAND_HEADER)
		return;

	fprint_drive_command(stderr, "intercepted", &cmd);

	struct drive_command *newcmd = prepare_drive_command(&cmd);
	if (!newcmd)
		return;

	if (process_vm_write(pid, buf, newcmd, sizeof(*newcmd)) < 0) {
		perror("process_vm_writev");
		return;
	}

	fprint_drive_command(stderr, "injecting  ", newcmd);
}

int wait_for_syscall(pid_t child)
{
	int status;
	for (;;) {
		if (ptrace(PTRACE_SYSCALL, child, 0, 0) < 0) {
			perror("PTRACE_SYSCALL");
			return -1;
		}

		if (wait(&status) < 0) {
			perror("wait");
			return -1;
		}

		if (WIFSTOPPED(status))
			return 0;
		if (WIFEXITED(status))
			return -1;
		fprintf(stderr, "[not stopped]\n");
	}
}

void do_trace(pid_t pid, const struct timespec *deadline)
{
	int status, syscall;
	int entering = 1;

	if (ptrace(PTRACE_ATTACH, pid, 0, 0) < 0) {
		perror("PTRACE_ATTACH");
		return;
	}

	if (wait(&status) < 0) {
		perror("wait");
		return;
	}
	fprintf(stderr, "Attached pid %d\n", pid);

	if (ptrace(PTRACE_SETOPTIONS, pid, 0, PTRACE_O_TRACESYSGOOD) < 0) {
		perror("PTRACE_O_TRACESYSGOOD");
		return;
	}

	struct timespec now = {0};
	for (; timespec_leq(&now, deadline); clock_gettime(CLOCK_MONOTONIC, &now)) {
		if (wait_for_syscall(pid) < 0)
			break;

		struct user_regs_struct regs = {0};

		if (ptrace(PTRACE_GETREGS, pid, 0, &regs) < 0)
			perror("PTRACE_GETREGS");

		syscall = regs.orig_rax;
		if (syscall != SYS_write)
			continue;

		if (!entering) {
			entering = 1;
			continue;
		}
		entering = 0;

		on_write(pid, regs.rdi, (void *)regs.rsi, regs.rdx);
	}
}

int main(int argc, char **argv)
{
	const char *pidstr = getenv("PID");
	const char *fdstr = getenv("FD");
	if ((argc != 3 && argc != 4) || !pidstr || !fdstr || !strlen(pidstr) || !strlen(fdstr)) {
		fprintf(stderr, "usage: PID=pid FD=fd %s lin_vel ang_vel [timeout=1]\n", argv[0]);
		return 1;
	}

	cmdbuf.header = DRIVE_COMMAND_HEADER;
	cmdbuf.mode = 1;
	cmdbuf.lin_vel = atof(argv[1]) * 65536;
	cmdbuf.ang_vel = atof(argv[2]) * 65536;
	cmdbuf.type = 0;
	cmdbuf.tag = -1;
	cmdbuf.lin_lim = 1.5 * 65536;
	cmdbuf.ang_lim = 1.5 * 65536;
	cmdbuf.lin_acc = 2.0 * 65536;
	cmdbuf.ang_acc = 0.5 * 65536;
	cmdbuf.crc = -1;
	
	st001_fd = atoi(fdstr);

	int timeout = (argc == 4 ? atoi(argv[3]) : 1);
	clock_gettime(CLOCK_MONOTONIC, &deadline);
	deadline.tv_sec += timeout;

	do_trace(atoi(pidstr), &deadline);

	return 0;
}
