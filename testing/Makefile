CFLAGS=-g -Wall -O2

OBJS=scanread

all: $(OBJS)

sendcmd-ptrace: sendcmd-ptrace.c
	$(CC) $(CFLAGS) $< -o $@ -lrt -L. -lst001_lib

scanread: scanread.c
	$(CC) $(CFLAGS) -Wno-unused-variable -Wno-unused-result $< -o $@ -lrt

clean:
	rm -f $(OBJS) *.o
