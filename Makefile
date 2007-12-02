PREFIX=/usr/local
DEVICE=/dev/ttyS0

CC=gcc
CFLAGS=-O2 -Wall -DDEVICE=\"$(DEVICE)\"

SRCS=garmini.c garmin.c
OBJS=$(SRCS:%.c=%.o)
BINS=garmini
LIBS=-lm

.PHONY: all clean setgidinstall install tarball

all: $(BINS)

tarball:
	mkdir garmini-$(VERSION)
	cp Makefile $(SRCS) garmini-$(VERSION)
	tar -czf garmini-$(VERSION).tar.gz garmini-$(VERSION)
	rm -Rf garmini-$(VERSION)

setgidinstall: install
	@echo "  CHGRP   garmini"
	@chgrp --reference=$(DEVICE) $(PREFIX)/bin/garmini
	@echo "  CHMOD   garmini"
	@chmod g+s $(PREFIX)/bin/garmini

install: $(BINS)
	@echo "  INSTALL garmini"
	@mkdir -p $(PREFIX)/bin
	@cp garmini $(PREFIX)/bin

garmini: $(OBJS)

clean:
	@echo "  CLEAN   $(BINS) $(OBJS)"
	@rm -f $(BINS) $(OBJS)

%.o: %.c
	@echo "  CC      $<"
	@$(CC) -c -o $@ $(CFLAGS) $<

%: %.o
	@echo "  LD      $<"
	@$(CC) -o $@ $(CFLAGS) $^ $(LIBS)
