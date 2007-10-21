PREFIX=/usr/local
DEVICE=/dev/ttyS0

CC=gcc
#CFLAGS=-O2 -Wall -Wextra
CFLAGS=-g -Wall -Wextra
INSTALL=install

SRCS=garmini.c garmin.c
OBJS=$(SRCS:%.c=%.o)
BINS=garmini

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
	@$(INSTALL) -D -m 755 garmini $(PREFIX)/bin/garmini

garmini: $(OBJS)

clean:
	@echo "  CLEAN   $(BINS) $(OBJS)"
	@rm -f $(BINS) $(OBJS)

%.o: %.c
	@echo "  CC      $<"
	@$(CC) -c -o $@ $(CFLAGS) $<

%: %.o
	@echo "  LD      $<"
	@$(CC) -o $@ $(CFLAGS) $^
