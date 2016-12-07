CFLAGS=
BINARY=sernet
SOURCES=sernet.c
LDFLAGS+=-lpthread

all: $(SOURCES)
	$(CC) -o $(BINARY) $(SOURCES) $(LDFLAGS) $(CFLAGS)

config:
	@echo CFLAGS: $(CFLAGS)
	@echo CC: $(CC)
	@echo LIB: $(LIB)

.PHONY: config
