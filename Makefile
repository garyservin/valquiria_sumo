program_NAME := valquiria_sumo

program_INCLUDE_DIRS :=
program_LIBRARY_DIRS :=
program_LIBRARIES := libc18f.lib

CC_FLAGS += $(foreach includedir,$(program_INCLUDE_DIRS),-I$(includedir))
LDFLAGS += $(foreach librarydir,$(program_LIBRARY_DIRS),-L$(librarydir))
LDFLAGS += $(foreach library,$(program_LIBRARIES),-l$(library))

CC=sdcc
FAMILY=pic16
PROC=18f14k50

all: build

build: main.c
	$(CC) $(CC_FLAGS) $(LDFLAGS) --use-non-free -m$(FAMILY) -p$(PROC) -o $(program_NAME) main.c

#main.o: main.c
#	$(CC) $(CC_FLAGS) $(LDFLAGS) --use-non-free -m$(FAMILY) -p$(PROC) -c main.c

clean:
	rm -f *.asm *.cod *.hex *.lst *.o

.PHONY: all clean
