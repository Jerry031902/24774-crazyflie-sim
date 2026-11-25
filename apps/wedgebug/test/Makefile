# Host test of the WedgeBug flight app (see test_app.c). Plain gcc, no ARM toolchain:
#   make -C apps/wedgebug/test           build and run every scenario
#   make -C apps/wedgebug/test build     build only
REPO := ../../..
CFLAGS := -std=gnu11 -O2 -g -Wall -Wextra -Werror -Wno-unused-parameter -Wno-missing-field-initializers
INCLUDES := -Istubs -I../src -I$(REPO)/src/modules/interface -I$(REPO)/src/hal/interface \
            -I$(REPO)/src/utils/interface/lighthouse -I$(REPO)/src/utils/interface
SOURCES := test_app.c ../src/wedgebug_app.c ../src/wedgebug.c $(REPO)/src/modules/src/range.c
HEADERS := $(wildcard stubs/*.h) ../src/wedgebug.h $(REPO)/src/modules/interface/range.h

.PHONY: all build clean
all: build
	./build/test_app

build: build/test_app

build/test_app: $(SOURCES) $(HEADERS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(SOURCES) -lm

clean:
	rm -rf build
