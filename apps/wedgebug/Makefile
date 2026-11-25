# WedgeBug mission app: the navigator in src/wedgebug.c flying on the drone.
#
#   make              build build/cf2.bin (merges app-config into the default CF2 config)
#   make cload        flash it over the Crazyradio (bootloader mode, see README.md)
#
# Out-of-tree app build, as examples/app_hello_world: the firmware Makefile
# does the work and compiles this folder's Kbuild files.
CRAZYFLIE_BASE := ../..

OOT_CONFIG := $(PWD)/app-config

include $(CRAZYFLIE_BASE)/tools/make/oot.mk
