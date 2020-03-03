# ESP IDF project Makefile for MyProjectThing

# basic config: CHANGE THESE for new projects
PROJECT_NAME := MyProjectThing
UNPHONE_DIR=$(shell cd $(CURDIR)/../../unphone && pwd)

# firmware versioning; device port
FIRMWARE_VERSION := $(shell grep 'int firmwareVersion =' main/main.cpp \
  |sed -e 's,[^0-9]*,,' -e 's,;.*$$,,')
ESPPORT ?= $(shell for p in ttyUSB0 ttyUSB1 cu.SLAB_USBtoUART; do \
  [ -r /dev/$${p} ] && echo /dev/$${p}; done)

# tone down compile warnings; make sure we see the correct pin defs header
CPPFLAGS += -Wno-all \
-I $(UNPHONE_DIR)/sdks/arduino-esp32/variants/feather_esp32 \
-Wno-error=return-type -Wno-write-strings \
-Wno-return-type -Wno-pointer-arith -Wno-cpp -Wno-unused-variable \
-Wno-missing-field-initializers

# pull in the arduino esp32 core IDF component
EXTRA_COMPONENT_DIRS += $(UNPHONE_DIR)/sdks/arduino-esp32

# burn and listen
default: flash monitor

# IDF targets and settings
include $(IDF_PATH)/make/project.mk

# push the firware for OTA
push-firmware: app
	@echo firmware version is $(FIRMWARE_VERSION)
	cp build/$(PROJECT_NAME).bin firmware/$(FIRMWARE_VERSION).bin
	git add -v firmware/$(FIRMWARE_VERSION).bin
	echo $(FIRMWARE_VERSION) >firmware/version
	git commit -vm 'added OTA firmware version $(FIRMWARE_VERSION)' \
          firmware/$(FIRMWARE_VERSION).bin firmware/version
	git push
