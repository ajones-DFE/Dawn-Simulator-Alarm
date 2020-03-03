# "main" pseudo-component makefile

UNPHONE_INC_DIR=../../../../unphone/src
UNPHONE_PARENT_DIR=../../../..
UNPHONE_SRC_DIR=../../../unphone/src
UNPHONE_LIB_DIR=../../../unphone/lib

CPPFLAGS += -I$(UNPHONE_INC_DIR) -I$(UNPHONE_PARENT_DIR) \
  -Wno-all -Wno-extra \
  -Wno-error=return-type -Wno-write-strings -Wno-conversion-null \
  -Wno-return-type -Wno-pointer-arith -Wno-cpp -Wno-unused-variable \
  -DUNPHONE_IDF_COMPILE=1

# library sources
BIGDEMOIDF_LIBS := $(UNPHONE_SRC_DIR) \
  $(UNPHONE_LIB_DIR)/Adafruit-GFX-Library \
  $(UNPHONE_LIB_DIR)/Adafruit_HX8357_Library \
  $(UNPHONE_LIB_DIR)/ESPAsyncWebServer/src \
  $(UNPHONE_LIB_DIR)/AsyncTCP/src/ \
  $(UNPHONE_LIB_DIR)/OneWire \
  $(UNPHONE_LIB_DIR)/ArduinoJson/src \
  $(UNPHONE_LIB_DIR)/WiFiManager \
  $(UNPHONE_LIB_DIR)/Adafruit_STMPE610 \
  $(UNPHONE_LIB_DIR)/Adafruit_LSM303DLHC \
  $(UNPHONE_LIB_DIR)/SD/src \
  $(UNPHONE_LIB_DIR)/Adafruit_Sensor \
  $(UNPHONE_LIB_DIR)/arduino-lmic/src \
  $(UNPHONE_LIB_DIR)/arduino-lmic/src/aes \
  $(UNPHONE_LIB_DIR)/arduino-lmic/src/aes/ideetron \
  $(UNPHONE_LIB_DIR)/arduino-lmic/src/hal \
  $(UNPHONE_LIB_DIR)/arduino-lmic/src/lmic \
  $(UNPHONE_LIB_DIR)/Adafruit_ImageReader \
  $(UNPHONE_LIB_DIR)/RCSwitch \
  $(UNPHONE_LIB_DIR)/DHTesp \
  $(UNPHONE_LIB_DIR)/Adafruit_VS1053_Library \
  $(UNPHONE_LIB_DIR)/Adafruit_NeoMatrix \
  $(UNPHONE_LIB_DIR)/Adafruit_NeoPixel \
  $(UNPHONE_LIB_DIR)/Adafruit_Motor_Shield_V2_Library \
  $(UNPHONE_LIB_DIR)/Adafruit_Motor_Shield_V2_Library/utility \
  $(UNPHONE_LIB_DIR)/GP2Y1010_DustSensor/src \
  $(UNPHONE_LIB_DIR)/ESP32-IRremote \
  $(UNPHONE_LIB_DIR)/Adafruit_TSL2591_Library \
  $(UNPHONE_LIB_DIR)/ArduinoHttpClient/src \
	../lib/NTPClient-master \

COMPONENT_SRCDIRS += $(BIGDEMOIDF_LIBS)
COMPONENT_ADD_INCLUDEDIRS := $(BIGDEMOIDF_LIBS)
