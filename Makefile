BOARD=stm32f103_bluepill

include tinyusb/examples/build_system/make/make.mk

INC += \
  src \
  $(TOP)/hw \

# Example source
EXAMPLE_SOURCE += $(wildcard src/*.c)
SRC_C += $(addprefix $(CURRENT_PATH)/, $(EXAMPLE_SOURCE))

include tinyusb/examples/build_system/make/rules.mk
