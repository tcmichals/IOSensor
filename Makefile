

#general build targets
TARGET ?= SPRACINGF4EVO
# SPRACINGF4EVO
# STM32F407VET6_BLUE_BOARD

DIR := ${CURDIR}

DEBUG ?= GDB
TARGET_FLAGS= -DSTM32F407xx
OBJECT_DIR = obj
V ?=1

#need protoc file
REQUIRED_BINS := protoc python3
$(foreach bin,$(REQUIRED_BINS),\
    $(if $(shell command -v $(bin) 2> /dev/null),$(info Found `$(bin)`),$(error Please install `$(bin)`)))

#compiler
CC := $(CCACHE) arm-none-eabi-gcc
CXX := arm-none-eabi-g++
OBJCOPY := arm-none-eabi-objcopy
SIZE := arm-none-eabi-size


STM32_home_dir=/media/tcmichals/workingDiskHS/STM32
FreeRTOS_home_dir=FreeRTOS



# *************** STM32 Cube directories *************************

STM32Cube_F4_VERSION = V1.19.0
STM32Cube_F4_DIR = $(STM32_home_dir)/STM32Cube_FW_F4_$(STM32Cube_F4_VERSION)
STM32Cube_F4_DRIVER_DIR = $(STM32Cube_F4_DIR)/Drivers/STM32F4xx_HAL_Driver
STM32Cube_F4_DRIVER_INC_DIR = $(STM32Cube_F4_DRIVER_DIR)/Inc
STM32Cube_F4_DRIVER_SRC_DIR= $(STM32Cube_F4_DRIVER_DIR)/Src
STM32Cube_F4_DRIVER_SRC =  $(filter-out $(wildcard  $(STM32Cube_F4_DRIVER_SRC_DIR)/*template*.c), $(wildcard $(STM32Cube_F4_DRIVER_SRC_DIR)/*.c))

STM32Cube_F4_CMSIS_DIR = $(STM32Cube_F4_DIR)/Drivers/CMSIS
STM32Cube_F4_CMSIS_INC_DIR = $(STM32Cube_F4_CMSIS_DIR)/Include

STM32Cube_F4_CMSIS_DEVICE_DIR = $(STM32Cube_F4_DIR)/Drivers/CMSIS/Device/ST/STM32F4xx
STM32Cube_F4_CMSIS_DEVICE_INC_DIR = $(STM32Cube_F4_CMSIS_DEVICE_DIR)/Include

CMSIS_FREERTOS_DIR = $(STM32Cube_F4_DIR)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
CMSIS_FREERTOS_SRC = $(wildcard $(CMSIS_FREERTOS_DIR)/*.c) 
CMSIS_FREERTOS_INC= $(CMSIS_FREERTOS_DIR)

USB_DEVICE_DIR= $(STM32Cube_F4_DIR)/Middlewares/ST/STM32_USB_Device_Library
USB_CORE_SRC =  $(filter-out $(wildcard  $(USB_DEVICE_DIR)/Core/Src/*template*.c), $(wildcard $(USB_DEVICE_DIR)/Core/Src/*.c))
USB_CORE_INC =  $(USB_DEVICE_DIR)/Core/Inc

# **************  protoThreads  *********************************
PROTOTHREADS_VERSION=1.4
PROTOTHREADS_SRC_DIR= depend/protoThreads/pt-$(PROTOTHREADS_VERSION)
PROTOTHREADS_INC= $(PROTOTHREADS_SRC_DIR)


# **************  FreeRTOS directories *********************************
FREERTOS_VERSION= v10.0.0
FREERTOS_SRC_DIR= $(FreeRTOS_home_dir)/FreeRTOS$(FREERTOS_VERSION)/FreeRTOS/Source
FREERTOS_RTOS_SRC = $(wildcard $(FREERTOS_SRC_DIR)/*.c)
FREERTOS_RTOS_INC= $(FREERTOS_SRC_DIR)/include
FREERTOS_PORTABLE_DIR = $(FREERTOS_SRC_DIR)/portable/GCC/ARM_CM4F
FREERTOS_PORTABLE_SRC = $(wildcard $(FREERTOS_PORTABLE_DIR)/*.c) $(FREERTOS_SRC_DIR)/portable/MemMang/heap_4.c
FREERTOS_PORTABLE_INC= $(FREERTOS_PORTABLE_DIR)



# **************  LwIP directories *********************************
LWIP_VERSION = git
LWIP_DIR=./depend/lwip
LWIP_CONTRIB=./depend/lwip-contrib
LWIP_CORE_SRC=  $(wildcard $(LWIP_DIR)/src/core/*.c) $(wildcard $(LWIP_DIR)/src/api/*.c) \
		$(wildcard $(LWIP_DIR)/src/core/ipv4/*.c) $(wildcard $(LWIP_DIR)/src/netif/*.c)
		
LWIP_APPS_SRC=$(wildcard $(LWIP_DIR)/src/apps/mdns/*.c) $(wildcard $(LWIP_DIR)/src/apps/lwiperf/*.c)
LWIP_SYSTEM_SRC=  $(wildcard $(LWIP_CONTRIB)/ports/freertos/*.c) 

LWIP_SRC= $(LWIP_SYSTEM_SRC) $(LWIP_CORE_SRC) $(LWIP_APPS_SRC)
LWIP_INC =   $(LWIP_CONTRIB)/ports/freertos/include $(LWIP_DIR)/src/include 


# **************  EEM USB  ********************************* 

USB_EEM_DIR=EEM
USB_EEM_SRC=$(wildcard $(USB_EEM_DIR)/Src/*.c) $(wildcard $(USB_EEM_DIR)/interface/*.cpp)
USB_EEM_INC=$(USB_EEM_DIR)/Inc $(USB_EEM_DIR)/interface

USB_SRC = $(USB_CORE_SRC) $(USB_EEM_SRC)
USB_INC = $(USB_CORE_INC) $(USB_EEM_INC)


# *************** TARGETS *************************************
BOARD_SRC:= $(wildcard $(TARGET)/Src/*.c) $(wildcard $(TARGET)/Src/*.cpp)
BOARD_INC:= $(TARGET)/Inc
BOARD_LINKER:= $(wildcard $(TARGET)/STM32F*.ld)
BOARD_STARTUP:=$(wildcard $(TARGET)/startup_*xx.s)

ifeq ($(TARGET),SPRACING4EVO)

TARGET_FLAGS= -DSTM32F405xx

else ifeq ($(TARGET),STM32F407VET6_BLUE_BOARD)

TARGET_FLAGS= -DSTM32F407xx

endif


# *************** APPS *************************************

APP_SRC:=  $(wildcard Src/*.c) $(wildcard Src/*.cpp)
APP_INCLUDE:= Inc



PROTOC_FILES:= $(wildcard protofiles/*.proto) 
GEN_NANOPB_C_FILES:= protofiles/generated/timestamp.pb.c protofiles/generated/messages.pb.c 
GEN_NANOPB_C_INCLUDE=protofiles/generated


#protofiles/generated/messages.pb.c: protofiles/messages.proto
#	protoc -I=protofiles -oprotofiles/generated/messages.pb  protofiles/messages.proto
#	python3 depend/nanopb/generator/nanopb_generator.py protofiles/generated/messages.pb

NANOPB_SRC:=$(wildcard depend/nanopb/*.c)
NANOPB_INC:= depend/nanopb

	

# ***************** SRC ******************************************


SRC = 	$(STM32Cube_F4_DRIVER_SRC) \
	$(FREERTOS_RTOS_SRC) $(CMSIS_FREERTOS_SRC)\
	$(FREERTOS_PORTABLE_SRC) \
	$(LWIP_SRC) \
	$(USB_SRC) \
	$(BOARD_SRC) \
	$(APP_SRC) \
	$(BOARD_STARTUP) \
	$(GEN_NANOPB_C_FILES) \
	$(NANOPB_SRC)
	
	
	
INCLUDE_DIRS = 	$(APP_INCLUDE)/msgProtocol \
		$(APP_INCLUDE) \
		$(BOARD_INC) \
		$(STM32Cube_F4_DRIVER_INC_DIR) \
		$(STM32Cube_F4_CMSIS_INC_DIR) \
		$(STM32Cube_F4_CMSIS_DEVICE_INC_DIR) \
		$(FREERTOS_RTOS_INC) \
		$(FREERTOS_PORTABLE_INC) \
		$(CMSIS_FREERTOS_INC) \
		$(LWIP_INC) \
		$(USB_INC) \
		$(PROTOTHREADS_INC) \
		$(GEN_NANOPB_C_INCLUDE) $(NANOPB_INC)
		

TARGET_OBJS	 = $(addsuffix .o,$(addprefix ./$(OBJECT_DIR)/$(TARGET)/,$(basename $(SRC))))
TARGET_DEPS	 = $(addsuffix .d,$(addprefix ./$(OBJECT_DIR)/$(TARGET)/,$(basename $(SRC))))	
		
ARCH_FLAGS	 = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DFREERTOS -DUSB_TRANSPORT
DEBUG_FLAGS	 = -ggdb3 -DDEBUG --specs=nano.specs

ifeq ($(DEBUG),GDB)
OPTIMIZE	 = -O0
LTO_FLAGS	 = $(OPTIMIZE)
else
OPTIMIZE	 = -O3
LTO_FLAGS	 = $(OPTIMIZE)
endif

CFLAGS= $(ARCH_FLAGS) \
		   $(LTO_FLAGS) \
		   $(WARN_FLAGS) \
		   $(addprefix -D,$(OPTIONS)) \
		   $(addprefix -I,$(INCLUDE_DIRS)) \
		   $(DEBUG_FLAGS)  \
		   $(SPECTROMETER_TRANSPORT) \
		   -std=gnu99 \
		   -Wall  -Wextra -Wunsafe-loop-optimizations -Wdouble-promotion -Wundef \
		   -ffunction-sections \
		   -fdata-sections  \
		   $(TARGET_FLAGS) \
		   -fverbose-asm -ffat-lto-objects \
		   -MMD -MP
		   
CXXFLAGS	 = $(ARCH_FLAGS) \
		   $(LTO_FLAGS) \
		   $(WARN_FLAGS) \
		   $(addprefix -D,$(OPTIONS)) \
		   $(addprefix -I,$(INCLUDE_DIRS)) \
		   $(DEBUG_FLAGS) \
		    $(SPECTROMETER_TRANSPORT) \
		   -std=c++1z \
		   -Wall  -Wextra -Wunsafe-loop-optimizations  -Wundef \
		   -ffunction-sections -fno-exceptions -fno-rtti \
		   -fdata-sections \
		   $(TARGET_FLAGS) \
		   -fverbose-asm -ffat-lto-objects \
		   -MMD -MP		   

ASFLAGS		 = $(ARCH_FLAGS) \
		   $(WARN_FLAGS) \
		   -x assembler-with-cpp -ggdb3 \
		   $(addprefix -I,$(INCLUDE_DIRS)) \
		  -MMD -MP

LDFLAGS		 = -lm \
		   -specs=nosys.specs \
		   -lnosys -Wl,--gc-section\
		   $(ARCH_FLAGS) \
		   $(LTO_FLAGS) \
		   $(WARN_FLAGS) \
		   $(DEBUG_FLAGS) \
		   -static \
		   -Wl,-gc-sections,-Map,$(TARGET).map \
		   -Wl,-L$(LINKER_DIR) \
		   -Wl,--cref \
		   -T$(BOARD_LINKER)

	   
all: $(TARGET).elf 
	
$(TARGET).elf: $(TARGET_OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)
	$(SIZE) $(TARGET).elf
	$(OBJCOPY) -O ihex --set-start 0x8000000 $(TARGET).elf $(TARGET).hex
	$(V0) $(OBJCOPY) -O binary  $(TARGET).elf $(TARGET).bin
	
	
# Compile
$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	$(CC) -c -o $@ $(CFLAGS) $<

# Assemble
$(OBJECT_DIR)/$(TARGET)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $<


	
# Compile
$(OBJECT_DIR)/$(TARGET)/%.o: %.cpp
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	$(CXX) -c -o $@ $(CXXFLAGS) $<

# create files
protofiles/generated/timestamp.pb.c: protofiles/timestamp.proto
	protoc -oprotofiles/generated/timestamp.pb  protofiles/timestamp.proto
	python3 depend/nanopb/generator/nanopb_generator.py protofiles/generated/timestamp.pb

protofiles/generated/messages.pb.c: protofiles/messages.proto
	protoc -I=protofiles -oprotofiles/generated/messages.pb  protofiles/messages.proto
	python3 depend/nanopb/generator/nanopb_generator.py protofiles/generated/messages.pb

debug:

	@echo $(STM32Cube_F4_DRIVER_SRC)
	
	
clean:
	rm -f $(TARGET).elf $(TARGET).map 
	rm -rf obj
	rm -f *.hex *.map *.elf *.bin
	rm -f protofiles/generated/*

debugtim:
	echo $(PROTOC_FILES)
	echo $(GEN_NANOPB_C_FILES)


-include $(TARGET_OBJS:%.o=%.d)	
