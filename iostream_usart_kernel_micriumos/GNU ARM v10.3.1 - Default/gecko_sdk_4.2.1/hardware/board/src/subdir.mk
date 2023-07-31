################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk/hardware/board/src/sl_board_control_gpio.c \
C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk/hardware/board/src/sl_board_init.c 

OBJS += \
./gecko_sdk_4.2.1/hardware/board/src/sl_board_control_gpio.o \
./gecko_sdk_4.2.1/hardware/board/src/sl_board_init.o 

C_DEPS += \
./gecko_sdk_4.2.1/hardware/board/src/sl_board_control_gpio.d \
./gecko_sdk_4.2.1/hardware/board/src/sl_board_init.d 


# Each subdirectory must supply rules for building sources it contributes
gecko_sdk_4.2.1/hardware/board/src/sl_board_control_gpio.o: C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk/hardware/board/src/sl_board_control_gpio.c gecko_sdk_4.2.1/hardware/board/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFM32GG11B820F2048GL192=1' '-DSL_BOARD_NAME="BRD2204A"' '-DSL_BOARD_REV="B07"' '-DconfigNUM_SDK_THREAD_LOCAL_STORAGE_POINTERS=2' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"C:\Users\runhu\SimplicityStudio\v5_workspace\iostream_usart_kernel_micriumos\config" -I"C:\Users\runhu\SimplicityStudio\v5_workspace\iostream_usart_kernel_micriumos\autogen" -I"C:\Users\runhu\SimplicityStudio\v5_workspace\iostream_usart_kernel_micriumos" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/Device/SiliconLabs/EFM32GG11B/Include" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/common/inc" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//hardware/board/inc" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/service/cli/inc" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/service/cli/src" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/CMSIS/Core/Include" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/CMSIS/RTOS2/Include" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/service/device_init/inc" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/dmadrv/inc" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/common/inc" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/inc" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/service/iostream/inc" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/micrium_os/common/source" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/micrium_os/common/include" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/micrium_os/cpu/include" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/micrium_os/ports/source" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/micrium_os" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/micrium_os/kernel/source" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/micrium_os/kernel/include" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/common/toolchain/inc" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/service/system/inc" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/service/sleeptimer/inc" -Os -Wall -Wextra -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv4-sp-d16 -mfloat-abi=softfp --specs=nano.specs -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_4.2.1/hardware/board/src/sl_board_control_gpio.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

gecko_sdk_4.2.1/hardware/board/src/sl_board_init.o: C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk/hardware/board/src/sl_board_init.c gecko_sdk_4.2.1/hardware/board/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFM32GG11B820F2048GL192=1' '-DSL_BOARD_NAME="BRD2204A"' '-DSL_BOARD_REV="B07"' '-DconfigNUM_SDK_THREAD_LOCAL_STORAGE_POINTERS=2' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"C:\Users\runhu\SimplicityStudio\v5_workspace\iostream_usart_kernel_micriumos\config" -I"C:\Users\runhu\SimplicityStudio\v5_workspace\iostream_usart_kernel_micriumos\autogen" -I"C:\Users\runhu\SimplicityStudio\v5_workspace\iostream_usart_kernel_micriumos" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/Device/SiliconLabs/EFM32GG11B/Include" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/common/inc" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//hardware/board/inc" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/service/cli/inc" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/service/cli/src" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/CMSIS/Core/Include" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/CMSIS/RTOS2/Include" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/service/device_init/inc" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/dmadrv/inc" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/common/inc" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/inc" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/service/iostream/inc" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/micrium_os/common/source" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/micrium_os/common/include" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/micrium_os/cpu/include" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/micrium_os/ports/source" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/micrium_os" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/micrium_os/kernel/source" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/micrium_os/kernel/include" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/common/toolchain/inc" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/service/system/inc" -I"C:/Users/runhu/SimplicityStudio/SDKs/gecko_sdk//platform/service/sleeptimer/inc" -Os -Wall -Wextra -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv4-sp-d16 -mfloat-abi=softfp --specs=nano.specs -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_4.2.1/hardware/board/src/sl_board_init.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

