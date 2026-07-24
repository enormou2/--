################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'GNU Compiler - building file: "$<"'
	"C:/TI/gcc_arm_none_eabi_9_2_1/bin/arm-none-eabi-gcc-9.2.1.exe" -c @"device.opt"  -mcpu=cortex-m0plus -march=armv6-m -mthumb -mfloat-abi=soft -I"C:/Users/10959/workspace_ccstheia/test" -I"C:/Users/10959/workspace_ccstheia/test/Debug" -I"D:/ti/ccs2051/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"D:/ti/ccs2051/mspm0_sdk_2_10_00_04/source" -I"C:/TI/gcc_arm_none_eabi_9_2_1/arm-none-eabi/include/newlib-nano" -I"C:/TI/gcc_arm_none_eabi_9_2_1/arm-none-eabi/include" -O2 -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)" -std=c99 $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-346327493: ../empty.syscfg
	@echo 'SysConfig - building file: "$<"'
	"D:/ti/ccs2051/ccs/utils/sysconfig_1.27.1/sysconfig_cli.bat" -s "D:/ti/ccs2051/mspm0_sdk_2_10_00_04/.metadata/product.json" --script "C:/Users/10959/workspace_ccstheia/test/empty.syscfg" -o "." --compiler gcc
	@echo 'Finished building: "$<"'
	@echo ' '

device_linker.lds: build-346327493 ../empty.syscfg
device.opt: build-346327493
device.lds.genlibs: build-346327493
ti_msp_dl_config.c: build-346327493
ti_msp_dl_config.h: build-346327493
Event.dot: build-346327493

%.o: ./%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'GNU Compiler - building file: "$<"'
	"C:/TI/gcc_arm_none_eabi_9_2_1/bin/arm-none-eabi-gcc-9.2.1.exe" -c @"device.opt"  -mcpu=cortex-m0plus -march=armv6-m -mthumb -mfloat-abi=soft -I"C:/Users/10959/workspace_ccstheia/test" -I"C:/Users/10959/workspace_ccstheia/test/Debug" -I"D:/ti/ccs2051/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"D:/ti/ccs2051/mspm0_sdk_2_10_00_04/source" -I"C:/TI/gcc_arm_none_eabi_9_2_1/arm-none-eabi/include/newlib-nano" -I"C:/TI/gcc_arm_none_eabi_9_2_1/arm-none-eabi/include" -O2 -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)" -std=c99 $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0g350x_gcc.o: D:/ti/ccs2051/mspm0_sdk_2_10_00_04/source/ti/devices/msp/m0p/startup_system_files/gcc/startup_mspm0g350x_gcc.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'GNU Compiler - building file: "$<"'
	"C:/TI/gcc_arm_none_eabi_9_2_1/bin/arm-none-eabi-gcc-9.2.1.exe" -c @"device.opt"  -mcpu=cortex-m0plus -march=armv6-m -mthumb -mfloat-abi=soft -I"C:/Users/10959/workspace_ccstheia/test" -I"C:/Users/10959/workspace_ccstheia/test/Debug" -I"D:/ti/ccs2051/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"D:/ti/ccs2051/mspm0_sdk_2_10_00_04/source" -I"C:/TI/gcc_arm_none_eabi_9_2_1/arm-none-eabi/include/newlib-nano" -I"C:/TI/gcc_arm_none_eabi_9_2_1/arm-none-eabi/include" -O2 -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)" -std=c99 $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


