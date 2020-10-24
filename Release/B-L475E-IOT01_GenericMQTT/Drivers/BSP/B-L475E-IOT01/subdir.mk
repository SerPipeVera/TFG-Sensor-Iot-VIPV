################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.c \
../B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.c \
../B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.c \
../B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.c \
../B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.c \
../B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.c \
../B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.c 

OBJS += \
./B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.o \
./B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.o \
./B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.o \
./B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.o \
./B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.o \
./B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.o \
./B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.o 

C_DEPS += \
./B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.d \
./B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.d \
./B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.d \
./B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.d \
./B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.d \
./B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.d \
./B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.d 


# Each subdirectory must supply rules for building sources it contributes
B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.o: ../B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DGENERICMQTT -DUSE_HAL_DRIVER '-DMQTTCLIENT_PLATFORM_HEADER=paho_mqtt_platform.h' '-DMBEDTLS_CONFIG_FILE=<genmqtt_mbedtls_config.h>' -DENABLE_IOT_WARNING -DUSE_WIFI -DSENSOR -DUSE_MBED_TLS -DENABLE_IOT_ERROR -DSTM32L475xx -DENABLE_IOT_INFO -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/MQTTPacket -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/LIS3MDL -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/LPS22HB -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/LSM6DSL -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/MQTTClient-C -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/MbedTLS -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/cJSON -I../B-L475E-IOT01_GenericMQTT/Application/Common -I../B-L475E-IOT01_GenericMQTT/Application/GenericMQTT -I../B-L475E-IOT01_GenericMQTT/Application/Wifi -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01 -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/HTS221 -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP -I../B-L475E-IOT01_GenericMQTT/Drivers/CMSIS -I../B-L475E-IOT01_GenericMQTT/Drivers/STM32L4xx_HAL_Driver -I../Core/Inc/minmea-master -I../Middlewares/ST/STM32_MotionFX_Library/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.o: ../B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DGENERICMQTT -DUSE_HAL_DRIVER '-DMQTTCLIENT_PLATFORM_HEADER=paho_mqtt_platform.h' '-DMBEDTLS_CONFIG_FILE=<genmqtt_mbedtls_config.h>' -DENABLE_IOT_WARNING -DUSE_WIFI -DSENSOR -DUSE_MBED_TLS -DENABLE_IOT_ERROR -DSTM32L475xx -DENABLE_IOT_INFO -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/MQTTPacket -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/LIS3MDL -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/LPS22HB -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/LSM6DSL -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/MQTTClient-C -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/MbedTLS -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/cJSON -I../B-L475E-IOT01_GenericMQTT/Application/Common -I../B-L475E-IOT01_GenericMQTT/Application/GenericMQTT -I../B-L475E-IOT01_GenericMQTT/Application/Wifi -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01 -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/HTS221 -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP -I../B-L475E-IOT01_GenericMQTT/Drivers/CMSIS -I../B-L475E-IOT01_GenericMQTT/Drivers/STM32L4xx_HAL_Driver -I../Core/Inc/minmea-master -I../Middlewares/ST/STM32_MotionFX_Library/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.o: ../B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DGENERICMQTT -DUSE_HAL_DRIVER '-DMQTTCLIENT_PLATFORM_HEADER=paho_mqtt_platform.h' '-DMBEDTLS_CONFIG_FILE=<genmqtt_mbedtls_config.h>' -DENABLE_IOT_WARNING -DUSE_WIFI -DSENSOR -DUSE_MBED_TLS -DENABLE_IOT_ERROR -DSTM32L475xx -DENABLE_IOT_INFO -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/MQTTPacket -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/LIS3MDL -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/LPS22HB -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/LSM6DSL -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/MQTTClient-C -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/MbedTLS -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/cJSON -I../B-L475E-IOT01_GenericMQTT/Application/Common -I../B-L475E-IOT01_GenericMQTT/Application/GenericMQTT -I../B-L475E-IOT01_GenericMQTT/Application/Wifi -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01 -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/HTS221 -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP -I../B-L475E-IOT01_GenericMQTT/Drivers/CMSIS -I../B-L475E-IOT01_GenericMQTT/Drivers/STM32L4xx_HAL_Driver -I../Core/Inc/minmea-master -I../Middlewares/ST/STM32_MotionFX_Library/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.o: ../B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DGENERICMQTT -DUSE_HAL_DRIVER '-DMQTTCLIENT_PLATFORM_HEADER=paho_mqtt_platform.h' '-DMBEDTLS_CONFIG_FILE=<genmqtt_mbedtls_config.h>' -DENABLE_IOT_WARNING -DUSE_WIFI -DSENSOR -DUSE_MBED_TLS -DENABLE_IOT_ERROR -DSTM32L475xx -DENABLE_IOT_INFO -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/MQTTPacket -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/LIS3MDL -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/LPS22HB -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/LSM6DSL -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/MQTTClient-C -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/MbedTLS -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/cJSON -I../B-L475E-IOT01_GenericMQTT/Application/Common -I../B-L475E-IOT01_GenericMQTT/Application/GenericMQTT -I../B-L475E-IOT01_GenericMQTT/Application/Wifi -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01 -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/HTS221 -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP -I../B-L475E-IOT01_GenericMQTT/Drivers/CMSIS -I../B-L475E-IOT01_GenericMQTT/Drivers/STM32L4xx_HAL_Driver -I../Core/Inc/minmea-master -I../Middlewares/ST/STM32_MotionFX_Library/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.o: ../B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DGENERICMQTT -DUSE_HAL_DRIVER '-DMQTTCLIENT_PLATFORM_HEADER=paho_mqtt_platform.h' '-DMBEDTLS_CONFIG_FILE=<genmqtt_mbedtls_config.h>' -DENABLE_IOT_WARNING -DUSE_WIFI -DSENSOR -DUSE_MBED_TLS -DENABLE_IOT_ERROR -DSTM32L475xx -DENABLE_IOT_INFO -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/MQTTPacket -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/LIS3MDL -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/LPS22HB -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/LSM6DSL -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/MQTTClient-C -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/MbedTLS -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/cJSON -I../B-L475E-IOT01_GenericMQTT/Application/Common -I../B-L475E-IOT01_GenericMQTT/Application/GenericMQTT -I../B-L475E-IOT01_GenericMQTT/Application/Wifi -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01 -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/HTS221 -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP -I../B-L475E-IOT01_GenericMQTT/Drivers/CMSIS -I../B-L475E-IOT01_GenericMQTT/Drivers/STM32L4xx_HAL_Driver -I../Core/Inc/minmea-master -I../Middlewares/ST/STM32_MotionFX_Library/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.o: ../B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DGENERICMQTT -DUSE_HAL_DRIVER '-DMQTTCLIENT_PLATFORM_HEADER=paho_mqtt_platform.h' '-DMBEDTLS_CONFIG_FILE=<genmqtt_mbedtls_config.h>' -DENABLE_IOT_WARNING -DUSE_WIFI -DSENSOR -DUSE_MBED_TLS -DENABLE_IOT_ERROR -DSTM32L475xx -DENABLE_IOT_INFO -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/MQTTPacket -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/LIS3MDL -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/LPS22HB -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/LSM6DSL -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/MQTTClient-C -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/MbedTLS -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/cJSON -I../B-L475E-IOT01_GenericMQTT/Application/Common -I../B-L475E-IOT01_GenericMQTT/Application/GenericMQTT -I../B-L475E-IOT01_GenericMQTT/Application/Wifi -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01 -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/HTS221 -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP -I../B-L475E-IOT01_GenericMQTT/Drivers/CMSIS -I../B-L475E-IOT01_GenericMQTT/Drivers/STM32L4xx_HAL_Driver -I../Core/Inc/minmea-master -I../Middlewares/ST/STM32_MotionFX_Library/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.o: ../B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DGENERICMQTT -DUSE_HAL_DRIVER '-DMQTTCLIENT_PLATFORM_HEADER=paho_mqtt_platform.h' '-DMBEDTLS_CONFIG_FILE=<genmqtt_mbedtls_config.h>' -DENABLE_IOT_WARNING -DUSE_WIFI -DSENSOR -DUSE_MBED_TLS -DENABLE_IOT_ERROR -DSTM32L475xx -DENABLE_IOT_INFO -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/MQTTPacket -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/LIS3MDL -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/LPS22HB -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/LSM6DSL -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/MQTTClient-C -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/MbedTLS -I../B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/cJSON -I../B-L475E-IOT01_GenericMQTT/Application/Common -I../B-L475E-IOT01_GenericMQTT/Application/GenericMQTT -I../B-L475E-IOT01_GenericMQTT/Application/Wifi -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01 -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/HTS221 -I../B-L475E-IOT01_GenericMQTT/Drivers/BSP -I../B-L475E-IOT01_GenericMQTT/Drivers/CMSIS -I../B-L475E-IOT01_GenericMQTT/Drivers/STM32L4xx_HAL_Driver -I../Core/Inc/minmea-master -I../Middlewares/ST/STM32_MotionFX_Library/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"

