################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/seekfree_libraries/zf_ccu6_pit.c \
../Libraries/seekfree_libraries/zf_eeprom.c \
../Libraries/seekfree_libraries/zf_eru.c \
../Libraries/seekfree_libraries/zf_eru_dma.c \
../Libraries/seekfree_libraries/zf_gpio.c \
../Libraries/seekfree_libraries/zf_gpt12.c \
../Libraries/seekfree_libraries/zf_gtm_pwm.c \
../Libraries/seekfree_libraries/zf_spi.c \
../Libraries/seekfree_libraries/zf_stm_systick.c \
../Libraries/seekfree_libraries/zf_uart.c \
../Libraries/seekfree_libraries/zf_vadc.c 

OBJS += \
./Libraries/seekfree_libraries/zf_ccu6_pit.o \
./Libraries/seekfree_libraries/zf_eeprom.o \
./Libraries/seekfree_libraries/zf_eru.o \
./Libraries/seekfree_libraries/zf_eru_dma.o \
./Libraries/seekfree_libraries/zf_gpio.o \
./Libraries/seekfree_libraries/zf_gpt12.o \
./Libraries/seekfree_libraries/zf_gtm_pwm.o \
./Libraries/seekfree_libraries/zf_spi.o \
./Libraries/seekfree_libraries/zf_stm_systick.o \
./Libraries/seekfree_libraries/zf_uart.o \
./Libraries/seekfree_libraries/zf_vadc.o 

COMPILED_SRCS += \
./Libraries/seekfree_libraries/zf_ccu6_pit.src \
./Libraries/seekfree_libraries/zf_eeprom.src \
./Libraries/seekfree_libraries/zf_eru.src \
./Libraries/seekfree_libraries/zf_eru_dma.src \
./Libraries/seekfree_libraries/zf_gpio.src \
./Libraries/seekfree_libraries/zf_gpt12.src \
./Libraries/seekfree_libraries/zf_gtm_pwm.src \
./Libraries/seekfree_libraries/zf_spi.src \
./Libraries/seekfree_libraries/zf_stm_systick.src \
./Libraries/seekfree_libraries/zf_uart.src \
./Libraries/seekfree_libraries/zf_vadc.src 

C_DEPS += \
./Libraries/seekfree_libraries/zf_ccu6_pit.d \
./Libraries/seekfree_libraries/zf_eeprom.d \
./Libraries/seekfree_libraries/zf_eru.d \
./Libraries/seekfree_libraries/zf_eru_dma.d \
./Libraries/seekfree_libraries/zf_gpio.d \
./Libraries/seekfree_libraries/zf_gpt12.d \
./Libraries/seekfree_libraries/zf_gtm_pwm.d \
./Libraries/seekfree_libraries/zf_spi.d \
./Libraries/seekfree_libraries/zf_stm_systick.d \
./Libraries/seekfree_libraries/zf_uart.d \
./Libraries/seekfree_libraries/zf_vadc.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/seekfree_libraries/%.src: ../Libraries/seekfree_libraries/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -D__CPU__=tc26xb -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\CODE" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\doc" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\Configurations" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\_Build" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\_Impl" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\_Lib" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\_Lib\DataHandling" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\_Lib\InternalMux" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\_PinMap" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Asclin" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Asclin\Asc" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Asclin\Lin" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Asclin\Spi" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Asclin\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Ccu6" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Ccu6\Icu" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Ccu6\PwmBc" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Ccu6\PwmHl" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Ccu6\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Ccu6\Timer" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Ccu6\TimerWithTrigger" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Ccu6\TPwm" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Cif" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Cif\Cam" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Cif\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Cpu" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Cpu\CStart" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Cpu\Irq" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Cpu\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Cpu\Trap" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Dma" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Dma\Dma" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Dma\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Dsadc" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Dsadc\Dsadc" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Dsadc\Rdc" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Dsadc\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Dts" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Dts\Dts" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Dts\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Emem" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Emem\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Eray" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Eray\Eray" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Eray\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Eth" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Eth\Phy_Pef7071" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Eth\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Fce" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Fce\Crc" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Fce\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Fft" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Fft\Fft" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Fft\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Flash" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Flash\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gpt12" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gpt12\IncrEnc" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gpt12\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gtm" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gtm\Atom" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gtm\Atom\Pwm" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gtm\Atom\PwmHl" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gtm\Atom\Timer" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gtm\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gtm\Tim" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gtm\Tim\In" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gtm\Tom" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gtm\Tom\Pwm" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gtm\Tom\PwmHl" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gtm\Tom\Timer" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gtm\Trig" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Hssl" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Hssl\Hssl" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Hssl\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\I2c" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\I2c\I2c" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\I2c\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Iom" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Iom\Driver" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Iom\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Msc" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Msc\Msc" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Msc\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Mtu" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Mtu\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Multican" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Multican\Can" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Multican\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Port" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Port\Io" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Port\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Psi5" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Psi5\Psi5" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Psi5\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Psi5s" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Psi5s\Psi5s" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Psi5s\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Qspi" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Qspi\SpiMaster" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Qspi\SpiSlave" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Qspi\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Scu" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Scu\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Sent" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Sent\Sent" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Sent\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Smu" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Smu\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Src" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Src\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Stm" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Stm\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Stm\Timer" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Vadc" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Vadc\Adc" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Vadc\Std" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\Infra" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\Infra\Platform" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\Infra\Platform\Tricore" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\Infra\Platform\Tricore\Compilers" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\Infra\Sfr" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\Infra\Sfr\TC26B" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\Infra\Sfr\TC26B\_Reg" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\Service" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\Service\CpuGeneric" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\Service\CpuGeneric\_Utilities" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\Service\CpuGeneric\If" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\Service\CpuGeneric\StdIf" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\infineon_libraries\Service\CpuGeneric\SysSe" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\seekfree_libraries" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\seekfree_libraries\common" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\Libraries\seekfree_peripheral" -I"C:\Users\Lenovo\Desktop\车车\seekfree-TC264_Library-master\TC264_Library\Example\9-PWM_Demo\USER" --iso=99 --c++14 --language=+volatile --anachronisms --fp-model=3 --fp-model=c --fp-model=f --fp-model=l --fp-model=n --fp-model=r --fp-model=z -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -o "$@"  "$<"  -cs --dep-file=$(@:.src=.d) --misrac-version=2012 -N0 -Z0 -Y0 2>&1; sed -i -e '/ctc\\include/d' -e '/Libraries\\iLLD/d' -e '/Libraries\\Infra/d' -e 's/\(.*\)".*\\9-PWM_Demo\(\\.*\)"/\1\.\.\2/g' -e 's/\\/\//g' $(@:.src=.d) && \
	echo $(@:.src=.d) generated
	@echo 'Finished building: $<'
	@echo ' '

Libraries/seekfree_libraries/%.o: ./Libraries/seekfree_libraries/%.src
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -o  "$@" "$<" --list-format=L1 --optimize=gs
	@echo 'Finished building: $<'
	@echo ' '


