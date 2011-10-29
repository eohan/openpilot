BOARD_TYPE          := 0x05
BOARD_REVISION      := 0x01
BOOTLOADER_VERSION  := 0x00
HW_TYPE             := 0x00

MCU                 := cortex-m4
CHIP                := STM32F405RGT
BOARD               := STM32205_PX2FMU_Rev1

MODEL               := HD
MODEL_SUFFIX        :=

OSCILLATOR_FREQ     := 24000000
SYSCLK_FREQ         := 120000000

# Note: These must match the values in link_$(BOARD)_memory.ld
FW_BANK_BASE        := 0x08008000	# Start of firmware flash
FW_BANK_SIZE        := 0x000f8000	# Size of firmware flash

FW_DESC_SIZE        := 0x00000064
