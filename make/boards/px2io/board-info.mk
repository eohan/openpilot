BOARD_TYPE          := 0x06
BOARD_REVISION      := 0x01
BOOTLOADER_VERSION  := 0x00
HW_TYPE             := 0x01

MCU                 := cortex-m3
CHIP                := STM32F100C6T6B
BOARD               := STM32F100C6T6B_PX2IO_Rev1

# should be MD_VL here but makefile's not ready for it
MODEL               := MD
MODEL_SUFFIX        :=

OSCILLATOR_FREQ     := 24000000
SYSCLK_FREQ         := 24000000

# Note: These must match the values in link_$(BOARD)_memory.ld
FW_BANK_BASE        := 0x08002000	# Start of firmware flash
FW_BANK_SIZE        := 0x0000e000	# Should include FW_DESC_SIZE

FW_DESC_SIZE        := 0x00000064
