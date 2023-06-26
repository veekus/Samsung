# name of your application
APPLICATION = task_lora

# If no BOARD is found in the environment, use this default:
BOARD ?= unwd-range-l1-r3

USEMODULE += periph_gpio
USEMODULE += periph_i2c
USEMODULE += xtimer
USEMODULE += lsm6ds3
USEPKG += cayenne-lpp

# LoRa definitions
USEMODULE += sx127x
USEPKG += semtech-loramac
USEMODULE += semtech_loramac_rx
USEMODULE += crypto

# We need CRYPTO_AES for loramac, as it uses Riot's crypto
CFLAGS += -DCRYPTO_AES

# Default radio driver is Semtech SX1276 (used by the B-L072Z-LRWAN1 board)
DRIVER ?= sx1276
CFLAGS += -DSX127X_USE_TX_SWITCH

# Default region is Europe and default band is 868MHz
LORA_REGION ?= RU864
# Disable LoRaWAN duty cycle restrictions
CFLAGS += -DDISABLE_LORAMAC_DUTYCYCLE
CFLAGS += -DLORAWAN_DONT_USE_FPORT

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
DEVELHELP ?= 1

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

include $(RIOTBASE)/Makefile.include
