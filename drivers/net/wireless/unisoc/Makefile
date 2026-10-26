sc23xx_wlan-y			+= cmd.o hw_param.o main.o reorder.o txrx.o vif.o
obj-$(CONFIG_SC23XX_CORE)	+= sc23xx_wlan.o
sc23xx_wlan_sipc-y		+= sipc.o
obj-$(CONFIG_SC23XX_SIPC)	+= sc23xx_wlan_sipc.o
sc23xx_wlan_sdio-y		+= sdio.o
obj-$(CONFIG_SC23XX_SDIO)	+= sc23xx_wlan_sdio.o

# the SDIO transport rides the BSP sprdwcn bus (sdiohal); its public
# headers live in that module's private include directory
CFLAGS_sdio.o			+= -I$(srctree)/drivers/net/wireless/sprdwcn/include
