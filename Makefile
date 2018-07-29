bcm47xxnflash-y				+= main.o
bcm47xxnflash-y				+= ops_bcm4706.o
bcm47xxnflash-y				+= ops_bcm5357.o

obj-$(CONFIG_MTD_NAND_BCM47XXNFLASH)	+= bcm47xxnflash.o
