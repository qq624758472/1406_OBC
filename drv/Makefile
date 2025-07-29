export ARCH=arm64
export CROSS_COMPILE=aarch64-linux-gnu-
ifneq ($(KERNELRELEASE),)
    obj-m = my_i2c_drv.o
else
#    KERNELDIR := /home/ucas/user/zhanghao/0878/petalinux/TP0808/build/tmp/work/plnx_zynq7-xilinx-linux-gnueabi/linux-xlnx/4.14-xilinx-v2018.3+gitAUTOINC+eeab73d120-r0/linux-plnx_zynq7-standard-build
#     KERNELDIR := /home/ucas/user/qbq/0878/zynq_prj/zynqprj_flashRamFS/zynqprj_RAMFS/build/tmp/work/plnx_zynq7-xilinx-linux-gnueabi/linux-xlnx/4.14-xilinx-v2018.3+git999-r0/linux-xlnx-4.14-xilinx-v2018.3+git999/
#      KERNELDIR :=/home/ucas/user/qbq/0878/zynq_prj/zynqprj_updatemtd/zynqprj_mtd/build/tmp/work/plnx_zynq7-xilinx-linux-gnueabi/linux-xlnx/4.14-xilinx-v2018.3+git999-r0/linux-xlnx-4.14-xilinx-v2018.3+git999 
#	KERNELDIR :=/home/ucas/user/qbq/0878/zynq_prj/zynq_prjzhengyang/BB_XING/V1.04/V1.04_A17_LXIICRESET_PPS_TEST/build/tmp/work/plnx_zynq7-xilinx-linux-gnueabi/linux-xlnx/4.14-xilinx-v2018.3+git999-r0/linux-xlnx-4.14-xilinx-v2018.3+git999/
	KERNELDIR :=/home/ucas/1406/linux-fslc-rt/
	PWD       := $(shell pwd)

CC = $(CROSS_COMPILE)gcc

modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
endif
 
clean:
	rm -rf .tmp_versions Module.symvers modules.order .tmp_versions .*.cmd *.o *.ko *.mod.c
