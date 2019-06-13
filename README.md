WHAT IS THIS?
=============

Linux Kernel source code for the devices:
* Archos 101b Oxygen (AC101BOX)

Maybe compatible with ARCHOS oxygen 70 / 80

Also branded as BUSH Spira B3 10 (or B2 10)


BUILD INSTRUCTIONS?
===================

Specific sources are separated by branches and each version is tagged with it's corresponding number. First, you should
clone the project:

        $ git clone https://github.com/carlitros900/android_kernel_archos_oxygen.git

After it, choose the version you would like to build:

* Archos 101b Oxygen Ultra *

        $ mv android_kernel_archos_oxygen kernel
        $ cd kernel
        $ git checkout 3.18.140-NovaKernel

At the same level of the "kernel" directory:

Download a prebuilt gcc:

        $ git clone https://android.googlesource.com/platform/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9 -b marshmallow-release 

Create KERNEL_OUT dir:

        $ mkdir KERNEL_OUT   

Your directory tree should look like this:
* kernel
* aarch64-linux-android-4.9
* KERNEL_OUT

Finally, build the kernel according the next table of product names:

| device                    | product                 |
| --------------------------|-------------------------|
| Archos 101b Oxygen        | ac101box                |


        $ make -C kernel O=../KERNEL_OUT/ ARCH=arm64 CROSS_COMPILE=../aarch64-linux-android-4.9/bin/aarch64-linux-android- {product}_defconfig
        $ make -C kernel O=../KERNEL_OUT/ ARCH=arm64 CROSS_COMPILE=../aarch64-linux-android-4.9/bin/aarch64-linux-android-                      
    
You can specify "-j CORES" argument to speed-up your compilation, example:

        $ make -C kernel O=../KERNEL_OUT/ ARCH=arm64 CROSS_COMPILE=../aarch64-linux-android-4.9/bin/aarch64-linux-android- -j 8

Some source code is unavailable, or fails, you have to link the prebuild binary shipped in the /prebuild folder. 

        $ mkdir -p KERNEL_OUT/drivers/power/mediatek
        $ cp kernel/prebuild/user/drivers/power/mediatek/battery_common.o KERNEL_OUT/drivers/power/mediatek/
        $ cp kernel/prebuild/user/drivers/power/mediatek/battery_meter.o KERNEL_OUT/drivers/power/mediatek/
