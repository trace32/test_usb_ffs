# Introduction

This repository contains some test code to demonstrate how to use the
proposed patch for the Linux Kernel to implement mmap for USB Gadget
FunctionFS.

# Setting up your USB Gadget device

To setup USB Gadget FunctionFS on your device you need to do some
configuration.

An example shell script is included here at `script/usb_gadget_mk.sh`.

The assumption here is, that neither ConfigFS nor FunctionFS are already
mounted.  The script will create the directories
- `/config` to mount ConfigFS
- `/usbfunc` to mount FunctionFS

If ConfigFS or FunctionFS is already mounted on your device you need to
modify the script (and subsequently the test application).

# Compiling C sources

## Compiling the test application for your USB Gadget device

Assumptions:
- Your USB Gadget device is running a Linux Kernel with the proposed patch
- You have a cross compiler to create ELF files for the USB Gadget device
- You have libaio available for your USB Gadget device

Here is an example how to compile the application for your USB Gadget
device.  In this example the targeted device has an SMP Cortex-A53 cluster. 
The device is running Linux with "musl libc".

The GCC cross compiler used here is called `aarch64-linux-musl-gcc`

    SYSROOT=/home/rohloff/linux_gadget/sysroot

    aarch64-linux-musl-gcc --sysroot=${SYSROOT} -mcpu=cortex-a53 -O2 -o usb_ffs_tst src/usb_ffs_tst.c -laio
    aarch64-linux-musl-strip usb_ffs_tst

You **very likely** need to modify the source code to:
- Use the right directories for ConfigFS and FunctionFS 
  (see definitions of `CFGFS_DIR` and `FFS_DIR`)
- Point to the right USB Device Controller (see definition of `g_udc_name[]`)

Now you have to copy the `usb_ffs_tst ELF` file to your USB gadget device.

## Compiling the test application for your PC

This application is intended to talk to the USB Gadget device via USB. 

Example compilation:

    gcc -Wall -Og -fno-inline -o usb_pc_tst pc/usb_pc_tst.c

# Running the test

## Start test application on your USB Gadget device.

Once the script to create the USB function has run, you have to start the
`usb_ffs_tst` application on your Gadget Device.

Make sure you have modified the source code to fit to your ConfigFS,
FunctionFS and USB Device Controller.

## Connect your USB Gadget device to the PC

After connecting your USB Gadget device to the PC, the device should be
listed by `lsusb`.

Example:

    rohloff@mypc:~/linux_gadget$ lsusb
    <...>
    Bus 002 Device 009: ID 1209:0005 Generic pid.codes Test PID
    <...>

The `usb_gadget_mk.sh` script configures the USB Gadget to use Vendor ID `1209` and Product ID `0005`.

## Run the test application on the PC

Use the bus and device numbers listed by `lsusb` as shown above.

Example run:

    rohloff@mypc:~/linux_gadget$ ./usb_pc_tst /dev/bus/usb/002/009 echo
    checkUsbDevice : Device is using SuperSpeed
    scanEndpointDescriptors : Found bulk IN  endpoint 0x81
    scanEndpointDescriptors : Found bulk OUT endpoint 0x01
    Started write of 4 bytes to OUT EP
    Finished write of 4 bytes to OUT EP
    Running ECHO test
    Transfering 4096 x 262128 bytes
    Total time in usecs        : 5730276
    Echo test successful, transferred 1073676288 bytes
    bytes per second: 187369035

Depending on the setup of your Linux PC you might need to run the
`usb_pc_tst` application as root; alternatively it might be possible to add
your user to the `usb` group to make sure you have access to your USB Gadget
device.
