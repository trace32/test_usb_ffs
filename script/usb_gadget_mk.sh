#!/bin/ash

# This script will:
#  - create the directory /config  and mount ConfigFS on top of it
#  - modprobe "libcomposite" to get a /config/usb_gadget entry
#  - create /config/usb_gadget/g1   to configure a USB gadget
#  - create .../g1/configs/c.1  as USB configuration
#  - create .../g1/functions/ffs.usb0
#    and attach it to configuration c.1
#  - create /usbfunc and mount FunctionFS for "usb0" on to of it
#
# After this setup you should be able to use /usbfunc/ep0
# to configure your function (via software).

if [ ! -d /config ] ; then
	mkdir /config
	mount -t configfs none /config
fi

if [ ! -d /config/usb_gadget ] ; then
	modprobe libcomposite
fi

cd /config/usb_gadget

if [ ! -d g1 ] ; then
	mkdir g1
	cd g1
else
	cd g1
fi

if [ ! -d strings/0x409 ] ; then
	mkdir strings/0x409
fi

echo "0x1209"            > idVendor
echo "0x0005"            > idProduct
echo "FFS Tester"        > strings/0x409/manufacturer
echo "FFS Tester"        > strings/0x409/product
echo "112233445566"      > strings/0x409/serialnumber

if [ ! -d configs/c.1 ] ; then
	mkdir configs/c.1
fi

if [ ! -d configs/c.1/strings/0x409 ] ; then
	mkdir configs/c.1/strings/0x409
fi

echo "default" > configs/c.1/strings/0x409/configuration

# The assumption is, that your USB device is self powered
# If not: Change here
echo 0 > configs/c.1/MaxPower

if [ ! -d functions/ffs.usb0 ] ; then
	mkdir functions/ffs.usb0
	ln -s functions/ffs.usb0 configs/c.1
fi

if [ ! -d /usbfunc ] ; then
	mkdir /usbfunc
	mount -t functionfs usb0 /usbfunc
fi

echo "Now start usb_ffs_tst"
