#1 bin/bash

source /etc/device_config

CONFIGFS=/sys/kernel/config/usb_gadget
GADGET=$CONFIGFS/composite_gadget
IIOD_OPTS="-D -n $ENDPOINTS -F /dev/iio_ffs"
UDC_HANDLE_SUSPEND=`fw_printenv -n udc_handle_suspend 2> /dev/null || echo 0`

create_iiod_context_attributes() {
	model=$1
	serial=$2
	model_variant=$3

	if [ "$USBPID" == "0xb673" ]; then
		# ADALM-PLUTO
		for dev in /sys/bus/iio/devices/*; do
			[ `cat ${dev}/name` == "ad9361-phy" ] && DEV_NAME=`basename ${dev}`
		done

		AD936X_TYPE=$(cat /sys/bus/iio/devices/${DEV_NAME}/of_node/compatible | sed 's/adi,//g')
		AD936X_TYPE_CAP=$(echo $AD936X_TYPE | tr a-z A-Z)
		MODEL=$(echo $model | sed "s/AD936[34]/$AD936X_TYPE_CAP/")
	else
		MODEL=$model
	fi

	echo "[Context Attributes]" > /etc/libiio.ini
	echo "hw_model=$MODEL" >> /etc/libiio.ini

	if [ "$model_variant" == "n25q256a" ]
	then
		echo "hw_model_variant=0" >> /etc/libiio.ini
	else
		echo "hw_model_variant=1" >> /etc/libiio.ini
	fi

	echo -e "hw_serial=$serial\n" >> /etc/libiio.ini
	echo "fw_version=`grep device-fw /opt/VERSIONS | cut -d ' ' -f 2`" >> /etc/libiio.ini
	if [ "$USBPID" == "0xb673" ]; then
		echo ad9361-phy,xo_correction=`cat /sys/bus/iio/devices/${DEV_NAME}/xo_correction` >> /etc/libiio.ini
		echo ad9361-phy,model=$AD936X_TYPE >> /etc/libiio.ini

	elif [ "$USBPID" == "0xb672" ]; then
		cat /opt/${CALIBFILENAME} | grep ^cal,* >> /etc/libiio.ini
	fi
}

#__________________STOP__________________________________________________
echo "Stopping UDC Gadgets"

if [ "$UDC_HANDLE_SUSPEND" == "1" ]; then
	start-stop-daemon -K -q -p /var/run/udc_handle_suspend.pid 2>/dev/null
fi

echo "" > $GADGET/UDC
start-stop-daemon -K -q -p /var/run/iiod.pid 2>/dev/null

rm $GADGET/configs/c.1/rndis.0
rm $GADGET/configs/c.1/mass_storage.0
#rm $GADGET/configs/c.1/acm.usb0
rm $GADGET/configs/c.1/ffs.iio_ffs

rmdir $GADGET/strings/0x409
rmdir $GADGET/configs/c.1/strings/0x409
rmdir $GADGET/configs/c.1

rmdir $GADGET/functions/ffs.iio_ffs
rmdir $GADGET/functions/acm.usb0
rmdir $GADGET/functions/rndis.0
rmdir $GADGET/functions/mass_storage.0

rmdir $GADGET 2> /dev/null

#__________________START__________________________________________________
echo -n "Starting UDC Gadgets: "
mount configfs -t configfs /sys/kernel/config 2> /dev/null

mkdir -p $GADGET

model=`cat /sys/firmware/devicetree/base/model | tr / -`
model_variant=`dmesg | grep m25p80 | grep Kbytes | cut -d ' ' -f 3`

serial=`dmesg | grep SPI-NOR-UniqueID`
serial=${serial#*SPI-NOR-UniqueID }

create_iiod_context_attributes "$model" "$serial" "$model_variant"

echo $serial > /etc/serial
sha1=`echo $serial | sha1sum`

echo 0x0456 > $GADGET/idVendor
echo $USBPID > $GADGET/idProduct
echo 0x1337 > $GADGET/bcdDevice

mkdir -p $GADGET/strings/0x409
echo "Analog Devices Inc." > $GADGET/strings/0x409/manufacturer
echo $PRODUCT > $GADGET/strings/0x409/product
echo $serial > $GADGET/strings/0x409/serialnumber

#        mkdir -p $GADGET/functions/ffs.iio_ffs
mkdir -p $GADGET/functions/acm.usb0
#        mkdir -p $GADGET/functions/rndis.0
mkdir -p $GADGET/functions/mass_storage.0

insmod /plutousbgadget/plutousbgadget.ko
mkdir -p $GADGET/functions/iiousb2.0
	
echo Y > $GADGET/functions/mass_storage.0/lun.0/removable

host_addr=`echo -n 00:E0:22; echo $sha1 | dd bs=1 count=6 2>/dev/null | hexdump -v -e '/1 ":%01c""%c"'`
dev_addr=`echo -n 00:05:F7; echo $sha1 | dd bs=1 count=6 skip=6 2>/dev/null | hexdump -v -e '/1 ":%01c""%c"'`

#        echo $host_addr > $GADGET/functions/rndis.0/host_addr
#        echo $dev_addr > $GADGET/functions/rndis.0/dev_addr

mkdir -p $GADGET/configs/c.1
mkdir -p $GADGET/configs/c.1/strings/0x409
#        echo "RNDIS/MSD/ACM/IIOUSBD" > $GADGET/configs/c.1/strings/0x409/configuration
echo "MSD/ACM/IIOUSBD" > $GADGET/configs/c.1/strings/0x409/configuration
echo 500 > $GADGET/configs/c.1/MaxPower

#        ln -s $GADGET/functions/rndis.0 $GADGET/configs/c.1
ln -s $GADGET/functions/mass_storage.0 $GADGET/configs/c.1
ln -s $GADGET/functions/acm.usb0 $GADGET/configs/c.1
#        ln -s $GADGET/functions/ffs.iio_ffs $GADGET/configs/c.1/ffs.iio_ffs
ln -s $GADGET/functions/iiousb2.0 $GADGET/configs/c.1

mkdir -p /dev/iio_ffs
mount iio_ffs -t functionfs /dev/iio_ffs 2> /dev/null

#start-stop-daemon -S -b -q -m -p /var/run/iiod.pid -x  /usr/sbin/iiod  -- $IIOD_OPTS
sleep 0.2

echo ci_hdrc.0 > $GADGET/UDC

[ $? = 0 ] && echo "OK" || echo "FAIL"

if [ "$UDC_HANDLE_SUSPEND" == "1" ]; then
        start-stop-daemon -S -b -q -m -p /var/run/udc_handle_suspend.pid -x  /sbin/udc_handle_suspend.sh
fi



