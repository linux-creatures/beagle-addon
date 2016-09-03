
TRIG1=P8_43
TRIG2=P8_45
TRIG3=P9_28
TRIG4=P9_30

ECHO1=P8_44
ECHO2=P8_46
ECHO3=P9_27
ECHO4=P9_31







echo "-Configuring pinmux"
	config-pin -a $TRIG1 pruout
	config-pin -a $TRIG2 pruout
	config-pin -a $TRIG3 pruout
	config-pin -a $TRIG4 pruout
	config-pin -a $ECHO1 pruin
	config-pin -a $ECHO2 pruin
	config-pin -a $ECHO3 pruin
	config-pin -a $ECHO4 pruin






echo "-Rebooting"
		echo "Rebooting pru-core 0"
		echo "4a334000.pru0" > /sys/bus/platform/drivers/pru-rproc/unbind 2>/dev/null
		echo "4a334000.pru0" > /sys/bus/platform/drivers/pru-rproc/bind
		echo "Rebooting pru-core 1"
		echo "4a338000.pru1"  > /sys/bus/platform/drivers/pru-rproc/unbind 2> /dev/null
		echo "4a338000.pru1" > /sys/bus/platform/drivers/pru-rproc/bind

