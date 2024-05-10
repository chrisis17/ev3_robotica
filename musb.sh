USB_LISTS=$(dmesg)

USB1=$(grep 1-1.1 <<< "$USB_LISTS")
USB1_PROD=$(grep idProduct <<< "$USB1" | head -1 )
USB1_ID=$(grep idProduct <<< "$USB1_PROD" | cut -d'=' -f 3 | cut -b 1-4 )
USB1_PORT_TMP=$(grep tty <<< "$USB1" | head -4 )
USB1_PORT_TMP=$(echo "$USB1_PORT_TMP" | tail -n1 ) 
USB1_PORT=$(echo "$USB1_PORT_TMP" | cut -d " " -f 11 )

USB2=$(grep 1-1.3 <<< "$USB_LISTS")
USB2_PROD=$(grep idProduct <<< "$USB2" | head -1 )
USB2_ID=$(grep idProduct <<< "$USB2_PROD" | cut -d'=' -f 3 | cut -b 1-4 )
USB2_PORT_TMP=$(grep tty <<< "$USB2" | head -4 )
USB2_PORT_TMP=$(echo "$USB2_PORT_TMP" | tail -n1 ) 
USB2_PORT=$(echo "$USB2_PORT_TMP" | cut -d " " -f 11 )

if [[ $USB1_ID -eq "c534" ]]; then
  PORT_RIGHT=${USB1_PORT}
  PORT_LEFT=${USB2_PORT}
elif [[ $USB1_ID -eq "7523" ]]; then
  PORT_RIGHT=${USB2_PORT}
  PORT_LEFT=${USB1_PORT}
fi

echo "The right port is ${PORT_RIGHT} and the left port is ${PORT_LEFT}"
echo ""

PORT_LEFT="/dev/${PORT_LEFT}"
PORT_RIGHT="/dev/${PORT_RIGHT}"

roslaunch merakm_bringup merakm_core.launch port_right:=${PORT_RIGHT} port_left:=${PORT_LEFT}
