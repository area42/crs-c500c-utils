deploy:
	screen -dmS ttyS0 /dev/ttyS0 57600
	screen -r ttyS0 -X stuff ^E
	screen -r ttyS0 -X stuff "cd /ros^M"
	../../file_transfer/filetransfer_screen.sh main.r3 rosserial.r3 rosmsg.r3
	screen -r ttyS0 -X stuff "r3c -Wall -orosserver main.r3^M"
	screen -r ttyS0 -X stuff "./rosserver^M"

stop:
	screen -dmS ttyS0 /dev/ttyS0 57600
	screen -r ttyS0 -X stuff ^E
	
start:
	screen -dmS ttyS0 /dev/ttyS0 57600
	screen -r ttyS0 -X stuff "/ros/rosserver^M"

attach: 
	screen -dmS ttyS0 /dev/ttyS0 57600
	screen -r ttyS0
