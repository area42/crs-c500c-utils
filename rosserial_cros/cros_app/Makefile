deploy:
	screen -dmS crs_con /dev/crs_con 57600
	screen -r crs_con -X stuff ^E
	screen -r crs_con -X stuff "cd /ros^M"
	../../file_transfer/filetransfer_screen.sh main.r3 rosserial.r3 rosmsg.r3
	screen -r crs_con -X stuff "r3c -Wall -orosserver main.r3^M"
	screen -r crs_con -X stuff "./rosserver^M"

