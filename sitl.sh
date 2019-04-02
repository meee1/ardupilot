mkdir 1; cd 1;  ../build/sitl/bin/arducopter  -M+ -s1 --home -35.3629387811354,149.165237426758,584.187969963776,0 --instance 0 --uartA tcp:0  --disable-fgview &
cd ..
pause
counter=1 
while [ $counter -le 50 ] 
do
	mkdir $counter
	cd $counter
	parentport="tcpclient:127.0.0.1:"
	parentport+=$((5752 + $counter*10))
	homeloc="-35.362"
	homeloc+=$counter
	homeloc+="387811354,149.165237426758,584.187969963776,0"
	echo SYSID_THISMAV=$(($counter+1)) > defaults.parm
	echo FRAME_CLASS=1 >> defaults.parm
	echo ARMING_CHECK=0 >> defaults.parm
	echo SCHED_LOOP_RATE=50 >> defaults.parm
	echo SCHED_DEBUG=3 >> defaults.parm	
	../build/sitl/bin/arducopter  -M+ -s1 --home $homeloc --instance $counter --uartA tcp:0  --defaults defaults.parm  --disable-fgview  &
	echo --uartD $parentport
	cd ..
	((counter++))
done

bash


"C:\Users\michael\Desktop\DIYDrones\ardupilot\build/sitl/bin/arducopter"


tcp://127.0.0.1:5760
tcp://127.0.0.1:5770
tcp://127.0.0.1:5780
tcp://127.0.0.1:5790
tcp://127.0.0.1:5800
tcp://127.0.0.1:5810
tcp://127.0.0.1:5820
tcp://127.0.0.1:5830
tcp://127.0.0.1:5840
tcp://127.0.0.1:5850
tcp://127.0.0.1:5860
tcp://127.0.0.1:5870
tcp://127.0.0.1:5880
tcp://127.0.0.1:5890
tcp://127.0.0.1:5900
tcp://127.0.0.1:5910
tcp://127.0.0.1:5920
tcp://127.0.0.1:5930
tcp://127.0.0.1:5940
tcp://127.0.0.1:5950
tcp://127.0.0.1:5960
tcp://127.0.0.1:5970
tcp://127.0.0.1:5980
tcp://127.0.0.1:5990
tcp://127.0.0.1:6000
tcp://127.0.0.1:6010
tcp://127.0.0.1:6020
tcp://127.0.0.1:6030
tcp://127.0.0.1:6040
tcp://127.0.0.1:6050
tcp://127.0.0.1:6060
tcp://127.0.0.1:6070
tcp://127.0.0.1:6080
tcp://127.0.0.1:6090
tcp://127.0.0.1:6100
tcp://127.0.0.1:6110
tcp://127.0.0.1:6120
tcp://127.0.0.1:6130
tcp://127.0.0.1:6140
tcp://127.0.0.1:6150
tcp://127.0.0.1:6160
tcp://127.0.0.1:6170
tcp://127.0.0.1:6180
tcp://127.0.0.1:6190
tcp://127.0.0.1:6200
tcp://127.0.0.1:6210
tcp://127.0.0.1:6220
tcp://127.0.0.1:6230
tcp://127.0.0.1:6240
tcp://127.0.0.1:6250
tcp://127.0.0.1:6260

