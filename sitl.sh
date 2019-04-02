mkdir 1; cd 1;  ../build/sitl/bin/arducopter  -M+ -s1 --home -35.3629387811354,149.165237426758,584.187969963776,0 --instance 0 --uartA tcp:0  --disable-fgview &
cd ..

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