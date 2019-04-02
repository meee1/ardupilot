
counter=0 
while [ $counter -le 50 ] 
do
	mkdir $counter
	cd $counter
	homeloc="-35.362"
	homeloc+=$(($counter+10))
	homeloc+="387811354,149.165237426758,584.187969963776,0"
	echo SYSID_THISMAV=$(($counter+1)) > defaults.parm
	echo FRAME_CLASS=1 >> defaults.parm
	echo ARMING_CHECK=0 >> defaults.parm
	echo SCHED_LOOP_RATE=50 >> defaults.parm
	echo SCHED_DEBUG=3 >> defaults.parm
    echo SIM_TERRAIN=0 >> defaults.parm
    echo TERRAIN_ENABLE=0 >> defaults.parm
    echo FS_GCS_ENABLE=0 >> defaults.parm
	../build/sitl/bin/arducopter  -M+ -s1 -r 50 --home $homeloc --instance $counter --uartA udpclient:127.0.0.1:14550  --defaults defaults.parm  --disable-fgview  &
	cd ..
	((counter++))
	perl -e "select(undef,undef,undef,0.2);"
done

bash


"C:\Users\michael\Desktop\DIYDrones\ardupilot\build/sitl/bin/arducopter"
