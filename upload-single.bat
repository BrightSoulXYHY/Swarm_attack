@echo off

REM run in wsl to avoid permission problem
REM tar -cpf _release.tar --exclude=.git --exclude=upload-single.bat _release

set /a i=16
	
	echo del file in server@%i%
	ssh q@192.168.111.%i% rm -rf ~/Swarm_ws/*;
	echo del file done, start upload;
	scp ../_release.tar q@192.168.111.%i%:~/Swarm_ws;
	
	REM start multi line with single quotes
	REM and use ^ wrapping lines
	ssh q@192.168.111.%i% '^
		cd ~/Swarm_ws; ^
		tar -xpf _release.tar; ^
		mv _release src; ^
		source ~/.${SHELL:5}rc; 	^
		source /opt/ros/melodic/setup.${SHELL:5};	^
		catkin_make	^
	'

pause