#!/bin/bash

if [ "$0" != "bash" ]
then # you are not bash

	echo "Usage: root# . $0"
	exit

else 

	if [ "$EUID" -ne 0 ]
    then 
    
	    echo "Usage: run as root."
	    echo "Use 'sudo su'"
	    
	else # you are root
		
	    # path to create shared lib
	    # export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
	    # path to shared lib (PWD) 
	    export LD_LIBRARY_PATH=/home/neurorobotics/cmCode/biosemi:$LD_LIBRARY_PATH
	    
	    #./cmlabview $1
	    #chown cm $1
	    echo "*** Biosemi is Ready ***"
	    
	fi

fi
