#!/bin/bash

if ! [ -x "$(command -v psrecord)" ]                                         
then                                                                         
    echo Please install psrecord and make it available via the PATH variable.
    echo \# pip install psrecord                                             
    echo \# export PATH=\${PATH}:\${HOME}/.local/bin/                        
    exit -1                                                                  
fi  

psrecord "ros2 run reference_system_autoware $1" --log $2 --plot $3 --duration $4