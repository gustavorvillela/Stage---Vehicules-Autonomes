#!/bin/bash

cancel1='cancel'
cancel2='can'
cancel3='c'

add1='add'
add2='a'

finish1='finish'
finish2='f'

read -p 'Insert the name(s) of packets to be installed:' command
echo Adding $command to list of packages...

while true; do

    read -p 'Do you want to add a package or cancel the operation or are you done?[add/cancel/finish]' continue_var

    if [ "$continue_var" = "$cancel1" ] || [ "$continue_var" = "$cancel2" ] || [ "$continue_var" = "$cancel3" ]; then
        echo Canceling...
        exit 0
    elif [ "$continue_var" = "$add1" ] || [ "$continue_var" = "$add2" ]; then
        read -p 'Which one?' add_var
        command="$command $add_var"
        echo "New list of packages: $command"
    elif [ "$continue_var" = "$finish1" ] || [ "$continue_var" = "$finish2" ]; then
        echo "OK! (This might take a while :P)"
        cd ~/catkin_ws
        rosinstall_generator robot $command --rosdistro noetic --deps --wet-only --tar > noetic-custom_ros.rosinstall &&
        wstool merge -t src melodic-custom_ros.rosinstall &&
        wstool update -t src &&
        rosdep install --from-paths src --ignore-src --rosdistro noetic -y -r --os=debian:buster &&
        sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/noetic
    else
        echo "Invalid input"
    fi

done
