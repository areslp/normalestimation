#!/bin/sh
# for file in `ls *.obj| grep -v noise`
# do
    file=box.obj
    echo $file
    ./addnoise $file 0.1
    ./addnoise $file 0.2
    ./addnoise $file 0.3
    ./addnoise $file 0.4
    ./addnoise $file 0.5
    # ./addnoise.exe $file 0.1
    # ./addnoise.exe $file 0.2
    # ./addnoise.exe $file 0.3
    # ./addnoise.exe $file 0.4
    # ./addnoise.exe $file 0.5
# done
