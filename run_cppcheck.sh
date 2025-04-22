#!/bin/bash

export PATH=$PATH:/home/simon/proj/vxl_toffy/repos/cppcheck-sarif/

PWD=`pwd`
DIR=`basename $PWD`
N="cppcheck_$DIR"

cppcheck --enable=all --std=c++17 ./*/ --xml 2>${N}.xml

cppcheck-sarif <${N}.xml >${N}.sarif

