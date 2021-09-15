#!bin/bash
sed -i "s/all(val >= 0 and val) < 256/all(ord(val) >= 0 and ord(val) < 256/" $1
