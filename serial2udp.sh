#!/bin/sh

socat -d -d pty,raw,echo=0,link=/tmp/ttyBRIDGE pty,raw,echo=0,link=/tmp/ttyMAVLINK > /dev/null 2>&1 &
socat -v udp4-listen:14540 open:/tmp/ttyBRIDGE,raw,nonblock,waitlock=/tmp/s0.locak,echo=0,b57600,crnl > /dev/null 2>&1 &
