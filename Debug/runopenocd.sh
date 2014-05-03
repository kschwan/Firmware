#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

openocd -f interface/ftdi/olimex-arm-usb-ocd-h.cfg -f $DIR/px4fmu-v1-board.cfg
