#!/bin/bash

mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551
