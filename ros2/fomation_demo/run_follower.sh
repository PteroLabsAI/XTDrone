#!/bin/bash
python3 leader.py $1 $2&
python3 follower.py $1 $2&
