#!/bin/bash

g++ -O0  -Wall --std=c++17 \
    -Wno-psabi -g \
    -I ../lib/cpp -I /opt/vc/include -L /opt/vc/lib -I../ruckig \
    -o mc_control \
    mc-moteus_control.cc \
    ../lib/cpp/mjbots/pi3hat/pi3hat.cc \
    ../ruckig/brake.cpp \
    ../ruckig/position-step1.cpp \
    ../ruckig/position-step2.cpp \
    ../ruckig/velocity-step1.cpp \
    ../ruckig/velocity-step2.cpp \
    joystick/joystick.cc \
    -lbcm_host \
    -lpthread
