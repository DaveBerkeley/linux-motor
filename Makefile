# Makefile for motor driver driver

obj-m += motors.o
motors-objs = stepper.o pwm.o motor.o

# FIN
