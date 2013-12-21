# Mouse Library
cmake_minimum_required(VERSION 2.4.6)

add_library(MOUSE
          manymouse.h

          linux_evdev.c
          macosx_hidmanager.c
          macosx_hidutilities.c
          manymouse.c
          windows_wminput.c
      	  x11_xinput2.c)