#Userlib directories
USRLIB = ./usrlib

# List of all the Userlib device files.
USRSRC := $(USRLIB)/IMU.c \
		  $(USRLIB)/Kalman.c \
		  $(USRLIB)/MadgwickAHRS.c \
		  $(USRLIB)/matrices.c  \
		  $(USRLIB)/GPS_Lib.c
					

# Required include directories
USRINC := $(USRLIB)

# Shared variables
ALLCSRC += $(USRSRC)
ALLINC  += $(USRINC)