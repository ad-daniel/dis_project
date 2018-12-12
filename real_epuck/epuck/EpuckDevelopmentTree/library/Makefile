##
##  Makefile for epuck_library
##
##  $Date: 2009-11-10 12:37:33 +0100 (Tue, 10 Nov 2009) $
##  $Revision: 102 $
##  $Author: cianci $
##  $HeadURL: https://grmapc10.epfl.ch/svn/students/Epuck/EpuckDevelopmentTree/library/Makefile $
##


COMPONENTS 	= a_d a_d/advance_ad_scan camera/fast_2_timer camera/slow_3_timer codec contrib/radio_swis contrib/radio_swis/v2 contrib/robot_id I2C motor_led uart


.PHONY: all
all:
	for i in $(COMPONENTS); do \
	    (cd $$i && $(MAKE)); \
	done


.PHONY:	clean
clean:
	for i in $(COMPONENTS); do \
	    (cd $$i && $(MAKE) clean); \
	done

#-------------------------------------------------------------------------------
