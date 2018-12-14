
Software : libIrcom 1.0
Authors : Alexandre Campo, Alvaro Guttierez


What is it ?
================================================================================
libIrcom is a library that can be used straight forward on the e-puck robots 
(www.e-puck.org) to achieve local range infrared communication. libIrcom relies 
on the infrared sensors of the robots to transmit and receive information. 
However, the communication system is multiplexed with the proximity sensing 
system commonly used on the robots. It is therefore possible to both communicate 
and avoid obstacles.

libIrcom allows communications at a rate of 30 bytes per seconds max from 
sender to receiver, including a 2bits CRC check in each byte to detect erroneous
messages. Messages are encoded using a frequency modulation that permits usage 
in a wide range of light conditions. Messages can be detected up to 25 cm of 
distance between emitter and receiver.



How to test the examples ?
================================================================================
With the libIrcom library 2 examples are provide. test and synchronize.

Before compiling the examples you have to compile all the libraries. Go to 
the root directory of the library and launch the compilation script by typing:

	./make.sh

test
----
This example shows the comunication between two robots. One will be the emitter
and the other the receiver. You will choose each one with the selector switch.
1-> Emiter 2-> Receiver.

The emitter will start transmiting a sequence of numbers from 0 to 255 (1 byte)
continuously.

The receiver will take the frame and demodulate it. If the frame is well decoded
the receiver will send it through bluetooth along with information about the
location of the emitter of the message (orientation and distance). 
The orientation is expressed in the local mark of the receiver starting from 
the camera and going CCW. The distance is between the center of the 2 robots.

Boths robots have to be connected to a computer through bluetooth and 
press enter to start the comunication between them.

The code for the robots it's on libIrcomXXX/e-puck/src/test/. Go to the 
directory and type:

	make
	
The binary will be generated on libIrcomXXX/e-puck/bin/. Download the file to 
the robots

	epuckupload -f test.hex <number_of_robots_to_download>

The code for the computer transmition is on libIrcomXXX/pc/ircomTest/. Go to 
the directory. 

	make

and after 

	./ircomTest <mac_of_robot>

Once you are connected to both robots, press enter and check the transmision.

Good luck !

synchronize
-----------

This example shows how a group of robots can communicate about directions where they
are pointing to surrounding neighbours and end up aligned towards the same direction.

The example is base on the libIrcom library. In a complete loop, the robots try to:

i)  Send information: Because of the number of robots, each robot will first send 
	his own ID, and after his own direction related to the robots which it has 
	receive any message.
i)  listen for some transmissions and decode the messages. 
	If a message is an ID it will store the direction where it cames from.
	If the message is an orientation addressed to it, it will change the orientation, 
	according to this new information.

The code for the robots is on libIrcomXXX/e-puck/src/synchronize/. Go to the 
directory and type:

	make
	
The binary will be generated on libIrcomXXX/e-puck/bin/. Download the file to 
the robots

	epuckupload -f sync.hex <number_of_robots_to_download>


More informations
================================================================================
Find more informations on http://www.e-puck.org, or on the GNA community 
https://gna.org/projects/e-puck/


Contact the authors
================================================================================
For specific requests you may also contact the authors of this software 
Alexandre Campo : acampo@ulb.ac.be
Alvaro Guttierez : aguti@etsit.upm.es
