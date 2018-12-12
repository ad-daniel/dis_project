##
## 	Makefile for TinyOS/Java host program
##

TARGET = WebSvcBase

TOS = $(shell ncc -print-tosdir)
#PACKAGE = net.tinyos.oscope
#APP = ../VoltageThresholdDemo

MIG = mig java -deprecation
JAVAC = javac -deprecation
ROOT = ${TOS}/../tools/java
include $(ROOT)/Makefile.include







# List of message classes to build
MSGS = $(addsuffix .java,$(shell fgrep struct *Msg.h | cut -d' ' -f2))





.PHONY: all
all: $(addsuffix .class,$(TARGET))

$(TARGET).class: $(addsuffix .class,$(basename $(MSGS)))

.PHONY: echo
echo:
	@echo MSGS = $(MSGS)
	@echo ALL  = $(addsuffix .class,$(basename $(MSGS)) $(TARGET))




%.class: %.java
	javac $<

%.java: *.h
	{ MIGMSGS=`fgrep struct $< | cut -d' ' -f2`; echo $${MIGMSGS}; for i in $${MIGMSGS}; do $(MIG) -java-classname=$(PACKAGE).$$i $< $$i -o $${i}.java; $(JAVAC) $${i}.java; done; }





#OTHER_CLEAN = cleanmig
#cleanmig:
clean:
	rm -f $(MSGS) $(addsuffix .class,$(basename $(MSGS)) $(TARGET))

