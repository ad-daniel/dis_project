.include "p30f6014a.inc"

.global _readrobotidfromeeprom

.section eeprom, eedata
EE_number:

.section .text
_readrobotidfromeeprom:
	
	; Setup pointer to EEPROM memory
	MOV #tblpage(EE_number),W1
	MOV W1,TBLPAG
	MOV #tbloffset(EE_number),W1
	; Read the EEPROM data
	TBLRDL [W1],W0
	return

.end
