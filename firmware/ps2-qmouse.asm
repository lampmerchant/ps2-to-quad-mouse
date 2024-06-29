;;; 80 characters wide please ;;;;;;;;;;;;;;;;;;;;;;;;;; 8-space tabs please ;;;


;
;;;
;;;;;  PS/2 to Macintosh Quadrature Mouse Converter
;;;
;


;;; Connections ;;;

;;;                                                                 ;;;
;                                .--------.                           ;
;                        Supply -|01 \/ 08|- Ground                   ;
;    Mouse X Interrupt <--- RA5 -|02    07|- RA0 <--> PS/2 Data       ;
;    Mouse Y Interrupt <--- RA4 -|03    06|- RA1 <--> PS/2 Clock      ;
;     Mouse Quadrature <--- RA3 -|04    05|- RA2 ---> Mouse Button    ;
;                                '--------'                           ;
;                                                                     ;
;    Mouse quadrature signal must be pulled down with a 1 Mohm        ; 
;    resistor and inverted using a 2N7000 MOSFET.  PS/2 data and      ;
;    clock must be pulled up with a resistor of between 1 and 10      ;
;    kohms.                                                           ;
;                                                                     ;
;;;                                                                 ;;;


;;; Assembler Directives ;;;

	list		P=PIC12F1501, F=INHX32, ST=OFF, MM=OFF, R=DEC, X=ON
	#include	P12F1501.inc
	errorlevel	-302	;Suppress "register not in bank 0" messages
	errorlevel	-224	;Suppress TRIS instruction not recommended msgs
	__config	_CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_ON & _MCLRE_OFF & _CP_OFF & _BOREN_OFF & _CLKOUTEN_OFF
			;_FOSC_INTOSC	Internal oscillator, I/O on RA5
			;_WDTE_OFF	Watchdog timer disabled
			;_PWRTE_ON	Keep in reset for 64 ms on start
			;_MCLRE_OFF	RA3/!MCLR is RA3
			;_CP_OFF	Code protection off
			;_BOREN_OFF	Brownout reset off
			;_CLKOUTEN_OFF	CLKOUT disabled, I/O on RA4
	__config	_CONFIG2, _WRT_OFF & _STVREN_ON & _BORV_LO & _LPBOR_OFF &_LVP_OFF
			;_WRT_OFF	Write protection off
			;_STVREN_ON	Stack over/underflow causes reset
			;_BORV_LO	Brownout reset voltage low trip point
			;_LPBOR_OFF	Low power brownout reset disabled
			;_LVP_OFF	High-voltage on Vpp to program


;;; Macros ;;;

DELAY	macro	value		;Delay 3*W cycles, set W to 0
	movlw	value
	decfsz	WREG,F
	bra	$-1
	endm

DNOP	macro
	bra	$+1
	endm


;;; Constants ;;;

;Pin Assignments:
MXI_PIN	equ	RA5	;Pin where mouse X interrupt signal is connected
MYI_PIN	equ	RA4	;Pin where mouse Y interrupt signal is connected
MQU_PIN	equ	RA3	;Pin where mouse quadrature signal is connected
MBT_PIN	equ	RA2	;Pin where mouse button is connected
PMC_PIN	equ	RA1	;Pin where PS/2 mouse clock is connected
PMD_PIN	equ	RA0	;Pin where PS/2 mouse data is connected

;PS/2 Constants:
PMRESET	equ	0xFF	;Reset PS/2 mouse
PMSAMPR	equ	0xF3	;Set sample rate of PS/2 mouse
PMRESOL	equ	0xE8	;Set resolution of PS/2 mouse
PMIDFY	equ	0xF2	;Identify PS/2 device
PMIDWHL	equ	0x03	;Identify response for PS/2 wheel mouse
PMSTART	equ	0xF4	;Enable reporting from PS/2 mouse
PMACK	equ	0xFA	;Acknowledge byte from PS/2 mouse

;FLAGS:
QUADST	equ	7	;Determines whether we toggle quad pin or interrupt pins
PMINIT	equ	6	;Set if PS/2 mouse is initialized
PMIDFYD	equ	5	;Set if PS/2 mouse has been identified
PMWHEEL	equ	4	;Set if PS/2 mouse is a wheel mouse


;;; Variable Storage ;;;

	cblock	0x70	;Bank-common registers
	
	FLAGS	;You've got to have flags
	PP_FSAP	;Pointer to where to resume PS/2 peripheral state machine
	PP_SR	;PS/2 peripheral shift register
	PM_FSAP	;Pointer to where to resume PS/2 mouse state machine
	PM_HEAD	;PS/2 mouse header
	PC_FSAP	;Pointer to where to resume PS/2 mouse config state machine
	PC_T1U	;Upper byte of Timer1 (incremented when it overflows)
	DELTAXH	;Mouse X delta (positive = right)
	DELTAXL	; "
	DELTAYH	;Mouse Y delta (positive = up)
	DELTAYL	; "
	MSPEED	;Mouse speed (delta counts per quadrature event)
	X3
	X2
	X1
	X0
	
	endc


;;; Vectors ;;;

	org	0x0		;Reset vector
	goto	Init

	org	0x4		;Interrupt vector
	;fall through


;;; Interrupt Handler ;;;

Interrupt
	movlb	7		;If we got a negative edge on the PS/2 clock
	btfsc	IOCAF,PMC_PIN	; pin, handle it
	bra	IntPs2Bit	; "
	movlb	0		;If Timer1 overflowed, handle it
	btfsc	PIR1,TMR1IF	; "
	bra	IntTimer1	; "
	bcf	PIR1,TMR2IF	;Else, Timer2 interrupted, so reset the PS/2
	movlw	low PPFsaStart	; peripheral state machine
	movwf	PP_FSAP		; "
	movlw	low PMFsaHeader	;Also reset the mouse state machine if the mouse
	btfsc	FLAGS,PMINIT	; is initialized
	movwf	PM_FSAP		; "
	retfie			;Return

IntPs2Bit
	bcf	IOCAF,PMC_PIN	;Clear the interrupt
	movlb	0		;Reset Timer2 as we've received a bit
	clrf	TMR2		; "
	bcf	PIR1,TMR2IF	; "
	movlb	30		;Copy the received bit into carry
	lsrf	CLCDATA,W	; "
	lsrf	WREG,W		; "
	movf	PP_FSAP,W	;Resume the PS/2 peripheral state machine
	callw			; "
	movwf	PP_FSAP		;On returning, save the address returned in W
	retfie			; "

IntTimer1
	bcf	PIR1,TMR1IF	;Clear the interrupt
	incf	PC_T1U,F	;Tick the upper byte of Timer1
	btfss	STATUS,Z	;If it didn't roll over, we're done here
	retfie			; "
	bcf	STATUS,C	;Resume the PS/2 mouse configuration state
	movf	PC_FSAP,W	; machine, clearing carry so it knows we're
	callw			; there because of a timer event
	movwf	PC_FSAP		;On returning, save the address returned in W
	retfie			; "


;;; Hardware Initialization ;;;

Init
	banksel	OSCCON		;16 MHz high-freq internal oscillator
	movlw	B'01111000'
	movwf	OSCCON

	banksel	IOCAN		;PS/2 mouse clock sets IOCAF on negative edge
	movlw	1 << PMC_PIN
	movwf	IOCAN

	banksel	OPTION_REG	;Timer0 has 1:16 prescaler, thus overflowing
	movlw	B'01010011'	; after 1.024 ms; weak pull-ups on
	movwf	OPTION_REG

	banksel	T1CON		;Timer 1 is 1:1 with instruction clock, thus
	movlw	B'00000001'	; overflowing after 16.384 ms
	movwf	T1CON

	banksel	T2CON		;Timer2 has 1:16 prescaler and 1:4 postscaler,
	movlw	B'00011110'	; thus interrupting after 4.096 ms
	movwf	T2CON

	banksel	CLC2CON		;CLC2 is a DFF which clocks in data from the
	clrf	CLC2SEL0	; PS/2 data pin (CLC2IN1) on the falling edge of
	movlw	B'01010000'	; the PS/2 clock pin (CLC2IN0)
	movwf	CLC2SEL1
	movlw	B'00000001'
	movwf	CLC2POL
	movlw	B'00000010'
	movwf	CLC2GLS0
	movlw	B'10000000'
	movwf	CLC2GLS1
	clrf	CLC2GLS2
	clrf	CLC2GLS3
	movlw	B'10000100'
	movwf	CLC2CON

	banksel	ANSELA		;All pins digital, not analog
	clrf	ANSELA

	banksel	LATA		;Default state of output pins is low, PS/2 pins
	clrf	LATA		; ready to be pulled low

	banksel	TRISA		;PS/2 pins open-collector outputs which are
	movlw	B'00001011'	; currently off, quadrature controlled by weak
	movwf	TRISA		; pullup, other signals outputs

	banksel	PIE1		;Timer2 interrupt enabled
	movlw	1 << TMR2IE
	movwf	PIE1

	movlb	0		;Initialize key globals
	movlp	high PPFsaStart
	clrf	FLAGS
	clrf	DELTAXH
	clrf	DELTAXL
	clrf	DELTAYH
	clrf	DELTAYL
	clrf	PC_T1U
	movlw	4
	movwf	MSPEED
	movlw	low PPFsaStart
	movwf	PP_FSAP
	movlw	low PMFsaInit
	movwf	PM_FSAP
	movlw	low PCFsaNormal
	movwf	PC_FSAP

	movlw	B'11001000'	;On-change interrupt, peripheral interrupts (for
	movwf	INTCON		; Timer2) and interrupt subsystem on

	;fall through


;;; Mainline ;;;

Main
	btfsc	INTCON,TMR0IF	;If Timer0 overflowed, do quadrature emulation
	call	DoQuad		; "
	bra	Main		;Loop

DoQuad
	bcf	INTCON,TMR0IF	;Clear the interrupt
	movlw	1 << QUADST	;Toggle quadrature state
	xorwf	FLAGS,F		; "
	btfss	FLAGS,QUADST	;If state bit is clear, toggle the quadrature
	bra	DoQuadToggle	; signal
	bcf	INTCON,GIE	;Else, do quadrature movement for X and Y by
	call	DoQuadX		; toggling interrupt pins as appropriate,
	bsf	INTCON,GIE	; disabling interrupts as we do so that the
	bcf	INTCON,GIE	; deltas don't change under us
	call	DoQuadY		; "
	bsf	INTCON,GIE	; "
	return			; "

DoQuadToggle
	movlb	4		;Toggle quadrature pin for next subprogram call
	movlw	1 << MQU_PIN	; "
	xorwf	WPUA,F		; "
	return			; "

DoQuadX
	movlb	0		;If the sum of the current state of the
	clrw			; interrupt pin, the quadrature pin (which is
	btfsc	PORTA,MXI_PIN	; inverted), and the sign bit of the delta is
	xorlw	1 << MXI_PIN	; odd, we can toggle the interrupt pin to move
	btfss	PORTA,MQU_PIN	; the mouse in the desired direction; otherwise
	xorlw	1 << MXI_PIN	; return
	movlb	2		; "
	btfsc	DELTAXH,7	; "
	xorlw	1 << MXI_PIN	; "
	btfsc	STATUS,Z	; "
	return			; "
	btfsc	DELTAXH,7	;If the delta is negative, skip ahead
	bra	DoQuaX0		; "
	call	SubDeltaX	;Delta is positive, try to subtract from it
	movlw	1 << MXI_PIN	;If the carry was set (delta is now zero or
	btfsc	STATUS,C	; still positive), toggle the interrupt pin and
	xorwf	LATA,F		; return
	btfss	STATUS,C	;Otherwise, restore the previous value
	call	AddDeltaX	; "
	return			; "
DoQuaX0	call	AddDeltaX	;Delta is negative, try to add to it
	movlw	1 << MXI_PIN	;If the carry was clear (delta is now zero or
	btfss	STATUS,C	; still negative), toggle the interrupt pin and
	xorwf	LATA,F		; return
	btfsc	STATUS,C	;Otherwise, restore the previous value
	call	SubDeltaX	; "
	return			; "

DoQuadY
	movlb	0		;If the sum of the current state of the
	clrw			; interrupt pin, the quadrature pin (which is
	btfsc	PORTA,MYI_PIN	; inverted), and the sign bit of the delta is
	xorlw	1 << MYI_PIN	; odd, we can toggle the interrupt pin to move
	btfss	PORTA,MQU_PIN	; the mouse in the desired direction; otherwise
	xorlw	1 << MYI_PIN	; return
	movlb	2		; "
	btfsc	DELTAYH,7	; "
	xorlw	1 << MYI_PIN	; "
	btfsc	STATUS,Z	; "
	return			; "
	btfsc	DELTAYH,7	;If the delta is negative, skip ahead
	bra	DoQuaY0		; "
	call	SubDeltaY	;Delta is positive, try to subtract from it
	movlw	1 << MYI_PIN	;If the carry was set (delta is now zero or
	btfsc	STATUS,C	; still positive), toggle the interrupt pin and
	xorwf	LATA,F		; return
	btfss	STATUS,C	;Otherwise, restore the previous value
	call	AddDeltaY	; "
	return			; "
DoQuaY0	call	AddDeltaY	;Delta is negative, try to add to it
	movlw	1 << MYI_PIN	;If the carry was clear (delta is now zero or
	btfss	STATUS,C	; still negative), toggle the interrupt pin and
	xorwf	LATA,F		; return
	btfsc	STATUS,C	;Otherwise, restore the previous value
	call	SubDeltaY	; "
	return			; "


;;; State Machines ;;;

	org	0x100

FsaIgnore
	retlw	low FsaIgnore	;Utility state to self-transition until timeout

PPFsaStart
	btfsc	STATUS,C	;If for some reason the start bit is a 1, ignore
	retlw	low PPFsaStart	; it
	movlw	0x80		;Otherwise, initialize shift register with a
	movwf	PP_SR		; sentinel bit, ready to take data LSb first
	retlw	low PPFsaBitP	;Transition to accept first bit

PPFsaBitP
	rrf	PP_SR,F		;Rotate received bit (in C) into SR from left
	btfsc	STATUS,C	;If a 1 fell out of the SR, we completed a byte,
	bra	PPFBiP0		; skip ahead
	btfsc	PP_SR,7		;If not, transition to receive next bit, keeping
	retlw	low PPFsaBitN	; track of parity as we go (if we just got a 1,
	retlw	low PPFsaBitP	; swap expected parity)
PPFBiP0	btfsc	PP_SR,7		;If we completed a byte, transition to expect
	retlw	low PPFsaParN	; the appropriate parity bit
	retlw	low PPFsaParP	; "

PPFsaBitN
	rrf	PP_SR,F		;Rotate received bit (in C) into SR from left
	btfsc	STATUS,C	;If a 1 fell out of the SR, we completed a byte,
	bra	PPFBiN0		; skip ahead
	btfsc	PP_SR,7		;If not, transition to receive next bit, keeping
	retlw	low PPFsaBitP	; track of parity as we go (if we just got a 1,
	retlw	low PPFsaBitN	; swap expected parity)
PPFBiN0	btfsc	PP_SR,7		;If we completed a byte, transition to expect
	retlw	low PPFsaParP	; the appropriate parity bit
	retlw	low PPFsaParN	; "

PPFsaParP
	btfsc	STATUS,C	;If we got the expected parity bit, transition
	retlw	low PPFsaStop	; to expect the stop bit, else ignore this byte
	retlw	low FsaIgnore	; "

PPFsaParN
	btfss	STATUS,C	;If we got the expected parity bit, transition
	retlw	low PPFsaStop	; to expect the stop bit, else ignore this byte
	retlw	low FsaIgnore	; "

PPFsaStop
	btfss	STATUS,C	;Stop bit should be high; if it's not, something
	retlw	low FsaIgnore	; is wrong, wait for bus timeout
	;fall through

PPFsaForce
	clrf	PC_T1U		;Reset the PS/2 mouse timeout timer
	movf	PM_FSAP,W	;Resume the PS/2 mouse state machine
	callw			; "
	movwf	PM_FSAP		;On returning, save the address returned in W
	btfsc	STATUS,C	;If carry bit is set (default), transition to
	retlw	low PPFsaStart	; receive the next byte
	movlb	1		;If carry bit was cleared, transmit requested
	bcf	TRISA,PMC_PIN	;Pull clock low for at least 100 us
	DELAY	150		; "
	bsf	TRISA,PMC_PIN	;Release clock, pull data low, and wait for
	bcf	TRISA,PMD_PIN	; device to start clocking
	movlb	7		;Clear the interrupt created by the falling edge
	bcf	IOCAF,PMC_PIN	; of the clock so we don't trigger on it
	retlw	low PPFsaXmit	; "

PPFsaXmit
	bsf	STATUS,C	;Rotate a sentinel bit into the shift register,
	rrf	PP_SR,F		; rotate the first bit to send out
	movlb	1		;Rejoin below to send first bit
	bra	PPFXP0		; "

PPFsaXmitP
	movlb	1		;Ready to pull data low or release it
	lsrf	PP_SR,F		;Rotate the next bit to send out
	btfsc	STATUS,Z	;If we rotated the sentinel bit out, skip ahead
	bra	PPFXP2		; to transmit a positive parity bit
PPFXP0	btfss	STATUS,C	;If bit is a one, release data pin and invert
	bra	PPFXP1		; current parity
	bsf	TRISA,PMD_PIN	; "
	retlw	low PPFsaXmitN	; "
PPFXP1	bcf	TRISA,PMD_PIN	;If bit is a zero, pull data pin low and keep
	retlw	low PPFsaXmitP	; current parity
PPFXP2	bsf	TRISA,PMD_PIN	;Release data pin to send positive parity bit
	retlw	low PPFsaXmitSp	;Transition to set up stop bit next

PPFsaXmitN
	movlb	1		;Ready to pull data low or release it
	lsrf	PP_SR,F		;Rotate the next bit to send out
	btfsc	STATUS,Z	;If we rotated the sentinel bit out, skip ahead
	bra	PPFXN1		; to transmit a negative parity bit
	btfss	STATUS,C	;If bit is a one, release data pin and invert
	bra	PPFXN0		; current parity
	bsf	TRISA,PMD_PIN	; "
	retlw	low PPFsaXmitP	; "
PPFXN0	bcf	TRISA,PMD_PIN	;If bit is a zero, pull data pin low and keep
	retlw	low PPFsaXmitN	; current parity
PPFXN1	bcf	TRISA,PMD_PIN	;Pull data pin low to send negative parity bit
	retlw	low PPFsaXmitSp	;Transition to set up stop bit next

PPFsaXmitSp
	movlb	1		;Release data pin for stop bit
	bsf	TRISA,PMD_PIN	; "
	retlw	low PPFsaXmitAck;Transition to await ack from device

PPFsaXmitAck
	movlb	0		;Check the data line (data and clock are pulled
	btfss	PORTA,PMD_PIN	; low at the same time); if the byte we sent was
	retlw	low PPFsaStart	; acknowledged, await the device's response
	movlw	low PMFsaReset	;Else reset the mouse state machine and rejoin
	movwf	PM_FSAP		; above to enter it
	bra	PPFsaForce	; "

PMFsaReset
	bcf	FLAGS,PMINIT	;Clear mouse state flags
	bcf	FLAGS,PMIDFYD	; "
	bcf	FLAGS,PMWHEEL	; "
	movlw	PMRESET		;Load a reset byte for transmission and flag
	movwf	PP_SR		; that we want it transmitted
	bcf	STATUS,C	; "
	retlw	low PMFsaInit	;Transition to await response

PMFsaInit
	movf	PP_SR,W		;If the byte received is anything but an 0x00
	btfss	STATUS,Z	; (identifying the device as a PS/2 mouse), loop
	retlw	low PMFsaInit	; to wait until we get that
	movlw	low PMFsaInitStr;Point to beginning of mouse init string
	movwf	PM_HEAD		; "
PMFIni0	movf	PM_HEAD,W	;Get the next byte of the mouse init string
	callw			; "
	movwf	PP_SR		;Load it for transmission
	bcf	STATUS,C	; "
	incf	PM_HEAD,F	;Advance the mouse init string pointer
	retlw	low PMFsaAck	;Transition to await acknowledgment

PMFsaAck
	movf	PP_SR,W		;If we didn't get an ack byte from the device,
	xorlw	PMACK		; start initialization sequence over
	btfss	STATUS,Z	; "
	bra	PMFsaReset	; "
	movf	PM_HEAD,W	;Check if the next byte in the init string is a
	callw			; zero
	movf	WREG,W		; "
	btfss	STATUS,Z	;If not, move on to send the next init byte
	bra	PMFIni0		; "
	incf	PM_HEAD,F	;If so, advance past the zero
	btfss	FLAGS,PMIDFYD	;If the device hasn't been identified yet, the
	retlw	low PMFsaIdfy	; next byte coming is the identity byte
	bsf	FLAGS,PMINIT	;Else set the initialized flag and transition to
	retlw	low PMFsaHeader	; await the header of the first packet

PMFsaIdfy
	movf	PP_SR,W		;If the identity byte we received says that this
	xorlw	PMIDWHL		; mouse has a wheel, set the bit accordingly
	btfsc	STATUS,Z	; "
	bsf	FLAGS,PMWHEEL	; "
	bsf	FLAGS,PMIDFYD	;In either case, mouse has been identified
	bra	PMFIni0		;Move on to send the next init byte

PMFsaHeader
	movf	PP_SR,W		;Save the header of the mouse packet for later
	movwf	PM_HEAD		; reference
	movf	PC_FSAP,W	;Call into the mouse configuration state machine
	callw			; "
	movwf	PC_FSAP		; "
	movlb	2		;Lower the quadrature mouse button signal if
	movf	LATA,W		; either the left or right button on the PS/2
	iorlw	1 << MBT_PIN	; mouse is pressed (if bit 0 or bit 1 is set),
	btfss	PM_HEAD,1	; this permits left-handed mouse operation too
	btfsc	PM_HEAD,0	; "
	andlw	~(1 << MBT_PIN)	; "
	movwf	LATA		; "
	retlw	low PMFsaXMove	;Transition to expect X movement byte

PMFsaXMove
	movf	PP_SR,W		;Add the X delta from the mouse to our X delta
	addwf	DELTAXL,F	; twice (registering 400 counts per inch),
	movlw	0		; extending the sign bit from the header
	btfsc	PM_HEAD,4	; "
	movlw	0xFF		; "
	addwfc	DELTAXH,F	; "
	movf	PP_SR,W		; "
	addwf	DELTAXL,F	; "
	movlw	0		; "
	btfsc	PM_HEAD,4	; "
	movlw	0xFF		; "
	addwfc	DELTAXH,F	; "
	bsf	STATUS,C	;Set carry to keep us in receive mode
	retlw	low PMFsaYMove	;Transition to expect Y movement byte

PMFsaYMove
	movf	PP_SR,W		;Add the Y delta from the mouse to our Y delta
	addwf	DELTAYL,F	; twice (registering 400 counts per inch),
	movlw	0		; extending the sign bit from the header
	btfsc	PM_HEAD,5	; "
	movlw	0xFF		; "
	addwfc	DELTAYH,F	; "
	movf	PP_SR,W		; "
	addwf	DELTAYL,F	; "
	movlw	0		; "
	btfsc	PM_HEAD,5	; "
	movlw	0xFF		; "
	addwfc	DELTAYH,F	; "
	bsf	STATUS,C	;Set carry to keep us in receive mode
	btfsc	FLAGS,PMWHEEL	;If the attached mouse has a wheel, transition
	retlw	low PMFsaZMove	; to expect its movement byte, else transition
	retlw	low PMFsaHeader	; to expect header of next packet

PMFsaZMove
	retlw	low PMFsaHeader	;Transition to expect header of next packet

PMFsaInitStr
	dt	PMSAMPR,200	;Set sample rate to 200, then 100, then 80; this
	dt	PMSAMPR,100	; sequence tells PS/2 mice with wheels to enable
	dt	PMSAMPR,80	; wheel position reporting
	dt	PMIDFY,0	;Identify mouse and set wheel flag accordingly
	dt	PMSAMPR,200	;Set sample rate to 200 for real
	dt	PMRESOL,3	;Set resolution to 8 counts/mm (~200 cpi)
	dt	PMSTART,0	;Enable mouse reporting then end init

PCFsaNormal
	clrf	PC_T1U		;Reset the event timer
	btfsc	STATUS,C	;If we're here because of a mouse event, stay in
	retlw	low PCFsaNormal	; this state and do nothing
	btfsc	PM_HEAD,1	;If this was a timer event and both buttons are
	btfss	PM_HEAD,0	; down, the user held them down for four seconds
	bra	PCFNor0		; and wants to enter configuration mode, so wait
	retlw	low PCFsaWaitRls; until they're released
PCFNor0	movlw	low PPFsaForce	;If this was a timer event and both buttons are
	movwf	PP_FSAP		; not held down, we've gone four seconds with no
	movlw	low PMFsaReset	; data from the mouse, so force a reset of it
	movwf	PM_FSAP		; "
	movlb	7		; "
	bsf	IOCAF,PMC_PIN	; "
	retlw	low PCFsaNormal	; "

PCFsaWaitRls
	clrf	PC_T1U		;Reset the event timer
	btfss	STATUS,C	;If we're here because of a timer event, we've
	bra	PCFNor0		; gone four seconds without data, reset mouse
	movf	PM_HEAD,W	;If both buttons have been released, enter
	andlw	B'00000011'	; config mode, else stay here; also make sure
	bcf	PM_HEAD,1	; caller doesn't click quadrature mouse button
	bcf	PM_HEAD,0	; while in config mode
	btfsc	STATUS,Z	; "
	retlw	low PCFsaConfig	; "
	retlw	low PCFsaWaitRls; "

PCFsaConfig
	clrf	PC_T1U		;Reset the event timer
	btfss	STATUS,C	;If we're here because of a timer event, we've
	bra	PCFNor0		; gone four seconds without data, reset mouse
	btfsc	PM_HEAD,0	;If the left button has been pressed, decrease
	bra	PCFCon0		; mouse speed
	btfsc	PM_HEAD,1	;If the right button has been pressed, increase
	bra	PCFCon1		; mouse speed
	bcf	PM_HEAD,1	;Make sure caller doesn't click quadrature mouse
	bcf	PM_HEAD,0	; button while in config mode
	retlw	low PCFsaConfig	;Return, await timeout or mouse click
PCFCon0	incf	MSPEED,F	;Increase the number of counts per quadrature
	btfsc	MSPEED,3	; movement up to a maximum of 7
	decf	MSPEED,F	; "
	retlw	low PCFsaWaitRls;Wait for the left button to be released
PCFCon1	decf	MSPEED,F	;Decrease the number of counts per quadrature
	btfsc	STATUS,Z	; movement down to a minimum of 1
	incf	MSPEED,F	; "
	retlw	low PCFsaWaitRls;Wait for the right button to be released


;;; Subprograms ;;;

ReadPage
	movlb	3		;Save the parameter in W into low byte of
	movwf	PMADRL		; program memory address
	movlw	0x03		;High byte of program memory address is always
	movwf	PMADRH		; the uppermost 256 bytes
	clrf	FSR1L		;Start pointer off at the beginning of buffer
ReadPg0	bcf	INTCON,GIE	;Read a word from program memory
	bsf	PMCON1,RD	; "
	nop			; "
	nop			; "
	bsf	INTCON,GIE	; "
	movf	PMDATH,W	;Copy read word to SRAM
	movwi	FSR1++		; "
	movf	PMDATL,W	; "
	movwi	FSR1++		; "
	incf	PMADRL,F	;Advance program memory address
	btfss	FSR1L,5		;If we haven't yet read out 16 words (32 bytes),
	bra	ReadPg0		; loop to read out the next word
	return			;Done

WritePage
	movlb	3		;Save the parameter in W into low byte of
	movwf	PMADRL		; program memory address
	movlw	0x03		;High byte of program memory address is always
	movwf	PMADRH		; the uppermost 256 bytes
	clrf	FSR0L		;Start pointer off at the beginning of buffer
	bcf	INTCON,GIE	;Interrupts must be off for this whole affair
	bsf	PMCON1,WREN	;Enable writes
	bsf	PMCON1,FREE	;Specify an erase operation
	movlw	0x55		;Execute erase of page
	movwf	PMCON2		; "
	movlw	0xAA		; "
	movwf	PMCON2		; "
	bsf	PMCON1,WR	; "
	nop			; "
	nop			; "
	bsf	PMCON1,LWLO	;Enable writing of latches
WriteP0	moviw	FSR1++		;Copy word from SRAM
	movwf	PMDATH		; "
	moviw	FSR1++		; "
	movwf	PMDATL		; "
	btfsc	FSR0L,5		;If this last word will fill all 16 latches, we
	bcf	PMCON1,LWLO	; want the next write to write to flash for real
	movlw	0x55		;Write to latches or to flash
	movwf	PMCON2		; "
	movlw	0xAA		; "
	movwf	PMCON2		; "
	bsf	PMCON1,WR	; "
	nop			; "
	nop			; "
	incf	PMADRL,F	;Advance program memory address
	btfss	FSR0L,5		;If we haven't yet written out 16 words (32
	bra	WriteP0		; bytes), loop to write out the next word
	bcf	PMCON1,WREN	;Disable writes again
	bsf	INTCON,GIE	;Reenable interrupts
	return			;Done

AddDeltaX
	movf	MSPEED,W	;Add the mouse speed to the X delta
	addwf	DELTAXL,F	; "
	movlw	0		; "
	addwfc	DELTAXH,F	; "
	movf	DELTAXL,W	;If we ticked over to zero, clear carry before
	iorwf	DELTAXH,W	; returning
	btfsc	STATUS,Z	; "
	bcf	STATUS,C	; "
	return			; "

SubDeltaX
	comf	MSPEED,W	;Subtract the mouse speed from the X delta
	addlw	1		; "
	addwf	DELTAXL,F	; "
	movlw	0xFF		; "
	addwfc	DELTAXH,F	; "
	return			; "

AddDeltaY
	movf	MSPEED,W	;Add the mouse speed to the Y delta
	addwf	DELTAYL,F	; "
	movlw	0		; "
	addwfc	DELTAYH,F	; "
	movf	DELTAYL,W	;If we ticked over to zero, clear carry before
	iorwf	DELTAYH,W	; returning
	btfsc	STATUS,Z	; "
	bcf	STATUS,C	; "
	return			; "

SubDeltaY
	comf	MSPEED,W	;Subtract the mouse speed from the Y delta
	addlw	1		; "
	addwf	DELTAYL,F	; "
	movlw	0xFF		; "
	addwfc	DELTAYH,F	; "
	return			; "


;;; Non-Volatile Parameter Storage ;;;

	org	0x3F0

	dw	0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000
	dw	0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000


;;; End of Program ;;;

	end
