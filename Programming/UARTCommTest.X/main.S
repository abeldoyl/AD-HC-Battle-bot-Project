// Hunter Clezie
// RCET3375
// Capstone Project Testing
// https://github.com/abeldoyl/AD-HC-Battle-bot-Project.git

; BattleBot Tank Drive Controller
; PIC16F1788
; UART Receiver with Two Motor PWM Outputs
;
; Based on the 2024 Universal Remote Controller packet:
; 24 ADDRESS 4A JOY1_UD JOY1_LR JOY2_UD
;
; Left motor  = Joystick 1 Up/Down
; Right motor = Joystick 2 Up/Down
;
; Hardware:
; RC2 = CCP1 PWM, Left Motor
; RC1 = CCP2 PWM, Right Motor
; RA0 = Left Motor Direction
; RA1 = Right Motor Direction
; RC7 = UART RX
; RC6 = UART TX (optional)
;
; Driver assumes PHASE/ENABLE inputs (DRV8835).
; It does NOT directly support RZ7886 FI/BI wiring.

; PIC16F1788 Configuration Bits
    CONFIG FOSC = INTOSC
    CONFIG WDTE = OFF
    CONFIG PWRTE = ON
    CONFIG MCLRE = ON
    CONFIG CP = OFF
    CONFIG CPD = OFF
    CONFIG BOREN = ON
    CONFIG CLKOUTEN = OFF
    CONFIG IESO = OFF
    CONFIG FCMEN = OFF
    CONFIG WRT = OFF
    CONFIG PLLEN = OFF
    CONFIG STVREN = ON
    CONFIG BORV = LO
    CONFIG LPBOR = OFF
    CONFIG LVP = OFF

#include <xc.inc>
    ; General Purpouse Registers
    ROBOT_ADDR EQU 0x01       ; Address selected on controller
    rx_state EQU 0x20         ; Current receive state
    rx_byte EQU 0x21          ; Most recent UART byte
    rx_address EQU 0x22       ; Robot address from packet
    rx_command EQU 0x23       ; Command byte
    rx_data1 EQU 0x24         ; First joystick data byte
    rx_data2 EQU 0x25         ; Second joystick data byte
    rx_data3 EQU 0x26         ; Third joystick data byte
    left_value EQU 0x27       ; Left joystick position
    right_value EQU 0x28      ; Right joystick position
    left_duty EQU 0x29        ; Left motor PWM value
    right_duty EQU 0x2A       ; Right motor PWM value
    left_dir EQU 0x2B         ; 0 = forward, 1 = reverse
    right_dir EQU 0x2C        ; 0 = forward, 1 = reverse
    armed EQU 0x2D           ; 1 = motor control enabled
    timeout_count EQU 0x2E   ; Counts 50 ms periods

; Reset and Interrupt Vectors 
PSECT resetVect,class=CODE,delta=2
ResetVector:
    GOTO Setup

PSECT isrVect,class=CODE,delta=2
InterruptVector:
    GOTO ISRHandler

PSECT code,class=CODE,delta=2

Setup:
; Bank 0 Setup
    ; Clear all program variables
    BANKSEL rx_state
    CLRF rx_state
    CLRF rx_byte
    CLRF rx_address
    CLRF rx_command
    CLRF rx_data1
    CLRF rx_data2
    CLRF rx_data3
    CLRF left_value
    CLRF right_value
    CLRF left_duty
    CLRF right_duty
    CLRF left_dir
    CLRF right_dir
    CLRF armed
    CLRF timeout_count

    ; Start output pins LOW
    BANKSEL LATA
    CLRF LATA
    CLRF LATC

    ; Clear interrupt flags
    BANKSEL PIR1
    BCF PIR1,0       ; Clear Timer1 flag
    BCF PIR1,1       ; Clear Timer2 flag

    ; Disable interrupts during setup
    BANKSEL INTCON
    BCF INTCON,7     ; Global interrupts OFF
    BCF INTCON,6     ; Peripheral interrupts OFF

; Bank 1 Setup
    ; Set internal oscillator to 16 MHz
    ; PLL disabled
    BANKSEL OSCCON
    MOVLW 0x78
    MOVWF OSCCON

    ; RA0 and RA1 are motor direction outputs
    BANKSEL TRISA
    BCF TRISA,0
    BCF TRISA,1

    ; Keep PWM outputs disabled during setup
    BANKSEL TRISC
    BSF TRISC,1      ; RC1 / CCP2
    BSF TRISC,2      ; RC2 / CCP1

    ; UART pin directions
    BCF TRISC,6      ; RC6 / TX
    BSF TRISC,7      ; RC7 / RX

    ; Timer2 period
    ; PWM Frequency = 3906.25 Hz
    BANKSEL PR2
    MOVLW 0xFF
    MOVWF PR2

    ; Enable Timer1 interrupt
    BANKSEL PIE1
    BSF PIE1,0       ; TMR1IE = 1

; Bank 2 Setup
    ; Set PORTA and PORTC to digital
    BANKSEL ANSELA
    CLRF ANSELA
    CLRF ANSELC

    ; Default UART and CCP pin locations
    ; CCP1 = RC2
    ; CCP2 = RC1
    ; TX = RC6
    ; RX = RC7
    BANKSEL APFCON1
    CLRF APFCON1

; UART Setup
    ; 115200 Baud at 16 MHz
    ; BRGH = 1, BRG16 = 1
    ; Actual baud = approximately 114286
    BANKSEL BAUD1CON
    MOVLW 0x08
    MOVWF BAUD1CON
    BANKSEL SP1BRGH
    CLRF SP1BRGH
    BANKSEL SP1BRGL
    MOVLW 34
    MOVWF SP1BRGL

    ; Asynchronous UART, 8-bit
    ; High-speed transmitter enabled
    BANKSEL TX1STA
    MOVLW 0x24
    MOVWF TX1STA

    ; Enable serial port and receiver
    BANKSEL RC1STA
    MOVLW 0x90
    MOVWF RC1STA

; PWM Setup
    ; Start both motors stopped
    BANKSEL CCPR1L
    CLRF CCPR1L
    BANKSEL CCPR2L
    CLRF CCPR2L

    ; Enable CCP1 PWM mode
    BANKSEL CCP1CON
    MOVLW 0x0C
    MOVWF CCP1CON

    ; Enable CCP2 PWM mode
    BANKSEL CCP2CON
    MOVLW 0x0C
    MOVWF CCP2CON

; Timer1 Setup
    ; 16 MHz oscillator = 4 MHz instruction clock
    ; Timer1 prescaler = 1:8
    ; Each tick = 2 microseconds
    ; 50 ms = 25000 counts
    ; 65536 - 25000 = 0x9E58
    BANKSEL T1CON
    CLRF T1CON      ; Timer1 OFF during setup
    BANKSEL T1GCON
    CLRF T1GCON
    BANKSEL TMR1H
    MOVLW 0x9E
    MOVWF TMR1H
    MOVLW 0x58
    MOVWF TMR1L

; Start Timers
    ; Clear old timer interrupt flags
    BANKSEL PIR1
    BCF PIR1,0       ; Clear Timer1 flag
    BCF PIR1,1       ; Clear Timer2 flag

    ; Start Timer2 for PWM
    BANKSEL T2CON
    MOVLW 0x04       ; Prescaler 1:1, Timer2 ON
    MOVWF T2CON

WaitPWM:
    ; Wait for first PWM period to finish
    BANKSEL PIR1
    BTFSS PIR1,1
    GOTO WaitPWM

    ; Enable PWM output pins
    BANKSEL TRISC
    BCF TRISC,2      ; CCP1 left motor
    BCF TRISC,1      ; CCP2 right motor

    ; Enable peripheral and global interrupts
    BANKSEL INTCON
    BSF INTCON,6     ; PEIE = 1
    BSF INTCON,7     ; GIE = 1

    ; Start Timer1 last
    BANKSEL T1CON
    MOVLW 0x31       ; FOSC/4, 1:8 prescaler, Timer1 ON
    MOVWF T1CON

GOTO Main

Main:
    CALL ServiceUART
    GOTO Main

; UART Receive
ServiceUART:
    BANKSEL RC1STA

    ; Check for overrun error
    BTFSC RC1STA,1
    GOTO UARTOverrun

    ; Check if a byte was received
    BANKSEL PIR1
    BTFSS PIR1,5
    RETURN

    ; Check framing error before reading data
    BANKSEL RC1STA
    BTFSC RC1STA,2
    GOTO UARTFraming

    ; Read received byte
    BANKSEL RC1REG
    MOVF RC1REG,W
    BANKSEL rx_byte
    MOVWF rx_byte
    CALL ParseUART
    RETURN

UARTOverrun:
    ; Restart the UART receiver
    BCF RC1STA,4
    BSF RC1STA,4
    BANKSEL rx_state
    CLRF rx_state
    RETURN

UARTFraming:
    ; Discard damaged byte
    BANKSEL RC1REG
    MOVF RC1REG,W
    BANKSEL rx_state
    CLRF rx_state
    RETURN

; Packet Parser
ParseUART:
    BANKSEL rx_state

    ; State 0: Wait for start byte
    MOVF rx_state,F
    BTFSC STATUS,2
    GOTO WaitStart

    ; State 1: Get robot address
    MOVF rx_state,W
    XORLW 0x01
    BTFSC STATUS,2
    GOTO GetAddress

    ; State 2: Get command
    MOVF rx_state,W
    XORLW 0x02
    BTFSC STATUS,2
    GOTO GetCommand

    ; State 3: Get Data 1
    MOVF rx_state,W
    XORLW 0x03
    BTFSC STATUS,2
    GOTO GetData1

    ; State 4: Get Data 2
    MOVF rx_state,W
    XORLW 0x04
    BTFSC STATUS,2
    GOTO GetData2

    ; State 5: Get Data 3
    MOVF rx_state,W
    XORLW 0x05
    BTFSC STATUS,2
    GOTO GetData3
    CLRF rx_state
    RETURN

WaitStart:
    ; Look for '$' = 0x24
    MOVF rx_byte,W
    XORLW 0x24
    BTFSS STATUS,2
    RETURN
    MOVLW 0x01
    MOVWF rx_state
    RETURN

GetAddress:
    MOVF rx_byte,W
    MOVWF rx_address
    MOVLW 0x02
    MOVWF rx_state
    RETURN

GetCommand:
    MOVF rx_byte,W
    MOVWF rx_command
    MOVLW 0x03
    MOVWF rx_state
    RETURN

GetData1:
    MOVF rx_byte,W
    MOVWF rx_data1
    MOVLW 0x04
    MOVWF rx_state
    RETURN

GetData2:
    MOVF rx_byte,W
    MOVWF rx_data2
    MOVLW 0x05
    MOVWF rx_state
    RETURN

GetData3:
    MOVF rx_byte,W
    MOVWF rx_data3

    ; All six bytes received
    CLRF rx_state
    CALL CheckPacket
    RETURN

; Packet Checking
CheckPacket:
    BANKSEL rx_address

    ; Check robot address
    MOVF rx_address,W
    XORLW ROBOT_ADDR
    BTFSS STATUS,2
    RETURN

    ; Only use the J packet for tank control
    MOVF rx_command,W
    XORLW 0x4A
    BTFSS STATUS,2
    RETURN

    ; Joystick 1 up/down controls left motor
    MOVF rx_data1,W
    MOVWF left_value

    ; Joystick 2 up/down controls right motor
    MOVF rx_data3,W
    MOVWF right_value

    ; Check whether motors are armed
    MOVF armed,F
    BTFSC STATUS,2
    GOTO CheckArm

    ; Valid control packet received
    CLRF timeout_count
    CALL SetLeftMotor
    CALL SetRightMotor
    RETURN

; Startup/Reconnect Safety
CheckArm:
    ; Both joysticks must be near center
    ; before allowing motors to operate
    ; Left joystick >= 120?
    MOVLW 120
    SUBWF left_value,W
    BTFSS STATUS,0
    RETURN

    ; Left joystick <= 136?
    MOVF left_value,W
    SUBLW 136
    BTFSS STATUS,0
    RETURN

    ; Right joystick >= 120?
    MOVLW 120
    SUBWF right_value,W
    BTFSS STATUS,0
    RETURN

    ; Right joystick <= 136?
    MOVF right_value,W
    SUBLW 136
    BTFSS STATUS,0
    RETURN

    ; Both are centered: arm motor control
    MOVLW 0x01
    MOVWF armed
    CLRF timeout_count
    RETURN

; Left Motor Below Here
SetLeftMotor:
    BANKSEL left_value

    ; Values 137-255 = forward
    MOVLW 137
    SUBWF left_value,W
    BTFSC STATUS,0
    GOTO LeftForward

    ; Values 120-136 = stop
    MOVLW 120
    SUBWF left_value,W
    BTFSC STATUS,0
    GOTO LeftStop

    ; Values 0-119 = reverse
    GOTO LeftReverse

LeftForward:
    ; If direction was reverse, stop first
    BTFSS left_dir,0
    GOTO LeftForwardDuty
    MOVF left_duty,F
    BTFSS STATUS,2
    GOTO LeftStop

    ; Motor stopped: change direction
    BCF left_dir,0
    BANKSEL LATA
    BCF LATA,0

LeftForwardDuty:
    BANKSEL left_value

    ; PWM = (Joystick - 128) * 2
    MOVLW 128
    SUBWF left_value,W
    MOVWF left_duty
    LSLF left_duty,F
    MOVF left_duty,W
    BANKSEL CCPR1L
    MOVWF CCPR1L
    RETURN

LeftReverse:
    ; If already in reverse, continue
    BTFSC left_dir,0
    GOTO LeftReverseDuty

    ; Stop before changing direction
    MOVF left_duty,F
    BTFSS STATUS,2
    GOTO LeftStop
    BSF left_dir,0
    BANKSEL LATA
    BSF LATA,0

LeftReverseDuty:
    BANKSEL left_value

    ; Special case: 0 maps to maximum duty
    MOVF left_value,F
    BTFSC STATUS,2
    GOTO LeftReverseFull

    ; PWM = (128 - Joystick) * 2
    MOVF left_value,W
    SUBLW 128
    MOVWF left_duty
    LSLF left_duty,F
    GOTO LeftReverseWrite

LeftReverseFull:
    MOVLW 0xFF
    MOVWF left_duty

LeftReverseWrite:
    MOVF left_duty,W
    BANKSEL CCPR1L
    MOVWF CCPR1L
    RETURN

LeftStop:
    BANKSEL left_duty
    CLRF left_duty
    BANKSEL CCPR1L
    CLRF CCPR1L
    RETURN

; Right Motor Below Here
SetRightMotor:
    BANKSEL right_value

    ; Values 137-255 = forward
    MOVLW 137
    SUBWF right_value,W
    BTFSC STATUS,0
    GOTO RightForward

    ; Values 120-136 = stop
    MOVLW 120
    SUBWF right_value,W
    BTFSC STATUS,0
    GOTO RightStop

    ; Values 0-119 = reverse
    GOTO RightReverse

RightForward:
    ; Stop first if changing from reverse
    BTFSS right_dir,0
    GOTO RightForwardDuty
    MOVF right_duty,F
    BTFSS STATUS,2
    GOTO RightStop
    BCF right_dir,0
    BANKSEL LATA
    BCF LATA,1

RightForwardDuty:
    BANKSEL right_value
    MOVLW 128
    SUBWF right_value,W
    MOVWF right_duty
    LSLF right_duty,F
    MOVF right_duty,W
    BANKSEL CCPR2L
    MOVWF CCPR2L
    RETURN

RightReverse:
    BTFSC right_dir,0
    GOTO RightReverseDuty

    ; Stop before reversing
    MOVF right_duty,F
    BTFSS STATUS,2
    GOTO RightStop
    BSF right_dir,0
    BANKSEL LATA
    BSF LATA,1

RightReverseDuty:
    BANKSEL right_value
    MOVF right_value,F
    BTFSC STATUS,2
    GOTO RightReverseFull
    MOVF right_value,W
    SUBLW 128
    MOVWF right_duty
    LSLF right_duty,F
    GOTO RightReverseWrite

RightReverseFull:
    MOVLW 0xFF
    MOVWF right_duty

RightReverseWrite:
    MOVF right_duty,W
    BANKSEL CCPR2L
    MOVWF CCPR2L
    RETURN

RightStop:
    BANKSEL right_duty
    CLRF right_duty
    BANKSEL CCPR2L
    CLRF CCPR2L
    RETURN

; Motor Shutdown Below Here
StopMotors:
    ; Shut off both PWM outputs
    BANKSEL CCPR1L
    CLRF CCPR1L

    BANKSEL CCPR2L
    CLRF CCPR2L

    BANKSEL left_duty
    CLRF left_duty
    CLRF right_duty
    CLRF left_dir
    CLRF right_dir

    ; Return direction signals LOW
    BANKSEL LATA
    BCF LATA,0
    BCF LATA,1

    RETURN

; Timer1 Interrupt/Failsafe
ISRHandler:
    ; Enhanced PIC automatically saves
    ; key CPU context on interrupt
    BANKSEL PIR1
    BTFSS PIR1,0
    RETFIE

    ; Clear Timer1 flag
    BCF PIR1,0

    ; Reload for approximately 50 ms
    BANKSEL TMR1H
    MOVLW 0x9E
    MOVWF TMR1H
    MOVLW 0x58
    MOVWF TMR1L

    ; Nothing to do if motors aren't armed
    BANKSEL armed
    MOVF armed,F
    BTFSC STATUS,2
    RETFIE

    ; Count another 50 ms
    INCF timeout_count,F

    ; Has 250 ms passed?
    MOVLW 5
    SUBWF timeout_count,W
    BTFSS STATUS,0
    RETFIE

    ; Communication timeout
    ; Stop motors and require rearming
    CLRF armed
    CALL StopMotors
    RETFIE

END
