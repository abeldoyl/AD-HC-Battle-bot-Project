; PIC16F1788 Configuration Bit Settings

; CONFIG1
  CONFIG  FOSC = INTOSC         ; Internal oscillator, I/O function on CLKIN pin
  CONFIG  WDTE = OFF            ; Watchdog Timer disabled
  CONFIG  PWRTE = OFF           ; Power-up Timer disabled
  CONFIG  MCLRE = ON            ; MCLR pin function is MCLR
  CONFIG  CP = OFF              ; Code protection disabled
  CONFIG  BOREN = OFF           ; Brown-out Reset disabled
  CONFIG  CLKOUTEN = OFF        ; CLKOUT function disabled
  CONFIG  IESO = OFF            ; Internal/External Switchover disabled
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor disabled

; CONFIG2
  CONFIG  WRT = OFF             ; Flash write protection off
  CONFIG  PLLEN = OFF           ; 4x PLL disabled
  CONFIG  STVREN = ON           ; Stack over/underflow causes Reset
  CONFIG  LVP = OFF             ; Low-voltage programming disabled

; config statements should precede project file includes.
#include <xc.inc>
val      EQU 0x70         ;shared RAM, ADC result (8 bit)
count    EQU 0x71         ;shared RAM, acquisition delay counter

; Reset Vector at 0000h. Execution starts here after reset.
PSECT resetVect,class=CODE,delta=2
ResetVector:
    goto Setup

; Main code section
PSECT code,class=CODE,delta=2

Setup:
    ; Bank 1
    BANKSEL OSCCON
    MOVLW 0x72
    MOVWF OSCCON    ;Internal oscillator at 8 MHz (IRCF = 1110)
    MOVLW 0x01
    MOVWF TRISA     ;RA0 input (pot), rest of PORTA outputs
    BCF TRISB,1     ;RB1 output (L293D 1A, direction)
    BCF TRISB,2     ;RB2 output (L293D 2A, direction)
    BCF TRISC,2     ;RC2 output (CCP1 PWM to L293D enable)
    MOVLW 0x50
    MOVWF ADCON1    ;ADFM = 0 (left justified), ADCS = 101 (FOSC/16), VDD/VSS references

    ; Bank 2
    BANKSEL LATB
    CLRF LATA       ;Clear output latches
    CLRF LATB
    CLRF LATC

    ; Bank 3
    BANKSEL ANSELA
    MOVLW 0x01
    MOVWF ANSELA    ;RA0 (AN0) analog, rest of PORTA digital
    CLRF ANSELB     ;PORTB digital I/O

    ; Bank 0
    BANKSEL PR2
    MOVLW 0xFF
    MOVWF PR2       ;PWM period = 255, about 7.8 kHz at 8 MHz
    MOVLW 0x04
    MOVWF T2CON     ;Timer2 on, prescaler 1:1

    ; Bank 5
    BANKSEL CCPR1L
    CLRF CCPR1L     ;Duty cycle = 0 (motor off)
    MOVLW 0x0C
    MOVWF CCP1CON   ;CCP1 in PWM mode

    ; Bank 1
    BANKSEL ADCON2
    MOVLW 0x0F      ;CHSN = 1111 (ADNREF, single-ended); TRIGSEL = 0000
    MOVWF ADCON2
    MOVLW 0x01
    MOVWF ADCON0    ;CHS = AN0, ADC on
    goto Main

Main:
    BANKSEL ADCON0
    MOVLW 10
    MOVWF count     ;Acquisition delay
    DECFSZ count,F
    goto $-1

    BSF ADCON0,1    ;Start conversion (GO)
    BTFSC ADCON0,1  ;Is conversion done?
    goto $-1

    MOVF ADRESH,W   ;0 to 255 (center = 128)
    MOVWF val

    ; Deadband: stop if val is 118 to 137
    MOVLW 158
    SUBWF val,W     ;C = 1 if val >= 158
    BTFSC STATUS,0
    goto Forward
    MOVLW 98
    SUBWF val,W     ;C = 0 if val < 98
    BTFSS STATUS,0
    goto Reverse

Stop:
    BANKSEL LATB
    BCF LATB,1
    BCF LATB,2
    CLRW            ;Duty = 0
    goto SetDuty

Forward:
    BANKSEL LATB
    BSF LATB,1      ;1A high, 2A low = forward
    BCF LATB,2
    MOVLW 128
    SUBWF val,W     ;W = val - 128 (10 to 127)
    MOVWF val
    LSLF val,W      ;x2 = duty (20 to 254)
    goto SetDuty

Reverse:
    BANKSEL LATB
    BCF LATB,1      ;1A low, 2A high = reverse
    BSF LATB,2
    MOVF val,W
    SUBLW 127       ;W = 127 - val (10 to 127)
    MOVWF val
    LSLF val,W      ;x2 = duty (20 to 254)

SetDuty:
    BANKSEL CCPR1L
    MOVWF CCPR1L    ;Set PWM duty cycle
    goto Main
End