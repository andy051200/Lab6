;------------------------------------------------------------------------------
;Archivo: lab6.s
;Microcontrolador: PIC16F887
;Autor: Andy Bonilla
;Compilador: pic-as (v2.30), MPLABX v5.45
    
;Programa: Laboratorio 6
;Hardware: PIC16F887
    
;Creado: 23 de marzo de 2021    
;Descripcion: incrementar variable con Tmr1, luz y displays titilantes con Tmr2  
; displays multiplexados con tmr0
;------------------------------------------------------------------------------

;---------libreras a emplementar-----------------------------------------------
PROCESSOR 16F887
#include <xc.inc>
    
;------configuration word 1----------------------------------------------------
CONFIG  FOSC=INTRC_NOCLKOUT ;se declara osc interno
CONFIG  WDTE=OFF            ; Watchdog Timer apagado
CONFIG  PWRTE=ON            ; Power-up Timer prendido
CONFIG  MCLRE=OFF           ; MCLRE apagado
CONFIG  CP=OFF              ; Code Protection bit apagado
CONFIG  CPD=OFF             ; Data Code Protection bit apagado

CONFIG  BOREN=OFF           ; Brown Out Reset apagado
CONFIG  IESO=OFF            ; Internal External Switchover bit apagado
CONFIG  FCMEN=OFF           ; Fail-Safe Clock Monitor Enabled bit apagado
CONFIG  LVP=ON		    ; low voltaje programming prendido

;----------configuration word 2-------------------------------------------------
CONFIG BOR4V=BOR40V	    ;configuraciÃ³n de brown out reset
CONFIG WRT = OFF	    ;apagado de auto escritura de cÃ?Â³digo
    
;---------------------macros---------------------------------------------------
;configuracion de macro para reinicio del timer0
reset_timer0	macro	    ; lo que anteriormente fue subrutina, se hizo macro
    movlw	223	    ; dada la configuración del prescaler
    movwf	TMR0	    ; se guarda en timer0
    bcf		T0IF	    ; bandera cuando no hay overflow
    endm       
    
;configuracion de macro para reinicio del timer1     
reset_timer1 macro
    movlw   0x0B	;se mueven MSB a TM1H
    movwf   TMR1H
    movlw   0xDC	;se mueven LSB a TMR1L
    movwf   TMR1L
    bcf	    TMR1IF	;PIR1, 0 ;bajo la bandera
    endm   

;--------------------- variables ----------------------------------------------
PSECT udata_bank0
    ;contadora para timer1
    cont_timer1:    DS 1
    cont_timer0:    DS 1
    cont_timer2:    DS 1 
    ;variable para el multiplexado
    banderas:	    DS 1
    nibble:	    DS 2 ; me lleva la cuenta de los displays
    display_var:    DS 2
        
;---------------------- variables de interrupcion -----------------------------
PSECT udata_shr	    
    W_TEMP:	    DS 1
    STATUS_TEMP:    DS 1  
    
    
;--------------------------- reset vector -------------------------------------
PSECT resVect, class=CODE, abs, delta=2 ;
ORG 00h
resetVector:
    PAGESEL main
    goto main
    
;------------------------ interrupt vector ------------------------------------
PSECT	intVect, class=code, abs, delta=2 
ORG 04h
push:
    movwf	W_TEMP
    swapf	STATUS, W
    movwf	STATUS_TEMP
       
isr:
    btfsc	TMR1IF		;ver si Interrupcion Tmr1 está prendida
    call    	suma_timer1
    
    btfsc	T0IF		;ver si interrupcion Tmr0 está prendida
    call	display_timer0
    
    btfsc 	TMR2IF		;ver si Interrupcion Tmr2 está prendida
    call	led_timer2
     
pop:
    swapf	STATUS_TEMP
    movwf	STATUS
    swapf	W_TEMP, F
    swapf	W_TEMP, W
    retfie            
    
;-------------------------- subrutinas de interruption ----------------------- 
;incremento de variable con timer1
suma_timer1:
    banksel	PORTA
    incf	cont_timer1
   ; incf	PORTB
    reset_timer1
    return
    
;led intermitente con timer2 
led_timer2:
    ;incf	PORTB
    btfsc	cont_timer2,0
    goto	luzprendida
    goto	luzapagado
    
luzprendida:
    bsf		PORTA,0
    bsf		PORTD,0
    bsf		PORTD,1
    goto	cambioled
    
luzapagado:
    bcf		PORTA,0
    bcf		PORTD,0
    bcf		PORTD,1
    goto	cambioled
    
cambioled:    
    movlw	1
    xorwf	cont_timer2,F
    bcf		TMR2IF
    return

    
;multiplexada de displays con timer0
display_timer0:
    reset_timer0
    clrf	PORTD
    btfsc	banderas,0
    goto	display0
    goto	display1
    
display0:
    movf	display_var, W
    movwf	PORTC
    bsf		PORTD, 1
    goto	otro_display
    
display1:
    movf	display_var+1, W
    movwf	PORTC
    bsf		PORTD, 0
    goto	otro_display
    
otro_display:    
    movlw	1
    xorwf	banderas,F
    bcf		T0IF
    return

;mux_timer0:

;----------------------------- codigo principal -------------------------------
PSECT code, delta=2, abs
ORG 100h
tabla:
    clrf    PCLATH	    ; asegurarase de estar en secciÃ³n
    bsf	    PCLATH, 0 	    ; 
    andlw   0x0f	    ; se eliminan los 4 MSB y se dejan los 4 LSB
    addwf   PCL, F	    ; se guarda en F
    retlw   00111111B	    ; 0
    retlw   00000110B	    ; 1
    retlw   01011011B	    ; 2
    retlw   01001111B	    ; 3
    retlw   01100110B	    ; 4
    retlw   01101101B	    ; 5 
    retlw   01111101B	    ; 6
    retlw   00000111B	    ; 7
    retlw   01111111B	    ; 8
    retlw   01101111B	    ; 9
    retlw   01110111B	    ; A
    retlw   01111100B	    ; B
    retlw   00111001B	    ; C
    retlw   01011110B	    ; D
    retlw   01111001B	    ; E
    retlw   01110001B	    ; F
 
;--------------------------configuraciones ------------------------------------
main:    
    clrf    cont_timer1	    ; asegurar que empieza en 0
    clrf    cont_timer0	    ; asegurar que empieza en 0
    ;se llaman las subrutinas de configuración
    call    io_config
    call    reloj_config
    call    timer0_config
    call    timer1_config
    call    timer2_config
    call    interruptiones_config
    
    
       
;------------------------ loop de programa ------------------------------------
loop:
    call	separar_nibbles
    call	variables_displays
    goto	loop

;------------------------ subrutinas regulares --------------------------------
io_config:
    banksel	ANSEL
    clrf	ANSEL
    clrf	ANSELH
   
    banksel	TRISA
    clrf	TRISA	    ; PortA como salida, led intermitente  
    clrf	TRISB	    ; PortA como salida, led intermitente  
    clrf	TRISC	    ; POrtC como salida, displays
    bcf		TRISD,0	    ; PortD como salida, transistores
    bcf		TRISD,1	    ; PortD como salida, transistores
    
    banksel	PORTA
    clrf	PORTA	    ; PortA como salida, led intermitente  
    clrf	PORTB	    ; PortA como salida, led intermitente  
    clrf	PORTC	    ; PortA como salida, displays  
    bcf		PORTD,0	    ; PortA como salida, transistores
    bcf		PORTD,1	    ; PortA como salida, transistores
    return
    
reloj_config:
    banksel	OSCCON	    ;se llama al registro
    bsf		OSCCON,0    ;reloj interno
    bcf		OSCCON,4    ; freq de oscilacion 1MHz, 110
    bcf		OSCCON,5    ; freq de oscilacion 1MHz, 110
    bsf		OSCCON,6    ; freq de oscilacion 1MHz, 110
    return
 
timer0_config:
    banksel	TRISA
    banksel	OPTION_REG
    bsf		OPTION_REG,0	; prescaler, 111 1:256
    bsf		OPTION_REG,1	; prescaler, 111 1:256
    bsf		OPTION_REG,2	; prescaler, 111 1:256
    bcf		OPTION_REG,3    ; preescaler a WDT apagado
    bcf		OPTION_REG,4	; oscilador interno encendido
    reset_timer0
    return
    
timer1_config:
    banksel TRISA
    ;bsf	    PIE1, 0	; se prende la interrupción del TMR1
    banksel T1CON
    bsf	    TMR1ON	;se prende el timer1
    bcf	    TMR1CS	; se activa el temporizador con intosc
    bcf	    T1SYNC	; sincronizacion apagada
    bsf	    T1CON, 3	; oscilador baja potencia
    bsf	    T1CON, 4	; prescaler 0, 11-> 1:8
    bsf	    T1CON, 5	; prescaler 1, 11-> 1:8
    bcf	    TMR1GE	; Gate enable apagado
    bcf	    T1GINV	;gate inverter apagado
    reset_timer1
    return

timer2_config:
    ;banksel	PORTA
    banksel	T2CON
    bsf		T2CON,0    ; configuracion prescaler tmr2 11 1:16
    bsf		T2CON,1    ; configuracion prescaler tmr2 11 1:16
    bsf		T2CON,2	   ; se enciende el timer2
    bsf		T2CON,3    ;configuracion del postscaler 1111 1:16
    bsf		T2CON,4    ;configuracion del postscaler 1111 1:16
    bsf		T2CON,5    ;configuracion del postscaler 1111 1:16
    bsf		T2CON,6    ;configuracion del postscaler 1111 1:16
    banksel	TRISA
    movlw	244
    movwf	PR2
    return
    
interruptiones_config:
    banksel	PORTA
    ;interrupciones del Timer0
    banksel	INTCON
    bcf		INTCON,2    ; interrupcion del tmr0
    bsf		INTCON,5    ; enable bit de interrupcion Tmr0
    bsf		INTCON,7    ; encendido interrupciones globales
    
    ;interrupciones del timer1 y timer2
    banksel	PIE1	 
    bsf		PIE1,0	    ; enable bit de interrupcion tmr1, encendido
    bsf		PIE1,1	    ; enable bit de interrupcion tmr2, encendido
    ;el resto van apagados
    
    banksel	PIR1
    bcf		PIR1,0	    ; interrupcion tmr1 encendida
    bcf		PIR1,1	    ; interrupcion tmr2 encendida 
    ;el resto van apagados
    return    

separar_nibbles:
    movf    PORTB,W
    andlw   0x0F
    movwf   nibble
    swapf   PORTB, W
    andlw   0x0F
    movwf   nibble+1
    return
 
variables_displays:
    ;msb
    movf	nibble+1, W	; display bit0 contador hex
    call	tabla		; llamo al valor de la tabla
    movwf	display_var+1	; muevo variable display_var+1 a f
    ;lsb
    movf	nibble, W	; display bit1 contador hex
    call	tabla		; llamo al valor de la tabla
    movwf	display_var	; muevo variable display_var a f
    return          
    
    
END


