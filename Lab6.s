
; Laboratorio 6
; Programación de Microcontroladores
; Herbert Barrios, Carné: 20199

; Compilador:	pic-as (v2.35), MPLABX V6.00

    
PROCESSOR 16F887
    
; PIC16F887 Configuration Bit Settings

; Assembly source line config statements

; CONFIG1
  CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
  CONFIG  PWRTE = ON            ; Power-up Timer Enable bit (PWRT enabled)
  CONFIG  MCLRE = OFF           ; RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
  CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
  CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection is disabled)
  CONFIG  BOREN = OFF           ; Brown Out Reset Selection bits (BOR disabled)
  CONFIG  IESO = OFF            ; Internal External Switchover bit (Internal/External Switchover mode is disabled)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
  CONFIG  LVP = ON              ; Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

; CONFIG2
  CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
  CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)

// config statements should precede project file includes.
#include <xc.inc>
  
 RESET_TMR0 MACRO TMR_VAR
    BANKSEL TMR0	    ; Cambiamos de banco
    MOVLW   TMR_VAR
    MOVWF   TMR0	    ; Configuración tiempo de retardo a 10 ms
    BCF	    T0IF	    ; Limpiamos bandera de interrupción
    ENDM
    
 RESET_TMR1 MACRO TMR1_H,TMR1_L  
    BANKSEL	TMR1H
    MOVLW	TMR1_H
    MOVWF	TMR1H
    MOVLW	TMR1_L
    MOVWF	TMR1L
    BCF		TMR1IF	    ; Limpiamos la bandera de interrupción del Timer .
    ENDM
  
; ------- VARIABLES EN MEMORIA --------
PSECT udata_shr		    ; Memoria compartida
    W_TEMP:		DS 1
    STATUS_TEMP:	DS 1
    
PSECT udata_bank0
    contador_segundos:	DS 1	   ; Contador en el PORTA
    valor:		DS 1	   ; Valor el cual se va a querer mostrar en los displays.
    numero:	        DS 2	   ; Utiliza tres bytes para mostrar el número deseado, capaz de llegar hasta 255.
    display:		DS 2	   ; Muestra las centenas, las decenas y las unidades en el display.
    banderas:		DS 1	   ; Indica cuál de los displays debe encenderse.
    
PSECT resVect, class=CODE, abs, delta=2
ORG 00h			    ; posición 0000h para el reset
;------------ VECTOR RESET --------------
resetVec:
    PAGESEL MAIN	    ; Cambio de pagina
    GOTO    MAIN
    
PSECT intVect, class=CODE, abs, delta=2
ORG 04h			    ; posición 0004h para interrupciones
;------- VECTOR INTERRUPCIONES ----------
PUSH:
    MOVWF   W_TEMP	    ; Guardamos W
    SWAPF   STATUS, W
    MOVWF   STATUS_TEMP	    ; Guardamos STATUS
    
ISR:
 
    BTFSC   T0IF	    ; Se verifica la bandera del TMR0
    CALL    INT_TMR0	    ; Llamamos a la subrutina de interrupción
    BTFSC   TMR1IF	    ; Interrupcion de TMR1
    CALL    INT_TMR1
    BTFSC   TMR2IF	    ; Interrupcion de TMR2
    CALL    INT_TMR2
    

    
    ;--------------------------------------------------------------------
    ; En caso de tener habilitadas varias interrupciones hay que evaluar
    ;	el estado de todas las banderas de las interrupciones habilitadas
    ;	para identificar que interrupción fue la que se activó.
    
    ;BTFSC   T0IF	    ; Fue interrupción del TMR0? No=0 Si=1
    ;CALL    INT_TMR0	    ; Si -> Subrutina o macro con codigo a ejecutar
			    ;	cuando se active interrupción de TMR0
    
    ;BTFSC   RBIF	    ; Fue interrupción del PORTB? No=0 Si=1
    ;CALL    INT_PORTB	    ; Si -> Subrutina o macro con codigo a ejecutar
			    ;	cuando se active interrupción de PORTB
    ;---------------------------------------------------------------------
    
POP:
    SWAPF   STATUS_TEMP, W  
    MOVWF   STATUS	    ; Recuperamos el valor de reg STATUS
    SWAPF   W_TEMP, F	    
    SWAPF   W_TEMP, W	    ; Recuperamos valor de W
    RETFIE		    ; Regresamos a ciclo principal
    
    ORG 200h
    
;-----------Tablas---------------------------
Tabla:
   CLRF    PCLATH	  ; Limpiamos registro PCLATH
   BSF	   PCLATH, 1	  ; Posicionamos el PC en dirección 02xxh
   ANDLW   0x0F		  ; No saltar más del tamaño de la tabla
   ADDWF   PCL , F	  ; Apuntamos el PC a caracter  PC= PCLATH+PCL +W
   RETLW   00111111B	  ; ASCII char 0
   RETLW   00000110B	  ; ASCII char 1
   RETLW   01011011B	  ; ASCII char 2
   RETLW   01001111B	  ; ASCII char 3
   RETLW   01100110B	  ; ASCII char 4
   RETLW   01101101B	  ; ASCII char 5
   RETLW   01111101B	  ; ASCII char 6
   RETLW   00000111B	  ; ASCII char 7
   RETLW   01111111B	  ; ASCII char 8
   RETLW   01101111B	  ; ASCII char 9
   RETLW   00111111B	  ; ASCII char 0    
   
   
PSECT code, delta=2, abs
ORG 250h		    ; posición 100h para el codigo
;------------- CONFIGURACION ------------
MAIN:
    CALL    CONFIG_IN_OUTS		    ; Configuración de I/O
    CALL    CONFIG_RELOJ		    ; Configuración de Oscilador
    CALL    CONFIG_TMR0			    ; Configuración del timer - TMR0
    CALL    CONFIG_TMR1			    ; Configuración de TMR1
    CALL    CONFIG_TMR2			    ; Configuración de TMR2
    CALL    CONFIG_INTERRUPCIONES	    ; Configuración de interrupciones
    BANKSEL PORTD			    ; Cambio a banco 00
    
LOOP:
    MOVF    PORTA, W		; Movemos el valor en el PORTA a W
    MOVWF   valor		; Movemos el valor de W a la variable de "valor"
    CALL    obtener_numero	; Se guardan los nibble bajo y alto.
    CALL    set_display		; Se guardan los valores para luego poder mostrarlos como valores en hexadecimal.
    CLRF    numero		; Limpiamos en el primer espacio de numero
    CLRF    numero+1		; Limpiamos en el primer espacio de numero
    GOTO    LOOP		; Nos mantenemos en el loop	    
    
;------------- SUBRUTINAS DE INTERRUPCIÓN ---------------
INT_TMR0:
    RESET_TMR0	252		    ;Tiempo = (10 * 10'-3) = (4 * 1/4*10'6)*(256-x)*256 = 252 y llamamos al macro del Timer_reset
    CALL    mostrar_valor
    RETURN
    
INT_TMR1:
    RESET_TMR1	0xC2, 0xF7	     ; TMR1 a 1000ms
    INCF    PORTA
   /* INCF   contador_segundos         ; Incrementamos en 1 y almacenamos el valor en el registro F
    MOVF   contador_segundos, 0	     ; Movemos el valor al registro W
    MOVWF  PORTA */		     ; Movemos el valor del registro al PORTA
    RETURN
    
INT_TMR2:
    BCF	    TMR2IF	    ; Limpiamos bandera de interrupcion de TMR1
    INCF    PORTB	    ; Incremento en PORTB
    RETURN
    
;------------- SUBRUTINAS ---------------
CONFIG_RELOJ:
    BANKSEL OSCCON	    ; cambiamos a banco 1
    BSF	    OSCCON, 0	    ; SCS -> 1, Usamos reloj interno
    BCF	    OSCCON, 6
    BSF	    OSCCON, 5
    BSF	    OSCCON, 4	    ; IRCF<2:0> -> 500 khz
    RETURN
    
CONFIG_TMR0:
   BANKSEL  OPTION_REG	    ; Cambiamos de banco
   BCF	    T0CS	    ; Establecemos el Timer0 como temporizador
   BCF	    PSA   	    ; Prescaler a TIMER0
   BSF	    PS2		    ; PS2 = 1
   BCF	    PS1		    ; PS1 = 1
   BSF	    PS0		    ; PS0 Prescaler de 1 : 256
    
   BANKSEL  TMR0	    ; Cambiamos de banco
   MOVLW    252		    ; Valor cargado para obtener un retraso de 2 ms.
   MOVWF    TMR0	    
   BCF	    T0IF	    ; Se limpia la bandera de interrupción
   RETURN 
; Cada vez que se cumple el tiempo del timer0 se debe de reiniciar   

CONFIG_TMR1:
    BANKSEL	T1CON
    BCF		TMR1CS	    ; Usar reloj interno
    BCF		T1OSCEN	    ; Apagamos el oscilador de baja frecuencia
    BSF		T1CKPS1	    ; Prescaler 1:8
    BSF		T1CKPS0
    BCF		TMR1GE	    ; Timer siempre contando
    BSF		TMR1ON	    ; Encendemos el Timer1
   
    RESET_TMR1	0xC2, 0xF7  ; TMR1 a 1000ms
    RETURN
    
; Cada vez que se cumple el tiempo del TMR0 es necesario reiniciarlo.
; ** Comentado porque lo cambiamos de subrutina a macro **
/*RESET_TMR0:
    BANKSEL TMR0	    ; cambiamos de banco
    MOVLW   61
    MOVWF   TMR0	    ; 50ms retardo
    BCF	    T0IF	    ; limpiamos bandera de interrupción
    return*/
    
CONFIG_TMR2:
    BANKSEL PR2		    ; Cambiamos a banco 01
    MOVLW   244		    ; Valor para interrupciones cada 500ms
    MOVWF   PR2		    ; Cargamos litaral a PR2
    
    BANKSEL T2CON	    ; Cambiamos a banco 00
    BSF	    T2CKPS1	    ; Prescaler 1:16
    BSF	    T2CKPS0
    
    BSF	    TOUTPS3	    ;Postscaler 1:13
    BSF	    TOUTPS2
    BCF	    TOUTPS1
    BCF	    TOUTPS0
    
    BSF	    TMR2ON	    ; Encendemos TMR2
    RETURN
    
 CONFIG_IN_OUTS:
    BANKSEL ANSEL
    CLRF    ANSEL
    CLRF    ANSELH	    ; I/O digitales
    BANKSEL TRISD
    CLRF    TRISD	    ; PORTD como salida
    CLRF    TRISA	    ; PORTA como salida
    CLRF    TRISC
    BCF	    TRISB, 0	    ; RB0 como salida
    BCF	    TRISD, 0	    ; RD0 como salida para display 
    BCF	    TRISD, 1	    ; RD1 como salida para display 
    BANKSEL PORTD
    CLRF    PORTD	    ; Apagamos PORTD
    CLRF    PORTA	    ; Apagamos PORTA
    CLRF    PORTB	    ; Apagamos PORTB
    CLRF    PORTC
    CLRF    banderas	    ; Limpiamos la variable de banderas (GPR)
    RETURN
    
CONFIG_INTERRUPCIONES:
    BANKSEL PIE1	    ; Habilitamos las interrupciones del TMR1
    BSF	    TMR1IE
    BSF	    TMR2IE	    ; Habilitamos int. TMR2
    BANKSEL INTCON
    BSF	    PEIE	    ; Habilitamos interrupciones de periféricos
    BSF	    GIE		    ; Habilitamos interrupciones globales
    BSF	    T0IE	    ; Habilitamos interrupcion TMR0
    BCF	    T0IF	    ; Limpiamos bandera de TMR0
    BCF	    TMR1IF	    ; Limpiramos la bandera del TMR1
    BCF	    TMR2IF	    ; Limpiamos bandera de TMR2
    RETURN


obtener_numero:             
   
    decenas:
        MOVLW   10		; W = 10
        SUBWF   valor,W		; W = valor - 10
        BTFSS   STATUS,0	; 10 > valor, Bandera de Carry apagada, C=0 // 10 < valor,Bandera de Cary se enciende, C=1
        GOTO    unidades	; Cuando la bandera es igual a 0 nos pasamos a las decenas para seguir restando, ya que no es posible restarle W al valor.
	MOVWF   valor
        INCF    numero		; Si la bandera es igual a 1 si es posible resstar W al valor y se incrementan las decenas.
        GOTO    decenas		; Regresamos a ejecutar nuevamente la subrutina de decenas.
	
    unidades:
        MOVLW   1		; W = 1
        SUBWF   valor,W		; W = valor - 1
        BTFSS   STATUS,0	; 1 > valor, Bandera de Carry apagada, C=0 // 1 < valor,Bandera de Cary se enciende, C=1
        RETURN			; Si ya no se puede restar la unidad hacemos un RETURN a la subrutina original.
	MOVWF   valor
        INCF    numero+1	; Si la bandera es igual a 1 si es posible resstar W al valor y se incrementan las unidades.
        GOTO    unidades	; Regresamos a ejecutar nuevamente la subrutina de unidades.
    
set_display:
    MOVF    numero, W		; Movemos la variable numero a F
    CALL    Tabla		; Buscamos el valor a cargar en PORTC
    MOVWF   display		; Se guarda en el display.

    MOVF    numero+1, W		; Movemos la variable numero+1 a F
    CALL    Tabla		; Buscamos valor a cargar en PORTC
    MOVWF   display+1		; Guardamos en display+1

    RETURN
    
mostrar_valor:

    BCF	    PORTD, 0		; Apagamos display
    BCF	    PORTD, 1		; Apagamos display
    BTFSC   banderas,0
    GOTO    display_1
    GOTO    display_0
    
    display_0:
	MOVF    display, W	; Se mueve el display a W
	MOVWF   PORTC		; Se mueve el valor de tabla a PORTC
	BSF	PORTD, 0		
	BSF	banderas,0
	RETURN
	
	;Nos preparamos para ejecutar el siguiente estado que serían las banderas en 10, es decir el siguiente display.
    
    display_1:
	MOVF    display+1, W	; Se mueve el display+1 a W
	MOVWF   PORTC		; Se mueve el valor de tabla a PORTC
	BSF	PORTD, 1	; Encendemos display de nibble alto
	BCF	banderas,0	;
	RETURN
	
	;Nos preparamos para ejecutar el siguiente estado que serían las banderas en 1X, es decir el siguiente display.
	
END
