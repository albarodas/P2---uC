/* 
 * File:   Slave.c
 * Author: Daniela Godinez & Alba Rodas
 *
 * Created on 17 de mayo de 2022, 17:03
 */

// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

//Constantes
#define _XTAL_FREQ 8000000  //Valor del oscilador de 8MHz

//Variables
char valor1 = 0;            //Variable para recibir el dato del maestro para el servo 1
char valor2 = 0;            //Variable para recibir el dato del maestro para el servo 2
char valor_servo1 = 0;      //Variable para mostrar el dato recibido del maestro para el servo 1
char valor_servo2 = 0;      //Variable para mostrar el dato recibido del maestro para el servo 2

/*
 * PROTOTIPOS DE FUNCIÓN
 */
void setup(void);           //Funcion de arranque

/*
 * INTERRUPCIONES
 */
void __interrupt() isr (void){  //INICIO DE LA INTERRUPCION
    if(PIR1bits.SSPIF){         //Verificacion de bandera SSPIF
        valor_servo1 = SSPBUF;  //Valor 1 recibe el dato enviado por el maestro para el servo 1
        __delay_us(1000);       //Espera 1000 us
        valor_servo2 = SSPBUF;  //Valor 2 recibe el dato enviado por el maestro para el servo 2
        PIR1bits.SSPIF = 0;     //Se limpia la bandera
    }                           //Finalizacion de verifiacion de bandera SSPIF
}                               //FIN INTERRUPCION

/*
 * ENTRADAS Y SALIDAS
 */
void setup(void){
    ANSEL = 0;
    ANSELH = 0;
    
    TRISAbits.TRISA6 = 0; //Salida Servo
    
    TRISAbits.TRISA5 = 1;
    TRISC = 0b00011000; // -> SDI y SCK entradas, SD0 como salida
    PORTC = 0;
    PORTA = 0;
    
    //CONFIGURACIÓN DE OSCILADOR
    OSCCONbits.IRCF = 0b111;    //8MHz
    OSCCONbits.SCS = 1;         //Reloj interno
    
    //CONFIGURACIÓN DEL PMW
    TRISCbits.TRISC2 = 1; //RC2/CCP1 como entrada potenciometro 1
    PR2 = 255; //Configuración del periodo
    CCP1CONbits.P1M = 0; //Configuración modo PWM
    CCP1CONbits.CCP1M = 0b1100;
    CCPR1L = 0x0f; //Ciclo de trabajo inicial
    CCP1CONbits.DC1B = 0;
    
    TRISCbits.TRISC1 = 1; //RC1/CCP2 como entrada potenciometro 2
    CCP2CONbits.CCP2M = 0b1100;
    CCPR2L = 0X0f;  
    CCP2CONbits.DC2B1 = 0;
    
    //CONFIGURACIÓN DEL TMR2
    PIR1bits.TMR2IF = 0; //Apagamos la bandera
    T2CONbits.T2CKPS = 0b11; //Prescaler 1:16
    T2CONbits.TMR2ON = 1;
    
    while(PIR1bits.TMR2IF == 0); //Esperamos un ciclo del TMR2
    PIR1bits.TMR2IF = 0;
    
    TRISCbits.TRISC2 = 0; //Salida del PMW
    TRISCbits.TRISC1 = 0; //Salida del PMW
    
    //CONFIGURACIÓN INTERUPCCIONES
    INTCONbits.GIE = 1;     //Se habilita las banderas globales
    INTCONbits.PEIE = 1;    //Habilitar interrupciones de periféricos
    PIR1bits.SSPIF = 0;         //Limpiar bandera SPI
    PIE1bits.SSPIE = 1;         //Se habilita interrupción de SPI
    
    //CONFIGURACIÓN CPI
    // SSPCON <5:0>
    SSPCONbits.SSPM = 0b0100;   //SPI Esclavo, SS entrada o salida
    SSPCONbits.CKP = 0;         //Reloj inactivo en 0
    SSPCONbits.SSPEN = 1;       //Habilitación de pines de SPI
    // SSPSTAT<7:6>
    SSPSTATbits.CKE = 1;        //Dato enviado a cada flanco de subida
    SSPSTATbits.SMP = 0;        //Dato al final del pulso del reloj
    return;
}

/*
 * FUNCIONES EXTRAS
 */


/*
 * MAIN
 */
void main(void) {
    setup();                                            //Funcion de arranque
    while(1){                                           //Loop principal
        if(valor1 != valor_servo1){                     //Verifica si el valor del servo 1 es igual al anterior
            CCPR1L = (valor_servo1>>1)+123;             //El valor del servo 1 se rota y se suma 123
            CCP1CONbits.DC1B1 = (valor_servo1 & 0b01);  //Al valor del servo 1 se le obtiene el bit menos significativo
            valor1 = valor_servo1;                      //Valor anterior se iguala al valor presente para servo 1
        }                                               //Finaliza la verificacion del servo 1
        if(valor2 != valor_servo2){                     //Verifica si el valor del servo 2 es igual al anterior
            CCPR2L = (valor_servo2>>1)+123;             //El valor del servo 2 se rota y se suma 123
            CCP2CONbits.DC2B1 = (valor_servo2 & 0b01);  //Al valor del servo 2 se le obtiene el bit menos significativo
            valor2 = valor_servo2;                      //Valor anterior se iguala al valor presente para servo 2
        }                                               //Finaliza la verificacion del servo 1
    }                                                   //Finaliza Loop
    return;                                             //Se repite
}