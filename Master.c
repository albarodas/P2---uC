/* 
 * File:   Master.c
 * Author: Daniela Godinez & Alba Rodas
 *
 * Created on 17 de mayo de 2022, 17:01
 */

// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT // Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF            // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF           // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF           // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF              // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF             // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF           // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF            // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF           // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF             // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V        // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF             // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include <stdint.h>

//Constantes
#define _XTAL_FREQ 8000000      //Valor de frecuencia de 8MHz

//Variables
uint8_t estado = 0;             //Variable para el Estado (Manual, Guardar, PC)
char valor_servo1 = 0;          //Variable para Servo 1 - Estado 0: Manual
char valor_servo2 = 0;          //Variable para Servo 2 - Estado 0: Manual
char valor_servo3 = 0;          //Variable para Servo 3 - Estado 0: Manual
char valor_servo4 = 0;          //Variable para Servo 4 - Estado 0: Manual
char valor3 = 0;                //Variable para Servo 3 - Estado 1: Guardar EEPROM
char valor4 = 0;                //Variable para Servo 4 - Estado 1: Guardar EEPROM
uint8_t address = 0;            //Variable para el address en la EEPROM - Estado 0 y 1
uint8_t numero_servo = 0;       //Variable para guardar los valores recibidos de la PC - Estado 2: PC 
char valor_servo1_read = 0;     //Variable para mostrar el valor guardado en la EEPROM del Servo 1 - Estado 1: Guardar EEPROM
char valor_servo2_read = 0;     //Variable para mostrar el valor guardado en la EEPROM del Servo 2 - Estado 1: Guardar EEPROM
char valor_servo3_read = 0;     //Variable para mostrar el valor guardado en la EEPROM del Servo 3 - Estado 1: Guardar EEPROM
char valor_servo4_read = 0;     //Variable para mostrar el valor guardado en la EEPROM del Servo 4 - Estado 1: Guardar EEPROM
char xservo = 0;                //Variable para guardar el numero de servo indicado por la PC - Estado 2: PC
char in_servo1 = 0;             //Variable para Servo 1 - Estado 2: PC
char in_servo2 = 0;             //Variable para Servo 2 - Estado 2: PC
char in_servo3 = 0;             //Variable para Servo 3 - Estado 2: PC
char in_servo4 = 0;             //Variable para Servo 4 - Estado 2: PC

/*
 * PROTOTIPOS DE FUNCIÓN
 */
void setup(void);                                   //Funcion de setup, al iniciar el PIC
void spi_envio(char data);                          //Funcion de envio de datos al esclavo
void spi_write(char dato);                          //Funcion de escritura de los datos que se enviaran al esclavo
uint8_t read_EEPROM(uint8_t address);               //Funcion de lectura de la EEPROM
void write_EEPROM(uint8_t address, uint8_t data);   //Funcion de escritura de la EEPROM

/*
 * INTERRUPCIONES
 */
void __interrupt() isr (void){                                      //INICIO DE INTERRUPCION
    if(RBIF){                                                       //Verifica la bandera de RBIF
        if(!PORTBbits.RB7){                                         //Verifica la activación (bit en set) del RB7
            estado++;                                               //Aumenta el estado, el cual sirve para cambiar de estado en el menu
            menu();                                                 //Llama a la funcion menu
        }                                                           //Finaliza de verificar la activacion de RB7
        if(estado == 0){                                            //Verifica si el estado es 0
            if(!PORTBbits.RB6){                                     //Verifica la activavion (bit en set) del RB6
                write_EEPROM(0, valor_servo1);                      //Escribe el valor del Servo 1 en la direccion 0 de la EEPROM
                __delay_ms(100);                                    //Espera 100 ms
                write_EEPROM(1, valor_servo2);                      //Escribe el valor del Servo 2 en la direccion 1 de la EEPROM
                __delay_ms(100);                                    //Espera 100 ms
                write_EEPROM(2, valor_servo3);                      //Escribe el valor del Servo 3 en la direccion 2 de la EEPROM
                __delay_ms(100);                                    //Espera 100 ms
                write_EEPROM(3, valor_servo4);                      //Escribe el valor del Servo 14 en la direccion 3 de la EEPROM
                __delay_ms(100);                                    //Espera 100 ms
            }                                                       //Finaliza la verificacion de activacion del RB6
            
            if(!PORTBbits.RB5){                                     //Verifica la activavion (bit en set) del RB6
                write_EEPROM(4, valor_servo1);                      //Escribe el valor del Servo 1 en la direccion 4 de la EEPROM
                __delay_ms(100);                                    //Espera 100 ms
                write_EEPROM(5, valor_servo2);                      //Escribe el valor del Servo 2 en la direccion 5 de la EEPROM
                __delay_ms(100);                                    //Espera 100 ms
                write_EEPROM(6, valor_servo3);                      //Escribe el valor del Servo 3 en la direccion 6 de la EEPROM
                __delay_ms(100);                                    //Espera 100 ms
                write_EEPROM(7, valor_servo4);                      //Escribe el valor del Servo 4 en la direccion 7 de la EEPROM
                __delay_ms(100);                                    //Espera 100 ms
            }                                                       //Finaliza la verificacion de activacion del RB6
        }                                                           //Finaliza la verificacion del Estado en 0
        if(estado == 1){                                            //Verifica si el estado es 1
            if(!PORTBbits.RB6){                                     //Verifica la activavion (bit en set) del RB6
                valor_servo1_read = read_EEPROM(0);                 //Lee el valor del Servo 1 en la direccion 0 de la EEPROM
                __delay_ms(50);                                     //Espera 50 ms
                valor_servo2_read = read_EEPROM(1);                 //Lee el valor del Servo 2 en la direccion 1 de la EEPROM
                __delay_ms(100);                                    //Espera 100 ms
                valor_servo3_read = read_EEPROM(2);                 //Lee el valor del Servo 3 en la direccion 2 de la EEPROM
                __delay_ms(100);                                    //Espera 100 ms
                valor_servo4_read = read_EEPROM(3);                 //Lee el valor del Servo 4 en la direccion 3 de la EEPROM
                __delay_ms(100);                                    //Espera 100 ms
                
                __delay_us(100);                                    //Espera 100 ms
                spi_envio(valor_servo1_read);                       //Envia el valor del Servo 1 al esclavo
                __delay_us(100);                                    //Espera 100 ms
                spi_envio(valor_servo2_read);                       //Envia el valor del Servo 2 al esclavo
                __delay_us(1000);                                   //Espera 100 ms
                if(valor3 != valor_servo3_read){                    //Verifica si el valor actual del servo 3 es distinto al viejo
                    CCPR1L = (valor_servo3_read>>1)+123;            //El valor del servo 3 se rota y se suma 123
                    CCP1CONbits.DC1B1 = (valor_servo3_read & 0b01); //Al valor del servo 3 se le obtiene el bit menos significativo
                    valor3 = valor_servo3_read;                     //El valor3 es igual al valor 3 del servo
                }                                                   //Finaliza la verificacion del valor actual y viejo del servo 3
                __delay_us(1000);                                   //Espera 1000 us
                if(valor4 != valor_servo4_read){                    //Verifica si el valor actual del servo 4 es distinto al viejo
                    CCPR2L = (valor_servo4_read>>1)+123;            //El valor del servo 4 se rota y se suma 123
                    CCP2CONbits.DC2B1 = (valor_servo4_read & 0b01); //Al valor del servo 4 se le obtiene el bit menos significativo
                    valor4 = valor_servo4_read;                     //El valor4 es igual al valor 4 del servo
                }                                                   //Finaliza la verificacion del valor actual y viejo del servo 4
            }                                                       //Finaliza la activavion (bit en set) del RB6
            
            if(!PORTBbits.RB5){                                     //Verifica la activavion (bit en set) del RB5
                valor_servo1_read = read_EEPROM(4);                 //Lee el valor del Servo 1 en la direccion 4 de la EEPROM
                __delay_ms(50);                                     //Espera 50 ms
                valor_servo2_read = read_EEPROM(5);                 //Lee el valor del Servo 2 en la direccion 5 de la EEPROM
                __delay_ms(100);                                    //Espera 100 ms
                valor_servo3_read = read_EEPROM(6);                 //Lee el valor del Servo 3 en la direccion 6 de la EEPROM
                __delay_ms(100);                                    //Espera 100 ms
                valor_servo4_read = read_EEPROM(7);                 //Lee el valor del Servo 4 en la direccion 7 de la EEPROM
                __delay_ms(100);                                    //Espera 100 ms
                
                __delay_us(100);                                    //Espera 100 ms
                spi_envio(valor_servo1_read);                       //Envia el valor del Servo 1 al esclavo
                __delay_us(100);                                    //Espera 100 ms
                spi_envio(valor_servo2_read);                       //Envia el valor del Servo 2 al esclavo
                __delay_us(1000);                                   //Espera 1000 us
                if(valor3 != valor_servo3_read){                    //Verifica si el valor actual del servo 3 es distinto al viejo
                    CCPR1L = (valor_servo3_read>>1)+123;            //El valor del servo 3 se rota y se suma 123
                    CCP1CONbits.DC1B1 = (valor_servo3_read & 0b01); //Al valor del servo 3 se le obtiene el bit menos significativo
                    valor3 = valor_servo3_read;                     //El valor3 es igual al valor 3 del servo
                }                                                   //Finaliza la verificacion del valor actual y viejo del servo 3
                __delay_us(1000);                                   //Espera 1000 us
                if(valor4 != valor_servo4_read){                    //Verifica si el valor actual del servo 4 es distinto al viejo
                    CCPR2L = (valor_servo4_read>>1)+123;            //El valor del servo 4 se rota y se suma 123
                    CCP2CONbits.DC2B1 = (valor_servo4_read & 0b01); //Al valor del servo 4 se le obtiene el bit menos significativo   
                    valor4 = valor_servo4_read;                     //El valor4 es igual al valor 4 del servo
                }                                                   //Finaliza la verificacion del valor actual y viejo del servo 4
            }                                                       //Finaliza la activavion (bit en set) del RB5
        }                                                           //Finaliza si el estado es 1
        PORTB = PORTB;                                              //Se iguala el PORTB al PORTB
        INTCONbits.RBIF = 0;                                        //Se limpia la bandera RBIF
    }                                                               //Finaliza la verificacion de la bandera de RBIF
    
    if(PIR1bits.ADIF){                                              //Verifica la bandera de ADIF
        if(estado == 0){                                            //Verifica si el estado es 0
            if (ADCON0bits.CHS == 0){                               //Verifica si AN0 es usado
                valor_servo1 = ADRESH;                              //Valor del servo 1 se iguala al ADRESH
            }                                                       //Finaliza la verificacion si AN0 es usado
            else if(ADCON0bits.CHS == 1){                           //Verifica si AN1 es usado
                valor_servo2 = ADRESH;                              //Valor del servo 2 se iguala al ADRESH
            }                                                       //Finaliza la verificacion si AN1 es usado
            else if(ADCON0bits.CHS == 2){                           //Verifica si AN2 es usado
                valor_servo3 = ADRESH;                              //Valor del servo 3 se iguala al ADRESH
                CCPR1L = (ADRESH>>1)+123;                           //El valor del servo 3 se rota y se suma 123
                CCP1CONbits.DC1B1 = (ADRESH & 0b01);                //Al valor del servo 3 se le obtiene el bit menos significativo
                CCP1CONbits.DC1B0 = 0;                              //DC1B0 se iguala a 0
            }                                                       //Finaliza la verificacion si AN2 es usado
            else if(ADCON0bits.CHS == 3){                           //Verifica si AN3 es usado
                valor_servo4 = ADRESH;                              //Valor del servo 4 se iguala al ADRESH
                CCPR2L = (ADRESH>>1)+123;                           //El valor del servo 4 se rota y se suma 123
                CCP2CONbits.DC2B1= (ADRESH & 0b01);                 //Al valor del servo 4 se le obtiene el bit menos significativo
                CCP2CONbits.DC2B0 = 0;                              //DC2B0 se iguala a 0
            }                                                       //Finaliza la verificacion si AN3 es usado
        }                                                           //Finaliza la verifiacion del estado 0
        PIR1bits.ADIF = 0;                                          //Se limpia la bandera del ADIF
    }                                                               //Finaliza la verificacion de la bandera de ADIF
    
    if(PIR1bits.RCIF){                                              //Verifica la bandera de RCIF
        if(estado == 2){                                            //Verifica si el estado es 2
            numero_servo = RCREG;                                   //Se iguala numero_servo a RCREG
        }                                                           //Finaliza la verifiacion del estado 2
        PIR1bits.RCIF = 0;                                          //Limpia la bandera de RCIF
    }                                                               //Finaliza la verificacion de la bandera de RCIF
    return;                                                         //Vuelve a repetir la interrupcion
}                                                                   //FIN DE INTERRUPCION

/*
 * ENTRADAS Y SALIDAS
 */
void setup(void){
    ANSEL = 0b00001111;       //AN0, AN1, AN2, AN3
    ANSELH = 0;
    
    TRISAbits.TRISA0 = 1;     //RA0-AN0 entrada Potenciometro 1
    TRISAbits.TRISA1 = 1;     //RA1-AN1 entrada Potenciometro 2
    TRISAbits.TRISA2 = 1;     //RA2-AN2 entrada Potenciometro 3
    TRISAbits.TRISA3 = 1;     //RA3-AN3 entrada Potenciometro 4
    
    TRISAbits.TRISA7 = 0;     //RA7 salida Comunicación con Esclavo
    TRISCbits.TRISC4 = 1;     //SDA entrada
    TRISCbits.TRISC3 = 0;     //SCK salida
    TRISCbits.TRISC5 = 0;     //SDO salida
    
    TRISEbits.TRISE0 = 0;     //RD6 salida LED de modo estado
    TRISEbits.TRISE1 = 0;     //RD7 salida LED de modo estado
    
    TRISD = 0;                //TRISD salida - Prueba de verificacion del RCREG
    
    TRISBbits.TRISB7 = 1;     //RB7 entrada Push-Button MODO
    TRISBbits.TRISB6 = 1;     //RB6 entrada Push-Button WRITE
    TRISBbits.TRISB5 = 1;     //RB5 entrada Push-Button READ
    OPTION_REGbits.nRBPU = 0; //Habilitación de PORTB pull-ups
    WPUBbits.WPUB7 = 1;       //Habilitamos Pull-Up en RB7
    WPUBbits.WPUB6 = 1;       //Habilitamos Pull-Up en RB6
    WPUBbits.WPUB5 = 1;       //Habilitamos Pull-Up en RB5
    
    TRISE = 0;                //TRISE SALIDA
    
    PORTA = 0;                //Limpiamos PORTA
    PORTB = 0;                //Limpiamos PORTB
    PORTC = 0;                //Limpiamos PORTC
    PORTD = 0;                //Limpiamos PORTD
    PORTE = 0;                //Limpiamos PORTE
    
    //CONFIGURACIÓN DE OSCILADOR
    OSCCONbits.IRCF = 0b111;  //8MHz
    OSCCONbits.SCS = 1;       //Reloj interno

    //CONFIGURACIÓN DEL ADC
    ADCON1bits.ADFM = 0;      //Justificado a la izquierda
    ADCON1bits.VCFG0 = 0;     //VDD *Referencias internas
    ADCON1bits.VCFG1 = 0;     //VSS *Referencias internas
    
    ADCON0bits.ADCS = 0b10;   //FOSC/32
    ADCON0bits.CHS = 0;       //AN0/RA0 para potenciometro 1
    ADCON0bits.CHS = 1;       //AN1/RA1 para potenciometro 2
    ADCON0bits.CHS = 2;       //AN2/RA2 para potenciometro 3
    ADCON0bits.CHS = 3;       //AN3/RA3 para potenciometro 4
    ADCON0bits.ADON = 1;      //Habilitamos Módulo ADC
    __delay_us(50);
    
    //CONFIGURACIÓN DEL PMW
    TRISCbits.TRISC2 = 1;     //RC2/CCP1 como entrada potenciometro 1
    PR2 = 255;                //Configuración del periodo
    CCP1CONbits.P1M = 0;      //Configuración modo PWM
    CCP1CONbits.CCP1M = 0b1100;
    CCPR1L = 0x0f;            //Ciclo de trabajo inicial
    CCP1CONbits.DC1B = 0;
    
    TRISCbits.TRISC1 = 1;     //RC1/CCP2 como entrada potenciometro 2
    CCP2CONbits.CCP2M = 0b1100;
    CCPR2L = 0X0f;  
    CCP2CONbits.DC2B1 = 0;
    
    //CONFIGURACIÓN DEL TMR2
    PIR1bits.TMR2IF = 0;      //Apagamos la bandera
    T2CONbits.T2CKPS = 0b11;  //Prescaler 1:16
    T2CONbits.TMR2ON = 1;
    
    while(PIR1bits.TMR2IF == 0); //Esperamos un ciclo del TMR2
    PIR1bits.TMR2IF = 0;
    
    TRISCbits.TRISC2 = 0;    //Salida del PMW
    TRISCbits.TRISC1 = 0;    //Salida del PMW
    
    //CONFIGURACIÓN DEL COMUNICACIÓN SERIAL
    //SYNC = 0, BRGH = 1; BRG16 = 1
    //SPBRG = 25
    TXSTAbits.SYNC = 0;      //Modo asíncrono
    TXSTAbits.BRGH = 1;      //Alta velocidad en modo asíncrono
    BAUDCTLbits.BRG16 = 1;   //16-bit Baud Rate Generator es usado
    
    SPBRG = 207;
    SPBRGH = 0;              //Bits más significativos en 0, ya que solo necestiamos los menos significativos
    
    RCSTAbits.SPEN = 1;      //Habilitamos comunicación
    RCSTAbits.RX9 = 0;       //Usamos solo 8 bits
    TXSTAbits.TX9 = 0;
    RCSTAbits.CREN = 1;      //Habilitamos recepción
    TXSTAbits.TXEN = 0;      //Habilitamos la transmision
    
    //CONFIGURACIÓN CPI
    //MAESTRO
    
    // SSPCON <5:0>
    SSPCONbits.SSPM = 0b0000;//SPI Maestro, Reloj de Fosc/4 (250kbits/s)
    SSPCONbits.CKP = 0;      //Reloj inactivo en 0
    SSPCONbits.SSPEN = 1;    //Habilitación de pines de SPI
    // SSPSTAT<7:6>
    SSPSTATbits.CKE = 1;     //Dato enviado a cada flanco de subida
    SSPSTATbits.SMP = 1;     //Dato al final del pulso de reloj
    SSPBUF = 0;              //Envio de dato inicial
    
    //CONFIGURACIÓN INTERUPCCIONES
    INTCONbits.GIE = 1;      //Se habilita las banderas globales
    INTCONbits.PEIE = 1;     //Habilitar interrupciones de periféricos
    PIR1bits.ADIF = 0;       //Limpiamos bandera de interrupción de ADC
    PIE1bits.ADIE = 1;       //Habilitamos interrupción de ADC
    PIE1bits.RCIE = 1;       //Interupcción de recepción
    IOCBbits.IOCB7 = 1;      //Interrupcion del RB7
    IOCBbits.IOCB6 = 1;      //Interrupcion del RB6
    IOCBbits.IOCB5 = 1;      //Interrupcion del RB5
    INTCONbits.RBIE = 1;     //Interrupcion del PORTB
    INTCONbits.RBIF = 0;     //Interrupcion del PORTB
}

/*
 * FUNCIONES EXTRAS
 */
int menu(){
    switch(estado){
        case 0: //MODO MANUAL (POTENCIOMETROS)
            PORTEbits.RE0 = 0;
            PORTEbits.RE1 = 0;
            break;
        case 1: //MODO AUTOMÁTICO (EEPROM READ)
            PORTEbits.RE0 = 1;
            PORTEbits.RE1 = 0; 
            break;
        case 2: //MODO CONTROL PC
            PORTEbits.RE0 = 0;
            PORTEbits.RE1 = 1;
            break;
        default:
            estado = 0;
            break;
    }
    return 0;
}

void spi_envio(char data){
    PORTAbits.RA7 = 0;      // Deshabilitamos el ss del esclavo
    spi_write(data);        //Se llama la funcion de spi_write
    PORTAbits.RA7 = 1;      // habilitamos nuevamente el escalvo
}

void spi_write(char dato){
    SSPBUF = dato;              //Se guarda en el BUFFER el dato
    while(SSPSTATbits.BF == 0); //Mientras BF este en 0 que continue
    while(PIR1bits.SSPIF == 0); //Mientras SSPIF este en 0 que continue
    PIR1bits.SSPIF = 0;         //SSPIF = 0
}

/*
 * MAIN
 */
void main(void){
    setup();                                                //Funcion de arranque
    ADCON0bits.GO = 1;                                      //Go en 1
    while(1){                                               //Loop principal
        menu();                                             //Se llama a la funcion de menu
        
        if (ADCON0bits.GO == 0) {                           // si esta en 0, revisa en qué canal está convirtiendo
            if (ADCON0bits.CHS == 0){                       //Verifica el AN0
                __delay_us(100);                            //Espera 100 us
                ADCON0bits.CHS = 1;                         //Se cambia al AN1
            }                                               //Termina verificacion del AN0
            else if (ADCON0bits.CHS == 1){                  //Verifica el AN1
                __delay_us(100);                            //Espera 100 us
                ADCON0bits.CHS = 2;                         //Se cambia al AN2
            }                                               //Termina verificacion del AN1
            else if (ADCON0bits.CHS == 2){                  //Verifica el AN2
                __delay_us(100);                            //Espera 100 us
                ADCON0bits.CHS = 3;                         //Se cambia al AN3
            }                                               //Termina verificacion del AN2
            else if (ADCON0bits.CHS == 3){                  //Verifica el AN3
                __delay_us(100);                            //Espera 100 us
                ADCON0bits.CHS = 0;                         //Se cambia al AN0
            }                                               //Termina verificacion del AN3
            __delay_us(100);                                //Espera 100 us
            ADCON0bits.GO = 1;                              //Go en 1
        }                                                   //Finaliza analisis de conversion
        
        if(estado == 0){                                    //Verifica si el estado es 0
            __delay_us(100);                                //Espera 100 us
            spi_envio(valor_servo1);                        //Se envia el valor del servo 1 al esclavo
            __delay_us(100);                                //Espera 100 us
            spi_envio(valor_servo2);                        //Se envia el valor del servo 2 al esclavo
        }                                                   //Finaliza verificacion de estado
        
        if(estado == 2){                                    //Verifica si el estado es 2
            xservo = numero_servo & 0b11000000;             //xservo tomara los 2 bits mas significativos (indican el slider que se esta usando en la interfaz)
            if(xservo == 0){                                //xservo = slider 1 en uso
                in_servo1 = numero_servo & 0b00111111;      //in_servo1 tomará los 6 bits mas significativos del valor recibido por la interfaz
                __delay_us(100);                            //Espera 100 us
                spi_envio(in_servo1);                       //Se envia el valor del servo 1 al esclavo
            }else if(xservo == 64){                         //xservo = slider 2 en uso
                in_servo2 = numero_servo & 0b00111111;      //in_servo2 tomará los 6 bits mas significativos del valor recibido por la interfaz
                __delay_us(100);                            //Espera 100 us
                spi_envio(in_servo2);                       //Se envia el valor del servo 2 al esclavo
            }else if(xservo == 128){                        //xservo = slider 3 en uso
                in_servo3 = numero_servo & 0b00111111;      //in_servo3 tomará los 6 bits mas significativos del valor recibido por la interfaz
                CCPR1L = (in_servo3>>1)+123;                //El valor del servo 3 se rota y se suma 123
                CCP1CONbits.DC1B1 = (in_servo3 & 0b01);     //Al valor del servo 3 se le obtiene el bit menos significativo
                valor3 = in_servo3;                         //Valor 3 se iguala al valor servo 3
            }else if(xservo == 192){                        //xservo = slider 4 en uso
                in_servo4 = numero_servo & 0b00111111;      //in_servo4 tomará los 6 bits mas significativos del valor recibido por la interfaz
                CCPR2L = (in_servo4>>1)+123;                //El valor del servo 4 se rota y se suma 123
                CCP2CONbits.DC2B1 = (in_servo4 & 0b01);     //Al valor del servo 4 se le obtiene el bit menos significativo
                valor4 = in_servo4;                         //Valor 4 se iguala al valor servo 4
            }                                               //Finaliza verificacion de slider
        }                                                   //Finaliza verificacion de estado  
    }                                                       //Finalzia loop pricipal
    return;                                                 //Se repite el proceso
}

uint8_t read_EEPROM(uint8_t address){
    EEADR = address;
    EECON1bits.EEPGD = 0;       // Lectura a la EEPROM
    EECON1bits.RD = 1;          // Obtenemos dato de la EEPROM
    return EEDAT;               // Regresamos dato 
}

void write_EEPROM(uint8_t address, uint8_t data){
    EEADR = address;
    EEDAT = data;
    EECON1bits.EEPGD = 0;       // Escritura a la EEPROM
    EECON1bits.WREN = 1;        // Habilitamos escritura en la EEPROM
    
    INTCONbits.GIE = 0;         // Deshabilitamos interrupciones
    EECON2 = 0x55;      
    EECON2 = 0xAA;
    
    EECON1bits.WR = 1;          // Iniciamos escritura
    
    EECON1bits.WREN = 0;        // Deshabilitamos escritura en la EEPROM
    INTCONbits.RBIF = 0;
    INTCONbits.GIE = 1;         // Habilitamos interrupciones
}