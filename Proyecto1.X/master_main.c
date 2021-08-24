/*
 * Archivo:   master_main.c
 * Dispositivo: PIC16F887
 * Autor: Margareth Vela 
 * 
 * Programa: I2C
 * Hardware: LCD en PORTD y FTDI en PORTC.
 * 
 * Creado: Agosto 22, 2021
 * Última modificación: Agosto, 2021
 */

//------------------------------------------------------------------------------
//                          Importación de librerías
//------------------------------------------------------------------------------
#include <xc.h>
#include <stdint.h>
#include "I2C.h"
#include "LCD.h" 

//------------------------------------------------------------------------------
//                          Directivas del compilador
//------------------------------------------------------------------------------
#define _XTAL_FREQ 8000000 //Oscilador

//------------------------------------------------------------------------------
//                          Variables
//------------------------------------------------------------------------------
uint8_t Primer_digito = 0; //Para conversion a decimal
uint8_t Segundo_digito = 0;
uint8_t Tercer_digito = 0;

uint8_t slave1;
uint8_t slave2;
uint8_t sensor1;
uint8_t sensor2;
uint8_t sensor3;

//------------------------------------------------------------------------------
//                          Palabras de configuración
//------------------------------------------------------------------------------
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT//Oscillator Selection bits(INTOSCIO 
                              //oscillator: I/O function on RA6/OSC2/CLKOUT pin, 
                              //I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF // Watchdog Timer Enable bit (WDT disabled and 
                          //can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR  
                                //pin function is digital input, MCLR internally 
                                //tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code 
                                //protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code 
                                //protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit 
                                //Internal/External Switchover mode is disabled
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit 
                                //(Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF         //Low Voltage Programming Enable bit(RB3/PGM pin 
                                //has PGM function, low voltage programming 
                                //enabled)
// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out 
                                //Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits 
                                //(Write protection off)

//------------------------------------------------------------------------------
//                          Prototipos
//------------------------------------------------------------------------------
void setup(void);  //Configuración
void Decimal(uint8_t variable); //Conversion a decimal

//------------------------------------------------------------------------------
//                          Código Principal
//------------------------------------------------------------------------------
void main(void) {
    setup(); 
    Lcd_Init();
    while(1){  
        /*
          I2C_Master_Start();         //Se inicializa la comunicacion I2C
        I2C_Master_Write(0x71);     //Direccion de lectura del primer esclavo
        slave1 = I2C_Master_Read(0); //Se agrega el valor del potenciometro
        I2C_Master_Stop();          //Termina la comunicacion 
        __delay_ms(200);
        
        I2C_Master_Start();         //Se inicializa la comunicacion I2C
        I2C_Master_Write(0x81);     //Direccion de lectura del segundo esclavo
        slave2 = I2C_Master_Read(0); //Se agrega el valor del contador
        I2C_Master_Stop();          //Termina la comunicacion 
        __delay_ms(200);
        
        I2C_Master_Start();         //Se inicializa la comunicacion I2C
        I2C_Master_Write(0x90);     //Direccion de lectura del sensor I2C
        I2C_Master_Write(0xEE);     //Configuracion del sensor
        I2C_Master_Stop();          //Termina la comunicacion 
        __delay_ms(200);
        
        I2C_Master_Start();         //Se inicializa la comunicacion I2C
        I2C_Master_Write(0x90);     //Direccion de lectura del sensor I2C
        I2C_Master_Write(0xAA);     //Lee el valor del sensor
        I2C_Master_Stop();          //Termina la comunicacion
        __delay_ms(200);
        
        I2C_Master_Start();         //Se inicializa la comunicacion I2C
        I2C_Master_Write(0x91);     //Direccion de lectura del sensor I2C
        val_temp = I2C_Master_Read(0); //Se agrega el valor del sensor
        I2C_Master_Stop();          //Termina la comunicacion
        __delay_ms(200);
        */
        
        Lcd_Clear();
        Lcd_Set_Cursor(1,1);
        Lcd_Write_String("S1:");
        Lcd_Write_String(" ");
        Lcd_Write_String("S2:");
        Lcd_Write_String(" ");
        Lcd_Write_String("OP1");
        Lcd_Write_String(" ");
        Lcd_Write_String("OP2");
        
        Lcd_Set_Cursor(2,1);
        Decimal(255);
        Lcd_Write_Char(Primer_digito);
        Lcd_Write_Char(Segundo_digito);
        Lcd_Write_Char(Tercer_digito);
        Lcd_Write_String(" ");
        Decimal(40);
        Lcd_Write_Char(Primer_digito);
        Lcd_Write_Char(Segundo_digito);
        Lcd_Write_Char(Tercer_digito);
        Lcd_Write_String(" ");
        Decimal(80);
        Lcd_Write_Char(Primer_digito);
        Lcd_Write_Char(Segundo_digito);
        Lcd_Write_Char(Tercer_digito);
        Lcd_Write_String(" ");
        Decimal(10);
        Lcd_Write_Char(Primer_digito);
        Lcd_Write_Char(Segundo_digito);
        Lcd_Write_Char(Tercer_digito);
        __delay_ms(2000);

    }
    return;
}

//------------------------------------------------------------------------------
//                          Interrupciones
//------------------------------------------------------------------------------
void __interrupt()isr(void){
    di();   
    ei();                           //POP
}

//------------------------------------------------------------------------------
//                          Subrutinas
//------------------------------------------------------------------------------
void Decimal(uint8_t variable){        // Función para obtener valor decimal
    uint8_t valor;
    valor = variable;                  
    Primer_digito = (valor/100) ;                //Valor del tercer digito
    valor = (valor - (Primer_digito*100));
    Segundo_digito = (valor/10);              //Valor del segundo digito
    valor = (valor - (Segundo_digito*10));
    Tercer_digito = valor;                //Valor del primer digito
    
    Primer_digito = Primer_digito + 48;          //Conversion a ascii
    Segundo_digito = Segundo_digito + 48;
    Tercer_digito = Tercer_digito + 48;
    
}

//------------------------------------------------------------------------------
//                          Configuración
//------------------------------------------------------------------------------
void setup(void){
    //Configuracion reloj
    OSCCONbits.IRCF2 = 1; //Frecuencia a 8MHZ
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.SCS = 1;
    
    //Configurar entradas y salidas
    ANSELH = 0x00;//Pines digitales
    ANSEL = 0x00; //Pines digitales
    
    TRISA = 0x00; //Salidas
    TRISB = 0x00;
    TRISC = 0x80; 
    TRISD = 0x00; 
    TRISE = 0x00; 
    
    PORTA = 0x00; //Se limpian los puertos
    PORTB = 0x00;
    PORTC = 0x00;    
    PORTD = 0x00;
    PORTE = 0x00;
        
   // I2C_Master_Init(100000); // Se inicializa la frecuencia del master a 100kHz
}

