/*
 * Sprint8.c
 */ 

#define __DELAY_BACKWARD_COMPATIBLE__
#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include "nokia5110.h"



//variaveisglobais
uint32_t TEMPO_ms_anterior = 0;
uint8_t flag_200ms =0,flag_resp_por16 = 1,flag_150ms=0,flag_1s,flag_2500ms;
uint8_t Freq_Respiracao = 5; //5~30
uint16_t Freq_cardiaca = 0; // 20~200
uint32_t tempo_ms=0;
uint8_t rede = 0;
float temperatura = 30; //30~45
uint16_t saturacao_o2 = 0; //0~100
uint8_t valvula = 0;
uint16_t escolhe = 0;
uint8_t retorno = 0;
uint8_t volume = 1;
uint8_t bateria = 100,sujeira = 0,desempenho = 100 ;
unsigned char pressao_arterial[8] = "HHHxLLL";
int16_t pressao_H,pressao_L;

uint8_t check_payload_HHHxLLL(char *payload,int16_t *HHH, int16_t *LLL);
void anima_servo2(void);
void anima_servo(uint8_t *flag_disparo,uint8_t volume,uint8_t bateria);
void leitura_sensores_ADC(uint8_t *flag_disparo);
void anima_barra_sub_desc(uint8_t *flag_disparo);
void anima_lcd(uint8_t *flag_disparo,uint8_t *flag_disparo2,uint8_t Freq_Respiracao,uint16_t Freq_cardiaca, float temperatura,uint16_t saturacao_o2,uint8_t valvula,uint16_t escolhe,unsigned char pressao_arteriall,uint8_t volume,uint8_t bateria,uint8_t sujeira,uint8_t desempenho);
void bateria_maquina(uint8_t *flag_disparo,uint8_t rede,uint8_t sujeira, uint8_t desempenho);
void sujeira_maquina(uint8_t *flag_disparo,uint8_t sujeira);
void desempenho_maquina(uint8_t *flag_disparo,uint8_t desempenho,uint8_t sujeira);
//Temperatura
//----------------------------\\
// 5V -> 
//1023 | 0V -> 0 | 

// |5V	   ->	 1023|
// |Tensão -> Binario|

// Binario = (1023/5) * tensão

//((T-30)/(45-30)) = ((B-409.6)/(716.8-409.6))
// T = (B/20.48) + 10
//------------------------------\\

//Sp02
//----------------------\\
// 5V -> 1023 | 0V -> 0 | 

// |5V	   ->	 1023|
// |Tensão -> Binario|

// Binario = (1023/5) * tensão

// Binario = (1023/5) * 0 = 0
// Binario = (1023/5) * 4 = 819.2

// ((SPO2/100) = (Binario/819.2)

// Spo2 = Binario/8.192


//--------------------------\\//

ISR(USART_RX_vect)
{
	static uint8_t start_stop_flag = 0,index = 0;
	char char_recebido = UDR0;
	
	switch(char_recebido)
	{
		case ';':
			start_stop_flag = 1;
			index = 0;
			break;
		case ':':
			start_stop_flag = 0;
			pressao_arterial[index] = '\0';
			if (check_payload_HHHxLLL(pressao_arterial,&pressao_H,&pressao_L)==0)
			{
				strcpy(pressao_arterial,"ERRO!");
			}
			break;
		default:
			if (start_stop_flag)
			{
				if (index<7)
				{
					pressao_arterial[index++] = char_recebido;
				}
				else{
					start_stop_flag = 0;
					strcpy(pressao_arterial,"ERRO!");
				}
			}
	}
}
uint8_t check_payload_HHHxLLL(char *payload,int16_t *HHH, int16_t *LLL){
	unsigned char Pressao_H[8], Pressao_L[8];
	char *split, *aux;
	
	aux = strdup(payload);
	split = strsep(&aux,"x");
	strcpy(Pressao_H,split);
	split = strsep(&aux,"x");
	strcpy(Pressao_L,split);
	
	*HHH = atoi(Pressao_H);
	*LLL = atoi(Pressao_L);
	
	if(*HHH>=0 & *HHH <=999 & *LLL>0 &*LLL<=999){
		return 1;
	}
	else{
		return 0;
	}
}


// Função para inicializção da USART
void USART_Init(unsigned int ubrr){
	UBRR0H = (unsigned char)(ubrr>>8); // Ajusta a taxa de transmissão
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);//Habilita o transmissor e o receptor
	UCSR0C = (3<<UCSZ00);//Habilita o formato do frame: 8bits de dados e 1 de parada
	
	DDRC |= 0b00000010; //Tx como saída
	DDRC &= 0b11111110; //Rx como saída
}
//Função para envio de um frame de 5 a 8 bits
void USART_Transmit(unsigned char data){
	while(!(UCSR0A & (1<<UDRE0))); // Espera a limpeza do registrador de transmissão
	UDR0 = data;//Coloca o dado no registrador e o envia
}
//Função para recepção de um frame de 5 a 8 bits
unsigned char USART_Recive(void){
	while(!(UCSR0A & (1<<RXC0))); // Espera o dado ser recebido
	return  UDR0;//Lê o dado recebido e retorna
}

ISR(ADC_vect)
{
	}

ISR(TIMER0_COMPA_vect){
	tempo_ms = (tempo_ms+1);
	PORTD ^= 0b00000001; //degub freq Timer0
	
	if((tempo_ms%200)==0)
	{
		flag_200ms = 1;
	}
	if((tempo_ms)%(3333/Freq_Respiracao)==0)
	{
		flag_resp_por16 = 1;
	}
	if((tempo_ms%150)==0)
	{
		flag_150ms = 1;
		ADCSRA |= 0b00001000;
	}
	if ((tempo_ms%1000)==0)
	{
		flag_1s = 1;
	}
	if ((tempo_ms%2500)==0)
	{
		flag_2500ms = 1;
	}
} 

ISR(INT0_vect){
switch(escolhe){
		case 6:
		volume = volume + 1;
		if (volume>8)
		{
			volume = 8;
		}
		break;
		
		case 4:
			if (valvula<100)
			{
				valvula = valvula + 10;

			}
			OCR1B = 2000+(200*(valvula/10));
			break;
		
		case 2:
		Freq_Respiracao = Freq_Respiracao + 1;
		if(Freq_Respiracao > 30)
		{
			Freq_Respiracao = 30;
		
		}
		break;
		case 8:
		break;
		
		
	}

	 
}
ISR(INT1_vect)
{
	switch(escolhe){
		case 6:
		volume = volume - 1;
		if (volume<1)
		{
			volume = 1;
		}
		break;
		
		case 4:
		if (valvula>0)
		{
			valvula = valvula - 10;

		}
		OCR1B = 2000+(200*(valvula/10));
		break;
		
		case 2:
		Freq_Respiracao = Freq_Respiracao - 1;
		if(Freq_Respiracao < 5)
		{
			Freq_Respiracao = 5;
			
		}
		break;
		case 8:
		break;
		
		}
}
ISR(PCINT2_vect)
{
	static uint32_t tempo_ms_frequencia_anterior = 0;
	Freq_cardiaca = 60000/((tempo_ms-tempo_ms_frequencia_anterior)*2);
	tempo_ms_frequencia_anterior = tempo_ms;
}
ISR(PCINT0_vect){
	if (escolhe < 9)
	{
		escolhe = escolhe + 1;
		
	}
	if (escolhe > 8)
		{
			escolhe = 0;
		}
}
ISR(PCINT1_vect){
	if (rede<=2)
	{
		rede=rede+1;

	}
	if (rede > 2)
	{
		rede = 0;
	}
}

//-------------------------------------------------------------------------------\\	
	// 10% -> 180° | 5% -> 0º | Variação(5%) = 180º | grau/porcent = (180/5) = 36º/1%
	// 18° -> x
	// 36º   -> 1%
	// Logo: x = (18/36) = 1/2 = 0.5%  
	// 5% -> 2000
	// 0.5% -> y
	// Logo: y = (0.5*2000)/5  = 200 //Devemos acrescentar 200 e diminuir 200.
//----------------------------------------------------------------------------------\\//
//-------------------------------------------------------------------------------\\
// 10% -> 180° | 5% -> 0º | Variação(5%) = 180º | grau/porcent = (180/5) = 36º/1%
// angulo° -> x
// 36º   -> 1%
// Logo: x = (angulo/36) = angulo/36 = (angulo/36)%
// 5% -> 2000
// (angulo/36)% -> y
// Logo: y = ((angulo/36)*2000)/5 //Devemos acrescentar ((angulo/36)*2000)/5  e diminuir ((angulo/36)*2000)/5 .
//
//	AngMax = (Volume*22,5) -> x 
//	36º  -> 1%
// AngMax/36 = x
//----------------------------------------------------------------------------------\\//
void anima_servo(uint8_t *flag_disparo,uint8_t volume,uint8_t bateria){
	static uint8_t subindo = 1;
	float angulomax,acrescentar;
	angulomax = ((volume+1)*(45/2));
	acrescentar = (angulomax/180)*2000;
	if (*flag_disparo)
	{
		if (bateria>0)
		{		
			if (subindo)
			{
 				//Buzzer--------------------
 						PORTD &= 0b01111111;
				//Buzzer-------------------	
			
				OCR1A += 250;
				if (OCR1A>(2000+acrescentar))
				{
					subindo = 0;
				}
			}
			if (!subindo)
			{
				if (OCR1A>=2000)
				{
					OCR1A -= 250;

				}			
 				//Buzzer---------------
				if (OCR1A == 2000)
 				{
 					PORTD|=0b10000000;
				}				
				//Buzzer---------------	
			
				if (OCR1A<2000)
				{
					subindo = 1;
					OCR1A = 2000;
				}
			}
		}
		else{
			OCR1A = 0;
		}
	}
	*flag_disparo = 0;
}
void bateria_maquina(uint8_t *flag_disparo,uint8_t rede,uint8_t sujeira,uint8_t desempenho){
	if (rede == 2)
	{
		if (*flag_disparo)
		{
			
		
			if (bateria>0)
			{
				if (sujeira<100)
			{
				sujeira = sujeira+2;
			}
			if (desempenho>50)
			{
				desempenho = desempenho - (sujeira/2);
			}
				bateria = bateria-5;
			}
			if (bateria<=0)
			{
				bateria=0;
			}
		}
	}
	if (rede==0)
	{
		if (*flag_disparo)
		{
			
			if (sujeira<100)
			{
				sujeira = sujeira+2;
			}
			if (desempenho>50)
			{
				desempenho = desempenho - (sujeira/2);
			}
			if (bateria<100)
			{
				bateria = bateria+2;
			}
			if (bateria>=100)
			{
				bateria = 100;
			}
		}
	}
	
	*flag_disparo = 0;
}
void sujeira_maquina(uint8_t *flag_disparo,uint8_t sujeira){
	
		
		if (*flag_disparo)
		{
			if (bateria>0)
			{
			
			if (sujeira<100)
			{
				sujeira = sujeira+2;
			}
			}
		}
	

	*flag_disparo = 0;
}
void desempenho_maquina(uint8_t *flag_disparo,uint8_t desempenho,uint8_t sujeira){
	
		if (*flag_disparo)
		{
			if (bateria>0)
			{
			
			if (desempenho>50)
			{
				desempenho = desempenho - (sujeira/2);
			}
			}
		}
	

	*flag_disparo = 0;
}
void anima_lcd(uint8_t *flag_disparo,uint8_t *flag_disparo2,uint8_t Freq_Respiracao, uint16_t Freq_cardiaca,float temperatura, uint16_t saturacao_o2,uint8_t valvulaa, uint16_t escolhee,unsigned char pressao_arteriall,uint8_t volume,uint8_t bateria,uint8_t sujeiraa,uint8_t desempenhoo){ 
	
	unsigned char Freq_Respiracao_string[3];
	unsigned char Freq_cardiaca_string[4];
	unsigned char temperatura_string[5];
	unsigned char saturacao_o2_string[3];
	unsigned char valvula_string[4];
	unsigned char volume_string[2];
	unsigned char bateria_string[4];
	unsigned char sujeira_string[4];
	unsigned char desempenho_string[4];
	
	sprintf(Freq_Respiracao_string,"%u",Freq_Respiracao);
	sprintf(Freq_cardiaca_string,"%u",Freq_cardiaca);
	sprintf(temperatura_string,"%u",(uint16_t)(10*temperatura));
	temperatura_string[4]=temperatura_string[3];
	temperatura_string[3]=temperatura_string[2];
	temperatura_string[2]=",";
	sprintf(saturacao_o2_string,"%u",saturacao_o2);
	sprintf(valvula_string,"%u",valvulaa);
	sprintf(volume_string,"%u",volume);
	sprintf(bateria_string,"%u",bateria);
	sprintf(sujeira_string,"%u",sujeiraa);
	sprintf(desempenho_string,"%u",desempenhoo);
	if (*flag_disparo)
	{
		if (bateria>0)
		{
		nokia_lcd_power(1);
		switch(escolhee){
			case 0:
 				nokia_lcd_clear();
// 				nokia_lcd_set_cursor(0,0);
// 				nokia_lcd_write_string(Freq_Respiracao_string,1);
// 				nokia_lcd_set_cursor(15,0);
// 				nokia_lcd_write_string("resp/min",1);
		
				nokia_lcd_set_cursor(0,0);
 				nokia_lcd_write_string("Sinais Vitais",1);
				
 				
		
				nokia_lcd_set_cursor(0,9);
				nokia_lcd_write_string("->",1);
				nokia_lcd_set_cursor(13,9);
				nokia_lcd_write_string(Freq_cardiaca_string,1);
				nokia_lcd_set_cursor(40,9);
				nokia_lcd_write_string("bpm",1);
				
				
				nokia_lcd_set_cursor(0,18);
				nokia_lcd_write_string("->",1);
 				nokia_lcd_set_cursor(13,18);
 				nokia_lcd_write_string(temperatura_string,1);
 				nokia_lcd_set_cursor(45,18);
 				nokia_lcd_write_string("C",1);
		
				nokia_lcd_set_cursor(0,27);
				nokia_lcd_write_string("->",1);
				nokia_lcd_set_cursor(13,27);
				nokia_lcd_write_string(saturacao_o2_string,1);
				nokia_lcd_set_cursor(35,27);
				nokia_lcd_write_string("%SpO2",1);
		
// 				nokia_lcd_set_cursor(0,36);
// 				nokia_lcd_write_string(valvula_string,1);
// 				nokia_lcd_set_cursor(20,36);
// 				nokia_lcd_write_string("%O2",1);
			
					nokia_lcd_set_cursor(0,36);
					nokia_lcd_write_string("->",1);
					nokia_lcd_set_cursor(13,36);
					nokia_lcd_write_string(pressao_arteriall,1);
					nokia_lcd_set_cursor(40,36);
					nokia_lcd_write_string("mmHg",1);
				
				nokia_lcd_render();
				break;
 				
			case 2:
			
				nokia_lcd_clear();
				nokia_lcd_set_cursor(0,0);
				nokia_lcd_write_string("Parametros",1);
				
				nokia_lcd_set_cursor(0,10);
				nokia_lcd_write_string("->",1);
				
				nokia_lcd_set_cursor(14, 10);
				nokia_lcd_write_string(Freq_Respiracao_string,1);
				
				nokia_lcd_set_cursor(27,10);
				nokia_lcd_write_string("*Resp/min",1);
				
				nokia_lcd_set_cursor(0,20);
				nokia_lcd_write_string("->",1);
				
				nokia_lcd_set_cursor(14,20);
				nokia_lcd_write_string(valvula_string,1);
				
				nokia_lcd_set_cursor(36, 20);
				nokia_lcd_write_string(" %02",1);
				
				nokia_lcd_set_cursor(0,30);
				nokia_lcd_write_string("->",1);
				
				nokia_lcd_set_cursor(14,30);
				nokia_lcd_write_string(volume_string,1);
				
				nokia_lcd_set_cursor(32, 30);
				nokia_lcd_write_string("vol",1);
				
				nokia_lcd_render();
				break;
				
			case 4:
				
				nokia_lcd_clear();
				nokia_lcd_set_cursor(0,0);
				nokia_lcd_write_string("Parametros",1);
				
				nokia_lcd_set_cursor(0,10);
				nokia_lcd_write_string("->",1);
				
				nokia_lcd_set_cursor(14, 10);
				nokia_lcd_write_string(Freq_Respiracao_string,1);
				
				nokia_lcd_set_cursor(25,10);
				nokia_lcd_write_string(" Resp/min",1);
				
				nokia_lcd_set_cursor(0,20);
				nokia_lcd_write_string("->",1);
				
				nokia_lcd_set_cursor(14,20);
				nokia_lcd_write_string(valvula_string,1);
				
				nokia_lcd_set_cursor(36, 20);
				nokia_lcd_write_string("*%02",1);
				
				nokia_lcd_set_cursor(0,30);
				nokia_lcd_write_string("->",1);
				
				nokia_lcd_set_cursor(14,30);
				nokia_lcd_write_string(volume_string,1);
				
				nokia_lcd_set_cursor(32, 30);
				nokia_lcd_write_string("vol",1);
				
				nokia_lcd_render();
				
				break;
				case 6:
				
				nokia_lcd_clear();
				nokia_lcd_set_cursor(0,0);
				nokia_lcd_write_string("Parametros",1);
				
				nokia_lcd_set_cursor(0,10);
				nokia_lcd_write_string("->",1);
				
				nokia_lcd_set_cursor(14, 10);
				nokia_lcd_write_string(Freq_Respiracao_string,1);
				
				nokia_lcd_set_cursor(25,10);
				nokia_lcd_write_string(" Resp/min",1);
				
				nokia_lcd_set_cursor(0,20);
				nokia_lcd_write_string("->",1);
				
				nokia_lcd_set_cursor(14,20);
				nokia_lcd_write_string(valvula_string,1);
				
				nokia_lcd_set_cursor(36, 20);
				nokia_lcd_write_string("%02",1);
				
				nokia_lcd_set_cursor(0,30);
				nokia_lcd_write_string("->",1);
				
				nokia_lcd_set_cursor(14,30);
				nokia_lcd_write_string(volume_string,1);
				
				nokia_lcd_set_cursor(32, 30);
				nokia_lcd_write_string("*vol",1);
				
				nokia_lcd_render();
				
				break;
				case 8:
				
				nokia_lcd_clear();
				nokia_lcd_set_cursor(8,0);
				nokia_lcd_write_string("MAQUINA",1);
				
				nokia_lcd_set_cursor(0,15);
				nokia_lcd_write_string("->",1);
				
				nokia_lcd_set_cursor(50+11, 15);
				nokia_lcd_write_string(bateria_string,1);
				nokia_lcd_set_cursor(65+14, 15);
				nokia_lcd_write_string("%",1);
				
				nokia_lcd_set_cursor(14,15);
				nokia_lcd_write_string("Bateria",1);
				
				
				nokia_lcd_set_cursor(0,25);
				nokia_lcd_write_string("->",1);
				
				nokia_lcd_set_cursor(50+11, 25);
				nokia_lcd_write_string(sujeira_string,1);
				nokia_lcd_set_cursor(65+14, 25);
				nokia_lcd_write_string("%",1);
				
				nokia_lcd_set_cursor(14,25);
				nokia_lcd_write_string("Sujeira",1);
				
				nokia_lcd_set_cursor(0,35);
				nokia_lcd_write_string("->",1);
				
				nokia_lcd_set_cursor(50+11, 35);
				nokia_lcd_write_string(desempenho_string,1);
				nokia_lcd_set_cursor(65+14, 35);
				nokia_lcd_write_string("%",1);
				
				nokia_lcd_set_cursor(14,35);
				nokia_lcd_write_string("Desempe",1);
				
				nokia_lcd_render();
				
				break;
				
			default:
			
				break;
		}
		}
		else{
			nokia_lcd_power(0);
		}
		*flag_disparo = 0;
		}
}

void leitura_sensores_ADC(uint8_t *flag_disparo){
	static uint8_t cont_canal = 0;
	
	if (*flag_disparo)
	{
		switch(cont_canal){
			case 0:
			temperatura = ((float)ADC/20.48)+10;
			ADMUX = 0b01000001; //Muda para o canal 1
			break;
			
			case 1:
			saturacao_o2 = (float)ADC/8.192;
			ADMUX = 0b01000000; //Muda para o canal 0
			break;
			
			default:
			temperatura = 0;
			saturacao_o2 = 0xFF;
		}
		if (cont_canal<1){
		cont_canal++;
		}
		else{
			cont_canal = 0;
		}
		//TESTA ALARME
		if ((saturacao_o2<60)||(temperatura<35)||(temperatura>41))
		{
			PORTD |=010000000;
		}
		else
		{
			PORTD &= ~0b10000000;
		}
	
	}
	*flag_disparo = 0;
}



int main(void)
{
	//GPIO
	USART_Init(MYUBRR);
	DDRC = 0b11111011; //Habilita os pinos PC0..7 como saídas	
	DDRD  = 0b10100001; // PD1...4, PD6 = ENTRADA, PD0, PD5, PD7 = SAÍDA	
	PORTD = 0b11101111;
	PORTC = 0b00000100;
	//GPIO
	DDRB = 0b10111111;// Habilita os pinos PB1(0CR1A) E PB2(0CR1B) PARA SAÍDA
	PORTB = 0b01000000;
	
	//TIMER TC1 - MODO PWM RÁPIDO VIA ICR1, prescaler = 8
	//TOP = (F_CPU/(PRE*F_PWM))-1 , com pre = 8 e F_PWM = 5Hz >>>> TOP = 39999
	ICR1 = 39999;//Configura o período do PWM (2ms) //Registrador icr1
	TCCR1A = 0b10100010;//modo pwm rapido via ICR1,ativa o pWm no OC1B n-invert //registador
	
	TCCR1B = 0b00011010;//Prescaler = 8 //registrador
	
	OCR1A = 2000; //2ms //valor de comparação ((399/400)*10% = 1995/4000)
	OCR1B = 2000; //1ms //valor de comparação  ((399/400)*5% = 1995/2000)
//-------------------------------------------------------------------------------\\	
	// 10% -> 180° | 5% -> 0º | Variação(5%) = 180º | grau/porcent = (180/5) = 36º/1%
	// 22.5° -> x
	// 36º   -> 1%
	// Logo: x = (22.5/36) = 5/8 = 0.625%  
	// 5% -> 2000
	// 0.625% -> y
	// Logo: y = (0.625*2000)/5  = 250 //Devemos acrescentar 250 e diminuir 250.
//----------------------------------------------------------------------------------\\//

	nokia_lcd_init();

	
	// Configuração das interrupções
	EICRA  = 0b00001010; // interrupções externas INT0 e INT1 na borda de descida
	EIMSK  = 0b00000011; // habilita as interrupções externas INT0 e INT1
	PCICR  = 0b00000111; // interrupções pin change 2 (porta D) e pin change porta B E pin change porta C
	PCMSK2 = 0b00010000; // interrupções pin change PD4 - Contador de BPM
	PCMSK0 = 0b01000000; // Interrupções pin change PB6 
	PCMSK1 = 0b00000100; // Interrupções pin change PB2
	
	//Configuração do timer
	TCCR0A = 0b00000010; // habilita modo CTC do TC0
	TCCR0B = 0b00000011; //liga TC0 com prescaler = 64
	OCR0A = 249; //ajusta o comparador para TC0 contar até 249
	TIMSK0 = 0b00000010; //habilita a interrupção na igualdade de comparação OCROA. 1m = (64*(249+1))/16MHz
	
	//habilita o flag de interrupções globais
	sei(); //Habilita interrupções globais ativando o bit I do SREG
	//Configuração ADC
	ADMUX = 0b01000000;
	ADCSRA = 0b11100111; //habilita o Ad, habilita interrupção, modo de conversão contínua, prescaler = 128
	ADCSRB = 0x00; //Modo de conversão contínua
	DIDR0 = 0b00111111;//habilita o pino PC0 como entrada do AC0
	
	while (1)
	{
		bateria_maquina(&flag_1s,rede,sujeira,desempenho);
// 		sujeira_maquina(&flag_1s,sujeiraa);
// 		desempenho_maquina(&flag_1s, desempenho, sujeiraa);
		leitura_sensores_ADC(&flag_150ms);
		anima_servo(&flag_resp_por16,volume,bateria);
		anima_lcd(&flag_200ms,&flag_150ms,Freq_Respiracao,Freq_cardiaca,temperatura,saturacao_o2,valvula,escolhe,pressao_arterial,volume,bateria,sujeira,desempenho);
		
	}
}



    



