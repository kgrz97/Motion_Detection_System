/* Poznan, maj 2019
 * Konrad Grzelczyk
 *
 * Oprogramowanie dla kontrolera Atmega8
 * Projekt: Czujnik ruchu z modulem GSM ver2.0
 * */

/* Zasada dzialania:
 * - czas na inicjalizacje: 2-3min
 * - jesli urzadzenia jest gotowe, odbiorca otrzymuje sms o tresci "System OK"
 * - urzadzenie co godzine sprawdza stan baterii, jesli jest za niski wysyla powiadomienie o potrzebnej wymianie
 * ------------------------------------------------ver 1.0
 * - urzadzenie wykrywa ruch dwukrotnie:
 *    -> pierwsze wykrycie wybudza kontroler oraz inicjalizuje modul GSM
 *    -> drugie wykrycie (nastepujace ok. 1,5min po pierwszym) wysyla wiadomosc o wykryciu ruchu
 *
 * ------------------------------------------------ver 2.0
 * -urzadzenie wykrywa ruch jednokrotnie:
 * 	  -> po pierwszym wykryciu nastepuje wybudzenie, inicjalizacja oraz wyslanie wiadomosci sms
 *
 * - po wyslaniu sms'a o wykryciu ruchu, kolejne wykrycia czujnika nie sa brane pod uwage przez 30 minut
 * - kontroler dziala w trybie uspienia
 * */

/* Uzywane piny:
 * PB0 - domyslnie stan wysoki, stan niski aktywuje przekaznik polaczony z modulem GSM
 * PB1 - sterowanie trybem pracy przetwornicy zasilajacej kontroler
 * PC5 - port ADC, sprawdza stan baterii poprzez dzielnik napiecia (20k + 10k)
 * PD1 - port UART, wysyla komendy do modulu GSM
 * PD2 - pin przerwania czujnika ruchu
 * */

// Numer odbiorcy wpisuje siê w sendSMS

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <avr/sleep.h>
#include <avr/power.h>

#define UART_BAUD 9600
#define __UBRR ((F_CPU+UART_BAUD*8UL) / (16UL*UART_BAUD)-1)


void USART_Init( uint16_t ubrr);
void USART_Off();
void Usart_Transmit(unsigned char data);
void Send_clause(char * napis);
void sendSMS(char * wiadomosc);
void Delay(int param);
void Initialization_GSM();
void CheckBattery();
void TMR2_init( void );
void Go_to_sleep();

volatile uint8_t cnt=0, cnt2 = 0, czy_wyslano = 0, stan_baterii = 0, wykrycie = 0, runda_timer = 0, po_wykryciu = 0,
		         adc = 0;

int main(void)
{
	PORTB  = 0xff; //w³¹cz wewnêtrzne rezystory pull-up
	PORTC  = 0xff; //w³¹cz wewnêtrzne rezystory pull-up
	PORTD  = 0x3f; //w³¹cz wewnêtrzne rezystory pull-up






	DDRB |= (1<<PB0); 	// Pin B0 jako wyjscie do przekaznika
	PORTB &= ~(1<<PB0); // podciagniete do masy poniewaz przekaznik sterowany jest stanem niskim
						// PB0 = 0 -> przekaznik zalaczony
						// PB0 = 1 -> przekaznik rozlaczony

	DDRB |= (1<<PB1);	// Pin B1 steruje trybem pracy przetwornicy
	PORTB |= (1<<PB1); // PB1 = 0 -> low power
						// PB1 = 1 -> high power

	_delay_ms(1000);

	DDRD &= ~(1<<PD2); /// Ustawienie pinu D2 jako wejscie (INT0)
	PORTD |= (1<<PD2);

	Delay(1);



	/////// ADC
	ADCSRA =   (1<<ADEN) |(1<<ADPS0)  |(1<<ADPS1) |(1<<ADPS2);
	adc = ADCSRA;
	ADMUX  =  (1<<REFS1) | (1<<REFS0) |(1<<MUX2) | (1<<MUX0); // wybór kana³u ADC5 na pinie PC5
	DDRC=0xff;  //Ustawienie pinu jako wejœcie
	DDRC &= ~(1<<PC5);

	//CheckBattery();

	USART_Init(__UBRR);
	// Ustawienie baud rate na 9600 w module GSM
	Send_clause("AT+IPR=9600\r\n");
	// Inicjalizacja i test dzialania systemu
	Initialization_GSM();

	if(stan_baterii==0)
	sendSMS("System OK");

	_delay_ms(1000);

	USART_Off();

	ADCSRA &= ~(1<<ADEN);

	PORTB |= (1<<PB0); // wylaczenie przekaznika

	//MCUCR &= ~((1 << ISC01) | (1 << ISC00)); // zmiana stanu, stan niski wywoluje przerwanie
	GICR |= (1<<INT0);


	TMR2_init();

	sei();//Globalne uruchomienie przerwañ

    while(1)
    {
    	Go_to_sleep();
    }// end while
}// end main

void sendSMS(char * wiadomosc)
{
	_delay_ms(1000);
	Send_clause("AT\r\n");

	_delay_ms(1000);
	Send_clause("AT+CMGF=1\r\n");
	_delay_ms(1000);
	Send_clause("AT+CMGS=\"+48668171925\"\r\n"); /* +48668171925 */
	_delay_ms(1000);
	Send_clause(wiadomosc);
	_delay_ms(1000);
	Usart_Transmit(26);
	_delay_ms(2000);
	}
void USART_Init( unsigned int ubrr)
{
		UBRRH = (unsigned char)(ubrr>>8);
		UBRRL = (unsigned char)ubrr;
		UCSRB = (1<<TXEN);
		UCSRC = (1<<URSEL)|(3<<UCSZ0);
}

void USART_Off()
{
	UCSRB &= ~(1<<TXEN);
}


void Usart_Transmit(unsigned char data)
{
	while ( !( UCSRA & (1<<UDRE)) );
	UDR = data;
	}


void Send_clause(char * napis)
{
	while(*napis)
		Usart_Transmit(*napis++);
}

void Delay(int param)
{
	if(param == 1)
	for(int i=0; i<1200; i++)
		_delay_ms(50);

	if(param == 2)
	for(int i=0; i<500; i++)
		_delay_ms(50);

	if(param == 3)
		for(int i=0; i<300; i++)
			_delay_ms(50);
	}

void Initialization_GSM()
{
	for(int i = 0; i < 10; i++)
	{
		Send_clause("AT\r\n");
		_delay_ms(800);
	}
}

void CheckBattery()
{
	ACSR &= ~(1<<ACD);
	ADCSRA = adc;

	int pomiary = 0, srednia=0;


	for(int i=0; i<3; i++)
	{
		ADCSRA |= (1<<ADSC); //ADSC: uruchomienie pojedynczej konwersji

		while(ADCSRA & (1<<ADSC)); //czeka na zakoñczenie konwersji

		pomiary += ADC;
	}



	srednia = pomiary/3;

		/*USART_Init(__UBRR);
		char b[32];
		itoa(srednia, b, 10);
		Send_clause(b);
		Send_clause("\r\n");
		USART_Off(); */

	if(srednia <= 887) // ADC = 890 ~ 5,32V
	{
		USART_Init(__UBRR);
		PORTB &= ~(1<<PB0);
		Delay(2);
		Initialization_GSM();
		sendSMS("Miejsce 1. Niski stan baterii!");
		stan_baterii = 1;
		USART_Off();
		PORTB |= (1<<PB0);
	}
	ACSR |= (1<<ACD);
}

void TMR2_init( void )
{
	ASSR |= (1<<AS2);

	 //preskaler
	 TCCR2 = (1<<CS22) | (1<<CS20); //1024 (8sek)

	 //zaczekaj, a¿ bêdzie mo¿na zmieniaæ ustawienia timera
	 //pracuj¹cego w trybie asynchronicznym (patrz datasheet)
	 while(ASSR & ((1<<TCN2UB) | (1<<OCR2UB) | (1<<TCR2UB)));

	 //zeruj flagê przerwania Timer2
	 TIFR |= (1<<TOV2);

	 //w³¹cz przerwanie od przepe³nienia timer2
	 TIMSK  |= (1<<TOIE2);

}


ISR(TIMER2_OVF_vect)
{
  if(stan_baterii==0)
  {

	  cnt++;

  if(cnt >= 240)
  {
	  runda_timer ++;

	  if(runda_timer >= 60) // sprawdzanie baterii +/- co 4 godziny
	  {
		  CheckBattery();
		  runda_timer = 0;
	  }
	  cnt=0;
	  TCNT2 = 0;          //Pocz¹tkowa wartoœæ licznika
  }
  }

 cnt2++;	 //zwieksza zmienna licznika ponownego wykrycia

  //  po wyslaniu sms'a o wykryciu czujnik NIE jest brany pod uwagê przez pol godziny
  if((cnt2 >= 30) & (wykrycie == 1))
  {
	  po_wykryciu++;

	  if(po_wykryciu >= 30) // interwal: 60=30min, 30=15min
	  {
		  wykrycie = 0;
	  }

	  cnt2 = 0;
  }
}

ISR (INT0_vect) // czujnik
{
	if(wykrycie == 0)
	{
	    		//Delay(2);
	    		USART_Init(__UBRR);
	    		PORTB &= ~(1<<PB0);
	    		Delay(2);
	    		Initialization_GSM();
	    		sendSMS("Miejsce 1. Wykryto ruch");
	    		wykrycie = 1;
	    		cnt2 = 0;

	    		PORTB |= (1<<PB0);
	    		USART_Off();
	}
}

void Go_to_sleep()
{
	ACSR |= (1<<ACD);
	//USART_Off();

	PORTB |= (1<<PB0);
	sei();
	set_sleep_mode(SLEEP_MODE_PWR_SAVE);
	_delay_ms(50);
	sleep_enable();
	sleep_cpu();


	sleep_disable();
	_delay_ms(75);
	cli();
}


/* Kod dzialania urzadzenia po wybudzeniu
 * if(wykrycie == 0)
	{
	    	czy_wyslano = 0;

	    	if ((PIND & ~(1<<PD2))) // pierwsze wykrycie
	    	{
	    		czy_wyslano = 1;
	    	}

	    	if(czy_wyslano == 1)
	    	{
	    		//Delay(2);
	    		USART_Init(__UBRR);
	    		PORTB &= ~(1<<PB0);
	    		Delay(2);
	    		Initialization_GSM();

	    		if ((PIND & (1<<PD2))) // drugie wykrycie
	    		{
	    			_delay_ms(50);
	    		}
	    		else
	    		{
	    			sendSMS("Miejsce 1. Wykryto ruch");
	    			wykrycie = 1;
	    			cnt2 = 0;
	    		}
	    		PORTB |= (1<<PB0);
	    		USART_Off();
	    		czy_wyslano = 0;
	    	}
	}
 *
 * */
