#define F_CPU 16000000 // CPU speed in Hz
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//*******************************************************************************
//                            debounce_switch                                  
// Adapted from Ganssel's "Guide to Debouncing"            
// Checks the state of pushbutton S0 It shifts in ones till the button is pushed. 
// Function returns a 1 only once per debounced button push so a debounce and toggle 
// function can be implemented at the same time.  Expects active low pushbutton on 
// Port D bit zero.  Debounce time is determined by external loop delay times 12. 
//*******************************************************************************
int8_t debounce_button(uint8_t button) {
  static uint16_t state = 0; //holds present state
  state = (state << 1) | (! bit_is_clear(PIND, button)) | 0xE000;
  if (state == 0xF000) return 1;
  return 0;
}

void spi_init(void){
//Set SS (PB0), MOSI, and SCLK as outputs */
	DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2);

	/* Set MISO (PB3) as an input */
	DDRB &= ~(1 << PB3);

	SPCR |= (1 << SPE) | (1 <<MSTR); //enable SPI and set master
	SPSR |= (1 << SPI2X); //run i/o clock divided by 2	

}

void tcnt0_init(void) {
	/* Enable Timer 0 with a 128 prescaler */
	/* TIMSK = Timer Mask; 1 means enable the timer interrupt */
	TIMSK |= (1 << TOIE0);
	/* TCCR0 = Timer 0 Control Register; 1 means enable that prescaler bit */
	TCCR0 |= (1 << CS02) | (1 << CS00);
//	TCCR0 |= (1 << CS01); //prescaler 8 = 1.907 Hz
}

ISR(TIMER0_OVF_vect){
	//polling style check for button presses
	//loop really quickly through each shift register

	uint8_t clock=PORTC;
	uint8_t load=PORTA;
	uint8_t led = PORTB;
	int i, j=0;	
	clock = clock & 0x0F;
	while(!clock){
		j++;
		clock = clock >> 1;			
	}
	clock = PINC; //restore clock
	//Latch all switch values to respective memory of shift registers by toggling sh/ld pin
	PORTA =0x00 ; 
	for (i=0; i<100; i++){
		_delay_ms(2);
	}
	PORTA = ~(PORTA); 
	//have to latch clock 8 times to serial shift out bits
	for(i=0; i<15; i++){   
		PORTC = (PINC ^ clock); //toggle clock on and off  
		if (debounce_button(j)){ //should read on falling edge of sh/ld
			PORTB ^= (debounce_button(j)) << (i/2); 
		}
//		PORTB ^= (1 << (i/2)); 
	}

//indicator to confirm how many roll overs - test
	PORTB = PORTB>>4;
	PORTB++;
	PORTB = (PORTB << 4) | (led & 0x0F);

	PORTC = clock; //restore original
	PORTA = load;
	if (!(PINC & 0x04))
		PORTC = 0x01; //loop back	
	else
		PORTC = PORTC << 1; //shift clock to next output 
}

//*******************************************************************************
// Check switch S0.  When found low for 12 passes of "debounc_switch(), increment
// PORTB.  This will make an incrementing count on the port B LEDS. 
//*******************************************************************************
int main()
{
tcnt0_init();
//spi_init();

//setup data direction , 1 for output - 0 for input
DDRC = 0xFF;  //Control the clock
DDRA = 0xFF;  //Control the Shift/Load
//DDRF = 0xFF; //Control clock inhibit pin
DDRE = 0x00; //Accept Inputs
//setup LEDs as outputs
DDRB = 0xFF;

PORTC = 0x01; //assert logic 1 to activate first
PORTB = 0x00;
PORTA = 0x0F; //keep sh/ld pin high, latch low to push current state of switches to register
//PORTF = 0x00; //
PORTE = 0x00; //no pullup resistor, so returns real state of pin

sei(); //set global interrupts

while(1){     //do forever




  } //while 
  


} //main
