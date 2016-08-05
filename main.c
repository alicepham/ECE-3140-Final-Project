#include <Board_Accelerometer.h>
#include <Board_Magnetometer.h>
#include <fsl_debug_console.h>
#include <Board_Buttons.h>
#include <board.h>
#include <math.h>
#include <Board_LED.h>
//#include "utils.c"

/* Structure which respesents a node in the robots path */
typedef struct path{
	int direction;
	int duration;
	struct path *previous;
	struct path *next;
}path_t;

typedef struct time {   
	unsigned int sec;   
	unsigned int msec; 
} time_t; 

time_t *current_time = NULL;	/* Amount of time passed since processes began running */
/* Pointer to the head of the robot's path list */
path_t* pathHead;
int move_Duration;
int turn_Duration;

/* Function Declarations */
path_t* Determine_Direction(void);
path_t* Take_Inputs(void);
int Get_Direction(void);
void LED_Direction(int,int);
void Drive(path_t*);
path_t* Dynamic_Driving(void);
void servo_Forward(void);
void servo_Backward(void);
void servo_Turn_Left(void);
void servo_Turn_Right(void);
void servo_Stop(void);
void right_Servo(void);
void left_Servo(void);
void init(void);
void delay(void);
void turn(void);
void delay_msec(void);
void delay1(void);
int Bluetooth(void);
path_t* Bluetooth_Driving(void);
	
int main() {
	/* Unsigned int to store which buttons are pressed */
	unsigned int buttonStates;
	/*Initialize buttons, Bluetooth, interupts, time, LEDs, board and accelerometer */
	init();
	Buttons_Initialize();
	LED_Initialize();
	hardware_init();
	Accelerometer_Initialize();
	
	/* Waits until the user presses a button */
	while(buttonStates==0) buttonStates = Buttons_GetState();
	
	/* If button 2 is pressed manual driving else dynamic driving */
	if(buttonStates == 1){
		/* Wait till button 2 is released */
		while(buttonStates == 1) buttonStates = Buttons_GetState();
		/* Wait until a button is pressed again */
		while(buttonStates == 0) buttonStates = Buttons_GetState();
		
		/* If button 2 is pressed use Take_Inputs() else use Determine_Direction() */
		if(buttonStates == 1){
			LED_Direction(1,1); /* Turn on green LED */
			pathHead = Take_Inputs();
		}
		else {
			LED_Direction(2,1); /* Turn on red LED */
			pathHead = Determine_Direction();
		}

		/* Set durations of servo actions */
		move_Duration = 5000;
		turn_Duration = 2000;
		Drive(pathHead); /* Execute actual driving of returned path */
	}
	else {
		/* Wait till button 2 is released */
		while(buttonStates == 2) buttonStates = Buttons_GetState();
		/* Wait until a button is pressed again */
		while(buttonStates == 0) buttonStates = Buttons_GetState();
		
		/* Set durations of servo actions */
		move_Duration = 500;
		turn_Duration = 500;
		/* If button 2 is pressed use Take_Inputs() else use Determine_Direction() */
		if(buttonStates == 1){
			LED_Direction(1,1); /* Turn on green LED */
			Bluetooth_Driving();
		}
		else {
			LED_Direction(2,1); /* Turn on red LED */
			Dynamic_Driving();
		}		
	}
	
	/* Dispaly driving sequence using LEDs */
	path_t* pointer = pathHead;
	while(pointer!=NULL){
		LED_Direction(pointer->direction,1);
		pointer = pointer->next;
	}	
}


/*------------------------------------------------------------------------
 * Main Methods
 *----------------------------------------------------------------------*/

/*------------------------------------------------------------------------
 * Determine_Direction
 * 		Has the user give direction inputs by tilting the board then  
 *		returns a pointer to a linked list which contains path structs each
 *		which has movement directions 1-North, 2-South, 3-West, 4-East for
 *		one node of the robot's path
 *----------------------------------------------------------------------*/
path_t* Determine_Direction(void) {
	unsigned int buttonStates;
	Buttons_Initialize();
	path_t *pathHead;
	int direction;
	
	buttonStates = Buttons_GetState();

	/* Allocate space for new path */
	path_t *newPath = malloc(sizeof(path_t));	
	newPath->next = NULL;
	newPath->direction = 0;
	pathHead = newPath;
	
	/* Keep taking inputs until button 2&3 are pressed */
	while(buttonStates!=3){
		buttonStates = Buttons_GetState();
		
		/* If button 2 is pressed get the current board direction*/
		if(buttonStates==1){
			direction = Get_Direction();
			newPath->direction = direction;
			LED_Direction(newPath->direction,1);
			newPath->next = malloc(sizeof(path_t));
			newPath = newPath->next;
			
			/* Wait until button 3 is pressed signaling the board is flat again */
			while(buttonStates!=2) buttonStates = Buttons_GetState();
		}
	}
	return pathHead;
}

/*------------------------------------------------------------------------
 * Take_Inputs
 *   User inputs driving directions using button inputs which are then
 *		stored in the nodes of the path linked list, it then returns a
 *		pointer to a linked list which contains path structs each
 *		which has movement directions 1-North, 2-South, 3-West, 4-East for
 *		one node of the robot's path
 *----------------------------------------------------------------------*/
path_t* Take_Inputs(void){
	unsigned int buttonStates;
	unsigned int lastButtonStates;
	Buttons_Initialize(); /*Initialize Buttons */
	path_t *pathHead;
	
	buttonStates = Buttons_GetState();

	/* Allocate space for new_process */
	path_t *newPath = malloc(sizeof(path_t));	
	newPath->next = NULL;
	newPath->direction = 0;
	pathHead = newPath;

	/* Keep taking inputs until button 2&3 are pressed */
	while(buttonStates!=3){
		lastButtonStates = buttonStates;
		buttonStates = Buttons_GetState();
		if(lastButtonStates!=buttonStates && buttonStates!=0){
			LED_Direction(0, 0); /* Turn off all LEDs */
			/* If button 2 is pressed add the current direction value
					to the path list */
			switch (buttonStates) {
				case 1: newPath->direction++;
				break;
				case 2: 
					if(newPath->direction > 4 || newPath->direction==0){
						newPath->direction = NULL;
					}
					LED_Direction(newPath->direction,1);
					newPath->next = malloc(sizeof(path_t));
					newPath = newPath->next;
				break;
				default: newPath->direction = NULL;
			}
		}
	}
	return pathHead;
}

/*------------------------------------------------------------------------
 * Dynamic_Driving
 *   Allows the user to drive the robot in real time by tilting the board
 *		the desired direction of movement for the robot then returns a pointer
 *		to a linked list which contains the path the robot drove while the
 *		user was controlling it.
 *----------------------------------------------------------------------*/
path_t* Dynamic_Driving(void){
	unsigned int buttonStates;
	int direction;
	int prev_Direction;
	
	/* Allocate space for a new path list */
	path_t *newPath = malloc(sizeof(path_t));	
	/* Initalize all path values */
	newPath->next = NULL;
	newPath->previous = NULL;
	newPath->direction = 0;
	newPath->duration = 0;
	pathHead = newPath;
	
	/* Initialize local variables */
	direction = NULL;
	prev_Direction = NULL;
	buttonStates = Buttons_GetState();
	
	/* Keep driving until button 2&3 are pressed */
	while(buttonStates!=3){		
		prev_Direction = direction;
		direction = Get_Direction();
		
	/*Update servo functions */
		LED_Direction(0,0); /* Turn off all LEDs */
		LED_Direction(direction, 0); /* Set the LED for the current direction */
	
		switch(direction){
			case 1: servo_Forward();
							break;
			case 2: servo_Backward();
							break;
			case 3: servo_Turn_Left();
							break;
			case 4: servo_Turn_Right();
							break;
			default: servo_Stop();
		}
		buttonStates = Buttons_GetState(); /* Get the new button states */
	}
	LED_Direction(0,0); /* Turn off all the LEDs */
	servo_Stop(); /* Stop all servos */
	
	return pathHead;
}

/*------------------------------------------------------------------------
 * Bluetooth_Driving
 *   Allows the user to drive the robot by issuing commands over Bluetooth
 *		from a connected mobile device, returns the path the robot traversed
 *----------------------------------------------------------------------*/
path_t* Bluetooth_Driving(void){
	unsigned int buttonStates;
	int direction;
	int prev_Direction;
	
	/* Allocate space for a new path list */
	path_t *newPath = malloc(sizeof(path_t));	
	/* Initalize all path values */
	newPath->next = NULL;
	newPath->previous = NULL;
	newPath->direction = 0;
	newPath->duration = 0;
	pathHead = newPath;
	
	/* Initialize local variables */
	direction = NULL;
	prev_Direction = NULL;
	buttonStates = Buttons_GetState();
	
	/* Keep driving until button 2&3 are pressed */
	while(buttonStates!=3){		
		prev_Direction = direction;
		direction = Bluetooth();
		
	/*Update servo functions */
		LED_Direction(0,0); /* Turn off all LEDs */
		LED_Direction(direction, 0); /* Set the LED for the current direction */
	
		switch(direction){
			case 1: servo_Forward();
							break;
			case 2: servo_Backward();
							break;
			case 3: servo_Turn_Left();
							break;
			case 4: servo_Turn_Right();
							break;
			default: servo_Stop();
		}
	}
	LED_Direction(0,0); /* Turn off all the LEDs */
	servo_Stop(); /* Stop all servos */
	
	return pathHead;
}

/*------------------------------------------------------------------------
 * Drive
 *	 Have the robot drive the path store in the pathHead list
 *		which was passed as an input
 *   Fields:
 *     pathHead - pointer to teh robot driving path which is a list
 *			of path structs each containing a direction for the robot
 *			to drive
 *----------------------------------------------------------------------*/
void Drive(path_t* pathHead){
	path_t* pointer = pathHead;
	while(pointer!=NULL){
		/* Blink the LED for the corresponding direction */
		LED_Direction(pointer->direction,0);
		switch(pointer->direction){
				case 1: servo_Forward();
								break;
				case 2: servo_Backward();
								break;
				case 3: servo_Turn_Left();
								break;
				case 4: servo_Turn_Right();
								break;
				default: servo_Stop();
			}
		pointer = pointer->next;
	}
	return;
}


/*------------------------------------------------------------------------
 * Servo Methods
 *----------------------------------------------------------------------*/
void servo_Forward(){
	for(int i=1;i<move_Duration;i++){
		right_Servo();
		left_Servo();
	}
	return;
}

void servo_Backward(){
	PTC->PSOR |=  1; //left servo	
	PTC->PSOR |=  (1 << 7); //right servo	
	delay1();
	PTC->PCOR |=  (1 << 7);			
	PTC->PCOR |=  1;	
	return;
}

void servo_Turn_Left(){
	for(int i=1;i<turn_Duration;i++){
		right_Servo();
		delay_msec();
	}
	return;
}

void servo_Turn_Right(){
	for(int i=1;i<turn_Duration;i++){
		left_Servo();
		delay_msec();
	}
	return;
}

void servo_Stop(){
	PTC->PCOR |=  (1 << 7);	
	PTC->PCOR |=  (1 << 7);	
	return;
}

void left_Servo(void){
	PTC->PSOR |=  1; //left servo
	delay_msec();
	PTC->PCOR |=  1;
	return;
}

void right_Servo(){
	PTC->PSOR |=  (1 << 7); //right servo	
	delay_msec();	
	PTC->PCOR |=  (1 << 7);	
}

/*------------------------------------------------------------------------
 * Helper Methods
 *----------------------------------------------------------------------*/

/*------------------------------------------------------------------------
 * LED_Direction
 *   Either blinks or turn on the corresponding LED to the direction value
 *		which was passed as an input
 *   Fields:
 *     direction - current direction value for the robot
 *     blink - Has value 1 if the LED should blink on then off otherwise
 *			has value 0 where the LED will just turn on
 *----------------------------------------------------------------------*/
void LED_Direction(int direction, int blink){
	int i;
	int val = 0x000;
	switch(direction){
		case 1: LED_On(0); /* Forwards, green LED */
						break;
		case 2: LED_On(1); /* Backwards, red LED */
						break;
		case 3: LED_On(2); /* Left, blue LED */
						break;
		case 4: LED_On(0); /* Right, white LED */
						LED_On(1);
						LED_On(2);
						break;
		default: LED_SetOut(val);
	}
	if(blink){	
		/* Wait for a little while */
		for(i=1; i<10000000; i++);
		LED_SetOut(val);
	}
		return;
}

/*------------------------------------------------------------------------
 * Get_Direction
 *   Returns the current orientation of the board as an int direciton value,
 *		either forwards, backwards, left, right or stop
 *----------------------------------------------------------------------*/
int Get_Direction(void){
	int direction;
	int delta_x, delta_y, delta_z;
	ACCELEROMETER_STATE state;
	
	/* Collect a thousand sample points */
	delta_x = 0; delta_y = 0;	delta_z = 0;
	for(int i=1;i<=100;i++){
		Accelerometer_GetState(&state);
		delta_x = delta_x + state.x;
		delta_y = delta_y + state.y;
		delta_z = delta_z + state.z;
	}
	/* Normalize the sample points */
	delta_x = delta_x/100;
	delta_y = delta_y/100;
	delta_z = delta_z/100;

	/* Based on the accelerometer data determine orientation */
	if(delta_x<100 && delta_x>-100 && delta_y>300) direction = 1; /* Forwards */
	else if(delta_x<100 && delta_x>-100 && delta_y<-300) direction = 2; /* Backwards */
	else if(delta_y<200 && delta_y>-200 && delta_x<-300) direction = 3; /* Left */
	else if(delta_y<200 && delta_y>-200 && delta_x>300) direction = 4; /* Right */
	else direction = NULL; /* Flat */
	
	return direction;
}

/*------------------------------------------------------------------------
 * Bluetooth
 *   Returns the direction corresponding to the value send from a mobile
 *		device to the robot's Bluetooth connection
 *----------------------------------------------------------------------*/
int Bluetooth(void){
	unsigned int data;
	int direction;

	data = UART4_D;
	if(data==237) direction = 1;
	else if(data == 238) direction = 2;
	else if(data == 239) direction = 3;
	else if(data == 232) direction = 4;
	else direction = 0;
	
	return direction;
}

/*------------------------------------------------------------------------
 * Interrupt Handler: Increments current_time by one millisecond
 *----------------------------------------------------------------------*/
void PIT1_IRQHandler(void) {
	/* Disable interrupts and clear flag */
	PIT->CHANNEL[1].TCTRL = 1;				
	PIT->CHANNEL[1].TFLG = 1; 	
	/* Add a msec to current_time */
	if(current_time->msec == 999) {
		current_time->sec += 1;
		current_time->msec = 0;
	} else {
		current_time->msec += 1;
	}
	/* Enable timer/enable interrupt */
	PIT->CHANNEL[1].TCTRL = 3;				
}

/*------------------------------------------------------------------------
 * init: Initializes servo pins, Bluetooth pins and PIT1 with interrupts
 *----------------------------------------------------------------------*/
void init(void){
	//Turn on port C and initialize pins
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	PORTC->PCR[0] = PORT_PCR_MUX(001);
	PORTC->PCR[7] = PORT_PCR_MUX(001);
	PTC->PDDR |= 1;
	PTC->PDDR |= (1 << 7);
	
	/* Enable Bluetooth */
	SIM_SCGC1 |= SIM_SCGC1_UART4_MASK;      /*Enable the UART clock*/
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;		/*Enable the PORTB clock*/
	PORTC_PCR14 |= PORT_PCR_MUX(3);
	PORTC_PCR15 |= PORT_PCR_MUX(3);

	UART4_C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );  /*Disable Tx and Rx*/
	UART4_C1 = 0; 		/*Dafault settings of the register*/
	UART4_BDH = 66; //sets BAUD to 960000
	UART4_BDL = 66; 
	UART4_C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK );    /* Enable receiver and transmitter */
	
	/* Allocate space for current_time */
	current_time = malloc(sizeof(time_t));
	
	/* Timer initialization */
	SIM->SCGC6 = SIM_SCGC6_PIT_MASK;
	PIT_MCR = 0x00;
	
	/* Set timer Interrupt Handlers */
	NVIC_EnableIRQ(PIT1_IRQn); 
	
	/* Initialize current_time to 0 (start) */
	current_time->sec = 0;
	current_time->msec = 0;
	
	/* Set the load value 1 msec */
	PIT->CHANNEL[1].LDVAL = 20970; 

	/* Timer things */
	NVIC_SetPriority(PIT1_IRQn, 0);
	PIT->CHANNEL[1].TCTRL = 3; 	
}

/*------------------------------------------------------------------------
 * Delay Functions
 *----------------------------------------------------------------------*/
void delay1(void){
	int j;
	for(j=0; j<100; j++);
}

void delay_msec(void) {
	int j;
	for(j=0;j<4718;j++);
}
