//Written by YIMING CHEN, 400230266, cheny466
//LSD = 6 -> Bus Speed = 12 MHz
//SLSD = 6 -> Distance Status = PF0

#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include <math.h>
#define PI 3.14159265359 //manually define the PI constant
void PortE0_Init(void){	// Port E is used to drive the button low
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;		  // activate the clock for Port E
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R4) == 0){};	  // allow time for clock to stabilize
	GPIO_PORTE_DEN_R= 0b00000001;
	GPIO_PORTE_DIR_R |= 0b00000001;                           // make PE0 output  
	GPIO_PORTE_DATA_R=0b00000000;                             // setting state to zero to drive the row, one to disable 
	return;
	}
void PortL0L1L2L3L4L5_Init(void){ //Port L0-L3 will be used to drive the motor
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;                 //activate the clock for Port L
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};        //allow time for clock to stabilize
	GPIO_PORTL_DIR_R=0b00101111;                              //make PL3-0 an output for LEDs
	GPIO_PORTL_DEN_R=0b00101111;
	return;
}
	
void PortM0_Init(void){ //port M is used as an INPUT for the button
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;                 //activate the clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};        //allow time for clock to stabilize 
	GPIO_PORTM_DIR_R |= 0b00000000;       			  // make PM0 an input, PM0 is reading the column 
        GPIO_PORTM_DEN_R |= 0b00000001;
	return;
}
void PortF_Init(void){ // onboard LED
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; //activate the clock for Port F
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};//allow time for clock to stabilize
	GPIO_PORTF_DIR_R=0b00000001; //Make PF0 output, to turn on LED
	GPIO_PORTF_DEN_R=0b00000001;
	return;
}
uint16_t	dev=0x52;
int status=0;
volatile int IntCount;

//device in interrupt mode (GPIO1 pin signal)
#define isInterrupt 1 /* If isInterrupt = 1 then device working in interrupt mode, else device working in polling mode */

void I2C_Init(void);
void UART_Init(void);
void PortG_Init(void);
void VL53L1X_XSHUT(void);

//capture values from VL53L1X for inspection
uint16_t debugArray[100];

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint8_t ToFSensor = 1; // 0=Left, 1=Center(default), 2=Right
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	
	PortE0_Init();
	PortM0_Init();
	PortF_Init();
	PortL0L1L2L3L4L5_Init();
	uint32_t  count = 0;  //used to turn the motor clockwise
	uint32_t  revcount = 0; //used to turn the motor counterclockwise
	float angle_deg =0; //holds the current angle in degrees
	float x_mm = 0; //x y z values to be sent to pc
	float y_mm = 0;
	float z_mm =0;
	
	 
	// hello world!
	UART_printf("Program Begins\r\n");
	int mynumber = 1;
	sprintf(printf_buffer,"2DX4 Program Studio Code %d\r\n",mynumber);
	UART_printf(printf_buffer);


/* Those basic I2C read functions can be used to check your own I2C functions */
  status = VL53L1_RdByte(dev, 0x010F, &byteData);					// This is the model ID.  Expected returned value is 0xEA
  myByteArray[i++] = byteData;

  status = VL53L1_RdByte(dev, 0x0110, &byteData);					// This is the module type.  Expected returned value is 0xCC
  myByteArray[i++] = byteData;
	
	status = VL53L1_RdWord(dev, 0x010F, &wordData);
	status = VL53L1X_GetSensorId(dev, &wordData);

	sprintf(printf_buffer,"Model_ID=0x%x , Module_Type=0x%x\r\n",myByteArray[0],myByteArray[1]);
	UART_printf(printf_buffer);

	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(100);
  }
	FlashAllLEDs();
	UART_printf("ToF Chip Booted!\r\n");
 	UART_printf("One moment...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/

	
  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);
status = VL53L1X_StartRanging(dev);   /* This function has to be called to enable the ranging */
	Status_Check("StartRanging", status);
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% BEGIN
	
	button:
	while(1){//keep checking if the button is pressed 
 
	// External LED ON HERE L5
	GPIO_PORTL_DATA_R = 0x20;
		
		if(((GPIO_PORTM_DATA_R&0b00000001)==0&&(GPIO_PORTE_DATA_R&0b000000001)==0))
			{
	
			GPIO_PORTL_DATA_R = 0x00; //turn off external LED
			goto motorclockwise;
	}
		
	
}
motorclockwise:
while(count<512){ //turn motor clockwise
			
		GPIO_PORTL_DATA_R = 0x0C;
	SysTick_Wait10ms(3);

		GPIO_PORTL_DATA_R = 0x06;
	SysTick_Wait10ms(3);
	
		GPIO_PORTL_DATA_R = 0x03;
	SysTick_Wait10ms(3);

		GPIO_PORTL_DATA_R = 0x09;
	SysTick_Wait10ms(3);
   count++;
   
 if ((count+1)%16==0) //if we reach 11.25 degrees, take a measurement
 {
	 angle_deg+=11.25;
		while (dataReady == 0){
		  status = VL53L1X_CheckForDataReady(dev, &dataReady);
          FlashLED3(1);
          VL53L1_WaitMs(dev, 5);
	  }

      dataReady = 0;
	  status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
	  status = VL53L1X_GetDistance(dev, &Distance);
      FlashLED4(1);

      debugArray[i] = Distance;
 status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
      //sprintf(printf_buffer,"%u %u %u, %u, %u\r\n", RangeStatus,Distance, SignalRate, AmbientRate,SpadNum);
		//sprintf(printf_buffer,"[%u, %f]\r\n", Distance,angle_deg);
		
		
		if (angle_deg>0 && angle_deg<= 90) //quad 4
		{
			float angle_rad = (angle_deg*PI)/180; //cos and sin only accept radian values of the angle
			x_mm = Distance*cos(angle_rad);
			y_mm = (-1)*Distance*sin(angle_rad);
		}
		
		else if (angle_deg>90 && angle_deg<= 180) //quad 3
		{
			float angle_rad = ((180-angle_deg)*PI)/180;
			x_mm = (-1)*Distance*cos(angle_rad);
			y_mm = (-1)*Distance*sin(angle_rad);
			
		}
			else if (angle_deg>180 && angle_deg<= 270) //quad 2
		{
			float angle_rad = ((angle_deg-180)*PI)/180;
			x_mm = (-1)*Distance*cos(angle_rad);
			y_mm = Distance*sin(angle_rad);
			
		}
		else if (angle_deg>270 && angle_deg<= 360) //quad 1 
		{
			float angle_rad = ((360-angle_deg)*PI)/180;
			x_mm = Distance*cos(angle_rad);
			y_mm = Distance*sin(angle_rad);
			
		}
		
    sprintf(printf_buffer,"%f %f %f\r\n",x_mm,y_mm,z_mm); //send x,y,z
		
		UART_printf(printf_buffer);
		
	  SysTick_Wait10ms(500);
 }
if ((count+1)%64 == 0) //blink onboard LED every 45 degrees
	{
	
  GPIO_PORTF_DATA_R = 0b000000001; //onboard LED ON
  SysTick_Wait10ms(10);	
	GPIO_PORTF_DATA_R = 0b000000000; //offboard LED OFF
	}
}
count = 0;
while(revcount<512){ //turn the motor 360 counter clockwise to prevent tangling of wires
		
	GPIO_PORTL_DATA_R = 0x09;
	SysTick_Wait10ms(3);
	GPIO_PORTL_DATA_R = 0x03;
	SysTick_Wait10ms(3);
	GPIO_PORTL_DATA_R = 0x06;
	SysTick_Wait10ms(3);
	GPIO_PORTL_DATA_R = 0x0C;
	SysTick_Wait10ms(3);
		
   revcount++;		}
angle_deg=0; //back to facing +x axis
z_mm-=200; //move forwards 20cm
revcount=0;
goto button; //wait for next scan command
}



//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable
#define MAXRETRIES              5           // number of receive attempts before giving up
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           // activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           // 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             // 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             // 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          // 7) disable analog functionality on PB2,3

                                                                                // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      // 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                                        // 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//XSHUT     This pin is an active-low shutdown input; the board pulls it up to VDD to enable the sensor by default. Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(100);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

