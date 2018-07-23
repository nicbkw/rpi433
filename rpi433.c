// A7129 test code - operates as master or slave, 
// depending on gcc command line value -DMASTER=1 or -DMASTER=0
// for example, gcc -Wall -o rpi433 rpi433.c -lwiringPi -DMASTER=1
//
// Code derived from Amiccom A7129 reference code and wiringPi
// Amiccom A7129 and Raspberry Pi - utilizes only GPIO for 3-wire SPI
// Nic Burkinshaw - nic@thinnovation.co.uk
// This code released under signed NDA, do not copy without consent
// version 0.1 10th July 2018
//
//  Module A7129   Rpi
//  1       Vcc     17  3V3
//  2       GND     9   0V
//  3       CK0     -   -
//  4       GIO2    16  GPIO #23
//  5       GIO1    18  GPIO #24
//  6       SDIO    22  GPIO #25
//  7       SCK     32  GPIO #12
//  8       SCS     36  GPIO #16
//  9       GND     34  0V
//  10              -   -

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>     // for usleep

// usleep used for delay as it is non-blocking
// usleep suspends execution of current thread, releasing CPU

#include <wiringPi.h>   // see http://wiringpi.com

#include "A7129reg.h"

#define NO		0
#define YES		1

#define LOW		0
#define HIGH	1

#define OFF		0
#define ON		1

#define DISABLE	0
#define ENABLE	1

#define GPIO23  4		// #23 physical pin 16
#define GIO2    GPIO23

#define GPIO24  5		// #24 physical pin 18
#define GIO1    GPIO24

#define RF_WTR  digitalRead(GIO2)

#define SDIO    6		// #25 physical pin 22
#define SCK     26		// #12 physical pin 32
#define SCS     27		// #16 physical pin 36

#define LED     29		// #21 physical pin 40

#define LED_ON		digitalWrite(LED, ON)
#define LED_OFF		digitalWrite(LED, OFF)

#define SCS_HI      digitalWrite(SCS, HIGH) // chip select
#define SCS_LO      digitalWrite(SCS, LOW)

#define SCK_HI      digitalWrite(SCK, HIGH) // clock
#define SCK_LO      digitalWrite(SCK, LOW)

#define SDIO_HI     digitalWrite(SDIO, HIGH)// MOSI / SDIO
#define SDIO_LO     digitalWrite(SDIO, LOW)

/*********************************************************************
**  Global Variable Declaration
*********************************************************************/

uint8_t		timeout;

uint16_t    RxCnt;
uint16_t	rssi;

uint32_t  	Err_ByteCnt;
uint32_t	Err_BitCnt;

uint8_t		CmdBuf[11];
uint8_t		tmpbuf[64];

uint8_t 	ID_Tab[4]={0x34,0x75,0xC5,0x8C};   // ID code, unique for network

const uint8_t BitCount_Tab[16]={0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4};

const uint8_t PN9_Tab[]=
{   0xFF,0x83,0xDF,0x17,0x32,0x09,0x4E,0xD1,
    0xE7,0xCD,0x8A,0x91,0xC6,0xD5,0xC4,0xC4,
    0x40,0x21,0x18,0x4E,0x55,0x86,0xF4,0xDC,
    0x8A,0x15,0xA7,0xEC,0x92,0xDF,0x93,0x53,
    0x30,0x18,0xCA,0x34,0xBF,0xA2,0xC7,0x59,
    0x67,0x8F,0xBA,0x0D,0x6D,0xD8,0x2D,0x7D,
    0x54,0x0A,0x57,0x97,0x70,0x39,0xD2,0x7A,
    0xEA,0x24,0x33,0x85,0xED,0x9A,0x1D,0xE0
};  // This table are 64bytes PN9 pseudo random code.

const uint8_t Dummy_TXT[]=
{	0x4e,0x6f,0x77,0x20,0x69,0x73,0x20,0x74,
 	0x68,0x65,0x20,0x74,0x69,0x6d,0x65,0x20,
 	0x66,0x6f,0x72,0x20,0x61,0x6c,0x6c,0x20,
 	0x67,0x6f,0x6f,0x64,0x20,0x6d,0x65,0x6e,
 	0x20,0x74,0x6f,0x20,0x63,0x6f,0x6d,0x65,
 	0x20,0x74,0x6f,0x20,0x74,0x68,0x65,0x20,
 	0x61,0x69,0x64,0x20,0x6f,0x66,0x20,0x74,
 	0x68,0x65,0x20,0x70,0x61,0x72,0x74,0x79
};
// Now is the time for all good men to come to the aid of the party

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//                                   NOTE                                    !!
// !!         THIS CONFIG TABLE ONLY USE ON RF CRYSTAL = 12.8MHz             !!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
const uint16_t A7129Config[16]=  
{
    0x0021,   //SYSTEM CLOCK register, was 1221 (0021)
    0x0A21,   //PLL1 register,
    0xDA05,   //PLL2 register,    433.301MHz
    0x0000,   //PLL3 register,
    0x0A20,   //PLL4 register, was 0E20 (0A20)
    0x0024,   //PLL5 register,
    0x0000,   //PLL6 register,
    0x0011,   //CRYSTAL register,
    0x0000,   //PAGEA,
    0x0000,   //PAGEB,
    0x18D4,   //RX1 register,   IFBW=100KHz, ETH=1  
    0x7009,   //RX2 register,   by preamble
    0xC000,   //ADC register,   0x4000 - 0xC000 to enable auto RSSI
    0x0800,   //PIN CONTROL register,   Use Strobe CMD
    0x4C45,   //CALIBRATION register,
    0x20C0    //MODE CONTROL register,  Use FIFO mode
};

const uint16_t A7129Config_PageA[16]=  
{
    0xF706,   //TX1 register,   Fdev = 37.5kHz
    0x0000,   //WOR1 register,
    0xF800,   //WOR2 register,
    0x1107,   //RFI register,   Enable Tx Ramp up/down was 1907
    0x9B70,   //PM register,    CST=1 was 9B70 (0170)
    0x0302,   //RTH register,
    0x400F,   //AGC1 register,  
    0x0AC0,   //AGC2 register,
    0x0045,   //GIO register      GIO2=WTR, GIO1=FSYNC 
    0xD181,   //CKO register capacitance for crystal trim
    0x0004,   //VCB register,
    0x0A21,   //CHG1 register,  430MHz
    0x0022,   //CHG2 register,  435MHz
    0x003F,   //FIFO register,  FEP=63+1=64bytes
    0x151F,   //CODE register,  Preamble=4bytes, ID=4bytes was 1507 - turn on FEC
    0x0000    //WCAL register,
};

const uint16_t A7129Config_PageB[5]= 
{
    0x0337,   //TX2 register,   
    0x8400,   //IF1 register,   Enable Auto-IF, IF=200KHz
    0x0000,   //IF2 register,
    0x0000,   //ACK register,
    0x0000    //ART register,
};

/*********************************************************************
**  function Declaration
*********************************************************************/

void A7129_POR(void);
uint8_t InitRF(void);
uint8_t A7129_Config(void);
uint8_t A7129_WriteID(void);
uint8_t A7129_Cal(void);
void StrobeCMD(uint8_t);
void ByteSend(uint8_t);
uint8_t ByteRead(void);
void A7129_WriteReg(uint8_t, uint16_t);
uint16_t A7129_ReadReg(uint8_t);
void A7129_WritePageA(uint8_t, uint16_t);
uint16_t A7129_ReadPageA(uint8_t);
void A7129_WritePageB(uint8_t, uint16_t);
uint16_t A7129_ReadPageB(uint8_t);
void A7129_WriteFIFO(void);
void RxPacket(void);
void entry_deep_sleep_mode(void);
void wake_up_from_deep_sleep_mode(void);
void ndelay(void);

/*********************************************************************
* main loop
*********************************************************************/
int main(void)
{
	wiringPiSetup();
    pinMode(SCS, OUTPUT);
    pinMode(SCK, OUTPUT);
    pinMode(LED, OUTPUT);
    
    pullUpDnControl (SDIO, PUD_UP);
    pinMode(SDIO, OUTPUT);
    
    pullUpDnControl (GIO1, PUD_UP);
    pinMode(GIO1, INPUT);
    
    pullUpDnControl (GIO2, PUD_UP);
    pinMode(GIO2, INPUT);
    
	SCS_HI;
    SCK_LO;
    SDIO_HI;
    LED_OFF;
	
	if(MASTER)
		printf("MASTER\r\n");
	else
		printf("SLAVE\r\n");

	A7129_POR();	//power on only

    while(1)
    {
        if(InitRF()) //init RF
        {
            entry_deep_sleep_mode();
            usleep(2000);
            wake_up_from_deep_sleep_mode();
        }
        else
        {
			printf("Initialised RF\r\n");
            break;
        }
    }

    if(MASTER)	// ((P4 & 0x04)==0x04)   //if P4.2=1, master
    {
        while(1)
        {
			LED_ON;
			printf("master loop\r\n");
            A7129_WriteFIFO();  	//write data to TX FIFO
            
			StrobeCMD(CMD_TX);

            while(RF_WTR)
				usleep(10); 			//wait transmit completed
			
            LED_OFF;
			StrobeCMD(CMD_RX);
            
			timeout = 200;
			
            while(RF_WTR && timeout--)		// ((GIO2==1)&&(TimeoutFlag==0));
				usleep(1000);			// wait for a max of 200ms

			if(timeout)
                StrobeCMD(CMD_STBY);
            else
            {
                RxPacket();
				printf("RxCnt: %i, Err_ByteCnt: %i, Err_BitCnt: %i\r\n", RxCnt, Err_ByteCnt, Err_BitCnt);				
				printf("{%s}\r\n", tmpbuf);
                usleep(100000);
            }
			usleep(200000);		// was 200ms
        }
    }
    else    // slave
    {
        RxCnt = 0;
        Err_ByteCnt = 0;
        Err_BitCnt = 0;

        while(1)
        {
			LED_ON;
			printf("slave loop\r\n");

			StrobeCMD(CMD_RX);
            
            while(RF_WTR)        //wait receive completed
            	usleep(10);
			
			A7129_WriteReg(ADC_REG, (A7129Config[ADC_REG] & 0xfeff));		// ensure CDM=0
			A7129_WriteReg(MODE_REG, (A7129Config[MODE_REG] | 0x0001));		// set ADCM to read RSSI
			while (A7129_ReadReg(MODE_REG) & 0x0001);						// wait until reading complete
			rssi = A7129_ReadReg(RX2_REG & 0x01ff);							// read bits 8:0
			A7129_WriteReg(ADC_REG, A7129Config[ADC_REG]);					// restore CDM
			
			RxPacket();
			
			printf("RxCnt: %i, Err_ByteCnt: %i, Err_BitCnt: %i, RSSI: %i\r\n", RxCnt, Err_ByteCnt, Err_BitCnt, rssi);
			printf("{%s}\r\n", tmpbuf);
			
            A7129_WriteFIFO();  //write data to TX FIFO
            StrobeCMD(CMD_TX);

			while(RF_WTR)       //wait transmit completed
				usleep(10);
			LED_OFF;
            usleep(100000);		// was 90ms
        }
    }
}

/*********************************************************************
** Strobe Command
*********************************************************************/
void StrobeCMD(uint8_t cmd)
{
    SCS_LO;
	ByteSend(cmd);
    SCS_HI;
	usleep(10);
}

/************************************************************************
**  ByteSend
************************************************************************/
void ByteSend(uint8_t src)
{
    uint8_t i;

    for(i=0; i<8; i++)
    {
        if(src & 0x80)
            SDIO_HI;
        else
            SDIO_LO;

        SCK_HI;
        ndelay();
		
        SCK_LO;
        ndelay();		
        
		src<<=1;
    }
}

/************************************************************************
**  ByteRead
************************************************************************/
uint8_t ByteRead(void)
{
    uint8_t i,tmp;

	pinMode(SDIO, INPUT);
	ndelay();
	
    for(i=0; i<8; i++)
    {
        if(digitalRead(SDIO))
            tmp = (tmp << 1) | 0x01;
        else
            tmp = tmp << 1;

        SCK_HI;
        ndelay();
        
		SCK_LO;
        ndelay();
    }
	
	pinMode(SDIO, OUTPUT);
    return tmp;
}

/************************************************************************
**  A7129_WriteReg
************************************************************************/
void A7129_WriteReg(uint8_t address, uint16_t dataWord)
{
    uint8_t i;

    SCS_LO;
	ndelay();
	
    address |= CMD_Reg_W;
    ByteSend(address);
	
    //send data word
    for(i=0; i<16; i++)
    {
        if(dataWord & 0x8000)
            SDIO_HI;
        else
            SDIO_LO;

        SCK_HI;
        ndelay();		// _nop_();

		SCK_LO;
		ndelay();		// _nop_();
        
		dataWord = dataWord << 1;	// dataWord<<=1;
    }
    SCS_HI;
}

/************************************************************************
**  A7129_ReadReg
************************************************************************/
uint16_t A7129_ReadReg(uint8_t address)
{
    uint8_t i;
    uint16_t tmp;

    SCS_LO;
	
    address |= CMD_Reg_R;
    ByteSend(address);
    
    //read data code
	pinMode(SDIO, INPUT);
	ndelay();
	
    for(i=0; i<16; i++)
    {
        if(digitalRead(SDIO))
            tmp = (tmp << 1) | 0x01;
        else
            tmp = tmp << 1;

        SCK_HI;
        ndelay();			// _nop_();
    
		SCK_LO;
        ndelay();			// _nop_();
    }
    SCS_HI;
	
	pinMode(SDIO, OUTPUT);
    return tmp;
}

/************************************************************************
**  A7129_WritePageA
************************************************************************/
void A7129_WritePageA(uint8_t address, uint16_t dataWord)
{
    uint16_t tmp;

    tmp = address;
    tmp = ((tmp << 12) | A7129Config[CRYSTAL_REG]);
    A7129_WriteReg(CRYSTAL_REG, tmp);
    A7129_WriteReg(PAGEA_REG, dataWord);
}

/************************************************************************
**  A7129_ReadPageA
************************************************************************/
uint16_t A7129_ReadPageA(uint8_t address)
{
    uint16_t tmp;

    tmp = address;
    tmp = ((tmp << 12) | A7129Config[CRYSTAL_REG]);
    A7129_WriteReg(CRYSTAL_REG, tmp);

    return A7129_ReadReg(PAGEA_REG);
}

/************************************************************************
**  A7129_WritePageB
************************************************************************/
void A7129_WritePageB(uint8_t address, uint16_t dataWord)
{
    uint16_t tmp;

    tmp = address;
    tmp = ((tmp << 7) | A7129Config[CRYSTAL_REG]);
    A7129_WriteReg(CRYSTAL_REG, tmp);
    A7129_WriteReg(PAGEB_REG, dataWord);
}

/************************************************************************
**  A7129_ReadPageB
************************************************************************/
uint16_t A7129_ReadPageB(uint8_t address)
{
    uint16_t tmp;

    tmp = address;
    tmp = ((tmp << 7) | A7129Config[CRYSTAL_REG]);
    A7129_WriteReg(CRYSTAL_REG, tmp);

    return A7129_ReadReg(PAGEB_REG);
}

/*********************************************************************
** A7129_POR
*********************************************************************/
void A7129_POR(void)
{
	//power on only
    usleep(10000);   			//for regulator settling time (power on only)
    
    StrobeCMD(CMD_RF_RST);  	//reset A7129 chip
    while(A7129_WriteID())		//check SPI
    {
		StrobeCMD(CMD_RF_RST);  //reset A7129 chip    	
    }
    A7129_WritePageA(PM_PAGEA, A7129Config_PageA[PM_PAGEA] | 0x1000);   //STS=1
    usleep(2000);
    
    entry_deep_sleep_mode();		//deep sleep
    usleep(2000);
    wake_up_from_deep_sleep_mode();	//wake up
    
    StrobeCMD(CMD_RF_RST);  	//reset A7129 chip
    while(A7129_WriteID())		//check SPI
    {
		StrobeCMD(CMD_RF_RST);  //reset A7129 chip    	
    }
    A7129_WritePageA(PM_PAGEA, A7129Config_PageA[PM_PAGEA] | 0x1000);   //STS=1
    usleep(2000);
}

/*********************************************************************
** InitRF
*********************************************************************/
uint8_t InitRF(void)
{
    usleep(1000);            //delay 1ms for regulator stabilized
    StrobeCMD(CMD_RF_RST);  //reset A7129 chip
    usleep(1000);
    
    if(A7129_Config())      //config A7129 chip
        return 1;

    usleep(800);          //delay 800us for crystal stabilized

    if(A7129_WriteID())     //write ID code
        return 1;

    if(A7129_Cal())         //IF and VCO Calibration
        return 1;

    return 0;
}

/*********************************************************************
** A7129_Config
*********************************************************************/
uint8_t A7129_Config(void)
{
    uint8_t i;

    for(i=0; i<8; i++)
        A7129_WriteReg(i, A7129Config[i]);

    for(i=10; i<16; i++)
        A7129_WriteReg(i, A7129Config[i]);

    for(i=0; i<16; i++)
        A7129_WritePageA(i, A7129Config_PageA[i]);

    for(i=0; i<5; i++)
        A7129_WritePageB(i, A7129Config_PageB[i]);

    //for check        
    if(A7129_ReadReg(SYSTEMCLOCK_REG) != A7129Config[SYSTEMCLOCK_REG]) return 1;
    
    return 0;
}

/************************************************************************
**  WriteID
************************************************************************/
uint8_t A7129_WriteID(void)
{
    uint8_t i;
    uint8_t d1, d2, d3, d4;

    SCS_LO;
    ByteSend(CMD_ID_W);
    
	for(i=0; i<4; i++)
        ByteSend(ID_Tab[i]);
	
    SCS_HI;
	ndelay();
    
	SCS_LO;
	ndelay();
    
	ByteSend(CMD_ID_R);
    
	d1=ByteRead();
    d2=ByteRead();
    d3=ByteRead();
    d4=ByteRead();
    SCS_HI;
    
    if((d1!=ID_Tab[0]) || (d2!=ID_Tab[1]) || (d3!=ID_Tab[2]) || (d4!=ID_Tab[3]))
        return 1;
    
    return 0;
}

/*********************************************************************
** A7129_Cal
*********************************************************************/
uint8_t A7129_Cal(void)
{
    /*============VCC & IF calibration==================================*/
  
    A7129_WriteReg(MODE_REG, (A7129Config[MODE_REG] | 0x0802));    
    
    while(A7129_ReadReg(MODE_REG) & 0x0802);
    
    /*=======================check fbcf =================================*/
  
    if ((A7129_ReadReg(CALIBRATION_REG) >> 4) & 0x01) return 1;
  
    /*=======================check vccf =================================*/
    
    if ((A7129_ReadPageA(VCB_PAGEA) >> 4) & 0x01) return 1;
    
    /*===========RSSI Calibration procedure==========================*/
    
    A7129_WriteReg(ADC_REG, 0x4C00);                  //set ADC average=64
    A7129_WritePageA(WOR2_PAGEA, 0xF800);               //set RSSC_D=40us and RS_DLY=80us
    A7129_WritePageA(TX1_PAGEA, A7129Config_PageA[TX1_PAGEA] | 0xE000); //set RC_DLY=1.5ms
    A7129_WriteReg(MODE_REG, (A7129Config[MODE_REG] | 0x1000));     //RSSI Calibration
    
    while(A7129_ReadReg(MODE_REG) & 0x1000);
    
    A7129_WriteReg(ADC_REG, A7129Config[ADC_REG]);
    A7129_WritePageA(WOR2_PAGEA, A7129Config_PageA[WOR2_PAGEA]);
    A7129_WritePageA(TX1_PAGEA, A7129Config_PageA[TX1_PAGEA]);
    
    /*===========VCO calibration procedure==========================*/      
    
    A7129_WriteReg(PLL1_REG, 0x0A21);
    A7129_WriteReg(PLL2_REG, 0xDA05);
    A7129_WriteReg(MODE_REG, (A7129Config[MODE_REG] | 0x0004));         //vco calibration
    while (A7129_ReadReg(MODE_REG) & 0x0004);
    
    /*=======================check vbcf =================================*/
    
    if ((A7129_ReadReg(CALIBRATION_REG) >> 8) & 0x01) return 1;
   
	return 0;        
}

/*********************************************************************
** A7129_WriteFIFO
*********************************************************************/
void A7129_WriteFIFO(void)
{
    uint8_t i;

    StrobeCMD(CMD_TFR);     //TX FIFO address pointer reset

    SCS_LO;
    ByteSend(CMD_FIFO_W);   //TX FIFO write command
    for(i=0; i <64; i++)
//        ByteSend(PN9_Tab[i]);
        ByteSend(Dummy_TXT[i]);
    SCS_HI;
}

/*********************************************************************
** RxPacket
*********************************************************************/
void RxPacket(void)
{
    uint8_t i;
    uint8_t recv;
    uint8_t tmp;

    RxCnt++;

    StrobeCMD(CMD_RFR);     //RX FIFO address pointer reset

    SCS_LO;
    ByteSend(CMD_FIFO_R);   //RX FIFO read command
    for(i=0; i <64; i++)
    {
        tmpbuf[i] = ByteRead();
    }
    SCS_HI;

    for(i=0; i<64; i++)
    {
        recv = tmpbuf[i];
        tmp = recv ^ Dummy_TXT[i];		// was PN9_Tab[i];
        if(tmp!=0)
        {
            Err_ByteCnt++;
            Err_BitCnt += (BitCount_Tab[tmp>>4] + BitCount_Tab[tmp & 0x0F]);
        }
    }
}

/*********************************************************************
** entry_deep_sleep_mode
*********************************************************************/
void entry_deep_sleep_mode(void)
{
    StrobeCMD(CMD_RF_RST);              //RF reset
    A7129_WriteReg(PIN_REG, A7129Config[PIN_REG] | 0x0800);             //SCMDS=1
    A7129_WritePageA(PM_PAGEA, A7129Config_PageA[PM_PAGEA] | 0x0010);   //QDS=1
    StrobeCMD(CMD_SLEEP);               //entry sleep mode
    usleep(600);             //delay 600us for VDD_A shutdown, C load=0.1uF
    StrobeCMD(CMD_DEEP_SLEEP);          //entry deep sleep mode
    usleep(200);             //delay 200us for VDD_D shutdown, C load=0.1uF
}

/*********************************************************************
** wake_up_from_deep_sleep_mode
*********************************************************************/
void wake_up_from_deep_sleep_mode(void)
{
    StrobeCMD(CMD_STBY);    //wake up
    usleep(2000);            //delay 2ms for VDD_D stabilized
    //InitRF();
}

//**********************************************************************
// nano delay, specifically excluded from compiler optimization
// trial & error gave me a loop value of 30 on RaspberryPi
//**********************************************************************
void __attribute__((optimize("O0"))) ndelay(void)
{
    uint8_t	i;

	for (i = 0; i < 30; i++)
	   asm("nop");
}

// ends
