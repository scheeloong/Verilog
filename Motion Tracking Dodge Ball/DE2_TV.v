// ============================================================================
// Copyright (c) 2012 by Terasic Technologies Inc.
// ============================================================================
//
// Permission:
//
//   Terasic grants permission to use and modify this code for use
//   in synthesis for all Terasic Development Boards and Altera Development 
//   Kits made by Terasic.  Other use of this code, including the selling 
//   ,duplication, or modification of any portion is strictly prohibited.
//
// Disclaimer:
//
//   This VHDL/Verilog or C/C++ source code is intended as a design reference
//   which illustrates how these types of functions can be implemented.
//   It is the user's responsibility to verify their design for
//   consistency and functionality through the use of formal
//   verification methods.  Terasic provides no warranty regarding the use 
//   or functionality of this code.
//
// ============================================================================
//           
//  Terasic Technologies Inc
//  9F., No.176, Sec.2, Gongdao 5th Rd, East Dist, Hsinchu City, 30070. Taiwan
//
//
//
//                     web: http://www.terasic.com/
//                     email: support@terasic.com
//
// ============================================================================
//
// Major Functions:	DE2 TV_BOX 
//
// ============================================================================
//
// Revision History :
// ============================================================================
//   Ver  :| Author            :| Mod. Date :| Changes Made:
//   V1.0 :| Joe Yang	       :| 05/07/05   :| Initial Revision
//   V1.1 :| Johnny Chen       :| 05/09/05   :| Changed YCbCr2RGB Block,
//											   RGB output 8 Bits => 10 Bits
//   V1.2 :| Johnny Chen	    :| 05/10/05   :| H_SYNC & V_SYNC Timing fixed.
//   V1.3 :| Johnny Chen       :| 05/11/16   :| Added FLASH Address FL_ADDR[21:20]
//   V1.4 :| Joe Yang	       :| 06/07/20   :| Modify Output Color
//	  V2.0 :| Johnny Chen	    :| 06/11/20	:| New Version for DE2 v2.X PCB.
//   V3.0 :| Dee Zeng          :| 2012/01/30 :| change TD decoder chip from ADV7181 to ADV7180. support PAL and NTSC Format
//   V3.1 :| Dee Zeng          :| 2012/04/20 :| add TV i2c software reset. small changes in TD_detect.
// ============================================================================
module DE2_TV
		(
		////////////////////	Clock Input	 	////////////////////	 
		CLOCK_27,						//	On Board 27 MHz
		CLOCK_50,						//	On Board 50 MHz
		EXT_CLOCK,						//	External Clock
		////////////////////	Push Button		////////////////////
		KEY,							//	Pushbutton[3:0]
		////////////////////	DPDT Switch		////////////////////
		SW,								//	Toggle Switch[17:0]
		////////////////////	7-SEG Dispaly	////////////////////
		HEX0,							//	Seven Segment Digit 0
		HEX1,							//	Seven Segment Digit 1
		HEX2,							//	Seven Segment Digit 2
		HEX3,							//	Seven Segment Digit 3
		HEX4,							//	Seven Segment Digit 4
		HEX5,							//	Seven Segment Digit 5
		HEX6,							//	Seven Segment Digit 6
		HEX7,							//	Seven Segment Digit 7
		////////////////////////	LED		////////////////////////
		LEDG,							//	LED Green[8:0]
		LEDR,							//	LED Red[17:0]
		////////////////////////	UART	////////////////////////
		UART_TXD,						//	UART Transmitter, breaks parallel into serial
		UART_RXD,						//	UART Receiver, combines serial into parallel 
		////////////////////////	IRDA	////////////////////////
		IRDA_TXD,						//	IRDA Transmitter
		IRDA_RXD,						//	IRDA Receiver
		/////////////////////	SDRAM Interface		////////////////
		DRAM_DQ,						//	SDRAM Data bus 16 Bits
		DRAM_ADDR,						//	SDRAM Address bus 12 Bits
		DRAM_LDQM,						//	SDRAM Low-byte Data Mask 
		DRAM_UDQM,						//	SDRAM High-byte Data Mask
		DRAM_WE_N,						//	SDRAM Write Enable
		DRAM_CAS_N,						//	SDRAM Column Address Strobe
		DRAM_RAS_N,						//	SDRAM Row Address Strobe
		DRAM_CS_N,						//	SDRAM Chip Select
		DRAM_BA_0,						//	SDRAM Bank Address 0
		DRAM_BA_1,						//	SDRAM Bank Address 1
		DRAM_CLK,						//	SDRAM Clock
		DRAM_CKE,						//	SDRAM Clock Enable
		////////////////////	Flash Interface		////////////////
		FL_DQ,							//	FLASH Data bus 8 Bits
		FL_ADDR,						//	FLASH Address bus 20 Bits
		FL_WE_N,						//	FLASH Write Enable
		FL_RST_N,						//	FLASH Reset
		FL_OE_N,						//	FLASH Output Enable
		FL_CE_N,						//	FLASH Chip Enable
		////////////////////	SRAM Interface		////////////////
		SRAM_DQ,						//	SRAM Data bus 16 Bits
		SRAM_ADDR,						//	SRAM Address bus 18 Bits
		SRAM_UB_N,						//	SRAM High-byte Data Mask
		SRAM_LB_N,						//	SRAM Low-byte Data Mask  
		SRAM_WE_N,						//	SRAM Write Enable
		SRAM_CE_N,						//	SRAM Chip Enable
		SRAM_OE_N,						//	SRAM Output Enable
		////////////////////	ISP1362 Interface	////////////////
		// Single Chip "Universal Serial Bus (USB) On The Go" Controller
		// On the Go means the USB can be used as peripheral or host 
		OTG_DATA,						//	ISP1362 Data bus 16 Bits
		OTG_ADDR,						//	ISP1362 Address 2 Bits
		OTG_CS_N,						//	ISP1362 Chip Select
		OTG_RD_N,						//	ISP1362 Read
		OTG_WR_N,						//	ISP1362 Write
		OTG_RST_N,						//	ISP1362 Reset
		OTG_FSPEED,						//	USB Full Speed,	0 = Enable, Z = Disable
		OTG_LSPEED,						//	USB Low Speed, 	0 = Enable, Z = Disable
		OTG_INT0,						//	ISP1362 Interrupt 0
		OTG_INT1,						//	ISP1362 Interrupt 1
		OTG_DREQ0,						//	ISP1362 DMA Request 0
		OTG_DREQ1,						//	ISP1362 DMA Request 1
		OTG_DACK0_N,					//	ISP1362 DMA Acknowledge 0
		OTG_DACK1_N,					//	ISP1362 DMA Acknowledge 1
		////////////////////	LCD Module 16X2		////////////////
		LCD_ON,							//	LCD Power ON/OFF
		LCD_BLON,						//	LCD Back Light ON/OFF
		LCD_RW,							//	LCD Read/Write Select, 0 = Write, 1 = Read
		LCD_EN,							//	LCD Enable
		LCD_RS,							//	LCD Command/Data Select, 0 = Command, 1 = Data
		LCD_DATA,						//	LCD Data bus 8 bits
		////////////////////	SD_Card Interface	////////////////
		SD_DAT,							//	SD Card Data
		SD_WP_N,						   //	SD Write protect 
		SD_CMD,							//	SD Card Command Signal
		SD_CLK,							//	SD Card Clock
		////////////////////	USB JTAG link	////////////////////
		TDI,  							//	CPLD -> FPGA (Data in)
		TCK,  							//	CPLD -> FPGA (Clock)
		TCS,  							//	CPLD -> FPGA (CS)
	   TDO,  							//	FPGA -> CPLD (Data out)
		////////////////////	I2C		////////////////////////////
		I2C_SDAT,						//	I2C Data
		I2C_SCLK,						//	I2C Clock
		////////////////////	PS2		////////////////////////////
		PS2_DAT,						//	PS2 Data
		PS2_CLK,						//	PS2 Clock
		////////////////////	VGA		////////////////////////////
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK,						//	VGA BLANK
		VGA_SYNC,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,  						//	VGA Blue[9:0]
		////////////	Ethernet Interface	////////////////////////
		ENET_DATA,						//	DM9000A DATA bus 16Bits
		ENET_CMD,						//	DM9000A Command/Data Select, 0 = Command, 1 = Data
		ENET_CS_N,						//	DM9000A Chip Select
		ENET_WR_N,						//	DM9000A Write
		ENET_RD_N,						//	DM9000A Read
		ENET_RST_N,						//	DM9000A Reset
		ENET_INT,						//	DM9000A Interrupt
		ENET_CLK,						//	DM9000A Clock 25 MHz
		////////////////	Audio CODEC		////////////////////////
		AUD_ADCLRCK,					//	Audio CODEC ADC LR Clock
		AUD_ADCDAT,						//	Audio CODEC ADC Data
		AUD_DACLRCK,					//	Audio CODEC DAC LR Clock
		AUD_DACDAT,						//	Audio CODEC DAC Data
		AUD_BCLK,						//	Audio CODEC Bit-Stream Clock
		AUD_XCK,						//	Audio CODEC Chip Clock
		////////////////	TV Decoder		////////////////////////
		TD_DATA,    					//	TV Decoder Data bus 8 bits
		TD_HS,							//	TV Decoder H_SYNC
		TD_VS,							//	TV Decoder V_SYNC
		TD_RESET,						//	TV Decoder Reset
		TD_CLK27,                  //	TV Decoder 27MHz CLK
		////////////////////	GPIO	////////////////////////////
		GPIO_0,							//	GPIO Connection 0
		GPIO_1							//	GPIO Connection 1
	);

//----------------------------------------------------------------------------------------------	
// Definining inputs and outputs 
////////////////////////	Clock Input	 	////////////////////////
input			   CLOCK_27;				//	On Board 27 MHz
input			   CLOCK_50;				//	On Board 50 MHz
input			   EXT_CLOCK;				//	External Clock
////////////////////////	Push Button		////////////////////////
input	 [3:0]	KEY;					//	Pushbutton[3:0]
////////////////////////	DPDT Switch		////////////////////////
input	[17:0]	SW;						//	Toggle Switch[17:0]
////////////////////////	7-SEG Display	////////////////////////
output	[6:0]	HEX0;					//	Seven Segment Digit 0
output	[6:0]	HEX1;					//	Seven Segment Digit 1
output	[6:0]	HEX2;					//	Seven Segment Digit 2
output	[6:0]	HEX3;					//	Seven Segment Digit 3
output	[6:0]	HEX4;					//	Seven Segment Digit 4
output	[6:0]	HEX5;					//	Seven Segment Digit 5
output	[6:0]	HEX6;					//	Seven Segment Digit 6
output	[6:0]	HEX7;					//	Seven Segment Digit 7
////////////////////////////	LED		////////////////////////////
output	[8:0]	LEDG;					//	LED Green[8:0]
output  [17:0]	LEDR;					//	LED Red[17:0]
////////////////////////////	UART	////////////////////////////
output			UART_TXD;				//	UART Transmitter
input			   UART_RXD;				//	UART Receiver
////////////////////////////	IRDA	////////////////////////////
output			IRDA_TXD;				//	IRDA Transmitter
input			   IRDA_RXD;				//	IRDA Receiver
///////////////////////		SDRAM Interface	////////////////////////
inout	 [15:0]	DRAM_DQ;				//	SDRAM Data bus 16 Bits
output [11:0]	DRAM_ADDR;				//	SDRAM Address bus 12 Bits
output			DRAM_LDQM;				//	SDRAM Low-byte Data Mask 
output			DRAM_UDQM;				//	SDRAM High-byte Data Mask
output			DRAM_WE_N;				//	SDRAM Write Enable
output			DRAM_CAS_N;				//	SDRAM Column Address Strobe
output			DRAM_RAS_N;				//	SDRAM Row Address Strobe
output			DRAM_CS_N;				//	SDRAM Chip Select
output			DRAM_BA_0;				//	SDRAM Bank Address 0
output			DRAM_BA_1;				//	SDRAM Bank Address 0
output			DRAM_CLK;				//	SDRAM Clock
output			DRAM_CKE;				//	SDRAM Clock Enable
////////////////////////	Flash Interface	////////////////////////
inout	 [7:0]	FL_DQ;					//	FLASH Data bus 8 Bits
output [21:0]	FL_ADDR;				//	FLASH Address bus 22 Bits
output			FL_WE_N;				//	FLASH Write Enable
output			FL_RST_N;				//	FLASH Reset
output			FL_OE_N;				//	FLASH Output Enable
output			FL_CE_N;				//	FLASH Chip Enable
////////////////////////	SRAM Interface	////////////////////////
inout	 [15:0]	SRAM_DQ;				//	SRAM Data bus 16 Bits
output [17:0]	SRAM_ADDR;				//	SRAM Address bus 18 Bits
output			SRAM_UB_N;				//	SRAM Low-byte Data Mask 
output			SRAM_LB_N;				//	SRAM High-byte Data Mask 
output			SRAM_WE_N;				//	SRAM Write Enable
output			SRAM_CE_N;				//	SRAM Chip Enable
output			SRAM_OE_N;				//	SRAM Output Enable
////////////////////	ISP1362 Interface	////////////////////////
inout	 [15:0]	OTG_DATA;				//	ISP1362 Data bus 16 Bits
output [1:0]	OTG_ADDR;				//	ISP1362 Address 2 Bits
output			OTG_CS_N;				//	ISP1362 Chip Select
output			OTG_RD_N;				//	ISP1362 Write
output			OTG_WR_N;				//	ISP1362 Read
output			OTG_RST_N;				//	ISP1362 Reset
output			OTG_FSPEED;				//	USB Full Speed,	0 = Enable, Z = Disable
output			OTG_LSPEED;				//	USB Low Speed, 	0 = Enable, Z = Disable
input			   OTG_INT0;				//	ISP1362 Interrupt 0
input			   OTG_INT1;				//	ISP1362 Interrupt 1
input			   OTG_DREQ0;				//	ISP1362 DMA Request 0
input			   OTG_DREQ1;				//	ISP1362 DMA Request 1
output			OTG_DACK0_N;			//	ISP1362 DMA Acknowledge 0
output			OTG_DACK1_N;			//	ISP1362 DMA Acknowledge 1
////////////////////	LCD Module 16X2	////////////////////////////
inout	 [7:0]	LCD_DATA;				//	LCD Data bus 8 bits
output			LCD_ON;					//	LCD Power ON/OFF
output			LCD_BLON;				//	LCD Back Light ON/OFF
output			LCD_RW;					//	LCD Read/Write Select, 0 = Write, 1 = Read
output			LCD_EN;					//	LCD Enable
output			LCD_RS;					//	LCD Command/Data Select, 0 = Command, 1 = Data
////////////////////	SD Card Interface	////////////////////////
inout	 [3:0]	SD_DAT;					//	SD Card Data
input			   SD_WP_N;				   //	SD write protect
inout			   SD_CMD;					//	SD Card Command Signal
output			SD_CLK;					//	SD Card Clock
////////////////////////	I2C		////////////////////////////////
// Used to initialize ADV7181B chip 
inout			   I2C_SDAT;				//	I2C Data
output			I2C_SCLK;				//	I2C Clock
////////////////////////	PS2		////////////////////////////////
input		 	   PS2_DAT;				//	PS2 Data
input			   PS2_CLK;				//	PS2 Clock
////////////////////	USB JTAG link	////////////////////////////
input  			TDI;					// CPLD -> FPGA (data in)
input  			TCK;					// CPLD -> FPGA (clk)
input  			TCS;					// CPLD -> FPGA (CS)
output 			TDO;					// FPGA -> CPLD (data out)
////////////////////////	VGA			////////////////////////////
output			VGA_CLK;   				//	VGA Clock
output			VGA_HS;					//	VGA H_SYNC
output			VGA_VS;					//	VGA V_SYNC
output			VGA_BLANK;				//	VGA BLANK
output			VGA_SYNC;				//	VGA SYNC
output	[9:0]	VGA_R;   				//	VGA Red[9:0]
output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
////////////////	Ethernet Interface	////////////////////////////
inout	[15:0]	ENET_DATA;				//	DM9000A DATA bus 16Bits
output			ENET_CMD;				//	DM9000A Command/Data Select, 0 = Command, 1 = Data
output			ENET_CS_N;				//	DM9000A Chip Select
output			ENET_WR_N;				//	DM9000A Write
output			ENET_RD_N;				//	DM9000A Read
output			ENET_RST_N;				//	DM9000A Reset
input			   ENET_INT;				//	DM9000A Interrupt
output			ENET_CLK;				//	DM9000A Clock 25 MHz
////////////////////	Audio CODEC		////////////////////////////
inout			   AUD_ADCLRCK;			//	Audio CODEC ADC LR Clock
input			   AUD_ADCDAT;				//	Audio CODEC ADC Data
inout			   AUD_DACLRCK;			//	Audio CODEC DAC LR Clock
output			AUD_DACDAT;				//	Audio CODEC DAC Data
inout			   AUD_BCLK;				//	Audio CODEC Bit-Stream Clock
output			AUD_XCK;				//	Audio CODEC Chip Clock
////////////////////	TV Decoder		////////////////////////////
input	 [7:0]	TD_DATA;    			//	TV Decoder Data bus 8 bits
input		   	TD_HS;					//	TV Decoder H_SYNC
input		   	TD_VS;					//	TV Decoder V_SYNC
output			TD_RESET;				//	TV Decoder Reset
input          TD_CLK27;            //	TV Decoder 27MHz CLK
////////////////////////	GPIO	////////////////////////////////
inout	[35:0]	GPIO_0;					//	GPIO Connection 0
inout	[35:0]	GPIO_1;					//	GPIO Connection 1
//----------------------------------------------------------------------------------------------	
// Wires & Registers

// For SRAM Buffer of VGA 
reg [17:0] addr_reg; //memory address register for SRAM
reg [15:0] data_reg; //memory data register  for SRAM
reg we ;		//write enable for SRAM, it writes on active low 
// but write enable for VGA wants to write on active high = writeEn

assign SRAM_DQ = 16'hzzzz;

// SRAM_control
assign SRAM_ADDR = addr_reg; // depends on position of current VGA_X and VGA_Y position 
assign SRAM_DQ = (we)? 16'hzzzz : data_reg ; // Depends on write enable 
					// if we = 1 => read, let data be impedance, it should read from proper values 
					// if we = 0 => write, let it write whatever is in the data register 

assign SRAM_UB_N = 0;					// hi byte select enabled
assign SRAM_LB_N = 0;					// lo byte select enabled
assign SRAM_CE_N = 0;					// chip is enabled
assign SRAM_WE_N = we;					// write when ZERO
assign SRAM_OE_N = 0;					//output enable is overidden by WE

//------------------------------------------------------------------------

// r4r3r2r1 g4g3g2g1 b4b3b2b1 xxxx // -> 12 bit color into 16 bit input 
reg [15:0] colorSDRAM, colorSDRAM2; // this is color input into SDRAM 
wire [9:0] mRedSRAM, mGreenSRAM, mBlueSRAM; 
// temporary for understanding: 
assign  mRedSRAM = {SRAM_DQ[15:12], 6'b0}; // The colors that output to the whole vga every 60HZ is 
													  // Note: every pixel updates every few ns depending 
													  // on how fast HSYNC and YSYNC runs 
													  // but the whole screen is updated every 60 HZ 
assign  mGreenSRAM = {SRAM_DQ[11:8], 6'b0}; // each color is 4 bits
assign  mBlueSRAM = {SRAM_DQ[7:4], 6'b0};
// color data written to SRAM is 12 bit color , but data is 16 bit 
// SRAM_DQ[3:0] is used for 
// but data register is 16'b0 => black 
								// 16'hFFFF => white 

//------------------------------------------------------------------------

//	For Audio CODEC
wire		AUD_CTRL_CLK;	//	For Audio Controller

//	For ITU-R 656 Decoder
wire	[15:0]	YCbCr;
wire	[9:0] 	TV_X;
wire			   TV_DVAL;

//	For VGA Controller
wire	[9:0]	mRed;
wire	[9:0]	mGreen;
wire	[9:0]	mBlue;
wire	[10:0]	VGA_X;
wire	[10:0]	VGA_Y;
wire			VGA_Read;	//	VGA data request
wire			m1VGA_Read;	//	Read odd field
wire			m2VGA_Read;	//	Read even field

//	For YUV 4:2:2 to YUV 4:4:4
wire	[7:0]	mY;
wire	[7:0]	mCb;
wire	[7:0]	mCr;

//	For field select
wire	[15:0]	mYCbCr;
wire	[15:0]	mYCbCr_d;
wire	[15:0]	m1YCbCr;
wire	[15:0]	m2YCbCr;
wire	[15:0]	m3YCbCr;

//	For Delay Timer
wire			TD_Stable;
wire			DLY0;
wire			DLY1;
wire			DLY2;

//	For Down Sample
wire	[3:0]	Remain;
wire	[9:0]	Quotient;

wire			mDVAL;

wire	[15:0]	m4YCbCr;
wire	[15:0]	m5YCbCr;
wire	[8:0]	Tmp1,Tmp2;
wire	[7:0]	Tmp3,Tmp4;

wire            NTSC;
wire            PAL;

wire    i2c_av_config_reset_n;
wire    tv_detect_reset_n;
//----------------------------------------------------------------------------------------------	
//=============================================================================
// Structural coding
//=============================================================================
//	Flash
assign	FL_RST_N	=	1'b1; // assign Flash Memory REset to 1 bit 

//	16*2 LCD Module
assign	LCD_ON		=	1'b1;	//	LCD ON
assign	LCD_BLON	=	1'b1;	//	LCD Back Light	

//	All inout port turn to tri-state
assign	SD_DAT		=	4'bzzzz; // SD Data to tristate 
assign	AUD_ADCLRCK	=	AUD_DACLRCK; // Audio codec ADC Clock = Audio Codec DAC Clock 

//	Disable USB speed select
assign	OTG_FSPEED	=	1'bz;
assign	OTG_LSPEED	=	1'bz;

// Set all IO Ports  (there are 2 36 pins IO ports)
assign	GPIO_0	=	36'hzzzzzzzzz; // Set all 36 pins of first IO ports to tristate 
assign	GPIO_1	=	36'hzzzzzzzzz; // Set all 36 pins of second IO ports to tristate 

//	Enable TV Decoder , TD => TV Decoder 
assign	TD_RESET	= 1'b1; // low active ...use name: TD_RESET_N? , never reset the TV Decoder 

//	For Audio CODEC
assign	AUD_XCK	=	AUD_CTRL_CLK; 
// AUD_XCK = AUDIO CODEC Chip Clock, 
// AUD_CTRL_CLK = A wire to control AUD_XCK 

// Let RED LED show the state of wire VGA_X 
// Let GREEN LED show the state of wire VGA_Y
assign	LEDG	=	VGA_Y; // [10:0]
// assign	LEDR	=	VGA_X; // [10:0]
// only uses the first LEDG0 to LEDG7 to display VGA_Y position cause not enough LEDG
// LEDR is always on cause the clock goes too fast so it appears that all of them are on at the same time. 

// For video interlacing 
// m1VGA_Read is wire to read odd file , m2VGA_Read is wire to read even field 
// VGA_Y is the current Y position given out by the VGA 
// Y incremements from 0 to ... from top of screen to btm. 
// when y is an oddfield, it the first bit (in binary) will jump from 0 1 0 1 0 1 => 0 -> odd, 1 => even 

// This determines which data is read from the SRAM BUFFER INTO THE YUV444 to YUV 442 converter 

// VGA_READ is a trigger generated by the VGA when it wants to read for X and Y. 
// so basically if VGA_read is 0, you are also not reading from the sdram
assign	m1VGA_Read	=	VGA_Y[0] 	?	1'b0		:	VGA_Read	; // odd field , if odd, m1VGA_READ is VGA_READ
assign	m2VGA_Read	=	VGA_Y[0]		?	VGA_Read	:	1'b0		; // even field 

// Note: Either m1VGA_Read or m2VGA_read will be 1 at anytime, both depends on status of VGA_Y[0]
// VGA_Y[0] = first bit of VGA_Y position 
// VGA_Read is a wire for signaling VGA Data request 

// Used for Field Select 
// mYCbCr_d = a wire for field select, selects either from these 2 depending on NOT VGA_Y[0]
// Every mY and m5Y thing here are wires 

assign	mYCbCr_d	=	!VGA_Y[0]		?	m1YCbCr	:  m2YCbCr;
assign	mYCbCr		=	m5YCbCr;

// These are used for deinterlacing from SDRAM before giving the interlaced version to YUV422 to YUV444 converter 
// mYCbCr_d is given by SDRAM directly from either odd or even field 
// m3YCbCr is given from mYCbCr_d after going through line buffer 1, activates from VGA_READ
// m4YCbCr is given from m3YCbCr after going through line buffer 2, activates from VGA_READ 
assign	Tmp1	=	m4YCbCr[7:0]+mYCbCr_d[7:0]; 		// adding 2 8 bits gives a 9 bit 
assign	Tmp2	=	m4YCbCr[15:8]+mYCbCr_d[15:8]; 	// adding 2 8 bits gives a 9 bit 
assign	Tmp3	=	Tmp1[8:2]+m3YCbCr[7:1];				 // adding a 7 bit and a 7 bit gives a 8 bit 
assign	Tmp4	=	Tmp2[8:2]+m3YCbCr[15:9]; 			// adding a 7 bit and a 7 bit gives a 8 bit 

// m5YCbCr = Tmp4 (first 8 bits) followed by Tmp3 (last 8 bits) => 16 bits 
assign	m5YCbCr	=	{Tmp4,Tmp3}; // This concatenates both bits together 

//	7 segment LUT for HEX Displays, controls all 7 HEX with 18 switches
// Note: Will only control HEX0 -HEX3 and a bit of HEX4 due to not enough switches 
SEG7_LUT_8 			u0	(.oSEG0(HEX0),
							.oSEG1(HEX1),
							.oSEG2(HEX2),
							.oSEG3(HEX3),
							.oSEG4(HEX4),
							.oSEG5(HEX5),
							.oSEG6(HEX6),
							.oSEG7(HEX7),
							.iDIG(SW) );


//	TV Decoder Stable Check and if it is NTSC or PAL 
TD_Detect			u2	(.oTD_Stable(TD_Stable),
							.oNTSC(NTSC),
							.oPAL(PAL),
							.iTD_VS(TD_VS), // TD_VS = input TV Decoder Vertical Sync 
							.iTD_HS(TD_HS), // TD_HS = input TV Decoder Horizontal Sync 
							.iRST_N(KEY[0])); // Reset key is KEY[0]
													// Note: When u click reset (KEY[0]), it might not detect camera anymore 
													// need to reprogram de2 after turning on and off the board to work 
													// when reprogram after restarting board , need make sure camcorder is off. 

													
													
//	Reset Delay Timer, generates 3 different resets where reset 0 appears first, follows by reset1 followed by reset2
// all 3 resets becomes 0 if TD_Stable becomes a 0 which happens when NTSC changes to PAL or vice versa 
// or if resetN from KEY[0] is given 
Reset_Delay			u3	(	.iCLK(CLOCK_50),
							.iRST(TD_Stable), 
							// Note: At key[0] pressed, all resets at same time
							// if not, it takes turns to reset
							.oRST_0(DLY0), // Reset delay for DIV && Sdram_Control_4Port && YUV422_to_444
							.oRST_1(DLY1), // Reset delay for ITU_656_Decoder && AUDIO_DAC
							.oRST_2(DLY2)); // Reset delay for YCbCr2RGB
							

// the output of this is YCbCr which gets written into the SDRAM
// Input of this is 8 bit data coming from the TD_DATA 
// this output is 16 bit, the 16 bit represents the color in YUV 4:2:2 mode 
//	ITU-R 656 to YUV 4:2:2
ITU_656_Decoder		u4	(	//	TV Decoder Input
							.iTD_DATA(TD_DATA), // input from TV Decoder, data bus 8 bits 
													  // its connected to pins, like CLOCK_50, its just a data given by TV 
							//	Position Output
							.oTV_X(TV_X), // a [9:0] wire representing TV_X's position 
							//	YUV 4:2:2 Output
							.oYCbCr(YCbCr), // a [15:0] wire  output
													//  Data value to represent color of the YCbCr coordinate system
							.oDVAL(TV_DVAL), // a [0:0] wire  output from this decoder, 
												  // tells SDRAM if data is currently valid to be stored 
													// DVAL = Data Valid 
													
							//	Control Signals
							.iSwap_CbCr(Quotient[0]), // Quotient is a [9:0] wire  for down sample 
															// Its input is influenced by divide function at 27MHZ clock 
							.iSkip(Remain==4'h0), // Remain is a [3:0] wire for down sample 
															// Its input is influenced by divide function at 27MHZ clock 
															// It only cares if remainder is 0 
							.iRST_N(DLY1),    // the reset1 delay from reset delay gives this the reset 
													// it resets by after delay before DIV resets
							.iCLK_27(CLOCK_27));

// Megafunction: 
// i) input numerator : 10 bits unsigned 
// ii) input denominator: 4 bits unsigned
// iii) Output latency of 1 clock cycle 
// iv) Always returns positive remainder 
//	For Down Sample 720 to 640, this down samples the x position given by 
// ITU_656_Decoder from 720 pixels wide to 640 pixels wide 
// Wizard generated file for divide 
// This divides input numerator by denominator and outputs quotient and its remainder 
DIV 				u5	(	.aclr(!DLY0),	// this resets after a delay before TD_Stable's negedge
							.clock(CLOCK_27),
							.denom(4'h9),   // input denominator  is always a constant which is 9
							.numer(TV_X), // input numerator [9:0] which is outputted from ITU_656_Decoder
												// which is the TV's current X position 
							.quotient(Quotient), // These are output for quotient [9:0]
							.remain(Remain)); // This is an output [3:0]
							
//	SDRAM frame buffer
Sdram_Control_4Port	u6	(	//	HOST Side
						    .REF_CLK(CLOCK_27), // Note: Runs at 27MHz clock 
							 .CLK_18(AUD_CTRL_CLK), // 18 MHZ clock 
						    .RESET_N(DLY0),
							//	FIFO Write Side 1
						    .WR1_DATA(YCbCr), // The color value in YCbCr from ITU_656_Decoder
													// [15:0] YCbCr 
													
							// Writes whenever TV decoder tells him its valid 
							.WR1(TV_DVAL), // enable signal by ITU_656_Decoder, lets you know if its time to write
												// Data Valid
							.WR1_FULL(),
							.WR1_ADDR(0), // start writing frrom this memory address 
							.WR1_MAX_ADDR(NTSC ? 640*507 : 640*576),		//	525-18 , the size of your write depending on tv type 
												// stop writing on this memory address 
							.WR1_LENGTH(9'h80), // this is 8 bits, 
							.WR1_LOAD(!DLY0),
							.WR1_CLK(CLOCK_27),
							//	FIFO Read Side 1, First in First Out 
						   .RD1_DATA(m1YCbCr),
				        	.RD1(m1VGA_Read), // tells to read odd field, this will be 1 when VGA requests for a read
													// and this reads out into m1YCbCr which is 16 bits 
													// Depending on whether it is even or add, 
													// either one of m1YCbCr or m2YCbCr will be read out and
													// mYCbCr_d = m1YCbCr when VGAY[0] is 0 
													// mYCbCr_d = m1YCbCr when VGAY[0] is 1 
													// assign	mYCbCr_d	=	!VGA_Y[0]		?	m1YCbCr	:  m2YCbCr		;
													
													// this final myCbCr gets combine in some special way to m5YCbCr
													// and this m5YCbCr register is finally connected to wire mYCbCr
													// assign	mYCbCr		=	m5YCbCr;
													// mYCbCr is then connected to YUV422_to_444
													// to get YUV_444. look there for continuation 
													
							//	Read odd field and bypass blanking
				        	.RD1_ADDR(NTSC ? 640*13: 640*22),   	// : 640*42),     
							.RD1_MAX_ADDR(NTSC ? 640*253 : 640*262),//: 640*282),
							.RD1_LENGTH(9'h80), // WTF is this? 
				        	.RD1_LOAD(!DLY0),
							.RD1_CLK(CLOCK_27),
							//	FIFO Read Side 2
						    .RD2_DATA(m2YCbCr),
							 // reads even field signal, this should be turn on when read signal 1 is done 
				        	.RD2(m2VGA_Read), 
							//	Read even field and bypass blanking
				        	.RD2_ADDR(NTSC ? 640*267 : 640*310),	// : 640*330),		
							.RD2_MAX_ADDR(NTSC ? 640*507 : 640*550),//: 640*570),
							.RD2_LENGTH(9'h80),
				        	.RD2_LOAD(!DLY0),
							.RD2_CLK(CLOCK_27),
							//	SDRAM Side
						  .SA(DRAM_ADDR),
						  .BA({DRAM_BA_1,DRAM_BA_0}),
						  .CS_N(DRAM_CS_N),
						  .CKE(DRAM_CKE),
						  .RAS_N(DRAM_RAS_N),
				          .CAS_N(DRAM_CAS_N),
				          .WE_N(DRAM_WE_N),
						  .DQ(DRAM_DQ),
				          .DQM({DRAM_UDQM,DRAM_LDQM}),
						  .SDR_CLK(DRAM_CLK)	);

//	Converts YUV 4:2:2 to YUV 4:4:4
YUV422_to_444		u7	(	//	YUV 4:2:2 Input
							.iYCbCr(mYCbCr), // from Sdram_Control_4Port
												  // although converted from fields to mYCbCr, it is converted via 
												  // wires so everything happens instantaneously. 
							//	YUV	4:4:4 Output
							.oY(mY),  			// outputs 4:4:4 output of Y,Cb and Cr for YCbCr2RGB to
							.oCb(mCb),			// convert to RGB 
							.oCr(mCr),
							//	Control Signals
							.iX(VGA_X-160),
							.iCLK(CLOCK_27),
							.iRST_N(DLY0));

// Converts	YCbCr 8-bit to RGB-10 bit 
YCbCr2RGB 			u8	(	//	Output Side
							.Red(mRed),   // outputs final red to use 
							.Green(mGreen), // outputs final blue to use 
							.Blue(mBlue), // outputs final green to use 
							// It takes time for it to get the proper mYmCbmCr, so 
							// when data is valid, you know everything is done 
							.oDVAL(mDVAL), // Data Valid  
												// it then gives out the colors for VGA 
							//	Input Side
							.iY(mY), // input color 
							.iCb(mCb),
							.iCr(mCr),
							.iDVAL(VGA_Read), // it knows that input is valid if vga_read wants to read 
													// cause everything is connected in wires, so data comes instantaneously
													// from the SDRAM although it goes through all the processing. 
													// which immediately generates mRed, mGreen, mBlue out. 
							//	Control Signal
							.iRESET(!DLY2),   // This resets after a delay after ITU656Decoder
							.iCLK(CLOCK_27));

wire [9:0] mRed2, mGreen2, mBlue2; // This is connected to actual VGA 
reg [9:0] mRed3, mGreen3, mBlue3, mRed4, mGreen4, mBlue4, mRed5, mGreen5, mBlue5; 
assign mRed2 = mRed5; 
assign mGreen2 = mGreen5; 
assign mBlue2 = mBlue5; 
//assign LEDR = mGreen; // let LEDR show what the value of mGreen is 

// If SW[16] is 1, it shows whatever that is stored in SRAM (actual game play)
// If SW[16] is 0, it shows whatever SW[17] determines
always @(*)
begin 
	mRed5 = SW[16] ? mRed4: mRedSRAM;
	mGreen5 = SW[16] ? mGreen4: mGreenSRAM;
	mBlue5 = SW[16] ? mBlue4: mBlueSRAM;
end 

// Only matters if SW[16] is 1 
// If SW[17] is 1, it shows the original picture captured by camera
// If SW[17]is 0, it shows the green picture deduce by camera 
// for anything that counts as 1 will be white
always @ (*)
begin 
	mRed4 = SW[17]  ? mRed3: mRed; 
	mGreen4 = SW[17] ? mGreen3: mGreen; 
	mBlue4 = SW[17] ? mBlue3: mBlue; 
end 

always @ (*)
begin 
	mRed3 	=   ((mGreen >= SW[9:0])  && ( mRed < {SW[14:10],5'b11111}))? 9'b111111111 : 9'b0; 
	mBlue3   =  ((mGreen >= SW[9:0])  && ( mRed < {SW[14:10],5'b11111})) ? 9'b111111111 : 9'b0;  	
	mGreen3  =   ((mGreen >= SW[9:0])  && ( mRed < {SW[14:10],5'b11111})) ? 9'b111111111 : 9'b0; 
end 
							
VGA_Ctrl			u9	(	//	Host Side, 
// Proof: Since each time VGA wants data, it gets the right data, that means everything in every module
// happens instantaneously once VGA_Read (the trigger in this file) is set 
							// When VGA wants to read, it reads directly from SDRAM and not a temporary buffer. 
//							.iRed(mRed), 
//							.iGreen(mGreen),
//							.iBlue(mBlue),
							.iRed(mRed2), // always connect to mRed2, if wanna connect to SRAM, 
												// connect SRAM to mRedSRAM 
							.iGreen(mGreen2),
							.iBlue(mBlue2), 
							.oCurrent_X(VGA_X), // tells you which position it is currently at, 
													  // updates itself every 27MHZ since not using PLL in this file 
							.oCurrent_Y(VGA_Y),
							.oRequest(VGA_Read), // it requests for reading 
							//	VGA Side (dont have to worry about this) 
							.oVGA_R(VGA_R),
							.oVGA_G(VGA_G),
							.oVGA_B(VGA_B),
							.oVGA_HS(VGA_HS),
							.oVGA_VS(VGA_VS),
							.oVGA_SYNC(VGA_SYNC),
							.oVGA_BLANK(VGA_BLANK),
							.oVGA_CLOCK(VGA_CLK),
							//	Control Signal
							.iCLK(CLOCK_27),
							.iRST_N(DLY2));

// Line buffer allows you to do operation on the left to right bits!!!
// it is basically a buffer for a line of pixels! 
// Megafunction // In this case, the parameters are: 
// i) Distance between taps: 640 
// ii) shiftin input bus:    16bits
// iii) shiftout output bus: 16bits  						
// altshift_taps megafunction is a parameterized shift register with
// taps. The taps provide data outputs from the shift register at certain
// points in the shift register chain. The tap points must be evenly spaced.
// Note: The camera is giving it an interlaced input 	
// This is used for deinterlacing from the camera input stored in the SDRAM before
// giving it to the 
Line_Buffer u10	(	.aclr(!DLY0), // this only shifts every 640 bits 
                    .clken(VGA_Read),
					.clock(CLOCK_27),
					.shiftin(mYCbCr_d), // this is the result of the odd or even from SDRAM 
					.shiftout(m3YCbCr)); // when VGA wants to read, 
											// mYCbCr_d automatically changes to this new m3YCbCr 
											
// This is used for deinterlacing 
Line_Buffer u11	(	.aclr(!DLY0), // this only shifts every 640 bits 
                    .clken(VGA_Read),
					.clock(CLOCK_27),
					.shiftin(m3YCbCr), // this automatically changes to 
					.shiftout(m4YCbCr)); // m4YCbCr 

AUDIO_DAC 	u12	(	//	Audio Side
					.oAUD_BCK(AUD_BCLK),
					.oAUD_DATA(AUD_DACDAT),
					.oAUD_LRCK(AUD_DACLRCK),
					//	Control Signals
					.iSrc_Select(2'b01),
			        .iCLK_18_4(AUD_CTRL_CLK),
					.iRST_N(DLY1));
					
// Initialize I2C to let it know you're using the I2C bus 
//	Audio CODEC and video decoder setting
I2C_AV_Config 	u1	(	//	Host Side
						.iCLK(CLOCK_50),
						.iRST_N(KEY[0]),
						//	I2C Side
						.I2C_SCLK(I2C_SCLK),
						.I2C_SDAT(I2C_SDAT)	);
//--------------------------------------------------------------------------------------------------------------
// FSM1: To store color Data into SRAM whenever we is 1, and x and y is given 
// Note: SDRAM only writes when its writeenable bit is 0 
//--------------------------------------------------------------------------------------------------------------
//Create SRAM to store whatever is in SDRAM 
// Need to sync x and y that you give it with x and y that VGA wants from SDRAM 
// addr_reg has to change depending on whether you are writing or reading from it 
reg lock; // to detect synching 

assign LEDR[8:0]=VGA_X[9:1];
assign LEDR[16:9]=CenterY;

assign reset = ~KEY[2]; // clear the SRAM buffer first before doing anything 
assign resett = ~KEY[3];

wire [8:0] CenterY2; 
assign CenterY2 = {1'b0, CenterY}; 

wire [8:0]xSUMed, ySUMed;

assign xSUMed = xSUM/(2*numGreenAdd);
assign ySUMed = ySUM/(2*numGreenAdd);

// SRAM doesnt depend on clock, just need wait 1 clock cycle before reading written data
always @ (posedge CLOCK_27) 
begin
data_reg <= colorout;
	// only write to SRAM if synching 
	// so that VGA won't read from it 
	if (reset)		//synch reset assumes KEY0 is held down 1/60 second
	begin
		//clear the screen
		addr_reg <= {VGA_X[9:1],VGA_Y[9:1]} ;	// [17:0]
		we <= 1'b0;								//write some memory
		data_reg <= 16'b0;						//write all zeros (black)		
	end

else
	if(writeEn)
	begin
		we<=1'b0;
		addr_reg <= {9'd319-x,y2};
	end
	else
	/*if (~VGA_VS | ~VGA_HS) 
	begin 
			//we <= 1'b0; // tell u are writing something 
			we<=1'b0;
			addr_reg <= {x,y};	// [17:0] // get address from where u need it to be 
			// write to SDRAM 
			// Note: can only read this data after 1 clock cycle which is fine
			// cause you are currently in VGA sync mode -> VGA wont be reading anything yet 
	end 
	else // if synching , VGA is reading from SDRAM */
	begin 
			we<=1'b1;
			addr_reg <= {VGA_X[9:1], VGA_Y[9:1]} ;	// [17:0] , access from VGA access 
		// do nothing 
	end 
end 


// FSM to count number of greens which are one and add all the x and y positions up divided 
// by number of times you added them up to get middle pixel 
//-----------------------------------------------------------
// FSM 2 : Determine center position 
//-----------------------------------------------------------
// Note: Green pixels => anything that passes the given threshold and is outputted as 1 
// numGreenAdd -> number of green pixels addd 
// ySUM -> total sum of all positions of y sum belonging to green pixel added 
// xSUM -> total sum of all positions of x sum belonging to green pixel added
// currY -> current y position of green pixel to be added
// currX -> current x position of green pixel to be added 
// greenValid -> if current pixel is considered green 
// resetN is tied to key0 
// CenterX & CenterY is used in the 2nd FSM to draw the ball out to the screen 

// clock is tied to whenever the data is read out from the SDRAM, VGA_Read in this case  
// NOT SURE: DO I ASSUME VGA X is always equal to DATAX that i am reading in? 

reg [8:0] CenterX ; 
reg [7:0] CenterY ; 
reg testtemp = 0;
// max. x sum if all bits are green is less than 640 x 480 x 640 
// max. y sum if all bits are green is less than 480 x 480 x 640 
reg [28:0] xSUM, ySUM; 
wire [10:0] currX, currY; 
wire greenValid; 
// maximum number of greens add is 640 x 480 
reg [18:0] numGreenAdd;
assign greenValid = ((mGreen >= SW[9:0]) && (mRed < {SW[14:10], 5'b11111}))? 1 : 0; 
assign currX = VGA_X; 
assign currY = VGA_Y; 

reg [2:0] currState, nextState; 
parameter 
	sReset = 3'd0, 
	sAdd = 3'd1, 
	sIdle =3'd2, 
	sCalc =3'd3, 
	sDone =3'd4; 

// State Operations 
always @ (posedge CLOCK_27)
begin 
	case(currState)
		sReset:
		begin 
			numGreenAdd  <= 1; 
			ySUM <= 0; 
			xSUM  <= 0; 
			CenterX <= 0;  
			CenterY <= 0; 
		end 
		sAdd:
		begin 
			ySUM <= ySUM + currY; 
			xSUM <= xSUM + currX; 	
			numGreenAdd <= numGreenAdd + 1; 
		end 
		
		sIdle: 
		begin 
			// do nothing 
		end 
		
		sCalc: 
		begin 
			CenterX <= xSUMed; 
			CenterY <= ySUMed; 
		end 
		
		sDone: 
		begin 
			numGreenAdd  <= 1; 
			ySUM <= 0; 
			xSUM  <= 0; 			
		end 
	endcase 
end 

// State Transitions 
	always@(*) // change during changes in VGA_X or VGA_Y or greenValid 
	begin
		case (currState)
			sReset:
				if ((VGA_X[9:1] > 317) && (VGA_Y[9:1] > 239) && (numGreenAdd > 20))
					nextState = sCalc; 
				else if (greenValid)
					nextState <= sAdd; 
				else 
					nextState <= sIdle;
			sAdd:
				if ((VGA_X[9:1] > 317) && (VGA_Y[9:1] > 239) && (numGreenAdd > 20))
					nextState = sCalc; 
				else if (greenValid)
					nextState <= sAdd; 
				else 
					nextState <= sIdle;
			sIdle:
				if ((VGA_X[9:1] > 317) && (VGA_Y[9:1] > 239) && (numGreenAdd > 20))
					nextState = sCalc; 
				else if (greenValid)
					nextState <= sAdd; 
				else 
					nextState <= sIdle;	
			sCalc: 
					nextState <= sDone; 
			sDone: 
				if (greenValid)
					nextState <= sAdd; 
				else 
					nextState <= sIdle;
			default: nextState <= sReset; // initialize to sReset state. 
		endcase
	end
	
// Note: You can't do any operations here if not will have multiple assignments errors
// To initialize values before entering a state such as resetN, need create a state
// just for initialization. 
// Change State 
	always@(posedge CLOCK_27 or posedge reset) 
	begin
		if (reset) // reset = ~KEY[2]
		begin 
			currState <= sReset;
		end
		else			currState <= nextState;
	end
			
//-----------------------------------------------------------
// FSM 3 : Drawing the white square and the obstacle boxes 
//-----------------------------------------------------------
wire writeEn;
wire [8:0]x;
wire [7:0]y;
wire [8:0]y2;
assign y2 = {1'b0,y};

wire [8:0]wirecx, wirecy;

assign wirecx = CenterX;
assign wirecy = CenterY;

wire [15:0]colorout;

// Located in sketch.v 
sketch sk123(CLOCK_27, ~KEY[1],x, y, colorout, writeEn, wirecx, wirecy);


endmodule // End of main function 
//-----------------------------------------------------------------------
// Pulse to increase x position. Speed is 1 pixel/pulsetime
//-----------------------------------------------------------------------
module tempclock(clock, reset, pulse);
input clock, reset;
output pulse;
reg [25:0]count = {26{1'b0}};
assign pulse = (count == 27'b110010110111001101010000000);

always@(posedge clock, negedge reset)
begin
if(!reset)
	count<={26{1'b0}};
else
	count <= pulse ? 0 : count+1'b1;
end
endmodule 