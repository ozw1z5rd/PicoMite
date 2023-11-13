/***********************************************************************************************************************
PicoMite MMBasic

SSD1963.c

<COPYRIGHT HOLDERS>  Geoff Graham, Peter Mather
Copyright (c) 2021, <COPYRIGHT HOLDERS> All rights reserved. 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met: 
1.	Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2.	Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer
    in the documentation and/or other materials provided with the distribution.
3.	The name MMBasic be used when referring to the interpreter in any documentation and promotional material and the original copyright message be displayed 
    on the console at startup (additional copyright messages may be added).
4.	All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed 
    by the <copyright holder>.
5.	Neither the name of the <copyright holder> nor the names of its contributors may be used to endorse or promote products derived from this software 
    without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY <COPYRIGHT HOLDERS> AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDERS> BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 

************************************************************************************************************************/

#include "MMBasic_Includes.h"
#include "Hardware_Includes.h"
int ScrollStart;
int Has100Pins = 0;

// parameters for the SSD1963 display panel (refer to the glass data sheet)
int SSD1963HorizPulseWidth, SSD1963HorizBackPorch, SSD1963HorizFrontPorch;
int SSD1963VertPulseWidth, SSD1963VertBackPorch, SSD1963VertFrontPorch;
int SSD1963PClock1, SSD1963PClock2, SSD1963PClock3;
int SSD1963Mode1, SSD1963Mode2;
unsigned int RDpin, RDport;


//#define dx(...) {char s[140];sprintf(s,  __VA_ARGS__); SerUSBPutS(s); SerUSBPutS("\r\n");}


////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Functions used by MMBasic to setup the display
//
////////////////////////////////////////////////////////////////////////////////////////////////////////

void MIPS16 ConfigDisplaySSD(unsigned char *p) {
    getargs(&p, 9, (unsigned char *)",");
    if((argc & 1) != 1 || argc < 3) error("Argument count");


    if(checkstring(argv[0], (unsigned char *)"SSD1963_4")) {                         // this is the 4" glass
        Option.DISPLAY_TYPE = SSD1963_4;
    } else if(checkstring(argv[0], (unsigned char *)"SSD1963_5")) {                  // this is the 5" glass
        Option.DISPLAY_TYPE = SSD1963_5;
    } else if(checkstring(argv[0], (unsigned char *)"SSD1963_5A")) {                 // this is the 5" glass alternative version
        Option.DISPLAY_TYPE = SSD1963_5A;
    } else if(checkstring(argv[0], (unsigned char *)"SSD1963_7")) {                  // there appears to be two versions of the 7" glass in circulation, this is type 1
        Option.DISPLAY_TYPE = SSD1963_7;
    } else if(checkstring(argv[0], (unsigned char *)"SSD1963_7A")) {                 // this is type 2 of the 7" glass (high luminosity version)
        Option.DISPLAY_TYPE = SSD1963_7A;
    } else if(checkstring(argv[0], (unsigned char *)"SSD1963_8")) {                  // this is the 8" glass (EastRising)
        Option.DISPLAY_TYPE = SSD1963_8;
    } else if(checkstring(argv[0], (unsigned char *)"ILI9341_8")) {                  // this is the 8" glass (EastRising)
        Option.DISPLAY_TYPE = ILI9341_8;
    } else
        return;

    if(!(argc == 3 || argc == 5 || argc == 7)) error("Argument count");

    if(checkstring(argv[2], (unsigned char *)"L") || checkstring(argv[2], (unsigned char *)"LANDSCAPE"))
        Option.DISPLAY_ORIENTATION = LANDSCAPE;
    else if(checkstring(argv[2], (unsigned char *)"P") || checkstring(argv[2], (unsigned char *)"PORTRAIT"))
        Option.DISPLAY_ORIENTATION = PORTRAIT;
    else if(checkstring(argv[2], (unsigned char *)"RL") || checkstring(argv[2], (unsigned char *)"RLANDSCAPE"))
        Option.DISPLAY_ORIENTATION = RLANDSCAPE;
    else if(checkstring(argv[2], (unsigned char *)"RP") || checkstring(argv[2], (unsigned char *)"RPORTRAIT"))
        Option.DISPLAY_ORIENTATION = RPORTRAIT;
    else
        error("Orientation");

    CheckPin(SSD1963_DAT1, OptionErrorCheck);
    CheckPin(SSD1963_DAT2, OptionErrorCheck);
    CheckPin(SSD1963_DAT3, OptionErrorCheck);
    CheckPin(SSD1963_DAT4, OptionErrorCheck);
    CheckPin(SSD1963_DAT5, OptionErrorCheck);
    CheckPin(SSD1963_DAT6, OptionErrorCheck);
    CheckPin(SSD1963_DAT7, OptionErrorCheck);
    CheckPin(SSD1963_DAT8, OptionErrorCheck);

    if(argc > 3 && *argv[4]) {
        char code;
        if(!(code=codecheck(argv[4])))argv[4]+=2;
        int pin = getinteger(argv[4]);
        if(!code)pin=codemap(pin);
        if(IsInvalidPin(pin)) error("Invalid pin");
        if(ExtCurrentConfig[pin] != EXT_NOT_CONFIG)  error("Pin %/| is in use",pin,pin);
        if((PinDef[pin].slice & 0x7f) == Option.AUDIO_SLICE) error("Channel in use for Audio");
        Option.DISPLAY_BL = pin;
    } else Option.DISPLAY_BL = 0;
    if(argc==7){
        char code;
        if(!(code=codecheck(argv[6])))argv[6]+=2;
        int pin = getinteger(argv[6]);
        if(!code)pin=codemap(pin);
        if(IsInvalidPin(pin)) error("Invalid pin");
        if(ExtCurrentConfig[pin] != EXT_NOT_CONFIG)  error("Pin %/| is in use",pin,pin);
        Option.SSD_DC = PinDef[pin].GPno;
        Option.SSD_WR=  Option.SSD_DC+1;
        Option.SSD_RD=  Option.SSD_DC+2;
        Option.SSD_RESET =  Option.SSD_DC+3;
    } else {
        Option.SSD_DC = 13;
        Option.SSD_WR = 14;
        Option.SSD_RD = 15;
        Option.SSD_RESET = 16;
    }
    CheckPin(SSD1963_DC_PIN, OptionErrorCheck);
    CheckPin(SSD1963_RESET_PIN, OptionErrorCheck);
    CheckPin(SSD1963_WR_PIN, OptionErrorCheck);
    CheckPin(SSD1963_RD_PIN, OptionErrorCheck);


    // disable the SPI LCD and touch
    Option.TOUCH_CS = Option.TOUCH_IRQ = Option.LCD_CD = Option.LCD_Reset = Option.LCD_CS = Option.TOUCH_Click = 0;
    Option.TOUCH_XZERO = Option.TOUCH_YZERO = 0;                    // record the touch feature as not calibrated
}



// initialise the display controller
// this is used in the initial boot sequence of the Micromite
void MIPS16 InitDisplaySSD(void) {
    if(Option.DISPLAY_TYPE<SSDPANEL || Option.DISPLAY_ORIENTATION>=VIRTUAL)return;

    // the parameters for the display panel are set here (refer to the data sheet for the glass)
    switch(Option.DISPLAY_TYPE) {
        case SSD1963_4: DisplayHRes = 480;                          // this is the 4.3" glass
                        DisplayVRes = 272;
                        SSD1963HorizPulseWidth = 41;
                        SSD1963HorizBackPorch = 2;
                        SSD1963HorizFrontPorch = 2;
                        SSD1963VertPulseWidth = 10;
                        SSD1963VertBackPorch = 2;
                        SSD1963VertFrontPorch = 2;
                        //Set LSHIFT freq, i.e. the DCLK with PLL freq 120MHz set previously
                        //Typical DCLK is 9MHz.  9MHz = 120MHz*(LCDC_FPR+1)/2^20.  LCDC_FPR = 78642 (0x13332)
                        SSD1963PClock1 = 0x01;
                        SSD1963PClock2 = 0x33;
                        SSD1963PClock3 = 0x32;
                        SSD1963Mode1 = 0x20;                        // 24-bit for 4.3" panel, data latch in rising edge for LSHIFT
                        SSD1963Mode2 = 0;                           // Hsync+Vsync mode
                        break;
        case SSD1963_5: DisplayHRes = 800;                          // this is the 5" glass
                        DisplayVRes = 480;
                        SSD1963HorizPulseWidth = 128;
                        SSD1963HorizBackPorch = 88;
                        SSD1963HorizFrontPorch = 40;
                        SSD1963VertPulseWidth = 2;
                        SSD1963VertBackPorch = 25;
                        SSD1963VertFrontPorch = 18;
                        //Set LSHIFT freq, i.e. the DCLK with PLL freq 120MHz set previously
                        //Typical DCLK is 33MHz.  30MHz = 120MHz*(LCDC_FPR+1)/2^20.  LCDC_FPR = 262143 (0x3FFFF)
                        SSD1963PClock1 = 0x03;
                        SSD1963PClock2 = 0xff;
                        SSD1963PClock3 = 0xff;
                        SSD1963Mode1 = 0x24;                        // 24-bit for 5" panel, data latch in falling edge for LSHIFT
                        SSD1963Mode2 = 0;                           // Hsync+Vsync mode
                        break;
        case SSD1963_5A: DisplayHRes = 800;                         // this is a 5" glass alternative version
                        DisplayVRes = 480;
                        SSD1963HorizPulseWidth = 128;
                        SSD1963HorizBackPorch = 88;
                        SSD1963HorizFrontPorch = 40;
                        SSD1963VertPulseWidth = 2;
                        SSD1963VertBackPorch = 25;
                        SSD1963VertFrontPorch = 18;
                        //Set LSHIFT freq, i.e. the DCLK with PLL freq 120MHz set previously
                        //Typical DCLK is 33MHz.  30MHz = 120MHz*(LCDC_FPR+1)/2^20.  LCDC_FPR = 262143 (0x3FFFF)
                        SSD1963PClock1 = 0x04;
                        SSD1963PClock2 = 0x93;
                        SSD1963PClock3 = 0xe0;
                        SSD1963Mode1 = 0x24;                        // 24-bit for 5" panel, data latch in falling edge for LSHIFT
                        SSD1963Mode2 = 0;                           // Hsync+Vsync mode
                        break;
        case SSD1963_7: DisplayHRes = 800;                          // this is the 7" glass
                        DisplayVRes = 480;
                        SSD1963HorizPulseWidth = 1;
                        SSD1963HorizBackPorch = 210;
                        SSD1963HorizFrontPorch = 45;
                        SSD1963VertPulseWidth = 1;
                        SSD1963VertBackPorch = 34;
                        SSD1963VertFrontPorch = 10;
                        //Set LSHIFT freq, i.e. the DCLK with PLL freq 120MHz set previously
                        //Typical DCLK is 33.3MHz(datasheet), experiment shows 30MHz gives a stable result
                        //30MHz = 120MHz*(LCDC_FPR+1)/2^20.  LCDC_FPR = 262143 (0x3FFFF)
                        //Time per line = (DISP_HOR_RESOLUTION+DISP_HOR_PULSE_WIDTH+DISP_HOR_BACK_PORCH+DISP_HOR_FRONT_PORCH)/30 us = 1056/30 = 35.2us
                        SSD1963PClock1 = 0x03;
                        SSD1963PClock2 = 0xff;
                        SSD1963PClock3 = 0xff;
                        SSD1963Mode1 = 0x10;                        // 18-bit for 7" panel
                        SSD1963Mode2 = 0x80;                        // TTL mode
                        break;
        case SSD1963_7A: DisplayHRes = 800;                         // this is a 7" glass alternative version (high brightness)
                        DisplayVRes = 480;
                        SSD1963HorizPulseWidth = 3;
                        SSD1963HorizBackPorch = 88;
                        SSD1963HorizFrontPorch = 37;
                        SSD1963VertPulseWidth = 3;
                        SSD1963VertBackPorch = 32;
                        SSD1963VertFrontPorch = 10;
                        SSD1963PClock1 = 0x03;
                        SSD1963PClock2 = 0xff;
                        SSD1963PClock3 = 0xff;
                        SSD1963Mode1 = 0x10;                        // 18-bit for 7" panel
                        SSD1963Mode2 = 0x80;                        // TTL mode
                        break;
        case SSD1963_8: DisplayHRes = 800;                          // this is the 8" glass (not documented because the 40 pin connector is non standard)
                        DisplayVRes = 480;
                        SSD1963HorizPulseWidth = 1;
                        SSD1963HorizBackPorch = 210;
                        SSD1963HorizFrontPorch = 45;
                        SSD1963VertPulseWidth = 1;
                        SSD1963VertBackPorch = 34;
                        SSD1963VertFrontPorch = 10;
                        //Set LSHIFT freq, i.e. the DCLK with PLL freq 120MHz set previously
                        //Typical DCLK is 33.3MHz(datasheet), experiment shows 30MHz gives a stable result
                        //30MHz = 120MHz*(LCDC_FPR+1)/2^20.  LCDC_FPR = 262143 (0x3FFFF)
                        //Time per line = (DISP_HOR_RESOLUTION+DISP_HOR_PULSE_WIDTH+DISP_HOR_BACK_PORCH+DISP_HOR_FRONT_PORCH)/30 us = 1056/30 = 35.2us
                        SSD1963PClock1 = 0x03;
                        SSD1963PClock2 = 0xff;
                        SSD1963PClock3 = 0xff;
                        SSD1963Mode1 = 0x20;
                        SSD1963Mode2 = 0x00;
                        break;
        case ILI9341_8: DisplayHRes = 320;                          // this is the 8" glass (not documented because the 40 pin connector is non standard)
                        DisplayVRes = 240;
                        break;
    }

    if(DISPLAY_LANDSCAPE) {
        VRes=DisplayVRes;
        HRes=DisplayHRes;
    } else {
        VRes=DisplayHRes;
        HRes=DisplayVRes;
    }

    // setup the pointers to the drawing primitives
    DrawRectangle= DrawRectangleSSD1963;
    DrawBitmap = DrawBitmapSSD1963;
    if(Option .DISPLAY_TYPE != ILI9341_8)ScrollLCD = ScrollSSD1963;
    else ScrollLCD=ScrollLCDSPI;
    DrawBuffer = DrawBufferSSD1963;
    ReadBuffer = ReadBufferSSD1963;
    DrawPixel = DrawPixelNormal;
    if(!(Option.DISPLAY_TYPE==ILI9341_8))InitSSD1963();
    else InitILI9341();
    SetFont(Option.DefaultFont);
    PromptFont = gui_font;
    PromptFC = gui_fcolour = Option.DefaultFC;
    PromptBC = gui_bcolour = Option.DefaultBC;
    ResetDisplay();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// The SSD1963 driver
//
////////////////////////////////////////////////////////////////////////////////////////////////////////


// Write a command byte to the SSD1963
void WriteComand(int cmd) {
    gpio_put_masked(0b11111111,cmd);
    gpio_put(SSD1963_DC_GPPIN,0);
    gpio_put(SSD1963_WR_GPPIN,0);nop;gpio_put(SSD1963_WR_GPPIN,1);
    gpio_put(SSD1963_DC_GPPIN,1);
}


// Write an 8 bit data word to the SSD1963
void WriteData(int data) {
    gpio_put_masked(0b11111111,data);
    gpio_put(SSD1963_WR_GPPIN,0);nop;gpio_put(SSD1963_WR_GPPIN,1);
}


// For the 100 pin chip write RGB colour over an 8 bit bus
void WriteColor(unsigned int c) {
    gpio_put_masked(0b11111111,(c >> 16));
    nop;gpio_put(SSD1963_WR_GPPIN,0);nop;nop;gpio_put(SSD1963_WR_GPPIN,1);
    gpio_put_masked(0b11111111,(c >> 8));
    nop;gpio_put(SSD1963_WR_GPPIN,0);nop;nop;gpio_put(SSD1963_WR_GPPIN,1);
    nop;gpio_put_masked(0b11111111,c);
    gpio_put(SSD1963_WR_GPPIN,0);nop;nop;gpio_put(SSD1963_WR_GPPIN,1);
}

// The next two functions are used in the initial setup where the SSD1963 cannot respond to fast signals

// Slowly write a command byte to the SSD1963
static void WriteComandSlow(int cmd) {
    gpio_put_masked(0b11111111,cmd);
    gpio_put(SSD1963_DC_GPPIN,0);
    gpio_put(SSD1963_WR_GPPIN,0);uSec(5);gpio_put(SSD1963_WR_GPPIN,1);
    gpio_put(SSD1963_DC_GPPIN,1);
}


// Slowly write an 8 bit data word to the SSD1963
void WriteDataSlow(int data) {
    gpio_put_masked(0b11111111,data );
    gpio_put(SSD1963_WR_GPPIN,0);uSec(5);gpio_put(SSD1963_WR_GPPIN,1);
}


// Read a byte from the SSD1963
unsigned char ReadData(void) {
    gpio_put(SSD1963_RD_GPPIN,0);nop;nop;nop;nop;nop;nop;gpio_put(SSD1963_RD_GPPIN,1);
    nop;nop;nop;nop;nop;
    return (gpio_get_all() & 0xFF);
}


// Slowly read a byte from the SSD1963
unsigned char ReadDataSlow(void) {
    gpio_put(SSD1963_RD_GPPIN,0);
    uSec(5);
    gpio_put(SSD1963_RD_GPPIN,1);
    return (gpio_get_all() & 0xFF);
}



/*********************************************************************
* defines start/end coordinates for memory access from host to SSD1963
* also maps the start and end points to suit the orientation
********************************************************************/
void SetAreaSSD1963(int x1, int y1, int x2, int y2) {
    int start_x, start_y, end_x, end_y;

    switch(Option.DISPLAY_ORIENTATION) {
        case LANDSCAPE:
        case RLANDSCAPE: start_x = x1;
                         end_x = x2;
                         start_y = y1;
                         end_y = y2;
                         break;
        case PORTRAIT:
        case RPORTRAIT:  start_x = y1;
                         end_x = y2;
                         start_y = (DisplayVRes - 1) - x2;
                         end_y = (DisplayVRes - 1) - x1;
                         break;
        default: return;
    }

  WriteComand(CMD_SET_COLUMN);
  WriteData(start_x>>8);
  WriteData(start_x);
  WriteData(end_x>>8);
  WriteData(end_x);
  WriteComand(CMD_SET_PAGE);
  WriteData(start_y>>8);
  WriteData(start_y);
  WriteData(end_y>>8);
  WriteData(end_y);
}


/*********************************************************************
* Set a GPIO pin to state high(1) or low(0)
*
* PreCondition: Set the GPIO pin an output prior using this function
*
* Arguments: BYTE pin   -     LCD_RESET
*                         LCD_SPENA
*                         LCD_SPCLK
*                         LCD_SPDAT
*
*          BOOL state -   0 for low
*                         1 for high
*********************************************************************/
static void GPIO_WR(int pin, int state) {
  int _gpioStatus = 0;

  if(state==1)
      _gpioStatus = _gpioStatus|pin;
  else
      _gpioStatus = _gpioStatus&(~pin);

  WriteComand(CMD_SET_GPIO_VAL);                                 // Set GPIO value
  WriteData(_gpioStatus);
}


/*********************************************************************
* SetBacklight(BYTE intensity)
* Some boards may use of PWM feature of ssd1963 to adjust the backlight
* intensity and this function supports that.  However, most boards have
* a separate PWM input pin and that is also supported by using the variable
*  display_backlight intimer.c
*
* Input:    intensity = 0 (off) to 100 (full on)
*
* Note: The base frequency of PWM set to around 300Hz with PLL set to 120MHz.
*     This parameter is hardware dependent
********************************************************************/
void SetBacklightSSD1963(int intensity) {
  WriteComand(CMD_SET_PWM_CONF);                                   // Set PWM configuration for backlight control

  WriteData(0x0E);                                                  // PWMF[7:0] = 2, PWM base freq = PLL/(256*(1+5))/256 = 300Hz for a PLL freq = 120MHz
  WriteData((intensity * 255)/100);                                 // Set duty cycle, from 0x00 (total pull-down) to 0xFF (99% pull-up , 255/256)
  WriteData(0x01);                                                  // PWM enabled and controlled by host (mcu)
  WriteData(0x00);
  WriteData(0x00);
  WriteData(0x00);

    display_backlight = intensity/5;                                // this is used in timer.c
}

/*********************************************************************
* SetTearingCfg(BOOL state, BOOL mode)
* This function enable/disable tearing effect
*
* Input:    BOOL state -  1 to enable
*                         0 to disable
*         BOOL mode -     0:  the tearing effect output line consists
*                             of V-blanking information only
*                         1:  the tearing effect output line consists
*                             of both V-blanking and H-blanking info.
********************************************************************/
void SetTearingCfg(int state, int mode)
{
  if(state == 1) {
      WriteComand(CMD_SET_TEAR_ON);
      WriteData(mode&0x01);
  } else {
      WriteComand(0x34);
  }

}


/***********************************************************************************************************************************
* Function:  void InitSSD1963()
* Initialise SSD1963 for PCLK,    HSYNC, VSYNC etc
***********************************************************************************************************************************/
void InitILI9341(void){
    PinSetBit(SSD1963_RESET_PIN, LATCLR);                             // reset the SSD1963
    uSec(10000);
    PinSetBit(SSD1963_RESET_PIN, LATSET);                           // release from reset state to sleep state
    uSec(10000);
  WriteComand(0xEF);
  WriteData(0x03);
  WriteData(0x80);
  WriteData(0x02);

  WriteComand(0xCF);
  WriteData(0x00);
  WriteData(0XC1);
  WriteData(0X30);

  WriteComand(0xED);
  WriteData(0x64);
  WriteData(0x03);
  WriteData(0X12);
  WriteData(0X81);

  WriteComand(0xE8);
  WriteData(0x85);
  WriteData(0x00);
  WriteData(0x78);

  WriteComand(0xCB);
  WriteData(0x39);
  WriteData(0x2C);
  WriteData(0x00);
  WriteData(0x34);
  WriteData(0x02);

  WriteComand(0xF7);
  WriteData(0x20);

  WriteComand(0xEA);
  WriteData(0x00);
  WriteData(0x00);

  WriteComand(ILI9341_PWCTR1);    //Power control
  WriteData(0x23);   //VRH[5:0]

  WriteComand(ILI9341_PWCTR2);    //Power control
  WriteData(0x10);   //SAP[2:0];BT[3:0]

  WriteComand(ILI9341_VMCTR1);    //VCM control
  WriteData(0x3e);
  WriteData(0x28);

  WriteComand(ILI9341_VMCTR2);    //VCM control2
  WriteData(0x86);  //--

  WriteComand(ILI9341_PIXFMT);
  WriteData(0x66);

  WriteComand(ILI9341_FRMCTR1);
  WriteData(0x00);
  WriteData(0x13); // 0x18 79Hz, 0x1B default 70Hz, 0x13 100Hz

  WriteComand(ILI9341_DFUNCTR);    // Display Function Control
  WriteData(0x08);
  WriteData(0x82);
  WriteData(0x27);

  WriteComand(0xF2);    // 3Gamma Function Disable
  WriteData(0x00);

  WriteComand(ILI9341_GAMMASET);    //Gamma curve selected
  WriteData(0x01);

  WriteComand(ILI9341_GMCTRP1);    //Set Gamma
  WriteData(0x0F);
  WriteData(0x31);
  WriteData(0x2B);
  WriteData(0x0C);
  WriteData(0x0E);
  WriteData(0x08);
  WriteData(0x4E);
  WriteData(0xF1);
  WriteData(0x37);
  WriteData(0x07);
  WriteData(0x10);
  WriteData(0x03);
  WriteData(0x0E);
  WriteData(0x09);
  WriteData(0x00);

  WriteComand(ILI9341_GMCTRN1);    //Set Gamma
  WriteData(0x00);
  WriteData(0x0E);
  WriteData(0x14);
  WriteData(0x03);
  WriteData(0x11);
  WriteData(0x07);
  WriteData(0x31);
  WriteData(0xC1);
  WriteData(0x48);
  WriteData(0x08);
  WriteData(0x0F);
  WriteData(0x0C);
  WriteData(0x31);
  WriteData(0x36);
  WriteData(0x0F);

  WriteComand(ILI9341_SLPOUT);    //Exit Sleep
  WriteComand(ILI9341_MADCTL);    // Memory Access Control
         switch(Option.DISPLAY_ORIENTATION) {
             case LANDSCAPE:     WriteData(ILI9341_Landscape); break;
             case PORTRAIT:      WriteData(ILI9341_Portrait); break;
             case RLANDSCAPE:    WriteData(ILI9341_Landscape180); break;
             case RPORTRAIT:     WriteData(ILI9341_Portrait180); break;
         }

    WriteComand(ILI9341_DISPON);    //Display on
    uSec(100000);
    ClearScreen(Option.DefaultBC);
}

void MIPS16 InitSSD1963(void) {

    PinSetBit(SSD1963_RESET_PIN, LATCLR);                             // reset the SSD1963
    uSec(10000);
    PinSetBit(SSD1963_RESET_PIN, LATSET);                           // release from reset state to sleep state

    // IMPORTANT: At this stage the SSD1963 is running at a slow speed and cannot respond to high speed commands
    //            So we use slow speed versions of WriteComand/WriteData with a 3 uS delay between each control signal change

  // Set MN(multipliers) of PLL, VCO = crystal freq * (N+1)
  // PLL freq = VCO/M with 250MHz < VCO < 800MHz
  // The max PLL freq is around 120MHz. To obtain 120MHz as the PLL freq

  WriteComandSlow(CMD_SET_PLL_MN);                                 // Set PLL with OSC = 10MHz (hardware), command is 0xE3
  WriteDataSlow(0x23);                                              // Multiplier N = 35, VCO (>250MHz)= OSC*(N+1), VCO = 360MHz
  WriteDataSlow(0x02);                                              // Divider M = 2, PLL = 360/(M+1) = 120MHz
  WriteDataSlow(0x54);                                              // Validate M and N values

  WriteComandSlow(CMD_PLL_START);                                  // Start PLL command
  WriteDataSlow(0x01);                                              // enable PLL

  uSec(1000);                                                       // wait for it to stabilise
  WriteComandSlow(CMD_PLL_START);                                  // Start PLL command again
  WriteDataSlow(0x03);                                              // now, use PLL output as system clock

  WriteComandSlow(CMD_SOFT_RESET);                                 // Soft reset
  uSec(10000);

#define parallel_write_data WriteData
#define TFT_Write_Data WriteData

  // Configure for the TFT panel, varies from individual manufacturer
  WriteComandSlow(CMD_SET_PCLK);                                   // set pixel clock (LSHIFT signal) frequency
  WriteDataSlow(SSD1963PClock1);                                    // paramaters set by DISPLAY INIT
  WriteDataSlow(SSD1963PClock2);
  WriteDataSlow(SSD1963PClock3);
    //    uSec(10000);


  // Set panel mode, varies from individual manufacturer
  WriteComand(CMD_SET_PANEL_MODE);
  WriteData(SSD1963Mode1);                                          // parameters set by DISPLAY INIT
  WriteData(SSD1963Mode2);
  WriteData((DisplayHRes - 1) >> 8);                                // Set panel size
  WriteData(DisplayHRes - 1);
  WriteData((DisplayVRes - 1) >> 8);
  WriteData(DisplayVRes - 1);
  WriteData(0x00);                                                  // RGB sequence


  // Set horizontal period
  WriteComand(CMD_SET_HOR_PERIOD);
  #define HT (DisplayHRes + SSD1963HorizPulseWidth + SSD1963HorizBackPorch + SSD1963HorizFrontPorch)
  WriteData((HT - 1) >> 8);
  WriteData(HT - 1);
  #define HPS (SSD1963HorizPulseWidth + SSD1963HorizBackPorch)
  WriteData((HPS - 1) >> 8);
  WriteData(HPS - 1);
  WriteData(SSD1963HorizPulseWidth - 1);
  WriteData(0x00);
  WriteData(0x00);
  WriteData(0x00);

  // Set vertical period
  WriteComand(CMD_SET_VER_PERIOD);
  #define VT (SSD1963VertPulseWidth + SSD1963VertBackPorch + SSD1963VertFrontPorch + DisplayVRes)
  WriteData((VT - 1) >> 8);
  WriteData(VT - 1);
  #define VSP (SSD1963VertPulseWidth + SSD1963VertBackPorch)
  WriteData((VSP - 1) >> 8);
  WriteData(VSP - 1);
  WriteData(SSD1963VertPulseWidth - 1);
  WriteData(0x00);
  WriteData(0x00);

  // Set pixel data interface
  WriteComand(CMD_SET_DATA_INTERFACE);
    WriteData(0x00);                                                // 8-bit colour format

    // initialise the GPIOs
  WriteComand(CMD_SET_GPIO_CONF);                                  // Set all GPIOs to output, controlled by host
  WriteData(0x0f);                                                  // Set GPIO0 as output
  WriteData(0x01);                                                  // GPIO[3:0] used as normal GPIOs

 // LL Reset to LCD!!!
  GPIO_WR(LCD_SPENA, 1);
  GPIO_WR(LCD_SPCLK, 1);
  GPIO_WR(LCD_SPDAT,1);
  GPIO_WR(LCD_RESET,1);
  GPIO_WR(LCD_RESET,0);
  uSec(1000);
  GPIO_WR(LCD_RESET,1);



    // setup the pixel write order
    WriteComand(CMD_SET_ADDR_MODE);
    switch(Option.DISPLAY_ORIENTATION) {
        case LANDSCAPE:     WriteData(SSD1963_LANDSCAPE); break;
        case PORTRAIT:      WriteData(SSD1963_PORTRAIT); break;
        case RLANDSCAPE:    WriteData(SSD1963_RLANDSCAPE); break;
        case RPORTRAIT:     WriteData(SSD1963_RPORTRAIT); break;
    }

    // Set the scrolling area
  WriteComand(CMD_SET_SCROLL_AREA);
  WriteData(0);
  WriteData(0);
  WriteData(DisplayVRes >> 8);
  WriteData(DisplayVRes);
  WriteData(0);
  WriteData(0);
    ScrollStart = 0;

  ClearScreen(Option.DefaultBC);
    SetBacklightSSD1963(Option.DefaultBrightness);
  WriteComand(CMD_ON_DISPLAY);                                     // Turn on display; show the image on display

}

void  SetAreaILI9341(int xstart, int ystart, int xend, int yend, int rw) {
    if(HRes == 0) error("Display not configured");
    WriteComand(ILI9341_COLADDRSET);
    WriteData(xstart >> 8);
    WriteData(xstart);
    WriteData(xend >> 8);
    WriteData(xend);
    WriteComand(ILI9341_PAGEADDRSET);
    WriteData(ystart >> 8);
    WriteData(ystart);
    WriteData(yend >> 8);
    WriteData(yend);
    if(rw){
    	WriteComand(ILI9341_MEMORYWRITE);
    } else {
    	WriteComand(ILI9341_RAMRD);
    }
}
/**********************************************************************************************
Draw a filled rectangle on the video output with physical frame buffer coordinates
     x1, y1 - the start physical frame buffer coordinate
     x2, y2 - the end physical frame buffer coordinate
     c - the colour to use for both the fill
 This is only called by DrawRectangleSSD1963() below
***********************************************************************************************/
void PhysicalDrawRect(int x1, int y1, int x2, int y2, int c) {
    int i;
    if(Option.DISPLAY_TYPE == ILI9341_8){
        SetAreaILI9341(x1, y1 , x2, y2, 1);
    } else {
        SetAreaSSD1963(x1, y1 , x2, y2);                                // setup the area to be filled
        WriteComand(CMD_WR_MEMSTART);
    }
    for(i = (x2 - x1 + 1) * (y2 - y1 + 1); i > 0; i--)
    WriteColor(c);
}



////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Drawing primitives used by the functions in GUI.c and Draw.c
//
////////////////////////////////////////////////////////////////////////////////////////////////////////


/**********************************************************************************************
Draw a filled rectangle on the video output with logical (MMBasic) coordinates
     x1, y1 - the start coordinate
     x2, y2 - the end coordinate
     c - the colour to use for both the fill
***********************************************************************************************/
void DrawRectangleSSD1963(int x1, int y1, int x2, int y2, int c) {
    int t;

    // make sure the coordinates are kept within the display area
    if(x2 <= x1) { t = x1; x1 = x2; x2 = t; }
    if(y2 <= y1) { t = y1; y1 = y2; y2 = t; }
    if(x1 < 0) x1 = 0; 
    if(x1 >= HRes) x1 = HRes - 1;
    if(x2 < 0) x2 = 0; 
    if(x2 >= HRes) x2 = HRes - 1;
    if(y1 < 0) y1 = 0; 
    if(y1 >= VRes) y1 = VRes - 1;
    if(y2 < 0) y2 = 0; 
    if(y2 >= VRes) y2 = VRes - 1;

    t = y2 - y1;                                                    // get the distance between the top and bottom

    // set y1 to the physical location in the frame buffer (only really has an effect when scrolling is in action)
    if(Option.DISPLAY_ORIENTATION == RLANDSCAPE)
        y1 = (y1 + (VRes - ScrollStart)) % VRes;
    else
        y1 = (y1 + ScrollStart) % VRes;
    y2 = y1 + t;                                                    // and set y2 to the same
    if(y2 >= VRes) {                                                // if the box splits over the frame buffer boundary
        PhysicalDrawRect(x1, y1, x2, VRes - 1, c);                  // draw the top part
        PhysicalDrawRect(x1, 0, x2, y2 - VRes , c);                 // and the bottom part
    } else
        PhysicalDrawRect(x1, y1, x2, y2, c);                        // the whole box is within the frame buffer - much easier
}


// written by Peter Mather (matherp on the Back Shed forum)
void DrawBufferSSD1963(int x1, int y1, int x2, int y2, unsigned char* p) {
    int i,t;
    union colourmap
    {
    char rgbbytes[4];
    unsigned int rgb;
    } c;
    // make sure the coordinates are kept within the display area
    if(x2 <= x1) { t = x1; x1 = x2; x2 = t; }
    if(y2 <= y1) { t = y1; y1 = y2; y2 = t; }
    if(x1 < 0) x1 = 0; 
    if(x1 >= HRes) x1 = HRes - 1;
    if(x2 < 0) x2 = 0; 
    if(x2 >= HRes) x2 = HRes - 1;
    if(y1 < 0) y1 = 0; 
    if(y1 >= VRes) y1 = VRes - 1;
    if(y2 < 0) y2 = 0; 
    if(y2 >= VRes) y2 = VRes - 1;

    t = y2 - y1;                                                    // get the distance between the top and bottom
    if(Option.DISPLAY_TYPE!=ILI9341_8){
        if(Option.DISPLAY_ORIENTATION == RLANDSCAPE)
            y1 = (y1 + (VRes - ScrollStart)) % VRes;
        else
            y1 = (y1 + ScrollStart) % VRes;
        y2 = y1 + t; 
    }                                                   // and set y2 to the same
    if(y2 >= VRes) {
        if(Option.DISPLAY_TYPE==ILI9341_8) SetAreaILI9341(x1, y1 , x2, y2, 1);
        else {
            SetAreaSSD1963(x1, y1, x2, VRes - 1);                       // if the box splits over the frame buffer boundary
            WriteComand(CMD_WR_MEMSTART);
        }
        for(i = (x2 - x1 + 1) * ((VRes - 1) - y1 + 1); i > 0; i--){
            c.rgbbytes[0] = *p++;                                   // this order swaps the bytes to match the .BMP file
            c.rgbbytes[1] = *p++;
            c.rgbbytes[2] = *p++;
            WriteColor(c.rgb);
        }
        SetAreaSSD1963(x1, 0, x2, y2 - VRes );
        WriteComand(CMD_WR_MEMSTART);
        for(i = (x2 - x1 + 1) * (y2 - VRes + 1); i > 0; i--) {
            c.rgbbytes[0] = *p++;                                   // this order swaps the bytes to match the .BMP file
            c.rgbbytes[1] = *p++;
            c.rgbbytes[2] = *p++;
            WriteColor(c.rgb);
        }
    } else {
        // the whole box is within the frame buffer - much easier
    	if(Option.DISPLAY_TYPE == ILI9341_8) {
    		SetAreaILI9341(x1, y1 , x2, y2, 1);
        } else {
            SetAreaSSD1963(x1, y1 , x2, y2);                            // setup the area to be filled
            WriteComand(CMD_RD_MEMSTART);
        }
        for(i = (x2 - x1 + 1) * (y2 - y1 + 1); i > 0; i--){
            c.rgbbytes[0] = *p++;                                   // this order swaps the bytes to match the .BMP file
            c.rgbbytes[1] = *p++;
            c.rgbbytes[2] = *p++;
            WriteColor(c.rgb);
        }
    }
}


// Read RGB colour over an 8 bit bus
inline __attribute((always_inline)) unsigned int ReadColor(void) {
    return(ReadData() << 16) | (ReadData() << 8) | ReadData();
}


// Read RGB colour over an 8 bit bus
// but do it slowly to avoid timing issues with the first pixel
unsigned int ReadColorSlow(void) {
    return(ReadDataSlow() << 16) | (ReadDataSlow() << 8) | ReadDataSlow();
}


// written by Peter Mather (matherp on the Back Shed forum)
void ReadBufferSSD1963(int x1, int y1, int x2, int y2, unsigned char* p) {
    int i, t;
    union colourmap {
      char rgbbytes[4];
      unsigned int rgb;
    } c;

    // make sure the coordinates are kept within the display area
    if(x2 <= x1) { t = x1; x1 = x2; x2 = t; }
    if(y2 <= y1) { t = y1; y1 = y2; y2 = t; }
    if(x1 < 0) x1 = 0; 
    if(x1 >= HRes) x1 = HRes - 1;
    if(x2 < 0) x2 = 0; 
    if(x2 >= HRes) x2 = HRes - 1;
    if(y1 < 0) y1 = 0; 
    if(y1 >= VRes) y1 = VRes - 1;
    if(y2 < 0) y2 = 0; 
    if(y2 >= VRes) y2 = VRes - 1;

    t = y2 - y1;                                                    // get the distance between the top and bottom
    if(Option.DISPLAY_TYPE!=ILI9341_8){
        if(Option.DISPLAY_ORIENTATION == RLANDSCAPE)
            y1 = (y1 + (VRes - ScrollStart)) % VRes;
        else
            y1 = (y1 + ScrollStart) % VRes;
        y2 = y1 + t;  
    }                                                  // and set y2 to the same

    if(y2 >= VRes) {
        SetAreaSSD1963(x1, y1, x2, VRes - 1);                       // if the box splits over the frame buffer boundary
        WriteComand(CMD_RD_MEMSTART);
        gpio_set_dir_in_masked(0xFF);
        i=(x2 - x1 + 1) * ((VRes - 1) - y1 + 1);
        uSec(10);
        for( ; i > 1; i--) {                                        // NB loop counter terminates 1 pixel earlier
            c.rgb = ReadColor();
            *p++ = c.rgbbytes[0];                                   // this order swaps the bytes to match the .BMP file
            *p++ = c.rgbbytes[1];
            *p++ = c.rgbbytes[2];
        }
        gpio_set_dir_out_masked(0xFF);
        uSec(10);
        SetAreaSSD1963(x1, 0, x2, y2 - VRes );
        WriteComand(CMD_RD_MEMSTART);
        gpio_set_dir_in_masked(0xFF);
        uSec(10);
         for(i = (x2 - x1 + 1) * (y2 - VRes + 1); i > 1; i--) {     // NB loop counter terminates 1 pixel earlier
            c.rgb = ReadColor();
            *p++ = c.rgbbytes[0];                                   // this order swaps the bytes to match the .BMP file
            *p++ = c.rgbbytes[1];
            *p++ = c.rgbbytes[2];
        }
        gpio_set_dir_out_masked(0xFF);
        uSec(10);
    } else {
    	if(Option.DISPLAY_TYPE == ILI9341_8) {
    		SetAreaILI9341(x1, y1 , x2, y2, 0);
        } else {
            SetAreaSSD1963(x1, y1 , x2, y2);                            // setup the area to be filled
            WriteComand(CMD_RD_MEMSTART);
        }
        gpio_set_dir_in_masked(0xFF);
        uSec(10);
        if(Option.DISPLAY_TYPE==ILI9341_8)ReadDataSlow();
        for(i = (x2 - x1 + 1) * (y2 - y1 + 1); i > 0; i--){
            c.rgb=ReadColor();
            *p++=c.rgbbytes[0];                                     // this order swaps the bytes to match the .BMP file
            *p++=c.rgbbytes[1];
            *p++=c.rgbbytes[2];
        }
        gpio_set_dir_out_masked(0xFF);
        uSec(10);
    }
}

void fun_getscanline(void){
    if(Option.DISPLAY_TYPE < SSDPANEL) error("Invalid on this display");

    WriteComand(CMD_GET_SCANLINE);
    gpio_set_dir_in_masked(0xFF);
    iret = (ReadData() << 8) | ReadData();                          // get the scan line
    gpio_set_dir_out_masked(0xFF);
    targ = T_INT;
}


/***********************************************************************************************
Print the bitmap of a char on the video output
Modifications by Peter Mather (matherp on the Back Shed forum) to support transparent text
  x1, y1 - the top left of the char
    width, height - size of the char's bitmap
    scale - how much to scale the bitmap
  fg, bg - foreground and background colour
    bitmap - pointer to the butmap
***********************************************************************************************/
void DrawBitmapSSD1963(int x1, int y1, int width, int height, int scale, int fg, int bg, unsigned char *bitmap ){
    int i, j, k, m, y, yt, n ;
    int vertCoord, horizCoord, XStart, XEnd;
    char *buff;
    union car {
        char rgbbytes[4];
        unsigned int rgb;
    } c;

    buff = NULL;

    // adjust when part of the bitmap is outside the displayable coordinates
    vertCoord = y1; if(y1 < 0) y1 = 0;                                 // the y coord is above the top of the screen
    XStart = x1; if(XStart < 0) XStart = 0;                            // the x coord is to the left of the left marginn
    XEnd = x1 + (width * scale) - 1; if(XEnd >= HRes) XEnd = HRes - 1; // the width of the bitmap will extend beyond the right margin
    if(bg == -1) {
        buff = GetMemory(width * height * scale * scale * 3 );
        ReadBuffer(XStart, y1, XEnd, (y1 + (height * scale) - 1) , (unsigned char *)buff);
        n = 0;
    }

    // set y and yt to the physical location in the frame buffer (only is important when scrolling is in action)
    if(Option.DISPLAY_ORIENTATION == RLANDSCAPE)
        yt = y = (y1 + (VRes - ScrollStart)) % VRes;
    else
        yt = y = (y1 + ScrollStart) % VRes;

    if(Option.DISPLAY_TYPE==ILI9341_8) SetAreaILI9341(XStart, y, XEnd, (y + (height * scale) - 1)  % VRes, 1);
    else {
        SetAreaSSD1963(XStart, y, XEnd, (y + (height * scale) - 1)  % VRes);
        WriteComand(CMD_WR_MEMSTART);
    }
    for(i = 0; i < height; i++) {                                   // step thru the font scan line by line
        for(j = 0; j < scale; j++) {                                // repeat lines to scale the font
            if(vertCoord++ < 0) continue;                           // we are above the top of the screen
            if(vertCoord > VRes) {                                  // we have extended beyond the bottom of the screen
              if(buff != NULL) FreeMemory((unsigned char *)buff);
              return;
            }
            // if we have scrolling in action we could run over the end of the frame buffer
            // if so, terminate this area and start a new one at the top of the frame buffer
            if(y++ == VRes) {
                if(Option.DISPLAY_TYPE==ILI9341_8) SetAreaILI9341(XStart, 0, XEnd, ((yt + (height * scale) - 1)  % VRes) - y, 1);
                else {
                    SetAreaSSD1963(XStart, 0, XEnd, ((yt + (height * scale) - 1)  % VRes) - y);
                    WriteComand(CMD_WR_MEMSTART);
                }
            }
            horizCoord = x1;
                // optimise by dedicating the code to just writing to the 100 pin chip
                for(k = 0; k < width; k++) {                        // step through each bit in a scan line
                    for(m = 0; m < scale; m++) {                    // repeat pixels to scale in the x axis
                        if(horizCoord++ < 0) continue;              // we have not reached the left margin
                        if(horizCoord > HRes) continue;             // we are beyond the right margin
                        if((bitmap[((i * width) + k)/8] >> (7 - (((i * width) + k) % 8))) & 1)
//                        if((bitmap[((i * width) + k)/8] >> (((height * width) - ((i * width) + k) - 1) %8)) & 1)
                            WriteColor(fg);
                        else {
                            if(buff != NULL){
                                c.rgbbytes[0] = buff[n];
                                c.rgbbytes[1] = buff[n+1];
                                c.rgbbytes[2] = buff[n+2];
                                WriteColor(c.rgb);
                            } else {
                                WriteColor(bg);
                            }
                        }
                        n+=3;
                    }
                }
        }
    }
    if(buff != NULL) FreeMemory((unsigned char *)buff);
}


/**********************************************************************************************
 Scroll the image by a number of scan lines
 Will only work in landscape or reverse landscape
 lines - the number of scan lines to scroll
        if positive the display will scroll up
        if negative it will scroll down
***********************************************************************************************/
void ScrollSSD1963(int lines) {
    int t;

    t = ScrollStart;

    if(lines >= 0) {
        DrawRectangleSSD1963(0, 0, HRes - 1, lines - 1, gui_bcolour); // erase the line to be scrolled off
        while(lines--) {
            if(Option.DISPLAY_ORIENTATION == LANDSCAPE) {
                if(++t >= VRes) t = 0;
            } else if(Option.DISPLAY_ORIENTATION == RLANDSCAPE) {
                if(--t < 0) t = VRes - 1;
            }
        }
    } else {
        while(lines++) {
            if(Option.DISPLAY_ORIENTATION == LANDSCAPE) {
                if(--t < 0) t = VRes - 1;
            } else if(Option.DISPLAY_ORIENTATION == RLANDSCAPE) {
                if(++t >= VRes) t = 0;
            }
        }
        DrawRectangleSSD1963(0, 0, HRes - 1, lines - 1, gui_bcolour); // erase the line introduced at the top
    }

  WriteComand(CMD_SET_SCROLL_START);
  WriteData(t >> 8);
  WriteData(t);

    ScrollStart = t;
}

