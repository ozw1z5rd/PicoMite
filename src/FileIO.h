/***********************************************************************************************************************
PicoMite MMBasic

FileIO.h

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

************************************************************************************************************************/#ifndef __FILEIO_H
#define __FILEIO_H

#ifdef __cplusplus
extern "C" {
#endif
#include "ff.h"
// File related I/O
unsigned char MMfputc(unsigned char c, int fnbr);
int MMfgetc(int filenbr);
void MMfopen(unsigned char *fname, unsigned char *mode, int fnbr);
int MMfeof(int filenbr);
void MMfclose(int fnbr);
int FindFreeFileNbr(void);
void CloseAllFiles(void);
void MMgetline(int filenbr, char *p);
void MMPrintString(char *s);
void CheckAbort(void);
char FileGetChar(int fnbr);
void FilePutStr(int count, char *c, int fnbr);
char FilePutChar(char c, int fnbr);
void CheckSDCard(void);
void LoadOptions(void);
void CrunchData(unsigned char **p, int c);
int FileEOF(int fnbr);
void ClearSavedVars(void);
int FileLoadProgram(unsigned char *fname);
void SaveOptions(void);
void ResetAllFlash(void);
void disable_interrupts(void);
void enable_interrupts(void);
int ForceFileClose(int fnbr);
void ErrorCheck(int fnbr);
extern int OptionFileErrorAbort;
extern unsigned char filesource[MAXOPENFILES + 1];
extern int FatFSFileSystemSave;
extern void positionfile(int fnbr, int idx);
struct option_s {
    int  Magic;
    char Autorun;
    char Tab;
    char Invert;
    char Listcase; //8
  //
    unsigned int PROG_FLASH_SIZE;
    unsigned int HEAP_SIZE;
    char Height;
    char Width;
    unsigned char DISPLAY_TYPE;
    char DISPLAY_ORIENTATION; //12=20
//
    int  PIN;
    int  Baudrate;
    int  ColourCode;
    int CPU_Speed; 
    unsigned int Telnet;    // used to store the size of the program flash (also start of the LIBRARY code)
    int DefaultFC, DefaultBC;      // the default colours
    int DefaultBrightness;         // default backlight brightness //40
    uint16_t VGAFC, VGABC;      // the default colours 36=56
//
    // display related
    unsigned char DefaultFont;
    unsigned char KeyboardConfig;
    unsigned char RTC_Clock;
    unsigned char RTC_Data; //4=60
//
    #ifdef PICOMITE
        int MaxCtrls;                // maximum number of controls allowed //48
    #endif
    #ifdef PICOMITEWEB
        uint16_t TCP_PORT;                // maximum number of controls allowed //48
        uint16_t ServerResponceTime;
    #endif
    #ifdef PICOMITEVGA
        int16_t X_TILE;                // maximum number of controls allowed //48
        int16_t Y_TILE;                // maximum number of controls allowed //48
    #endif
        // for the SPI LCDs 4=64
    unsigned char LCD_CD;
    unsigned char LCD_CS;
    unsigned char LCD_Reset;
    // touch related
    unsigned char TOUCH_CS;
    unsigned char TOUCH_IRQ;
    char TOUCH_SWAPXY; 
    unsigned char repeat;
    char disabletftp;//56   8=72
    int  TOUCH_XZERO;
    int  TOUCH_YZERO;
    float TOUCH_XSCALE;
    float TOUCH_YSCALE; //72 16=88
    unsigned int fullrefresh;
    unsigned int FlashSize; //8=96
    unsigned char SD_CS;
    unsigned char SYSTEM_MOSI;
    unsigned char SYSTEM_MISO;
    unsigned char SYSTEM_CLK;
    unsigned char DISPLAY_BL;
    unsigned char DISPLAY_CONSOLE;
    unsigned char TOUCH_Click;
    char LCD_RD;                   // used for the RD pin for SSD1963  //8=104
    unsigned char AUDIO_L;
    unsigned char AUDIO_R;
    unsigned char AUDIO_SLICE; 
    unsigned char SDspeed;
    unsigned char pins[8];  //12=116                // general use storage for CFunctions written by PeterM //86
    char LCDVOP;
    char I2Coffset;
    unsigned char NoHeartbeat; 
    char Refresh;
    unsigned char SYSTEM_I2C_SDA;
    unsigned char SYSTEM_I2C_SCL;
    unsigned char RTC;
    char PWM;  //8=124
    unsigned char INT1pin;
    unsigned char INT2pin;
    unsigned char INT3pin; 
    unsigned char INT4pin;
    unsigned char SD_CLK_PIN;
    unsigned char SD_MOSI_PIN;
    unsigned char SD_MISO_PIN;
    unsigned char SerialConsole; //8=132
    unsigned char SerialTX;
    unsigned char SerialRX;
    unsigned char numlock; 
    unsigned char capslock; //4=136
    unsigned int  LIBRARY_FLASH_SIZE; // 4=140
    unsigned char AUDIO_CLK_PIN;
    unsigned char AUDIO_MOSI_PIN;
    unsigned char SYSTEM_I2C_SLOW;
    unsigned char AUDIO_CS_PIN;
    #ifdef PICOMITEWEB
        uint16_t UDP_PORT;                // maximum number of controls allowed //48
        uint16_t UDPServerResponceTime;
        char hostname[32];
        char ipaddress[16];
        char mask[16];
        char gateway[16];
        unsigned char x[2]; //108=256
    #else
        unsigned char x[86]; //112=256
    #endif
    unsigned char BGR;
    unsigned char NoScroll;
    unsigned char CombinedCS;
    unsigned char USBKeyboard;
    unsigned char VGA_HSYNC;
    unsigned char VGA_BLUE;
    unsigned char AUDIO_MISO_PIN;
    unsigned char AUDIO_DCS_PIN;
    unsigned char AUDIO_DREQ_PIN;
    unsigned char AUDIO_RESET_PIN;
    unsigned char SSD_DC;
    unsigned char SSD_WR;
    unsigned char SSD_RD;
    signed char SSD_RESET;
    unsigned char BackLightLevel;
    unsigned char NoReset;
    unsigned char AllPins;
    unsigned char modbuff;
	short RepeatStart;
	short RepeatRate;
    int modbuffsize;
    unsigned char F1key[MAXKEYLEN]; //204
    unsigned char F5key[MAXKEYLEN]; //268
    unsigned char F6key[MAXKEYLEN]; //332
    unsigned char F7key[MAXKEYLEN]; //396
    unsigned char F8key[MAXKEYLEN]; //460
    unsigned char F9key[MAXKEYLEN]; //524
    unsigned char SSID[MAXKEYLEN];  //588
    unsigned char PASSWORD[MAXKEYLEN]; //652=768
    unsigned char platform[32];
    unsigned char extensions[96];
    // To enable older CFunctions to run any new options *MUST* be added at the end of the list
} __attribute__((packed));
extern unsigned char *CFunctionFlash, *CFunctionLibrary;
extern struct option_s Option;
extern int FlashLoad;
extern void ResetOptions(void);
extern void FlashWriteBlock(void);
extern void FlashWriteWord(unsigned int i);
extern void FlashWriteByte(unsigned char b);
extern void FlashWriteAlign(void);
extern void FlashWriteClose(void);
extern void FlashWriteInit(int region);
void FlashSetAddress(int address);  //new
extern void FlashWriteAlignWord(void);  //new
extern void ResetFlashStorage(int umount);
extern volatile uint32_t realflashpointer;
extern int drivecheck(char *p, int *waste);
extern void getfullfilename(char *fname, char *q);
extern int FSerror;
extern int lfs_FileFnbr;
extern struct lfs_config pico_lfs_cfg;
#define SAVED_OPTIONS_FLASH 5
#define LIBRARY_FLASH 6
#define SAVED_VARS_FLASH 7
#define PROGRAM_FLASH 8
typedef union uFileTable
{
    unsigned int com;
    FIL *fptr;
    lfs_file_t *lfsptr;
}u_file;
enum {
    NONEFILE,
    FLASHFILE,
    FATFSFILE
};
extern union uFileTable FileTable[MAXOPENFILES + 1];

#ifdef __cplusplus
}
#endif
#endif /* __FILEIO_H */