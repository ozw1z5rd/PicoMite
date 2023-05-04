/***********************************************************************************************************************
PicoMite MMBasic

MM_Misc.c

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
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include <time.h>
#include "upng.h"
#include <complex.h>
#include "pico/bootrom.h"
#include "hardware/structs/systick.h"
#include "hardware/structs/pwm.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/flash.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "hardware/pio_instructions.h"
#include <malloc.h>

uint32_t getTotalHeap(void) {
   extern char __StackLimit, __bss_end__;
   
   return &__StackLimit  - &__bss_end__;
}

uint32_t getFreeHeap(void) {
   struct mallinfo m = mallinfo();

   return getTotalHeap() - m.uordblks;
}

uint32_t getProgramSize(void) {
   extern char __flash_binary_start, __flash_binary_end;

   return &__flash_binary_end - &__flash_binary_start;
}

uint32_t getFreeProgramSpace() {
   return PICO_FLASH_SIZE_BYTES - getProgramSize();
}
extern int busfault;
//#include "pico/stdio_usb/reset_interface.h"
const char *OrientList[] = {"LANDSCAPE", "PORTRAIT", "RLANDSCAPE", "RPORTRAIT"};
const char *KBrdList[] = {"", "US", "FR", "GR", "IT", "BE", "UK", "ES" };
extern const void * const CallTable[];
struct s_inttbl inttbl[NBRINTERRUPTS];
unsigned char *InterruptReturn;
extern const char *FErrorMsg[];
#ifdef PICOMITEWEB
	char *MQTTInterrupt=NULL;
	volatile int MQTTComplete=0;
#endif
extern const uint8_t *flash_target_contents;
int TickPeriod[NBRSETTICKS]={0};
volatile int TickTimer[NBRSETTICKS]={0};
unsigned char *TickInt[NBRSETTICKS]={NULL};
volatile unsigned char TickActive[NBRSETTICKS]={0};
unsigned char *OnKeyGOSUB = NULL;
unsigned char *OnPS2GOSUB = NULL;
const char *daystrings[] = {"dummy","Monday","Tuesday","Wednesday","Thursday","Friday","Saturday","Sunday"};
const char *CaseList[] = {"", "LOWER", "UPPER"};
int OptionErrorCheck;
unsigned char EchoOption = true;
unsigned long long int __attribute__((section(".my_section"))) saved_variable;  //  __attribute__ ((persistent));  // and this is the address
unsigned int CurrentCpuSpeed;
unsigned int PeripheralBusSpeed;
extern char *ADCInterrupt;
extern volatile int ConsoleTxBufHead;
extern volatile int ConsoleTxBufTail;
extern char *LCDList[];
extern volatile BYTE SDCardStat;
extern volatile int keyboardseen;
extern uint64_t TIM12count;
extern char id_out[12];
extern void WriteCommand(int cmd);
extern void WriteData(int data);
char *CSubInterrupt;
MMFLOAT optionangle=1.0;
int optionfastaudio=0;
volatile int CSubComplete=0;
uint64_t timeroffset=0;
int SaveOptionErrorSkip=0;
char SaveErrorMessage[MAXERRMSG] = { 0 };
int Saveerrno = 0;
void integersort(int64_t *iarray, int n, long long *index, int flags, int startpoint){
    int i, j = n, s = 1;
    int64_t t;
    if((flags & 1) == 0){
		while (s) {
			s = 0;
			for (i = 1; i < j; i++) {
				if (iarray[i] < iarray[i - 1]) {
					t = iarray[i];
					iarray[i] = iarray[i - 1];
					iarray[i - 1] = t;
					s = 1;
			        if(index!=NULL){
			        	t=index[i-1+startpoint];
			        	index[i-1+startpoint]=index[i+startpoint];
			        	index[i+startpoint]=t;
			        }
				}
			}
			j--;
		}
    } else {
		while (s) {
			s = 0;
			for (i = 1; i < j; i++) {
				if (iarray[i] > iarray[i - 1]) {
					t = iarray[i];
					iarray[i] = iarray[i - 1];
					iarray[i - 1] = t;
					s = 1;
			        if(index!=NULL){
			        	t=index[i-1+startpoint];
			        	index[i-1+startpoint]=index[i+startpoint];
			        	index[i+startpoint]=t;
			        }
				}
			}
			j--;
		}
    }
}
void floatsort(MMFLOAT *farray, int n, long long *index, int flags, int startpoint){
    int i, j = n, s = 1;
    int64_t t;
    MMFLOAT f;
    if((flags & 1) == 0){
		while (s) {
			s = 0;
			for (i = 1; i < j; i++) {
				if (farray[i] < farray[i - 1]) {
					f = farray[i];
					farray[i] = farray[i - 1];
					farray[i - 1] = f;
					s = 1;
			        if(index!=NULL){
			        	t=index[i-1+startpoint];
			        	index[i-1+startpoint]=index[i+startpoint];
			        	index[i+startpoint]=t;
			        }
				}
			}
			j--;
		}
    } else {
		while (s) {
			s = 0;
			for (i = 1; i < j; i++) {
				if (farray[i] > farray[i - 1]) {
					f = farray[i];
					farray[i] = farray[i - 1];
					farray[i - 1] = f;
					s = 1;
			        if(index!=NULL){
			        	t=index[i-1+startpoint];
			        	index[i-1+startpoint]=index[i+startpoint];
			        	index[i+startpoint]=t;
			        }
				}
			}
			j--;
		}
    }
}

void stringsort(unsigned char *sarray, int n, int offset, long long *index, int flags, int startpoint){
	int ii,i, s = 1,isave;
	int k;
	unsigned char *s1,*s2,*p1,*p2;
	unsigned char temp;
	int reverse= 1-((flags & 1)<<1);
    while (s){
      s=0;
      for(i=1;i<n;i++){
        s2=i*offset+sarray;
        s1=(i-1)*offset+sarray;
        ii = *s1 < *s2 ? *s1 : *s2; //get the smaller  length
        p1 = s1 + 1; p2 = s2 + 1;
        k=0; //assume the strings match
        while((ii--) && (k==0)) {
          if(flags & 2){
			  if(toupper(*p1) > toupper(*p2)){
				k=reverse; //earlier in the array is bigger
			  }
			  if(toupper(*p1) < toupper(*p2)){
				 k=-reverse; //later in the array is bigger
			  }
          } else {
			  if(*p1 > *p2){
				k=reverse; //earlier in the array is bigger
			  }
			  if(*p1 < *p2){
				 k=-reverse; //later in the array is bigger
			  }
          }
          p1++; p2++;
        }
      // if up to this point the strings match
      // make the decision based on which one is shorter
      if(k==0){
        if(*s1 > *s2) k=reverse;
        if(*s1 < *s2) k=-reverse;
      }
      if (k==1){ // if earlier is bigger swap them round
        ii = *s1 > *s2 ? *s1 : *s2; //get the bigger length
        ii++;
        p1=s1;p2=s2;
        while(ii--){
          temp=*p1;
          *p1=*p2;
          *p2=temp;
          p1++; p2++;
        }
        s=1;
        if(index!=NULL){
        	isave=index[i-1+startpoint];
        	index[i-1+startpoint]=index[i+startpoint];
        	index[i+startpoint]=isave;
        }
      }
    }
  }
}
void cmd_sort(void){
    void *ptr1 = NULL;
    void *ptr2 = NULL;
    MMFLOAT *a3float=NULL;
    int64_t *a3int=NULL,*a4int=NULL;
    unsigned char *a3str=NULL;
    int i, size, truesize,flags=0, maxsize=0, startpoint=0;
	getargs(&cmdline,9,",");
    ptr1 = findvar(argv[0], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
    if(vartbl[VarIndex].type & T_NBR) {
        if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
        if(vartbl[VarIndex].dims[0] <= 0) {		// Not an array
            error("Argument 1 must be array");
        }
        a3float = (MMFLOAT *)ptr1;
    } else if(vartbl[VarIndex].type & T_INT) {
        if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
        if(vartbl[VarIndex].dims[0] <= 0) {		// Not an array
            error("Argument 1 must be array");
        }
        a3int = (int64_t *)ptr1;
    } else if(vartbl[VarIndex].type & T_STR) {
        if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
        if(vartbl[VarIndex].dims[0] <= 0) {		// Not an array
            error("Argument 1 must be array");
        }
        a3str = (unsigned char *)ptr1;
        maxsize=vartbl[VarIndex].size;
    } else error("Argument 1 must be array");
	if((uint32_t)ptr1!=(uint32_t)vartbl[VarIndex].val.s)error("Argument 1 must be array");
    truesize=size=(vartbl[VarIndex].dims[0] - OptionBase);
    if(argc>=3 && *argv[2]){
    	ptr2 = findvar(argv[2], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
    	if(vartbl[VarIndex].type & T_INT) {
    		if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
    		if(vartbl[VarIndex].dims[0] <= 0 ) {		// Not an array
    			error("Argument 2 must be integer array");
    		}
    		a4int = (int64_t *)ptr2;
    	} else error("Argument 2 must be integer array");
    	if((vartbl[VarIndex].dims[0] - OptionBase) !=size)error("Arrays should be the same size");
		if((uint32_t)ptr2!=(uint32_t)vartbl[VarIndex].val.s)error("Argument 2 must be array");
    }
    if(argc>=5 && *argv[4])flags=getint(argv[4],0,3);
    if(argc>=7 && *argv[6])startpoint=getint(argv[6],OptionBase,size+OptionBase);
    size-=startpoint;
    if(argc==9)size=getint(argv[8],1,size+1+OptionBase)-1;
    if(startpoint)startpoint-=OptionBase;
    if(a3float!=NULL){
    	a3float+=startpoint;
    	if(a4int!=NULL)for(i=0;i<truesize+1;i++)a4int[i]=i+OptionBase;
    	floatsort(a3float, size+1, a4int, flags, startpoint);
    } else if(a3int!=NULL){
    	a3int+=startpoint;
    	if(a4int!=NULL)for(i=0;i<truesize+1;i++)a4int[i]=i+OptionBase;
    	integersort(a3int,  size+1, a4int, flags, startpoint);
    } else if(a3str!=NULL){
    	a3str+=((startpoint)*(maxsize+1));
    	if(a4int!=NULL)for(i=0;i<truesize+1;i++)a4int[i]=i+OptionBase;
    	stringsort(a3str,  size+1,maxsize+1, a4int, flags, startpoint);
    }
}
// this is invoked as a command (ie, TIMER = 0)
// search through the line looking for the equals sign and step over it,
// evaluate the rest of the command and save in the timer
void cmd_timer(void) {
  uint64_t mytime=time_us_64();
  while(*cmdline && tokenfunction(*cmdline) != op_equal) cmdline++;
  if(!*cmdline) error("Syntax");
  timeroffset=mytime-(uint64_t)getint(++cmdline,0,mytime/1000)*1000;
}
// this is invoked as a function
void __not_in_flash_func(fun_timer)(void) {
    fret = (MMFLOAT)(time_us_64()-timeroffset)/1000.0;
    targ = T_NBR;
}
void fun_datetime(void){
    sret = GetTempMemory(STRINGSIZE);                                    // this will last for the life of the command
    if(checkstring(ep, "NOW")){
        IntToStrPad(sret, day, '0', 2, 10);
        sret[2] = '-'; IntToStrPad(sret + 3, month, '0', 2, 10);
        sret[5] = '-'; IntToStr(sret + 6, year, 10);
        sret[10] = ' ';
        IntToStrPad(sret+11, hour, '0', 2, 10);
        sret[13] = ':'; IntToStrPad(sret + 14, minute, '0', 2, 10);
        sret[16] = ':'; IntToStrPad(sret + 17, second, '0', 2, 10);
    } else {
        struct tm  *tm;
        struct tm tma;
        tm=&tma;
        time_t timestamp = getinteger(ep); /* See README.md if your system lacks timegm(). */
        tm=gmtime(&timestamp);
        IntToStrPad(sret, tm->tm_mday, '0', 2, 10);
        sret[2] = '-'; IntToStrPad(sret + 3, tm->tm_mon+1, '0', 2, 10);
        sret[5] = '-'; IntToStr(sret + 6, tm->tm_year+1900, 10);
        sret[10] = ' ';
        IntToStrPad(sret+11, tm->tm_hour, '0', 2, 10);
        sret[13] = ':'; IntToStrPad(sret + 14, tm->tm_min, '0', 2, 10);
        sret[16] = ':'; IntToStrPad(sret + 17, tm->tm_sec, '0', 2, 10);
    }
    CtoM(sret);
    targ = T_STR;
}

void fun_epoch(void){
    unsigned char *arg;
    struct tm  *tm;
    struct tm tma;
    tm=&tma;
    int d, m, y, h, min, s;
    if(!checkstring(ep, "NOW"))
    {
        arg = getCstring(ep);
        getargs(&arg, 11, "-/ :");                                      // this is a macro and must be the first executable stmt in a block
        if(!(argc == 11)) error("Syntax");
            d = atoi(argv[0]);
            m = atoi(argv[2]);
            y = atoi(argv[4]);
			if(d>1000){
				int tmp=d;
				d=y;
				y=tmp;
			}
            if(y >= 0 && y < 100) y += 2000;
            if(d < 1 || d > 31 || m < 1 || m > 12 || y < 1902 || y > 2999) error("Invalid date");
            h = atoi(argv[6]);
            min  = atoi(argv[8]);
            s = atoi(argv[10]);
            if(h < 0 || h > 23 || min < 0 || m > 59 || s < 0 || s > 59) error("Invalid time");
//            day = d;
//            month = m;
//            year = y;
            tm->tm_year = y - 1900;
            tm->tm_mon = m - 1;
            tm->tm_mday = d;
            tm->tm_hour = h;
            tm->tm_min = min;
            tm->tm_sec = s;
    } else {
        tm->tm_year = year - 1900;
        tm->tm_mon = month - 1;
        tm->tm_mday = day;
        tm->tm_hour = hour;
        tm->tm_min = minute;
        tm->tm_sec = second;
    }
        time_t timestamp = timegm(tm); /* See README.md if your system lacks timegm(). */
        iret=timestamp;
        targ = T_INT;
}

void cmd_pause(void) {
	static int interrupted = false;
    MMFLOAT f;
    static int64_t end,count;
    int64_t start, stop, tick;
    f = getnumber(cmdline);                                         // get the pulse width
    if(f < 0) error("Number out of bounds");
    if(f < 0.05) return;

	if(f < 1.5) {
		uSec(f * 1000);                                             // if less than 1.5mS do the pause right now
		return;                                                     // and exit straight away
    }
    if(!interrupted){
        count=(int64_t)(f*1000);
        start=time_us_64();
        tick=PauseTimer;
        while(PauseTimer==tick){}  //wait for the next clock tick
        stop=time_us_64();
        count-=(stop-start);
        end = (count % 1000); //get the number of ticks remaining
        count/=1000;
        PauseTimer=0;
    }
    if(count){
		if(InterruptReturn == NULL) {
			// we are running pause in a normal program
			// first check if we have reentered (from an interrupt) and only zero the timer if we have NOT been interrupted.
			// This means an interrupted pause will resume from where it was when interrupted
			if(!interrupted) PauseTimer = 0;
			interrupted = false;

			while(PauseTimer < count) {
				CheckAbort();
				if(check_interrupt()) {
					// if there is an interrupt fake the return point to the start of this stmt
					// and return immediately to the program processor so that it can send us off
					// to the interrupt routine.  When the interrupt routine finishes we should reexecute
					// this stmt and because the variable interrupted is static we can see that we need to
					// resume pausing rather than start a new pause time.
					while(*cmdline && *cmdline != cmdtoken) cmdline--;	// step back to find the command token
					InterruptReturn = cmdline;							// point to it
					interrupted = true;								    // show that this stmt was interrupted
					return;											    // and let the interrupt run
				}
			}
			interrupted = false;
		}
		else {
			// we are running pause in an interrupt, this is much simpler but note that
			// we use a different timer from the main pause code (above)
			IntPauseTimer = 0;
			while(IntPauseTimer < FloatToInt32(f)) CheckAbort();
		}
    }
	uSec(end);
}

void cmd_longString(void){
    unsigned char *tp;
    tp = checkstring(cmdline, "SETBYTE");
    if(tp){
        void *ptr1 = NULL;
        int64_t *dest=NULL;
        int p=0;
        uint8_t *q=NULL;
        int nbr;
        int j=0;
    	getargs(&tp, 5, ",");
        if(argc != 5)error("Argument count");
        ptr1 = findvar(argv[0], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(vartbl[VarIndex].type & T_INT) {
            if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
            if(vartbl[VarIndex].dims[0] <= 0) {		// Not an array
                error("Argument 1 must be integer array");
            }
            j=(vartbl[VarIndex].dims[0] - OptionBase)*8-1;
            dest = (long long int *)ptr1;
            q=(uint8_t *)&dest[1];
        } else error("Argument 1 must be integer array");
        p = getint(argv[2],OptionBase,j-OptionBase);
        nbr=getint(argv[4],0,255);
        q[p-OptionBase]=nbr;
         return;
    }
    tp = checkstring(cmdline, "APPEND");
    if(tp){
        void *ptr1 = NULL;
        int64_t *dest=NULL;
        char *p= NULL;
        char *q= NULL;
        int i,j,nbr;
        getargs(&tp, 3, ",");
        if(argc != 3)error("Argument count");
        ptr1 = findvar(argv[0], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(vartbl[VarIndex].type & T_INT) {
            if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
            if(vartbl[VarIndex].dims[0] <= 0) {      // Not an array
                error("Argument 1 must be integer array");
            }
            dest = (long long int *)ptr1;
            q=(char *)&dest[1];
            q+=dest[0];
        } else error("Argument 1 must be integer array");
        j=(vartbl[VarIndex].dims[0] - OptionBase);
        p=getstring(argv[2]);
        nbr = i = *p++;
         if(j*8 < dest[0]+i)error("Integer array too small");
        while(i--)*q++=*p++;
        dest[0]+=nbr;
        return;
    }
    tp = checkstring(cmdline, "TRIM");
    if(tp){
        void *ptr1 = NULL;
        int64_t *dest=NULL;
        uint32_t trim;
        char *p, *q=NULL;
        int i;
        getargs(&tp, 3, ",");
        if(argc != 3)error("Argument count");
        ptr1 = findvar(argv[0], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(vartbl[VarIndex].type & T_INT) {
            if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
            if(vartbl[VarIndex].dims[0] <= 0) {      // Not an array
                error("Argument 1 must be integer array");
            }
            dest = (long long int *)ptr1;
            q=(char *)&dest[1];
        } else error("Argument 1 must be integer array");
        trim=getint(argv[2],1,dest[0]);
        i = dest[0]-trim;
        p=q+trim;
        while(i--)*q++=*p++;
        dest[0]-=trim;
        return;
    }
    tp = checkstring(cmdline, "REPLACE");
    if(tp){
        void *ptr1 = NULL;
        int64_t *dest=NULL;
        char *p=NULL;
        char *q=NULL;
        int i,nbr;
        getargs(&tp, 5, ",");
        if(argc != 5)error("Argument count");
        ptr1 = findvar(argv[0], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(vartbl[VarIndex].type & T_INT) {
            if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
            if(vartbl[VarIndex].dims[0] <= 0) {      // Not an array
                error("Argument 1 must be integer array");
            }
            dest = (long long int *)ptr1;
            q=(char *)&dest[1];
        } else error("Argument 1 must be integer array");
        p=getstring(argv[2]);
        nbr=getint(argv[4],1,dest[0]-*p+1);
        q+=nbr-1;
        i = *p++;
        while(i--)*q++=*p++;
        return;
    }
    tp = checkstring(cmdline, "LOAD");
    if(tp){
        void *ptr1 = NULL;
        int64_t *dest=NULL;
        char *p;
        char *q=NULL;
        int i,j;
        getargs(&tp, 5, ",");
        if(argc != 5)error("Argument count");
        int64_t nbr=getinteger(argv[2]);
        i=nbr;
        ptr1 = findvar(argv[0], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(vartbl[VarIndex].type & T_INT) {
            if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
            if(vartbl[VarIndex].dims[0] <= 0) {      // Not an array
                error("Argument 1 must be integer array");
            }
            dest = (long long int *)ptr1;
            dest[0]=0;
            q=(char *)&dest[1];
        } else error("Argument 1 must be integer array");
        j=(vartbl[VarIndex].dims[0] - OptionBase);
        p=getstring(argv[4]);
        if(nbr> *p)nbr=*p;
        p++;
        if(j*8 < dest[0]+nbr)error("Integer array too small");
        while(i--)*q++=*p++;
        dest[0]+=nbr;
        return;
    }
    tp = checkstring(cmdline, "LEFT");
    if(tp){
        void *ptr1 = NULL;
        void *ptr2 = NULL;
        int64_t *dest=NULL, *src=NULL;
        char *p=NULL;
        char *q=NULL;
        int i,j,nbr;
        getargs(&tp, 5, ",");
        if(argc != 5)error("Argument count");
        ptr1 = findvar(argv[0], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(vartbl[VarIndex].type & T_INT) {
            if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
            if(vartbl[VarIndex].dims[0] <= 0) {      // Not an array
                error("Argument 1 must be integer array");
            }
            dest = (int64_t *)ptr1;
            q=(char *)&dest[1];
        } else error("Argument 1 must be integer array");
        j=(vartbl[VarIndex].dims[0] - OptionBase);
        ptr2 = findvar(argv[2], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(vartbl[VarIndex].type & T_INT) {
            if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
            if(vartbl[VarIndex].dims[0] <= 0) {      // Not an array
                error("Argument 2 must be integer array");
            }
            src = (int64_t *)ptr2;
            p=(char *)&src[1];
        } else error("Argument 2 must be integer array");
        nbr=i=getinteger(argv[4]);
        if(nbr>src[0])nbr=i=src[0];
        if(j*8 < i)error("Destination array too small");
        while(i--)*q++=*p++;
        dest[0]=nbr;
        return;
    }
    tp = checkstring(cmdline, "RIGHT");
    if(tp){
        void *ptr1 = NULL;
        void *ptr2 = NULL;
        int64_t *dest=NULL, *src=NULL;
        char *p=NULL;
        char *q=NULL;
        int i,j,nbr;
        getargs(&tp, 5, ",");
        if(argc != 5)error("Argument count");
        ptr1 = findvar(argv[0], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(vartbl[VarIndex].type & T_INT) {
            if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
            if(vartbl[VarIndex].dims[0] <= 0) {      // Not an array
                error("Argument 1 must be integer array");
            }
            dest = (int64_t *)ptr1;
            q=(char *)&dest[1];
        } else error("Argument 1 must be integer array");
        j=(vartbl[VarIndex].dims[0] - OptionBase);
        ptr2 = findvar(argv[2], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(vartbl[VarIndex].type & T_INT) {
            if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
            if(vartbl[VarIndex].dims[0] <= 0) {      // Not an array
                error("Argument 2 must be integer array");
            }
            src = (int64_t *)ptr2;
            p=(char *)&src[1];
        } else error("Argument 2 must be integer array");
        nbr=i=getinteger(argv[4]);
        if(nbr>src[0]){
            nbr=i=src[0];
        } else p+=(src[0]-nbr);
        if(j*8 < i)error("Destination array too small");
        while(i--)*q++=*p++;
        dest[0]=nbr;
        return;
    }
    tp = checkstring(cmdline, "MID");
    if(tp){
        void *ptr1 = NULL;
        void *ptr2 = NULL;
        int64_t *dest=NULL, *src=NULL;
        char *p=NULL;
        char *q=NULL;
        int i,j,nbr,start;
        getargs(&tp, 7, ",");
        if(argc < 5)error("Argument count");
        ptr1 = findvar(argv[0], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(vartbl[VarIndex].type & T_INT) {
            if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
            if(vartbl[VarIndex].dims[0] <= 0) {      // Not an array
                error("Argument 1 must be integer array");
            }
            dest = (int64_t *)ptr1;
            q=(char *)&dest[1];
        } else error("Argument 1 must be integer array");
        j=(vartbl[VarIndex].dims[0] - OptionBase);
        ptr2 = findvar(argv[2], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(vartbl[VarIndex].type & T_INT) {
            if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
            if(vartbl[VarIndex].dims[0] <= 0) {      // Not an array
                error("Argument 2 must be integer array");
            }
            src = (int64_t *)ptr2;
            p=(char *)&src[1];
        } else error("Argument 2 must be integer array");
        start=getint(argv[4],1,src[0]);
        if(argc==7)nbr=getinteger(argv[6]);
        else nbr=src[0];
        p+=start-1;
        if(nbr+start>src[0]){
            nbr=src[0]-start+1;
        }
        i=nbr;
        if(j*8 < nbr)error("Destination array too small");
        while(i--)*q++=*p++;
        dest[0]=nbr;
        return;
    }
    tp = checkstring(cmdline, "CLEAR");
    if(tp){
        void *ptr1 = NULL;
        int64_t *dest=NULL;
        getargs(&tp, 1, ",");
        if(argc != 1)error("Argument count");
        ptr1 = findvar(argv[0], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(vartbl[VarIndex].type & T_INT) {
            if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
            if(vartbl[VarIndex].dims[0] <= 0) {      // Not an array
                error("Argument 1 must be integer array");
            }
            dest = (long long int *)ptr1;
        } else error("Argument 1 must be integer array");
        dest[0]=0;
        return;
    }
    tp = checkstring(cmdline, "RESIZE");
    if(tp){
        void *ptr1 = NULL;
        int64_t *dest=NULL;
        int j=0;
        getargs(&tp, 3, ",");
        if(argc != 3)error("Argument count");
        ptr1 = findvar(argv[0], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(vartbl[VarIndex].type & T_INT) {
            if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
            if(vartbl[VarIndex].dims[0] <= 0) {		// Not an array
                error("Argument 1 must be integer array");
            }
            j=(vartbl[VarIndex].dims[0] - OptionBase)*8;
            dest = (long long int *)ptr1;
        } else error("Argument 1 must be integer array");
        dest[0] = getint(argv[2], 0, j);
        return;
    }
    tp = checkstring(cmdline, "UCASE");
    if(tp){
        void *ptr1 = NULL;
        int64_t *dest=NULL;
        char *q=NULL;
        int i;
        getargs(&tp, 1, ",");
        if(argc != 1)error("Argument count");
        ptr1 = findvar(argv[0], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(vartbl[VarIndex].type & T_INT) {
            if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
            if(vartbl[VarIndex].dims[0] <= 0) {      // Not an array
                error("Argument 1 must be integer array");
            }
            dest = (long long int *)ptr1;
            q=(char *)&dest[1];
        } else error("Argument 1 must be integer array");
        i=dest[0];
        while(i--){
        if(*q >= 'a' && *q <= 'z')
            *q -= 0x20;
        q++;
        }
        return;
    }
    tp = checkstring(cmdline, "PRINT");
    if(tp){
        void *ptr1 = NULL;
        int64_t *dest=NULL;
        char *q=NULL;
        int i, j, fnbr;
        getargs(&tp, 5, ",;");
        if(argc < 1 || argc > 4)error("Argument count");
        if(argc > 0 && *argv[0] == '#') {                                // check if the first arg is a file number
            argv[0]++;
            fnbr = getinteger(argv[0]);                                 // get the number
            i = 1;
            if(argc >= 2 && *argv[1] == ',') i = 2;                      // and set the next argument to be looked at
        }
        else {
            fnbr = 0;                                                   // no file number so default to the standard output
            i = 0;
        }
        if(argc>=1){
            ptr1 = findvar(argv[i], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
            if(vartbl[VarIndex].type & T_INT) {
                if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
                if(vartbl[VarIndex].dims[0] <= 0) {      // Not an array
                    error("Argument must be integer array");
                }
                dest = (long long int *)ptr1;
                q=(char *)&dest[1];
            } else error("Argument must be integer array");
            j=dest[0];
            while(j--){
                MMfputc(*q++, fnbr);
            }
            i++;
        }
        if(argc > i){
            if(*argv[i] == ';') return;
        }
        MMfputs("\2\r\n", fnbr);
        return;
    }
    tp = checkstring(cmdline, "LCASE");
    if(tp){
        void *ptr1 = NULL;
        int64_t *dest=NULL;
        char *q=NULL;
        int i;
        getargs(&tp, 1, ",");
        if(argc != 1)error("Argument count");
        ptr1 = findvar(argv[0], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(vartbl[VarIndex].type & T_INT) {
            if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
            if(vartbl[VarIndex].dims[0] <= 0) {      // Not an array
                error("Argument 1 must be integer array");
            }
            dest = (long long int *)ptr1;
            q=(char *)&dest[1];
        } else error("Argument 1 must be integer array");
        i=dest[0];
        while(i--){
            if(*q >= 'A' && *q <= 'Z')
                *q += 0x20;
            q++;
        }
        return;
    }
    tp = checkstring(cmdline, "COPY");
    if(tp){
        void *ptr1 = NULL;
        void *ptr2 = NULL;
        int64_t *dest=NULL, *src=NULL;
        char *p=NULL;
        char *q=NULL;
        int i=0,j;
        getargs(&tp, 3, ",");
        if(argc != 3)error("Argument count");
        ptr1 = findvar(argv[0], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(vartbl[VarIndex].type & T_INT) {
            if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
            if(vartbl[VarIndex].dims[0] <= 0) {      // Not an array
                error("Argument 1 must be integer array");
            }
            dest = (int64_t *)ptr1;
            dest[0]=0;
            q=(char *)&dest[1];
        } else error("Argument 1 must be integer array");
        j=(vartbl[VarIndex].dims[0] - OptionBase);
        ptr2 = findvar(argv[2], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(vartbl[VarIndex].type & T_INT) {
            if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
            if(vartbl[VarIndex].dims[0] <= 0) {      // Not an array
                error("Argument 2 must be integer array");
            }
            src = (int64_t *)ptr2;
            p=(char *)&src[1];
            i=src[0];
        } else error("Argument 2 must be integer array");
        if(j*8 <i)error("Destination array too small");
        while(i--)*q++=*p++;
        dest[0]=src[0];
        return;
    }
    tp = checkstring(cmdline, "CONCAT");
    if(tp){
        void *ptr1 = NULL;
        void *ptr2 = NULL;
        int64_t *dest=NULL, *src=NULL;
        char *p=NULL;
        char *q=NULL;
        int i=0,j,d=0,s=0;
        getargs(&tp, 3, ",");
        if(argc != 3)error("Argument count");
        ptr1 = findvar(argv[0], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(vartbl[VarIndex].type & T_INT) {
            if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
            if(vartbl[VarIndex].dims[0] <= 0) {      // Not an array
                error("Argument 1 must be integer array");
            }
            dest = (int64_t *)ptr1;
            d=dest[0];
            q=(char *)&dest[1];
        } else error("Argument 1 must be integer array");
        j=(vartbl[VarIndex].dims[0] - OptionBase);
        ptr2 = findvar(argv[2], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(vartbl[VarIndex].type & T_INT) {
            if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
            if(vartbl[VarIndex].dims[0] <= 0) {      // Not an array
                error("Argument 2 must be integer array");
            }
            src = (int64_t *)ptr2;
            p=(char *)&src[1];
            i = s = src[0];
        } else error("Argument 2 must be integer array");
        if(j*8 < (d+s))error("Destination array too small");
        q+=d;
        while(i--)*q++=*p++;
        dest[0]+=src[0];
        return;
    }
    error("Invalid option");
}
void fun_LGetStr(void){
        void *ptr1 = NULL;
        char *p;
        char *s=NULL;
        int64_t *src=NULL;
        int start,nbr,j;
        getargs(&ep, 5, ",");
        if(argc != 5)error("Argument count");
        ptr1 = findvar(argv[0], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(vartbl[VarIndex].type & T_INT) {
            if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
            if(vartbl[VarIndex].dims[0] <= 0) {      // Not an array
                error("Argument 1 must be integer array");
            }
            src = (int64_t *)ptr1;
            s=(char *)&src[1];
        } else error("Argument 1 must be integer array");
        j=(vartbl[VarIndex].dims[0] - OptionBase)*8;
        start = getint(argv[2],1,j);
    nbr = getinteger(argv[4]);
    if(nbr < 1 || nbr > MAXSTRLEN) error("Number out of bounds");
        if(start+nbr>src[0])nbr=src[0]-start+1;
    sret = GetTempMemory(STRINGSIZE);                                       // this will last for the life of the command
        s+=(start-1);
        p=sret+1;
        *sret=nbr;
        while(nbr--)*p++=*s++;
        *p=0;
        targ = T_STR;
}

void fun_LGetByte(void){
        void *ptr1 = NULL;
        uint8_t *s=NULL;
        int64_t *src=NULL;
        int start,j;
    	getargs(&ep, 3, ",");
        if(argc != 3)error("Argument count");
        ptr1 = findvar(argv[0], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(vartbl[VarIndex].type & T_INT) {
            if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
            if(vartbl[VarIndex].dims[0] <= 0) {		// Not an array
                error("Argument 1 must be integer array");
            }
            src = (int64_t *)ptr1;
            s=(uint8_t *)&src[1];
        } else error("Argument 1 must be integer array");
        j=(vartbl[VarIndex].dims[0] - OptionBase)*8-1;
        start = getint(argv[2],OptionBase,j-OptionBase);
        iret=s[start-OptionBase];
        targ = T_INT;
}


void fun_LInstr(void){
        void *ptr1 = NULL;
        int64_t *dest=NULL;
        char *srch;
        char *str=NULL;
        int slen,found=0,i,j,n;
        getargs(&ep, 5, ",");
        if(argc <3  || argc > 5)error("Argument count");
        int64_t start;
        if(argc==5)start=getinteger(argv[4])-1;
        else start=0;
        ptr1 = findvar(argv[0], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(vartbl[VarIndex].type & T_INT) {
            if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
            if(vartbl[VarIndex].dims[0] <= 0) {      // Not an array
                error("Argument 1 must be integer array");
            }
            dest = (long long int *)ptr1;
            str=(char *)&dest[0];
        } else error("Argument 1 must be integer array");
        j=(vartbl[VarIndex].dims[0] - OptionBase);
        srch=getstring(argv[2]);
        slen=*srch;
        iret=0;
        if(start>dest[0] || start<0 || slen==0 || dest[0]==0 || slen>dest[0]-start)found=1;
        if(!found){
            n=dest[0]- slen - start;

            for(i = start; i <= n + start; i++) {
                if(str[i + 8] == srch[1]) {
                    for(j = 0; j < slen; j++)
                        if(str[j + i + 8] != srch[j + 1])
                            break;
                    if(j == slen) {iret= i + 1; break;}
                }
            }
        }
        targ = T_INT;
}

void fun_LCompare(void){
        void *ptr1 = NULL;
        void *ptr2 = NULL;
        int64_t *dest, *src;
        char *p=NULL;
        char *q=NULL;
        int d=0,s=0,found=0;
        getargs(&ep, 3, ",");
        if(argc != 3)error("Argument count");
        ptr1 = findvar(argv[0], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(vartbl[VarIndex].type & T_INT) {
            if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
            if(vartbl[VarIndex].dims[0] <= 0) {      // Not an array
                error("Argument 1 must be integer array");
            }
            dest = (int64_t *)ptr1;
            q=(char *)&dest[1];
            d=dest[0];
        } else error("Argument 1 must be integer array");
        ptr2 = findvar(argv[2], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(vartbl[VarIndex].type & T_INT) {
            if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
            if(vartbl[VarIndex].dims[0] <= 0) {      // Not an array
                error("Argument 2 must be integer array");
            }
            src = (int64_t *)ptr2;
            p=(char *)&src[1];
            s=src[0];
        } else error("Argument 2 must be integer array");
    while(!found) {
        if(d == 0 && s == 0) {found=1;iret=0;}
        if(d == 0 && !found) {found=1;iret=-1;}
        if(s == 0 && !found) {found=1;iret=1;}
        if(*q < *p && !found) {found=1;iret=-1;}
        if(*q > *p && !found) {found=1;iret=1;}
        q++;  p++;  d--; s--;
    }
        targ = T_INT;
}

void fun_LLen(void) {
    void *ptr1 = NULL;
    int64_t *dest=NULL;
    getargs(&ep, 1, ",");
    if(argc != 1)error("Argument count");
    ptr1 = findvar(argv[0], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
    if(vartbl[VarIndex].type & T_INT) {
        if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
        if(vartbl[VarIndex].dims[0] <= 0) {      // Not an array
            error("Argument 1 must be integer array");
        }
        dest = (long long int *)ptr1;
    } else error("Argument 1 must be integer array");
    iret=dest[0];
    targ = T_INT;
}



void update_clock(void){
/*    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;
    sTime.Hours = hour;
    sTime.Minutes = minute;
    sTime.Seconds = second;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
    {
        error("RTC hardware error");
    }
    sDate.WeekDay = day_of_week;
    sDate.Month = month;
    sDate.Date = day;
    sDate.Year = year-2000;

    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
    {
        error("RTC hardware error");
    }*/
}


// this is invoked as a command (ie, date$ = "6/7/2010")
// search through the line looking for the equals sign and step over it,
// evaluate the rest of the command, split it up and save in the system counters
void cmd_date(void) {
	unsigned char *arg;
	struct tm  *tm;
	struct tm tma;
	tm=&tma;
	int dd, mm, yy;
	while(*cmdline && tokenfunction(*cmdline) != op_equal) cmdline++;
	if(!*cmdline) error("Syntax");
	++cmdline;
	arg = getCstring(cmdline);
	{
		getargs(&arg, 5, "-/");										// this is a macro and must be the first executable stmt in a block
		if(argc != 5) error("Syntax");
		dd = atoi(argv[0]);
		mm = atoi(argv[2]);
		yy = atoi(argv[4]);
        if(dd>1000){
            int tmp=dd;
            dd=yy;
            yy=tmp;
        }
		if(yy >= 0 && yy < 100) yy += 2000;
	    //check year
	    if(yy>=1900 && yy<=9999)
	    {
	        //check month
	        if(mm>=1 && mm<=12)
	        {
	            //check days
	            if((dd>=1 && dd<=31) && (mm==1 || mm==3 || mm==5 || mm==7 || mm==8 || mm==10 || mm==12))
	                {}
	            else if((dd>=1 && dd<=30) && (mm==4 || mm==6 || mm==9 || mm==11))
	                {}
	            else if((dd>=1 && dd<=28) && (mm==2))
	                {}
	            else if(dd==29 && mm==2 && (yy%400==0 ||(yy%4==0 && yy%100!=0)))
	                {}
	            else
	                error("Day is invalid");
	        }
	        else
	        {
	            error("Month is not valid");
	        }
	    }
	    else
	    {
	        error("Year is not valid");
	    }

		mT4IntEnable(0);       										// disable the timer interrupt to prevent any conflicts while updating
		day = dd;
		month = mm;
		year = yy;
	    tm->tm_year = year - 1900;
	    tm->tm_mon = month - 1;
	    tm->tm_mday = day;
	    tm->tm_hour = hour;
	    tm->tm_min = minute;
	    tm->tm_sec = second;
	    time_t timestamp = timegm(tm); /* See README.md if your system lacks timegm(). */
	    tm=gmtime(&timestamp);
	    day_of_week=tm->tm_wday;
	    if(day_of_week==0)day_of_week=7;
		update_clock();
		mT4IntEnable(1);       										// enable interrupt
	}
}

// this is invoked as a function
void fun_date(void) {
    sret = GetTempMemory(STRINGSIZE);                                    // this will last for the life of the command
    IntToStrPad(sret, day, '0', 2, 10);
    sret[2] = '-'; IntToStrPad(sret + 3, month, '0', 2, 10);
    sret[5] = '-'; IntToStr(sret + 6, year, 10);
    CtoM(sret);
    targ = T_STR;
}

// this is invoked as a function
void fun_day(void) {
    unsigned char *arg;
    struct tm  *tm;
    struct tm tma;
    tm=&tma;
    time_t time_of_day;
    int i;
    sret = GetTempMemory(STRINGSIZE);                                    // this will last for the life of the command
    int d, m, y;
    if(!checkstring(ep, "NOW"))
    {
        arg = getCstring(ep);
        getargs(&arg, 5, "-/");                                     // this is a macro and must be the first executable stmt in a block
        if(!(argc == 5))error("Syntax");
        d = atoi(argv[0]);
        m = atoi(argv[2]);
        y = atoi(argv[4]);
		if(d>1000){
			int tmp=d;
			d=y;
			y=tmp;
		}
        if(y >= 0 && y < 100) y += 2000;
        if(d < 1 || d > 31 || m < 1 || m > 12 || y < 1902 || y > 2999) error("Invalid date");
        tm->tm_year = y - 1900;
        tm->tm_mon = m - 1;
        tm->tm_mday = d;
        tm->tm_hour = 0;
        tm->tm_min = 0;
        tm->tm_sec = 0;
        time_of_day = timegm(tm);
        tm=gmtime(&time_of_day);
        i=tm->tm_wday;
        if(i==0)i=7;
        strcpy(sret,daystrings[i]);
    } else {
        tm->tm_year = year - 1900;
        tm->tm_mon = month - 1;
        tm->tm_mday = day;
        tm->tm_hour = 0;
        tm->tm_min = 0;
        tm->tm_sec = 0;
        time_of_day = timegm(tm);
        tm=gmtime(&time_of_day);
        i=tm->tm_wday;
        if(i==0)i=7;
        strcpy(sret,daystrings[i]);
    }
    CtoM(sret);
    targ = T_STR;
}

// this is invoked as a command (ie, time$ = "6:10:45")
// search through the line looking for the equals sign and step over it,
// evaluate the rest of the command, split it up and save in the system counters
void cmd_time(void) {
	unsigned char *arg;
	int h = 0;
	int m = 0;
	int s = 0;
    MMFLOAT f;
    long long int i64;
    unsigned char *ss;
    int t=0;
    int offset;
	while(*cmdline && tokenfunction(*cmdline) != op_equal) cmdline++;
	if(!*cmdline) error("Syntax");
	++cmdline;
    evaluate(cmdline, &f, &i64, &ss, &t, false);
	if(t & T_STR){
	arg = getCstring(cmdline);
	{
		getargs(&arg, 5, ":");								// this is a macro and must be the first executable stmt in a block
		if(argc%2 == 0) error("Syntax");
		h = atoi(argv[0]);
		if(argc >= 3) m = atoi(argv[2]);
		if(argc == 5) s = atoi(argv[4]);
		if(h < 0 || h > 23 || m < 0 || m > 59 || s < 0 || s > 59) error("Invalid time");
		mT4IntEnable(0);       										// disable the timer interrupt to prevent any conflicts while updating
		hour = h;
		minute = m;
		second = s;
		SecondsTimer = 0;
		update_clock();
    	mT4IntEnable(1);       										// enable interrupt
    }
	} else {
		struct tm  *tm;
		struct tm tma;
		tm=&tma;
		offset=getinteger(cmdline);
		tm->tm_year = year - 1900;
		tm->tm_mon = month - 1;
		tm->tm_mday = day;
		tm->tm_hour = hour;
		tm->tm_min = minute;
		tm->tm_sec = second;
	    time_t timestamp = timegm(tm); /* See README.md if your system lacks timegm(). */
	    timestamp+=offset;
	    tm=gmtime(&timestamp);
		mT4IntEnable(0);       										// disable the timer interrupt to prevent any conflicts while updating
		hour = tm->tm_hour;
		minute = tm->tm_min;
		second = tm->tm_sec;
		SecondsTimer = 0;
		update_clock();
    	mT4IntEnable(1);       										// enable interrupt
	}
}




// this is invoked as a function
void fun_time(void) {
    sret = GetTempMemory(STRINGSIZE);                                  // this will last for the life of the command
     IntToStrPad(sret, hour, '0', 2, 10);
    sret[2] = ':'; IntToStrPad(sret + 3, minute, '0', 2, 10);
    sret[5] = ':'; IntToStrPad(sret + 6, second, '0', 2, 10);
    CtoM(sret);
    targ = T_STR;
}



void cmd_ireturn(void){
    if(InterruptReturn == NULL) error("Not in interrupt");
    checkend(cmdline);
    nextstmt = InterruptReturn;
    InterruptReturn = NULL;
    if(LocalIndex)    ClearVars(LocalIndex--);                        // delete any local variables
    TempMemoryIsChanged = true;                                     // signal that temporary memory should be checked
    *CurrentInterruptName = 0;                                        // for static vars we are not in an interrupt
#ifndef PICOMITEVGA
#ifndef PICOMITEWEB
    if(DelayedDrawKeyboard) {
        DrawKeyboard(1);                                            // the pop-up GUI keyboard should be drawn AFTER the pen down interrupt
        DelayedDrawKeyboard = false;
    }
    if(DelayedDrawFmtBox) {
        DrawFmtBox(1);                                              // the pop-up GUI keyboard should be drawn AFTER the pen down interrupt
        DelayedDrawFmtBox = false;
    }
#endif
#endif
	if(SaveOptionErrorSkip>0)OptionErrorSkip=SaveOptionErrorSkip+1;
    strcpy(MMErrMsg , SaveErrorMessage);
    MMerrno = Saveerrno;
}

void cmd_library(void) {  
            
    /********************************************************************************************************************
     ******* LIBRARY SAVE **********************************************************************************************/
    if(checkstring(cmdline, "SAVE")) {  
        int pageno=0;  
        unsigned char *p,  *pp , *m, *MemBuff, *TempPtr, rem;
        int i, j, k, InCFun, InQuote, CmdExpected;
        unsigned int CFunDefAddr[100], *CFunHexAddr[100] , *CFunHexLibAddr[100] ;
        if(CurrentLinePtr) error("Invalid in a program");
        if(*ProgMemory != 0x01) return;
        checkend(p);
        ClearRuntime();
        TempPtr = m = MemBuff = GetTempMemory(EDIT_BUFFER_SIZE);

        rem = GetCommandValue("Rem");
        InQuote = InCFun = j = 0;
        CmdExpected = true;
       if(Option.LIBRARY_FLASH_SIZE != MAX_PROG_SIZE){
           uint32_t *c = (uint32_t *)(flash_progmemory - MAX_PROG_SIZE);
           if (*c != 0xFFFFFFFF)
            error("Flash Slot % already in use",MAXFLASHSLOTS);
        ;
       }
        // first copy the current program code residing in the Library area to RAM
        if(Option.LIBRARY_FLASH_SIZE == MAX_PROG_SIZE){
            p = ProgMemory - Option.LIBRARY_FLASH_SIZE;
            while(!(p[0] == 0 && p[1] == 0)) *m++ = *p++;
              *m++ = 0;                                               // terminate the last line
        }
        //dump(m, 256);
        // then copy the current contents of the program memory to RAM
        
       //MMPrintString("\r\n Size=1 ");PInt(m - MemBuff);
        p = ProgMemory;
        while(*p != 0xff) {
            if(p[0] == 0 && p[1] == 0) break;                       // end of the program

            if(*p == T_NEWLINE) {
                TempPtr = m;
                CurrentLinePtr = p;
                *m++ = *p++;
                CmdExpected = true;                                 // if true we can expect a command next (possibly a CFunction, etc)
                if(*p == 0) {                                       // if this is an empty line we can skip it
                    p++;
                    if(*p == 0) break;                              // end of the program or module
                    m--;
                    continue;
                }
            }

            if(*p == T_LINENBR) {
//                TempPtr = m;
                *m++ = *p++; *m++ = *p++; *m++ = *p++;              // copy the line number
                skipspace(p);
            }

            if(*p == T_LABEL) {
                for(i = p[1] + 2; i > 0; i--) *m++ = *p++;          // copy the label
//                TempPtr = m;
                skipspace(p);
            }
            //if(CmdExpected && ( *p == GetCommandValue("End CFunction") || *p == GetCommandValue("End CSub") || *p == GetCommandValue("End DefineFont"))) {
            if(CmdExpected && (  *p == GetCommandValue("End CSub") || *p == GetCommandValue("End DefineFont"))) {
                InCFun = false;                                     // found an  END CSUB or END DEFINEFONT token
            }
            if(InCFun) {
                skipline(p);                                        // skip the body of a CFunction
                m = TempPtr;                                        // don't copy the new line
                continue;
            }

            if(CmdExpected && ( *p == cmdCSUB || *p == GetCommandValue("DefineFont"))) {    // found a  CSUB or DEFINEFONT token
                CFunDefAddr[++j] = (unsigned int)m;                 // save its address so that the binary copy in the library can point to it
                while(*p) *m++ = *p++;                              // copy the declaration
                InCFun = true;
            }

            if(CmdExpected && *p == rem) {                          // found a REM token
                skipline(p);
                m = TempPtr;                                        // don't copy the new line tokens
                continue;
            }

            if(*p >= C_BASETOKEN || isnamestart(*p))
                CmdExpected = false;                                // stop looking for a CFunction, etc on this line

            if(*p == '"') InQuote = !InQuote;                       // found the start/end of a string

            if(*p == '\'' && !InQuote) {                            // found a modern remark char
                skipline(p);
                //PIntHC(*(m-3)); PIntHC(*(m-2)); PIntHC(*(m-1)); PIntHC(*(m));
                //MMPrintString("\r\n");
                //if(*(m-3) == 0x01) {        //Original condition from Micromites
                /* Check to see if comment is the first thing on the line or its only preceded by spaces.
                Spaces have been reduced to a single space so we treat a comment with 1 space before it
                as a comment line to be omitted.
                */    
                if((*(m-1) == 0x01) ||  ((*(m-2) == 0x01) && (*(m-1) == 0x20))){    
                    m = TempPtr;                                    // if the comment was the only thing on the line don't copy the line at all
                    continue;
                } else
                    p--;
            }

            if(*p == ' ' && !InQuote) {                             // found a space
                if(*(m-1) == ' ') m--;                              // don't copy the space if the preceeding char was a space
            }

            if(p[0] == 0 && p[1] == 0) break;                       // end of the program
            *m++ = *p++;
        }
      
       // MMPrintString("\r\n Size2= ");PInt(m - MemBuff);
       //The picomite will have any fonts or CSUBs binary starting on a new 256 byte block so there can be many 
       //0x00 bytes at the end of the program. We only need two of them .
        // At the end of the program so get the two 0x00 bytes
        *m++ = *p++; 
        *m++ = *p++; 
        // Step the program memory up to the first 0xFF of the 4 that mark the beginning of the CSub binaries.
        while(*p != 0xff) *p++; 
        p++;p++; p++;p++;                                           // step over the header of the four 0xff bytes
                                    
        //step the memory to the next 4 word boundary
        // while((unsigned int)p & 0b11) p++;
        while((unsigned int)m & 0b11) *m++ = 0x00;                  // step memory to the next word boundary
        *m++=0xFF;*m++=0xFF;*m++=0xFF;*m++=0xFF;                    //write 4 byte of the csub binary header 
        
     
        // now copy the CFunction/CSub/Font data
        // =====================================
        // the format of a CFunction in flash is:
        //   Unsigned Int - Address of the CFunction/CSub/Font in program memory (points to the token).  For a font it is zero.
        //   Unsigned Int - The length of the CFunction/CSub/Font in bytes including the Offset (see below)
        //   Unsigned Int - The Offset (in words) to the main() function (ie, the entry point to the CFunction/CSub).  The specs for the font if it is a font.
        //   word1..wordN - The CFunction code
        // The next CFunction starts immediately following the last word of the previous CFunction

        // first, copy any CFunctions residing in the library area to RAM
       // MMPrintString("\r\n Copying CSUBS from library");
        k = 0;                                                      // this is used to index CFunHexLibAddr[] for later adjustment of a CFun's address
        if(CFunctionLibrary != NULL) {
            pp = (char *)CFunctionLibrary;
            while(*(unsigned int *)pp != 0xffffffff) {
                CFunHexLibAddr[++k] = (unsigned int *)m;            // save the address for later adjustment
                j = (*(unsigned int *)(pp + 4)) + 8;                // calculate the total size of the CFunction
                while(j--) *m++ = *pp++;                            // copy it into RAM
            }
        }
      
        // then, copy any CFunctions in program memory to RAM
        
        i = 0;                                                      // this is used to index CFunHexAddr[] for later adjustment of a CFun's address
       // while(*(unsigned int *)p != 0xffffffff && (int)p < Option.PROG_FLASH_SIZE) {
        while(*(unsigned int *)p != 0xffffffff) {    
            CFunHexAddr[++i] = (unsigned int *)m;                   // save the address for later adjustment
            j = (*(unsigned int *)(p + 4)) + 8;                     // calculate the total size of the CFunction
            while(j--) *m++ = *p++;                                 // copy it into RAM
        }
        // we have now copied all the CFunctions into RAM

        // calculate the size of the library code  to  end on a word boundary
        j=(((m - MemBuff) + (0x4 - 1)) & (~(0x4 - 1)));
       
         //We only have reserved MAX_PROG_SIZE of flash for library code .
        //Error if we try to use too much
        if (j > MAX_PROG_SIZE) error("Library too big");
              
        TempPtr = (LibMemory);
     
        // now adjust the addresses of the declaration in each CFunction header
        // do not adjust a font who's "address" is  fontno-1.
        // NO ADJUSTMENT REQUIRED FOR PICOMITE as ADDRESS is RELATIVE and LIBRARY is at a fixed location.
        // first, CFunctions that were already in the library
        //for(; k > 0; k--) {
             //if ((*CFunHexLibAddr[k]>>31)==0)  *CFunHexLibAddr[k] -= ((unsigned int)( MAX_PROG_SIZE);
        //}

        // then, CFunctions that are being added to the library
        for(; i > 0; i--) {
          if ((*CFunHexAddr[i]>>31)==0)  *CFunHexAddr[i] = (CFunDefAddr[i] - (unsigned int)MemBuff);
        }

       
  //******************************************************************************
        //now write the library from ram to the library flash area
        // initialise for writing to the flash
        FlashWriteInit(LIBRARY_FLASH);
        flash_range_erase(realflashpointer, MAX_PROG_SIZE);
        i=MAX_PROG_SIZE/4;
       
        int *ppp=(int *)(flash_progmemory - MAX_PROG_SIZE);
        while(i--)if(*ppp++ != 0xFFFFFFFF){
            enable_interrupts();
            error("Flash erase problem");
        }
   
        i=0;
        for(k = 0; k < m - MemBuff; k++){        // write to the flash byte by byte
           FlashWriteByte(MemBuff[k]);
        }
        FlashWriteClose();
        Option.LIBRARY_FLASH_SIZE = MAX_PROG_SIZE;
        SaveOptions();

        if(MMCharPos > 1) MMPrintString("\r\n");                    // message should be on a new line
        MMPrintString("Library Saved ");
        IntToStr(tknbuf, k, 10);
        MMPrintString(tknbuf);
        MMPrintString(" bytes\r\n");
        fflush(stdout);
        uSec(2000);
     

        //Now call the new command that will clear the current program memory then
        //write the library code at Option.ProgFlashSize by copying it from the windbond
        //and return to the command prompt.
        cmdline = ""; CurrentLinePtr = NULL;    // keep the NEW command happy
        cmd_new();                              //  delete the program,add the library code and return to the command prompt
    }
     /********************************************************************************************************************
     ******* LIBRARY DELETE **********************************************************************************************/

     if(checkstring(cmdline, "DELETE")) {
        int i;
        if(CurrentLinePtr) error("Invalid in a program");
        if(Option.LIBRARY_FLASH_SIZE != MAX_PROG_SIZE) return;
        
        FlashWriteInit(LIBRARY_FLASH);
        flash_range_erase(realflashpointer, MAX_PROG_SIZE);
        i=MAX_PROG_SIZE/4;
       
        int *ppp=(int *)(flash_progmemory - MAX_PROG_SIZE);
        while(i--)if(*ppp++ != 0xFFFFFFFF){
            enable_interrupts();
            error("Flash erase problem");
        }
        enable_interrupts();

        Option.LIBRARY_FLASH_SIZE= 0;
        SaveOptions();
        return;
        // Clear Program Memory and also the Library at the end.
//        cmdline = ""; CurrentLinePtr = NULL;    // keep the NEW command happy
//        cmd_new();                              //  delete any program,and the library code and return to the command prompt
        
     }

      /********************************************************************************************************************
      ******* LIBRARY LIST **********************************************************************************************/

     if(checkstring(cmdline, "LIST ALL")) {
        if(CurrentLinePtr) error("Invalid in a program");
        if(Option.LIBRARY_FLASH_SIZE != MAX_PROG_SIZE) return;
        ListProgram(ProgMemory - Option.LIBRARY_FLASH_SIZE, true);
        return;
     }
     if(checkstring(cmdline, "LIST")) {
        if(CurrentLinePtr) error("Invalid in a program");
        if(Option.LIBRARY_FLASH_SIZE != MAX_PROG_SIZE) return;
        ListProgram(ProgMemory - Option.LIBRARY_FLASH_SIZE, false);
        return;
     }
  
     error("Invalid syntax");
    }
     

// set up the tick interrupt
void cmd_settick(void){
    int period;
    int irq=0;;
    int pause=0;
    char *s=GetTempMemory(STRINGSIZE);
    getargs(&cmdline, 5, ",");
    strcpy(s,argv[0]);
    if(!(argc == 3 || argc == 5)) error("Argument count");
    if(argc == 5) irq = getint(argv[4], 1, NBRSETTICKS) - 1;
    if(strcasecmp(argv[0],"PAUSE")==0){
        TickActive[irq]=0;
        return;
    } else if(strcasecmp(argv[0],"RESUME")==0){
        TickActive[irq]=1;
        return;
    } else period = getint(argv[0], -1, INT_MAX);
    if(period == 0) {
        TickInt[irq] = NULL;                                        // turn off the interrupt
        TickPeriod[irq] = 0;
        TickTimer[irq] = 0;                                         // set the timer running
        TickActive[irq]=0;
    } else {
        TickPeriod[irq] = period;
        TickInt[irq] = GetIntAddress(argv[2]);                      // get a pointer to the interrupt routine
        TickTimer[irq] = 0;                                         // set the timer running
        InterruptUsed = true;
        TickActive[irq]=1;

    }
}
void PO(char *s) {
    MMPrintString("OPTION "); MMPrintString(s); MMPrintString(" ");
}

void PO2Str(char *s1, const char *s2) {
    PO(s1); MMPrintString((char *)s2); MMPrintString("\r\n");
}
void PO3Str(char *s1, const char *s2, const char *s3) {
    PO(s1); MMPrintString((char *)s2); MMPrintString(", ");MMPrintString((char *)s3); MMPrintString("\r\n");
}


void PO2Int(char *s1, int n) {
    PO(s1); PInt(n); MMPrintString("\r\n");
}
void PO2IntH(char *s1, int n) {
    PO(s1); PIntH(n); MMPrintString("\r\n");
}
void PO3Int(char *s1, int n1, int n2) {
    PO(s1); PInt(n1); PIntComma(n2); MMPrintString("\r\n");
}
void PO4Int(char *s1, int n1, int n2, int n3) {
    PO(s1); PInt(n1); PIntComma(n2);  PIntComma(n3);  MMPrintString("\r\n");
}
void PO5Int(char *s1, int n1, int n2, int n3, int n4) {
    PO(s1); PInt(n1); PIntComma(n2);  PIntComma(n3);  PIntComma(n4); MMPrintString("\r\n");
}

void printoptions(void){
//	LoadOptions();
    int i=Option.DISPLAY_ORIENTATION;
#ifdef PICOMITEVGA
    MMPrintString("\rPicoMiteVGA MMBasic Version " VERSION "\r\n");
#endif    
#ifdef PICOMITEWEB
    MMPrintString("\rWebMite MMBasic Version " VERSION "\r\n");    
#endif 
#ifdef PICOMITE
    MMPrintString("\rPicoMite MMBasic Version " VERSION "\r\n");
#endif     

    if(Option.SerialConsole){
        MMPrintString("OPTION SERIAL CONSOLE COM");
        MMputchar((Option.SerialConsole & 3)+48,1);
        MMputchar(',',1);
        MMPrintString((char *)PinDef[Option.SerialTX].pinname);MMputchar(',',1);
        MMPrintString((char *)PinDef[Option.SerialRX].pinname);
        if(Option.SerialConsole & 4)MMPrintString((char *)",BOTH");
        PRet();
    }
    if(Option.SYSTEM_CLK){
        PO("SYSTEM SPI");
        MMPrintString((char *)PinDef[Option.SYSTEM_CLK].pinname);MMputchar(',',1);;
        MMPrintString((char *)PinDef[Option.SYSTEM_MOSI].pinname);MMputchar(',',1);;
        MMPrintString((char *)PinDef[Option.SYSTEM_MISO].pinname);MMPrintString("\r\n");
    }
    if(Option.SYSTEM_I2C_SDA){
        PO("SYSTEM I2C");
        MMPrintString((char *)PinDef[Option.SYSTEM_I2C_SDA].pinname);MMputchar(',',1);
        MMPrintString((char *)PinDef[Option.SYSTEM_I2C_SCL].pinname);
        if(Option.SYSTEM_I2C_SLOW)MMPrintString(", SLOW\r\n");
        else PRet();
    }
    if(Option.Autorun>0 && Option.Autorun<=MAXFLASHSLOTS) PO2Int("AUTORUN", Option.Autorun);
    if(Option.Autorun==MAXFLASHSLOTS+1)PO2Str("AUTORUN", "ON");
    if(Option.Baudrate != CONSOLE_BAUDRATE) PO2Int("BAUDRATE", Option.Baudrate);
    if(Option.FlashSize !=2048*1024) PO2Int("FLASH SIZE", Option.FlashSize);
    if(MAX_PROG_SIZE == Option.LIBRARY_FLASH_SIZE) PO2IntH("LIBRARY_FLASH_SIZE ", Option.LIBRARY_FLASH_SIZE);
    if(Option.Invert == true) PO2Str("CONSOLE", "INVERT");
    if(Option.Invert == 2) PO2Str("CONSOLE", "AUTO");
    if(Option.ColourCode == true) PO2Str("COLOURCODE", "ON");
    if(Option.PWM == true) PO2Str("POWER PWM", "ON");
    if(Option.Listcase != CONFIG_TITLE) PO2Str("CASE", CaseList[(int)Option.Listcase]);
    if(Option.Tab != 2) PO2Int("TAB", Option.Tab);
    if(!(Option.KeyboardConfig == NO_KEYBOARD ||Option.KeyboardConfig == CONFIG_I2C)){
        PO("KEYBOARD"); MMPrintString((char *)KBrdList[(int)Option.KeyboardConfig]); 
        if(Option.capslock || Option.numlock!=1 || Option.repeat!=0b00101100){
            PIntComma(Option.capslock);PIntComma(Option.numlock);PIntComma(Option.repeat>>5);
            PIntComma(Option.repeat & 0x1f);
        }
        PRet();
    } 
    if(Option.KeyboardConfig == CONFIG_I2C)PO2Str("KEYBOARD", "I2C");
    if(Option.NoHeartbeat)PO2Str("HEARTBEAT", "OFF");
#ifdef PICOMITEVGA
    if(Option.CPU_Speed!=126000)PO2Int("CPUSPEED (KHz)", Option.CPU_Speed);
    if(Option.DISPLAY_TYPE==COLOURVGA)PO2Str("DEFAULT MODE", "2");
    if(Option.Height != 40 || Option.Width != 80) PO3Int("DISPLAY", Option.Height, Option.Width);
    if(Option.X_TILE==40)PO2Str("TILE SIZE", "LARGE");
    if(Option.VGAFC !=0xFFFF || Option.VGABC !=0){
        PO("DEFAULT COLOURS");
            if(Option.VGAFC==0xFFFF)MMPrintString("WHITE, ");
            else if(Option.VGAFC==0xEEEE)MMPrintString("YELLOW,");
            else if(Option.VGAFC==0xDDDD)MMPrintString("LILAC,");
            else if(Option.VGAFC==0xCCCC)MMPrintString("BROWN,");
            else if(Option.VGAFC==0xBBBB)MMPrintString("FUSCHIA,");
            else if(Option.VGAFC==0xAAAA)MMPrintString("RUST, ");
            else if(Option.VGAFC==0x9999)MMPrintString("MAGENTA,");
            else if(Option.VGAFC==0x8888)MMPrintString("RED,");
            else if(Option.VGAFC==0x7777)MMPrintString("CYAN,");
            else if(Option.VGAFC==0x6666)MMPrintString("GREEN,");
            else if(Option.VGAFC==0x5555)MMPrintString("CERULEAN,");
            else if(Option.VGAFC==0x4444)MMPrintString("MIDGREEN,");
            else if(Option.VGAFC==0x3333)MMPrintString("COBALT,");
            else if(Option.VGAFC==0x2222)MMPrintString("MYRTLE,");
            else if(Option.VGAFC==0x1111)MMPrintString("BLUE,");
            else if(Option.VGAFC==0x0000)MMPrintString("BLACK,");
            if(Option.VGABC==0xFFFF)MMPrintString(" WHITE");
            else if(Option.VGABC==0xEEEE)MMPrintString(" YELLOW");
            else if(Option.VGABC==0xDDDD)MMPrintString(" LILAC");
            else if(Option.VGABC==0xCCCC)MMPrintString(" BROWN");
            else if(Option.VGABC==0xBBBB)MMPrintString(" FUSCHIA");
            else if(Option.VGABC==0xAAAA)MMPrintString(" RUST");
            else if(Option.VGABC==0x9999)MMPrintString(" MAGENTA");
            else if(Option.VGABC==0x8888)MMPrintString(" RED");
            else if(Option.VGABC==0x7777)MMPrintString(" CYAN");
            else if(Option.VGABC==0x6666)MMPrintString(" GREEN");
            else if(Option.VGABC==0x5555)MMPrintString(" CERULEAN");
            else if(Option.VGABC==0x4444)MMPrintString(" MIDGREEN");
            else if(Option.VGABC==0x3333)MMPrintString(" COBALT");
            else if(Option.VGABC==0x2222)MMPrintString(" MYRTLE");
            else if(Option.VGABC==0x1111)MMPrintString(" BLUE");
            else if(Option.VGABC==0x0000)MMPrintString(" BLACK");
        PRet();
    }
#else
    if(Option.CPU_Speed!=133000)PO2Int("CPUSPEED (KHz)", Option.CPU_Speed);
    if(Option.DISPLAY_CONSOLE == true) PO2Str("LCDPANEL", "CONSOLE");
    if(Option.Height != 24 || Option.Width != 80) PO3Int("DISPLAY", Option.Height, Option.Width);
    if(Option.DISPLAY_TYPE == DISP_USER) PO3Int("LCDPANEL USER", HRes, VRes);
    if(Option.DISPLAY_TYPE > I2C_PANEL && Option.DISPLAY_TYPE < DISP_USER) {
        i=Option.DISPLAY_ORIENTATION;
        if(Option.DISPLAY_TYPE==ST7789 || Option.DISPLAY_TYPE == ST7789A)i=(i+2) % 4;
        PO("LCDPANEL"); MMPrintString((char *)display_details[Option.DISPLAY_TYPE].name); MMPrintString(", "); MMPrintString((char *)OrientList[(int)i - 1]);
        MMputchar(',',1);;MMPrintString((char *)PinDef[Option.LCD_CD].pinname);
        MMputchar(',',1);;MMPrintString((char *)PinDef[Option.LCD_Reset].pinname);
        if(Option.DISPLAY_TYPE!=ST7920){
            MMputchar(',',1);;MMPrintString((char *)PinDef[Option.LCD_CS].pinname);
        }
        if(!(Option.DISPLAY_TYPE<=I2C_PANEL || Option.DISPLAY_TYPE>=BufferedPanel ) && Option.DISPLAY_BL){
            MMputchar(',',1);MMPrintString((char *)PinDef[Option.DISPLAY_BL].pinname);
        }
        if(Option.DISPLAY_TYPE==SSD1306SPI && Option.I2Coffset)PIntComma(Option.I2Coffset);
        if(Option.DISPLAY_TYPE==N5110 && Option.LCDVOP!=0xC8)PIntComma(Option.LCDVOP);
        MMPrintString("\r\n");
    }
    if(Option.DISPLAY_TYPE > 0 && Option.DISPLAY_TYPE <= I2C_PANEL) {
        PO("LCDPANEL"); MMPrintString((char *)display_details[Option.DISPLAY_TYPE].name); MMPrintString(", "); MMPrintString((char *)OrientList[(int)i - 1]);
        if(Option.DISPLAY_TYPE==SSD1306I2C && Option.I2Coffset)PIntComma(Option.I2Coffset);
        MMPrintString("\r\n");
    }
    if(Option.DISPLAY_TYPE >= SSDPANEL && Option.DISPLAY_TYPE<VIRTUAL) {
        PO("LCDPANEL"); MMPrintString((char *)display_details[Option.DISPLAY_TYPE].name); MMPrintString(", "); MMPrintString((char *)OrientList[(int)i - 1]);
		if(Option.DISPLAY_BL){
            MMputchar(',',1);MMPrintString((char *)PinDef[Option.DISPLAY_BL].pinname);
		}
        PRet();
    }
    if(Option.DISPLAY_TYPE >= VIRTUAL){
        PO("LCDPANEL"); MMPrintString((char *)display_details[Option.DISPLAY_TYPE].name); PRet();
    } 
    #ifdef PICOMITE
    if(Option.MaxCtrls)PO2Int("GUI CONTROLS", Option.MaxCtrls);
    #endif
    #ifdef PICOMITEWEB
    if(*Option.SSID){
        char password[]="****************************************************************";
        password[strlen(Option.PASSWORD)]=0;
        PO3Str("WIFI",Option.SSID,password);
    }
    if(Option.TCP_PORT && Option.ServerResponceTime!=5000)PO3Int("TCP SERVER PORT", Option.TCP_PORT, Option.ServerResponceTime);
    if(Option.TCP_PORT && Option.ServerResponceTime==5000)PO2Int("TCP SERVER PORT", Option.TCP_PORT);
    if(Option.Telnet==1)PO2Str("TELNET", "CONSOLE ON");
    if(Option.Telnet==-1)PO2Str("TELNET", "CONSOLE ONLY");
    if(Option.disabletftp==1)PO2Str("TFTP", "OFF");
    #endif
    if(Option.TOUCH_CS) {
        PO("TOUCH"); 
        MMPrintString((char *)PinDef[Option.TOUCH_CS].pinname);MMputchar(',',1);;
        MMPrintString((char *)PinDef[Option.TOUCH_IRQ].pinname);
        if(Option.TOUCH_Click) {
            MMputchar(',',1);;MMPrintString((char *)PinDef[Option.TOUCH_Click].pinname);
        }
        MMPrintString("\r\n");
        if(Option.TOUCH_XZERO != 0 || Option.TOUCH_YZERO != 0) {
            MMPrintString("GUI CALIBRATE "); PInt(Option.TOUCH_SWAPXY); PIntComma(Option.TOUCH_XZERO); PIntComma(Option.TOUCH_YZERO);
            PIntComma(Option.TOUCH_XSCALE * 10000); PIntComma(Option.TOUCH_YSCALE * 10000); MMPrintString("\r\n");
        }
    }
#endif
    if(Option.SD_CS){
        PO("SDCARD");
        MMPrintString((char *)PinDef[Option.SD_CS].pinname);
        if(Option.SD_CLK_PIN){
            MMPrintString(", "); MMPrintString((char *)PinDef[Option.SD_CLK_PIN].pinname);
            MMPrintString(", "); MMPrintString((char *)PinDef[Option.SD_MOSI_PIN].pinname);
            MMPrintString(", "); MMPrintString((char *)PinDef[Option.SD_MISO_PIN].pinname);
        }
        MMPrintString("\r\n");
    }
    if(Option.AUDIO_L || Option.AUDIO_CLK_PIN){
        PO("AUDIO");
        if(Option.AUDIO_L){
            MMPrintString((char *)PinDef[Option.AUDIO_L].pinname);MMputchar(',',1);
            MMPrintString((char *)PinDef[Option.AUDIO_R].pinname);
        } else {
            MMPrintString((char *)PinDef[Option.AUDIO_CS_PIN].pinname);MMputchar(',',1);
            MMPrintString((char *)PinDef[Option.AUDIO_CLK_PIN].pinname);MMputchar(',',1);
            MMPrintString((char *)PinDef[Option.AUDIO_MOSI_PIN].pinname);
        }
        MMPrintString(", ON PWM CHANNEL ");
        PInt(Option.AUDIO_SLICE);MMPrintString("\r\n");
    }
    if(Option.RTC)PO2Str("RTC AUTO", "ENABLE");

    if(Option.INT1pin!=9 || Option.INT2pin!=10 || Option.INT3pin!=11 || Option.INT4pin!=12){
        PO("COUNT"); MMPrintString((char *)PinDef[Option.INT1pin].pinname);
        MMputchar(',',1);;MMPrintString((char *)PinDef[Option.INT2pin].pinname);
        MMputchar(',',1);;MMPrintString((char *)PinDef[Option.INT3pin].pinname);
        MMputchar(',',1);;MMPrintString((char *)PinDef[Option.INT4pin].pinname);PRet();
    }
    if(*Option.F1key)PO2Str("F1", Option.F1key);
    if(*Option.F5key)PO2Str("F5", Option.F5key);
    if(*Option.F6key)PO2Str("F6", Option.F6key);
    if(*Option.F7key)PO2Str("F7", Option.F7key);
    if(*Option.F8key)PO2Str("F8", Option.F8key);
    if(*Option.F9key)PO2Str("F9", Option.F9key);
    if(Option.DefaultFont!=1)PO3Int("DEFAULT FONT",(Option.DefaultFont>>4)+1, Option.DefaultFont & 0xF);

}
int checkslice(int pin1,int pin2, int ignore){
    if((PinDef[pin1].slice & 0xf) != (PinDef[pin2].slice &0xf)) error("Pins not on same PWM slice");
    if(!ignore){
        if(!((PinDef[pin1].slice - PinDef[pin2].slice == 128) || (PinDef[pin2].slice - PinDef[pin1].slice == 128))) error("Pins both same channel");
    }
    return PinDef[pin1].slice & 0xf;
}
void setterminal(void){
	  char sp[20]={0};
	  strcpy(sp,"\033[8;");
	  IntToStr(&sp[strlen(sp)],Option.Height,10);
	  strcat(sp,";");
	  IntToStr(&sp[strlen(sp)],Option.Width+1,10);
	  strcat(sp,"t");
	  SSPrintString(sp);						//
}

void cmd_update(void){
    uint gpio_mask = 0u;
    reset_usb_boot(gpio_mask, PICO_STDIO_USB_RESET_BOOTSEL_INTERFACE_DISABLE_MASK);
}
void disable_systemspi(void){
    if(!IsInvalidPin(Option.SYSTEM_MOSI))ExtCurrentConfig[Option.SYSTEM_MOSI] = EXT_DIG_IN ;   
    if(!IsInvalidPin(Option.SYSTEM_MISO))ExtCurrentConfig[Option.SYSTEM_MISO] = EXT_DIG_IN ;   
    if(!IsInvalidPin(Option.SYSTEM_CLK))ExtCurrentConfig[Option.SYSTEM_CLK] = EXT_DIG_IN ;   
    if(!IsInvalidPin(Option.SYSTEM_MOSI))ExtCfg(Option.SYSTEM_MOSI, EXT_NOT_CONFIG, 0);
    if(!IsInvalidPin(Option.SYSTEM_MISO))ExtCfg(Option.SYSTEM_MISO, EXT_NOT_CONFIG, 0);
    if(!IsInvalidPin(Option.SYSTEM_CLK))ExtCfg(Option.SYSTEM_CLK, EXT_NOT_CONFIG, 0);
    Option.SYSTEM_MOSI=0;
    Option.SYSTEM_MISO=0;
    Option.SYSTEM_CLK=0;
}
void disable_systemi2c(void){
    if(!IsInvalidPin(Option.SYSTEM_I2C_SCL))ExtCurrentConfig[Option.SYSTEM_I2C_SCL] = EXT_DIG_IN ;   
    if(!IsInvalidPin(Option.SYSTEM_I2C_SDA))ExtCurrentConfig[Option.SYSTEM_I2C_SDA] = EXT_DIG_IN ;   
    if(!IsInvalidPin(Option.SYSTEM_I2C_SCL))ExtCfg(Option.SYSTEM_I2C_SCL, EXT_NOT_CONFIG, 0);
    if(!IsInvalidPin(Option.SYSTEM_I2C_SDA))ExtCfg(Option.SYSTEM_I2C_SDA, EXT_NOT_CONFIG, 0);
    Option.SYSTEM_I2C_SCL=0;
    Option.SYSTEM_I2C_SDA=0;
}
void disable_sd(void){
    if(!IsInvalidPin(Option.SD_CS))ExtCurrentConfig[Option.SD_CS] = EXT_DIG_IN ;   
    if(!IsInvalidPin(Option.SD_CS))ExtCfg(Option.SD_CS, EXT_NOT_CONFIG, 0);
    Option.SD_CS=0;
    if(!IsInvalidPin(Option.SD_CLK_PIN))ExtCurrentConfig[Option.SD_CLK_PIN] = EXT_DIG_IN ;   
    if(!IsInvalidPin(Option.SD_CLK_PIN))ExtCfg(Option.SD_CLK_PIN, EXT_NOT_CONFIG, 0);
    Option.SD_CLK_PIN=0;
    if(!IsInvalidPin(Option.SD_MOSI_PIN))ExtCurrentConfig[Option.SD_MOSI_PIN] = EXT_DIG_IN ;   
    if(!IsInvalidPin(Option.SD_MOSI_PIN))ExtCfg(Option.SD_MOSI_PIN, EXT_NOT_CONFIG, 0);
    Option.SD_MOSI_PIN=0;
    if(!IsInvalidPin(Option.SD_MISO_PIN))ExtCurrentConfig[Option.SD_MISO_PIN] = EXT_DIG_IN ;   
    if(!IsInvalidPin(Option.SD_MISO_PIN))ExtCfg(Option.SD_MISO_PIN, EXT_NOT_CONFIG, 0);
    Option.SD_MISO_PIN=0;
}
void disable_audio(void){
    if(!IsInvalidPin(Option.AUDIO_L))ExtCurrentConfig[Option.AUDIO_L] = EXT_DIG_IN ;   
    if(!IsInvalidPin(Option.AUDIO_L))ExtCfg(Option.AUDIO_L, EXT_NOT_CONFIG, 0);
    if(!IsInvalidPin(Option.AUDIO_R))ExtCurrentConfig[Option.AUDIO_R] = EXT_DIG_IN ;   
    if(!IsInvalidPin(Option.AUDIO_R))ExtCfg(Option.AUDIO_R, EXT_NOT_CONFIG, 0);
    if(!IsInvalidPin(Option.AUDIO_CLK_PIN))ExtCurrentConfig[Option.AUDIO_CLK_PIN] = EXT_DIG_IN ;   
    if(!IsInvalidPin(Option.AUDIO_CLK_PIN))ExtCfg(Option.AUDIO_CLK_PIN, EXT_NOT_CONFIG, 0);
    if(!IsInvalidPin(Option.AUDIO_CS_PIN))ExtCurrentConfig[Option.AUDIO_CS_PIN] = EXT_DIG_IN ;   
    if(!IsInvalidPin(Option.AUDIO_CS_PIN))ExtCfg(Option.AUDIO_CS_PIN, EXT_NOT_CONFIG, 0);
    if(!IsInvalidPin(Option.AUDIO_MOSI_PIN))ExtCurrentConfig[Option.AUDIO_MOSI_PIN] = EXT_DIG_IN ;   
    if(!IsInvalidPin(Option.AUDIO_MOSI_PIN))ExtCfg(Option.AUDIO_MOSI_PIN, EXT_NOT_CONFIG, 0);
    Option.AUDIO_L=0;
    Option.AUDIO_R=0;
    Option.AUDIO_CLK_PIN=0;
    Option.AUDIO_CS_PIN=0;
    Option.AUDIO_MOSI_PIN=0;
    Option.AUDIO_SLICE=99;
}
void cmd_option(void) {
    unsigned char *tp;

    tp = checkstring(cmdline, "BASE");
    if(tp) {
        if(DimUsed) error("Must be before DIM or LOCAL");
        OptionBase = getint(tp, 0, 1);
        return;
    }

    tp = checkstring(cmdline, "EXPLICIT");
    if(tp) {
//        if(varcnt != 0) error("Variables already defined");
        OptionExplicit = true;
        return;
    }
	tp = checkstring(cmdline, "ANGLE");
	if(tp) {
		if(checkstring(tp, "DEGREES"))	{ optionangle=RADCONV; return; }
		if(checkstring(tp, "RADIANS"))	{ optionangle=1.0; return; }
	}

	tp = checkstring(cmdline, "FAST AUDIO");
	if(tp) {
		if(checkstring(tp, "OFF"))	{ optionfastaudio=0; return; }
		if(checkstring(tp, "ON"))	{ optionfastaudio=1; return; }
	}

    tp = checkstring(cmdline, "ESCAPE");
    if(tp) {
        OptionEscape = true;
        return;
    }
    tp = checkstring(cmdline, "DEFAULT");
    if(tp) {
        if(checkstring(tp, "INTEGER"))  { DefaultType = T_INT;  return; }
        if(checkstring(tp, "FLOAT"))    { DefaultType = T_NBR;  return; }
        if(checkstring(tp, "STRING"))   { DefaultType = T_STR;  return; }
        if(checkstring(tp, "NONE"))     { DefaultType = T_NOTYPE;   return; }
    }


    tp = checkstring(cmdline, "BREAK");
    if(tp) {
        BreakKey = getinteger(tp);
        return;
    }
    tp = checkstring(cmdline, "F1");
	if(tp) {
		char p[STRINGSIZE];
		strcpy(p,getCstring(tp));
		if(strlen(p)>=sizeof(Option.F1key))error("Maximum 63 characters");
		else strcpy((char *)Option.F1key, p);
		SaveOptions();
		return;
	}
    tp = checkstring(cmdline, "F5");
	if(tp) {
		char p[STRINGSIZE];
		strcpy(p,getCstring(tp));
		if(strlen(p)>=sizeof(Option.F5key))error("Maximum % characters",MAXKEYLEN-1);
		else strcpy((char *)Option.F5key, p);
		SaveOptions();
		return;
	}
    tp = checkstring(cmdline, "F6");
	if(tp) {
		char p[STRINGSIZE];
		strcpy(p,getCstring(tp));
		if(strlen(p)>=sizeof(Option.F6key))error("Maximum % characters",MAXKEYLEN-1);
		else strcpy((char *)Option.F6key, p);
		SaveOptions();
		return;
	}
    tp = checkstring(cmdline, "F7");
	if(tp) {
		char p[STRINGSIZE];
		strcpy(p,getCstring(tp));
		if(strlen(p)>=sizeof(Option.F7key))error("Maximum % characters",MAXKEYLEN-1);
		else strcpy((char *)Option.F7key, p);
		SaveOptions();
		return;
	}
    tp = checkstring(cmdline, "F8");
	if(tp) {
		char p[STRINGSIZE];
		strcpy(p,getCstring(tp));
		if(strlen(p)>=sizeof(Option.F8key))error("Maximum % characters",MAXKEYLEN-1);
		else strcpy((char *)Option.F8key, p);
		SaveOptions();
		return;
	}
    tp = checkstring(cmdline, "F9");
	if(tp) {
		char p[STRINGSIZE];
		strcpy(p,getCstring(tp));
		if(strlen(p)>=sizeof(Option.F9key))error("Maximum % characters",MAXKEYLEN-1);
		else strcpy((char *)Option.F9key, p);
		SaveOptions();
		return;
	}
    tp = checkstring(cmdline, "KEYBOARD");
	if(tp) {
    	if(CurrentLinePtr) error("Invalid in a program");
		if(checkstring(tp, "DISABLE")){
			Option.KeyboardConfig = NO_KEYBOARD;
            Option.capslock=0;
            Option.numlock=0;
            SaveOptions();
            _excep_code = RESET_COMMAND;
            SoftReset();
		} else {
        getargs(&tp,9,",");
        if(ExtCurrentConfig[KEYBOARD_CLOCK] != EXT_NOT_CONFIG && Option.KeyboardConfig == NO_KEYBOARD)  error("Pin %/| is in use",KEYBOARD_CLOCK,KEYBOARD_CLOCK);
        if(ExtCurrentConfig[KEYBOARD_DATA] != EXT_NOT_CONFIG && Option.KeyboardConfig == NO_KEYBOARD)  error("Pin %/| is in use",KEYBOARD_DATA,KEYBOARD_DATA);
        else if(checkstring(argv[0], "US"))	Option.KeyboardConfig = CONFIG_US;
		else if(checkstring(argv[0], "FR"))	Option.KeyboardConfig = CONFIG_FR;
		else if(checkstring(argv[0], "GR"))	Option.KeyboardConfig = CONFIG_GR;
		else if(checkstring(argv[0], "IT"))	Option.KeyboardConfig = CONFIG_IT;
		else if(checkstring(argv[0], "BE"))	Option.KeyboardConfig = CONFIG_BE;
		else if(checkstring(argv[0], "UK"))	Option.KeyboardConfig = CONFIG_UK;
		else if(checkstring(argv[0], "ES"))	Option.KeyboardConfig = CONFIG_ES;
		else if(checkstring(argv[0], "I2C")) Option.KeyboardConfig = CONFIG_I2C;
        else error("Syntax");
        Option.capslock=0;
        Option.numlock=1;
        int rs=0b00100000;
        int rr=0b00001100;
        if(Option.KeyboardConfig!=CONFIG_I2C){
            if(argc>=3 && *argv[2])Option.capslock=getint(argv[2],0,1);
            if(argc>=5 && *argv[4])Option.numlock=getint(argv[4],0,1);
            if(argc>=7 && *argv[6])rs=getint(argv[6],0,3)<<5;
            if(argc==9 && *argv[8])rr=getint(argv[8],0,31);
            Option.repeat = rs | rr;
        } else {
            if(!Option.SYSTEM_I2C_SCL)error("Option System I2C not set");
        }
        SaveOptions();
        _excep_code = RESET_COMMAND;
        SoftReset();
        }
	}

    tp = checkstring(cmdline, "BAUDRATE");
    if(tp) {
        if(CurrentLinePtr) error("Invalid in a program");
        int i;
        i = getint(tp,Option.CPU_Speed*1000/16/65535,921600);	
        if(i < 100) error("Number out of bounds");
        Option.Baudrate = i;
        SaveOptions();
        if(!Option.SerialConsole)MMPrintString("Value saved but Serial Console not enabled");
        else MMPrintString("Restart to activate");                // set the console baud rate
        return;
    }

    tp = checkstring(cmdline, "SERIAL CONSOLE");
    if(tp) {
   	    if(CurrentLinePtr) error("Invalid in a program");
        unsigned char *p=NULL;
        if(checkstring(tp, "DISABLE")) {
            Option.SerialTX=0;
            Option.SerialRX=0;
            Option.SerialConsole = 0; 
            SaveOptions(); 
            SoftReset();
            return;
        } else {
            int pin,pin2,value,value2;
            getargs(&tp,5,",");
            if(!(argc==3 || argc==5))error("Syntax");
            char code;
            if(!(code=codecheck(argv[0])))argv[0]+=2;
            pin = getinteger(argv[0]);
            if(!code)pin=codemap(pin);
            if(!(code=codecheck(argv[2])))argv[2]+=2;
            pin2 = getinteger(argv[2]);
            if(!code)pin2=codemap(pin2);
            if(ExtCurrentConfig[pin] != EXT_NOT_CONFIG)  error("Pin %/| is in use",pin,pin);
            if(ExtCurrentConfig[pin2] != EXT_NOT_CONFIG)  error("Pin %/| is in use",pin2,pin2);
            if(PinDef[pin].mode & UART0TX)Option.SerialTX = pin;
            else if(PinDef[pin].mode & UART0RX)Option.SerialRX = pin;
            else goto checkcom2;
            if(PinDef[pin2].mode & UART0TX)Option.SerialTX = pin2;
            else if(PinDef[pin2].mode & UART0RX)Option.SerialRX = pin2;
            else error("Invalid configuration");
            if(Option.SerialTX==Option.SerialRX)error("Invalid configuration");
            Option.SerialConsole = 1; 
            if(argc==5)Option.SerialConsole=(checkstring(argv[4],"B") ? 5: 1);
            SaveOptions(); 
            SoftReset();
            return;
        checkcom2:
            if(PinDef[pin].mode & UART1TX)Option.SerialTX = pin;
            else if(PinDef[pin].mode & UART1RX)Option.SerialRX = pin;
            else error("Invalid configuration");
            if(PinDef[pin2].mode & UART1TX)Option.SerialTX = pin2;
            else if(PinDef[pin2].mode & UART1RX)Option.SerialRX = pin2;
            else error("Invalid configuration");
            if(Option.SerialTX==Option.SerialRX)error("Invalid configuration");
            Option.SerialConsole = 2; 
            if(argc==5)Option.SerialConsole=(checkstring(argv[4],"B") ? 6: 2);
            SaveOptions(); 
            SoftReset();
        }  
    }

    tp = checkstring(cmdline, "AUTORUN");
    if(tp) {
        if(checkstring(tp, "OFF"))      { Option.Autorun = 0; SaveOptions(); return;  }
        if(checkstring(tp, "ON"))      { Option.Autorun = MAXFLASHSLOTS+1; SaveOptions(); return;  }
        Option.Autorun=getint(tp,0,MAXFLASHSLOTS);
        SaveOptions(); return; 
    }
    tp = checkstring(cmdline, "HEARTBEAT");
    if(tp) {
        if(checkstring(tp, "OFF"))      Option.NoHeartbeat = 1; 
        if(checkstring(tp, "ON"))      Option.NoHeartbeat = 0; 
        SaveOptions();
#ifndef PICOMITEWEB
        if(CheckPin(43, CP_NOABORT | CP_IGNORE_INUSE | CP_IGNORE_RESERVED)){
            if(Option.NoHeartbeat==0){
                (PinDef[HEARTBEATpin].GPno);
                gpio_set_dir(PinDef[HEARTBEATpin].GPno, GPIO_OUT);
                ExtCurrentConfig[PinDef[HEARTBEATpin].pin]=EXT_HEARTBEAT;
            } else ExtCfg(HEARTBEATpin, EXT_NOT_CONFIG, 0); 
        } else error("Pin %/| is reserved", HEARTBEATpin, HEARTBEATpin);
#endif
        return;
    }
    tp = checkstring(cmdline, "LCDPANEL NOCONSOLE");
    if(tp){
        Option.Height = SCREENHEIGHT; Option.Width = SCREENWIDTH;
        Option.DISPLAY_CONSOLE = 0;
        Option.ColourCode = false;
        Option.DefaultFC = WHITE;
        Option.DefaultBC = BLACK;
        SetFont((Option.DefaultFont = (Option.DISPLAY_TYPE==COLOURVGA? (6<<4) | 1 : 0x01 )));
        Option.DefaultBrightness = 100;
        if(!CurrentLinePtr) {
            SaveOptions();
            ClearScreen(Option.DefaultBC);
        }
        return;
    }
    tp = checkstring(cmdline, "LCDPANEL CONSOLE");
    if(tp) {
        if(!(Option.DISPLAY_TYPE==ST7789B || Option.DISPLAY_TYPE==ILI9488 || Option.DISPLAY_TYPE==ILI9341 || Option.DISPLAY_TYPE==ILI9481IPS || Option.DISPLAY_TYPE>=VGADISPLAY))error("Display does not support console");
        if(!Option.DISPLAY_ORIENTATION== DISPLAY_LANDSCAPE) error("Landscape only");
        skipspace(tp);
        Option.DefaultFC = WHITE;
        Option.DefaultBC = BLACK;
        Option.DefaultBrightness = 100;
        if(!(*tp == 0 || *tp == '\'')) {
            getargs(&tp, 7, ",");                              // this is a macro and must be the first executable stmt in a block
            if(argc > 0) {
                if(*argv[0] == '#') argv[0]++;                  // skip the hash if used
                Option.DefaultFont=(((getint(argv[0], 1, FONT_BUILTIN_NBR) - 1) << 4) | 1);
            }
            if(argc > 2) Option.DefaultFC = getint(argv[2], BLACK, WHITE);
            if(argc > 4) Option.DefaultBC = getint(argv[4], BLACK, WHITE);
            if(Option.DefaultFC == Option.DefaultBC) error("Same colours");
            if(argc > 6) {
                if(!Option.DISPLAY_BL)error("Backlight not available on this display");
                Option.DefaultBrightness = getint(argv[6], 0, 100);
            }
        }
        if(Option.DISPLAY_BL){
			MMFLOAT frequency=1000.0,duty=Option.DefaultBrightness;
            int wrap=(Option.CPU_Speed*1000)/frequency;
            int high=(int)((MMFLOAT)Option.CPU_Speed/frequency*duty*10.0);
            int div=1;
            while(wrap>65535){
                wrap>>=1;
                if(duty>=0.0)high>>=1;
                div<<=1;
            }
            wrap--;
            if(div!=1)pwm_set_clkdiv(BacklightSlice,(float)div);
            pwm_set_wrap(BacklightSlice, wrap);
            pwm_set_chan_level(BacklightSlice, BacklightChannel, high);
        }
#ifdef PICOMITEVGA
        int  fcolour = ((Option.DefaultFC & 0x800000)>> 20) | ((Option.DefaultFC & 0xC000)>>13) | ((Option.DefaultFC & 0x80)>>7);
        fcolour= (fcolour<<12) | (fcolour<<8) | (fcolour<<4) | fcolour;
        int bcolour = ((Option.DefaultBC & 0x800000)>> 20) | ((Option.DefaultBC & 0xC000)>>13) | ((Option.DefaultBC & 0x80)>>7);
        bcolour= (bcolour<<12) | (bcolour<<8) | (bcolour<<4) | bcolour;
        for(int xp=0;xp<X_TILE;xp++){
            for(int yp=0;yp<Y_TILE;yp++){
                tilefcols[yp*X_TILE+xp]=(uint16_t)fcolour;
                tilebcols[yp*Y_TILE+xp]=(uint16_t)bcolour;
            }
        }
        Option.VGAFC=fcolour;
        Option.VGABC=bcolour;
#endif
        Option.DISPLAY_CONSOLE = true; 
        if(!CurrentLinePtr) {
            ResetDisplay();
            setterminal();
            SaveOptions();
            if(!(Option.DISPLAY_TYPE==MONOVGA || Option.DISPLAY_TYPE==COLOURVGA))ClearScreen(Option.DefaultBC);
        }
        return;
    }

    tp = checkstring(cmdline, "LEGACY");
    if(tp) {
        if(checkstring(tp, "OFF"))      { CMM1=0; return;  }
        if(checkstring(tp, "ON"))      { CMM1=1; return;  }
        error("Syntax");
    }
#ifdef PICOMITEWEB
    tp = checkstring(cmdline, "WIFI");
    if(tp) {
        getargs(&tp,3,",");
        if(argc!=3)error("Syntax");
   	    if(CurrentLinePtr) error("Invalid in a program");
        char *ssid=GetTempMemory(STRINGSIZE);
        char *password=GetTempMemory(STRINGSIZE);
        strcpy(ssid,getCstring(argv[0]));
        strcpy(password,getCstring(argv[2]));
        if(strlen(ssid)>MAXKEYLEN-1)error("SSID too long, max 63 chars");
        if(strlen(password)>MAXKEYLEN-1)error("Password too long, max 63 chars");
        strcpy(Option.SSID,ssid);
        strcpy(Option.PASSWORD,password);
        SaveOptions();
         _excep_code = RESET_COMMAND;
        SoftReset();
        return;
    }
    tp = checkstring(cmdline, "TCP SERVER PORT");
    if(tp) {
        getargs(&tp,3,",");
   	    if(CurrentLinePtr) error("Invalid in a program");
        Option.TCP_PORT=getint(argv[0],0,65535);
        Option.ServerResponceTime=5000;
        if(argc==3)Option.ServerResponceTime=getint(argv[2],1000,20000);
        SaveOptions();
         _excep_code = RESET_COMMAND;
        SoftReset();
        return;
    }
    tp = checkstring(cmdline, "TELNET CONSOLE");
    if(tp) {
   	    if(CurrentLinePtr) error("Invalid in a program");
        if(checkstring(tp, "OFF"))Option.Telnet=0;
        else if(checkstring(tp, "ON"))Option.Telnet=1;
        else if(checkstring(tp, "ONLY")) Option.Telnet=-1;
        else error("Syntax");
        SaveOptions();
         _excep_code = RESET_COMMAND;
        SoftReset();
        return;
    }
    tp = checkstring(cmdline, "TFTP");
    if(tp) {
   	    if(CurrentLinePtr) error("Invalid in a program");
        if(checkstring(tp, "OFF"))Option.disabletftp=1;
        else if(checkstring(tp, "ON"))Option.disabletftp=0;
        else error("Syntax");
        SaveOptions();
         _excep_code = RESET_COMMAND;
        SoftReset();
        return;
    }

#endif

#ifdef PICOMITEVGA
    tp = checkstring(cmdline, "CPUSPEED");
    if(tp) {
   	    if(CurrentLinePtr) error("Invalid in a program");
         int CPU_Speed=getinteger(tp);
        if(!(CPU_Speed==126000 || CPU_Speed==252000 || CPU_Speed==378000))error("CpuSpeed 126000, 252000 or 378000 only");
        Option.CPU_Speed=CPU_Speed;
        Option.X_TILE=80;
        Option.Y_TILE=40;
        SaveOptions();
        _excep_code = RESET_COMMAND;
        SoftReset();
        return;
    }

    tp = checkstring(cmdline, "DEFAULT MODE");
    if(tp) {
        int mode=getint(tp,1,2);
        if(mode==2){
            Option.DISPLAY_TYPE=COLOURVGA; 
            Option.DefaultFont=(6<<4) | 1 ;
        } else {
            Option.DISPLAY_TYPE=MONOVGA;
            Option.DefaultFont= 1 ;
        }
        SaveOptions();
        DISPLAY_TYPE= Option.DISPLAY_TYPE;
	    memset(WriteBuf, 0, 38400);
        ResetDisplay();
        CurrentX = CurrentY =0;
        if(Option.DISPLAY_TYPE!=MONOVGA)ClearScreen(Option.DefaultBC);
        SetFont(Option.DefaultFont);
        return;
    }
    tp = checkstring(cmdline, "DEFAULT COLOURS");
    if(tp==NULL)tp = checkstring(cmdline, "DEFAULT COLORS");
    if(tp){
        uint16_t forcol=0xFFFF;
        uint16_t backcol=0x0000;
        int DefaultFC=WHITE;
        int DefaultBC=BLACK;
        getargs(&tp,3, ",");
        if(checkstring(argv[0], "WHITE"))        { forcol=0xFFFF; DefaultFC=WHITE;}
        else if(checkstring(argv[0], "YELLOW"))  { forcol=0xEEEE; DefaultFC=YELLOW;}
        else if(checkstring(argv[0], "LILAC"))   { forcol=0xDDDD; DefaultFC=LILAC;}
        else if(checkstring(argv[0], "BROWN"))   { forcol=0xCCCC; DefaultFC=BROWN;}
        else if(checkstring(argv[0], "FUCHSIA")) { forcol=0xBBBB; DefaultFC=FUCHSIA;}
        else if(checkstring(argv[0], "RUST"))    { forcol=0xAAAA; DefaultFC=RUST;}
        else if(checkstring(argv[0], "MAGENTA")) { forcol=0x9999; DefaultFC=MAGENTA;}
        else if(checkstring(argv[0], "RED"))     { forcol=0x8888; DefaultFC=RED;}
        else if(checkstring(argv[0], "CYAN"))    { forcol=0x7777; DefaultFC=CYAN;}
        else if(checkstring(argv[0], "GREEN"))   { forcol=0x6666; DefaultFC=GREEN;}
        else if(checkstring(argv[0], "CERULEAN")){ forcol=0x5555; DefaultFC=CERULEAN;}
        else if(checkstring(argv[0], "MIDGREEN")){ forcol=0x4444; DefaultFC=MIDGREEN;}
        else if(checkstring(argv[0], "COBALT"))  { forcol=0x3333; DefaultFC=COBALT;}
        else if(checkstring(argv[0], "MYRTLE"))  { forcol=0x2222; DefaultFC=MYRTLE;}
        else if(checkstring(argv[0], "BLUE"))    { forcol=0x1111; DefaultFC=BLUE;}
        else if(checkstring(argv[0], "BLACK"))   { forcol=0x0000; DefaultFC=BLACK;}
        else error("Invalid colour: $", argv[0]); 
        if(argc==3){
            if(checkstring(argv[2], "WHITE"))        { backcol=0xFFFF; DefaultBC=WHITE;}
            else if(checkstring(argv[2], "YELLOW"))  { backcol=0xEEEE; DefaultBC=YELLOW;}
            else if(checkstring(argv[2], "LILAC"))   { backcol=0xDDDD; DefaultBC=LILAC;}
            else if(checkstring(argv[2], "BROWN"))   { backcol=0xCCCC; DefaultBC=BROWN;}
            else if(checkstring(argv[2], "FUCHSIA")) { backcol=0xBBBB; DefaultBC=FUCHSIA;}
            else if(checkstring(argv[2], "RUST"))    { backcol=0xAAAA; DefaultBC=RUST;}
            else if(checkstring(argv[2], "MAGENTA")) { backcol=0x9999; DefaultBC=MAGENTA;}
            else if(checkstring(argv[2], "RED"))     { backcol=0x8888; DefaultBC=RED;}
            else if(checkstring(argv[2], "CYAN"))    { backcol=0x7777; DefaultBC=CYAN;}
            else if(checkstring(argv[2], "GREEN"))   { backcol=0x6666; DefaultBC=GREEN;}
            else if(checkstring(argv[2], "CERULEAN")){ backcol=0x5555; DefaultBC=CERULEAN;}
            else if(checkstring(argv[2], "MIDGREEN")){ backcol=0x4444; DefaultBC=MIDGREEN;}
            else if(checkstring(argv[2], "COBALT"))  { backcol=0x3333; DefaultBC=COBALT;}
            else if(checkstring(argv[2], "MYRTLE"))  { backcol=0x2222; DefaultBC=MYRTLE;}
            else if(checkstring(argv[2], "BLUE"))    { backcol=0x1111; DefaultBC=BLUE;}
            else if(checkstring(argv[2], "BLACK"))   { backcol=0x0000; DefaultBC=BLACK;}
            else error("Invalid colour: $", argv[2]); 
        }      
        if(backcol==forcol)error("Foreground and Background colours are the same");
        Option.VGABC=backcol;
        Option.VGAFC=forcol;
        for(int x=0;x<X_TILE;x++){
            for(int y=0;y<Y_TILE;y++){
                tilefcols[y*X_TILE+x]=Option.VGAFC;
                tilebcols[y*X_TILE+x]=Option.VGABC;
            }
        }
        Option.DefaultBC=DefaultBC;
        Option.DefaultFC=DefaultFC;
        SaveOptions();
        ResetDisplay();
        if(Option.DISPLAY_TYPE!=MONOVGA)ClearScreen(gui_bcolour);
        return;
    }
#else
#ifdef PICOMITE
    tp = checkstring(cmdline,"GUI CONTROLS");
    if(tp) {
        getargs(&tp, 1, ",");
    	if(CurrentLinePtr) error("Invalid in a program");
        Option.MaxCtrls=getint(argv[0],0,MAXCONTROLS-1);
        if(Option.MaxCtrls)Option.MaxCtrls++;
        SaveOptions();
        _excep_code = RESET_COMMAND;
        SoftReset();
    }
#endif
    tp = checkstring(cmdline, "CPUSPEED");
    if(tp) {
   	    if(CurrentLinePtr) error("Invalid in a program");
        Option.CPU_Speed=getint(tp,MIN_CPU,MAX_CPU);
        SaveOptions();
        _excep_code = RESET_COMMAND;
        SoftReset();
        return;
    }

    tp = checkstring(cmdline, "AUTOREFRESH");
	if(tp) {
	    if((Option.DISPLAY_TYPE==ILI9341 || Option.DISPLAY_TYPE == ILI9163 || Option.DISPLAY_TYPE == ST7735 || Option.DISPLAY_TYPE == ST7789 || Option.DISPLAY_TYPE == ST7789A)) error("Not valid for this display");
		if(checkstring(tp, "ON"))		{
			Option.Refresh = 1;
			Display_Refresh();
			return;
		}
		if(checkstring(tp, "OFF"))		{ Option.Refresh = 0; return; }
	}
    
    tp = checkstring(cmdline, "LCDPANEL");
    if(tp) {
    	if(CurrentLinePtr) error("Invalid in a program");
        if(checkstring(tp, "DISABLE")) {
            Option.LCD_CD = Option.LCD_CS = Option.LCD_Reset = Option.DISPLAY_TYPE = HRes = VRes = 0;
            DrawRectangle = (void (*)(int , int , int , int , int ))DisplayNotSet;
            DrawBitmap =  (void (*)(int , int , int , int , int , int , int , unsigned char *))DisplayNotSet;
            ScrollLCD = (void (*)(int ))DisplayNotSet;
            DrawBuffer = (void (*)(int , int , int , int , unsigned char * ))DisplayNotSet;
            ReadBuffer = (void (*)(int , int , int , int , unsigned char * ))DisplayNotSet;
			Option.DISPLAY_CONSOLE = false;
		} else {
            if(Option.DISPLAY_TYPE && !CurrentLinePtr) error("Display already configured");
            ConfigDisplaySPI(tp);
            if(!Option.DISPLAY_TYPE)ConfigDisplayVirtual(tp);
            if(!Option.DISPLAY_TYPE)ConfigDisplaySSD(tp);
            if(!Option.DISPLAY_TYPE)ConfigDisplayI2C(tp);
        }
        SaveOptions();
        _excep_code = RESET_COMMAND;
        SoftReset();
        return;
    }
    tp = checkstring(cmdline, "TOUCH");
    if(tp) {
      if(CurrentLinePtr) error("Invalid in a program");
      if(checkstring(tp, "DISABLE")) {
            TouchIrqPortAddr = 0;
            Option.TOUCH_Click = Option.TOUCH_CS = Option.TOUCH_IRQ = false;
        } else  {
            if(Option.TOUCH_CS) error("Touch already configured");
            ConfigTouch(tp);
        }
        SaveOptions();
        _excep_code = RESET_COMMAND;
        SoftReset();
        return;
  }
#endif
    tp = checkstring(cmdline, "DISPLAY");
    if(tp) {
        getargs(&tp, 3, ",");
        if(Option.DISPLAY_CONSOLE) error("Cannot change LCD console");
        Option.Height = getint(argv[0], 5, 100);
        if(argc == 3) Option.Width = getint(argv[2], 37, 240);
        setterminal();
        SaveOptions();
        return;
    }
    tp = checkstring(cmdline, "CASE");
    if(tp) {
        if(checkstring(tp, "LOWER"))    { Option.Listcase = CONFIG_LOWER; SaveOptions(); return; }
        if(checkstring(tp, "UPPER"))    { Option.Listcase = CONFIG_UPPER; SaveOptions(); return; }
        if(checkstring(tp, "TITLE"))    { Option.Listcase = CONFIG_TITLE; SaveOptions(); return; }
    }

    tp = checkstring(cmdline, "TAB");
    if(tp) {
        if(checkstring(tp, "2"))        { Option.Tab = 2; SaveOptions(); return; }
		if(checkstring(tp, "3"))		{ Option.Tab = 3; SaveOptions(); return; }
        if(checkstring(tp, "4"))        { Option.Tab = 4; SaveOptions(); return; }
        if(checkstring(tp, "8"))        { Option.Tab = 8; SaveOptions(); return; }
    }
    tp = checkstring(cmdline, "VCC");
    if(tp) {
        MMFLOAT f;
        f = getnumber(tp);
        if(f > 3.6) error("VCC too high");
        if(f < 1.8) error("VCC too low");
        VCC=f;
        return;
    }

    tp = checkstring(cmdline, "PIN");
    if(tp) {
        int i;
        i = getint(tp, 0, 99999999);
        Option.PIN = i;
        SaveOptions();
        return;
    }

    tp = checkstring(cmdline, "POWER");
    if(tp) {
        if(checkstring(tp, "PWM"))  Option.PWM = true;
        if(checkstring(tp, "PFM"))  Option.PWM = false;
        SaveOptions();
        if(Option.PWM){
            gpio_init(23);
            gpio_put(23,GPIO_PIN_SET);
            gpio_set_dir(23, GPIO_OUT);
        } else {
            gpio_init(23);
            gpio_put(23,GPIO_PIN_RESET);
            gpio_set_dir(23, GPIO_OUT);
    	}
        return;
    }

    tp = checkstring(cmdline, "COLOURCODE");
    if(tp == NULL) tp = checkstring(cmdline, "COLORCODE");
    if(tp) {
        if(checkstring(tp, "ON"))       { Option.ColourCode = true; SaveOptions(); return; }
        else if(checkstring(tp, "OFF"))      { Option.ColourCode = false; SaveOptions(); return;  }
        else error("Syntax");
    }

    tp = checkstring(cmdline, "RTC AUTO");
    if(tp) {
        if(checkstring(tp, "ENABLE"))       { Option.RTC = true; SaveOptions(); RtcGetTime(0); return; }
        if(checkstring(tp, "DISABLE"))      { Option.RTC = false; SaveOptions(); return;  }
    }

	tp = checkstring(cmdline, "LIST");
    if(tp) {
    	printoptions();
    	return;
    }
    tp = checkstring(cmdline, "AUDIO");
    if(tp) {
        int pin1,pin2, slice;
        unsigned char *p;
        if(checkstring(tp, "DISABLE")){
   	        if(CurrentLinePtr) error("Invalid in a program");
             disable_audio();
            SaveOptions();
            _excep_code = RESET_COMMAND;
            SoftReset();
            return;                                // this will restart the processor ? only works when not in debug
        }
        if((p=checkstring(tp, "SPI"))){
            int pin1,pin2,pin3;
            getargs(&p,5,",");
            if(CurrentLinePtr) error("Invalid in a program");
            if(argc!=5)error("Syntax");
            if(Option.AUDIO_CLK_PIN)error("Audio SPI already configured");
            unsigned char code;
//
            if(!(code=codecheck(argv[0])))argv[0]+=2;
            pin1 = getinteger(argv[0]);
            if(!code)pin1=codemap(pin1);
            if(IsInvalidPin(pin1)) error("Invalid pin");
            if(ExtCurrentConfig[pin1] != EXT_NOT_CONFIG)  error("Pin %/| is in use",pin1,pin1);
//
            if(!(code=codecheck(argv[2])))argv[2]+=2;
            pin2 = getinteger(argv[2]);
            if(!code)pin2=codemap(pin2);
            if(IsInvalidPin(pin2)) error("Invalid pin");
            if(ExtCurrentConfig[pin2] != EXT_NOT_CONFIG)  error("Pin %/| is in use",pin2,pin2);
//
            if(!(code=codecheck(argv[4])))argv[4]+=2;
            pin3 = getinteger(argv[4]);
            if(!code)pin3=codemap(pin3);
            if(IsInvalidPin(pin3)) error("Invalid pin");
            if(ExtCurrentConfig[pin3] != EXT_NOT_CONFIG)  error("Pin %/| is in use",pin3,pin3);
//
/*            if(!(code=codecheck(argv[6])))argv[6]+=2;
            pin2 = getinteger(argv[6]);
            if(!code)pin2=codemap(pin2);
            if(IsInvalidPin(pin2)) error("Invalid pin");
            if(ExtCurrentConfig[pin2] != EXT_NOT_CONFIG)  error("Pin %/| is in use",pin2,pin2);*/
//
            if(!(PinDef[pin2].mode & SPI0SCK && PinDef[pin3].mode & SPI0TX) &&
            !(PinDef[pin2].mode & SPI1SCK && PinDef[pin3].mode & SPI1TX))error("Not valid SPI pins");
            Option.AUDIO_CS_PIN=pin1;
            Option.AUDIO_CLK_PIN=pin2;
            Option.AUDIO_MOSI_PIN=pin3;
            slice=checkslice(pin2,pin2, 1);
            if((PinDef[Option.DISPLAY_BL].slice & 0x7f) == slice) error("Channel in use for backlight");
            Option.AUDIO_SLICE=slice;
            SaveOptions();
            _excep_code = RESET_COMMAND;
            SoftReset();
            return;
        }
    	getargs(&tp,3,",");
   	    if(CurrentLinePtr) error("Invalid in a program");
         if(argc!=3)error("Syntax");
        if(Option.AUDIO_L)error("Audio already configured");
        unsigned char code;
        if(!(code=codecheck(argv[0])))argv[0]+=2;
        pin1 = getinteger(argv[0]);
        if(!code)pin1=codemap(pin1);
        if(IsInvalidPin(pin1)) error("Invalid pin");
        if(ExtCurrentConfig[pin1] != EXT_NOT_CONFIG)  error("Pin %/| is in use",pin1,pin1);
        if(!(code=codecheck(argv[2])))argv[2]+=2;
        pin2 = getinteger(argv[2]);
        if(!code)pin2=codemap(pin2);
        if(IsInvalidPin(pin2)) error("Invalid pin");
        if(ExtCurrentConfig[pin2] != EXT_NOT_CONFIG)  error("Pin %/| is in use",pin2,pin2);
        slice=checkslice(pin1,pin2, 0);
        if((PinDef[Option.DISPLAY_BL].slice & 0x7f) == slice) error("Channel in use for backlight");
        Option.AUDIO_L=pin1;
        Option.AUDIO_R=pin2;
        Option.AUDIO_SLICE=slice;
        SaveOptions();
        _excep_code = RESET_COMMAND;
        SoftReset();
        return;
    }

    tp = checkstring(cmdline, "SYSTEM I2C");
    if(tp) {
        int pin1,pin2,channel=-1;
        if(checkstring(tp, "DISABLE")){
   	    if(CurrentLinePtr) error("Invalid in a program");
 #ifdef PICOMITEVGA
        if(Option.RTC_Clock || Option.RTC_Data)error("In use");
#else
        if(Option.DISPLAY_TYPE == SSD1306I2C || Option.DISPLAY_TYPE == SSD1306I2C32 || Option.RTC_Clock || Option.RTC_Data)error("In use");
#endif
            disable_systemi2c();
            SaveOptions();
            _excep_code = RESET_COMMAND;
            SoftReset();
            return;                                // this will restart the processor ? only works when not in debug
        }
    	getargs(&tp,5,",");
   	    if(CurrentLinePtr) error("Invalid in a program");
        if(argc<3)error("Syntax");
        if(Option.SYSTEM_I2C_SCL)error("I2C already configured");
        unsigned char code;
        if(!(code=codecheck(argv[0])))argv[0]+=2;
        pin1 = getinteger(argv[0]);
        if(!code)pin1=codemap(pin1);
        if(IsInvalidPin(pin1)) error("Invalid pin");
        if(ExtCurrentConfig[pin1] != EXT_NOT_CONFIG)  error("Pin %/| is in use",pin1,pin1);
        if(!(code=codecheck(argv[2])))argv[2]+=2;
        pin2 = getinteger(argv[2]);
        if(!code)pin2=codemap(pin2);
        if(IsInvalidPin(pin2)) error("Invalid pin");
        if(ExtCurrentConfig[pin2] != EXT_NOT_CONFIG)  error("Pin %/| is in use",pin2,pin2);
        if(PinDef[pin1].mode & I2C0SDA && PinDef[pin2].mode & I2C0SCL)channel=0;
        if(PinDef[pin1].mode & I2C1SDA && PinDef[pin2].mode & I2C1SCL)channel=1;
        if(channel==-1)error("Invalid I2C pins");
        if(argc==5){
            if(checkstring(argv[4], "SLOW"))Option.SYSTEM_I2C_SLOW=1;
            else if(checkstring(argv[4], "FAST"))Option.SYSTEM_I2C_SLOW=0;
            else error("Syntax");
        }
        Option.SYSTEM_I2C_SDA=pin1;
        Option.SYSTEM_I2C_SCL=pin2;
        SaveOptions();
        _excep_code = RESET_COMMAND;
        SoftReset();
        return;
    }
    tp = checkstring(cmdline, "COUNT");
    if(tp) {
        int pin1,pin2,pin3,pin4;
        if(CallBackEnabled==2) gpio_set_irq_enabled_with_callback(PinDef[Option.INT1pin].GPno, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false, &gpio_callback);
        else if(CallBackEnabled & 2){
            gpio_set_irq_enabled(PinDef[Option.INT1pin].GPno, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
            CallBackEnabled &= (~2);
        }
        if(CallBackEnabled==4) gpio_set_irq_enabled_with_callback(PinDef[Option.INT2pin].GPno, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false, &gpio_callback);
        else if(CallBackEnabled & 4){
            gpio_set_irq_enabled(PinDef[Option.INT2pin].GPno, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
            CallBackEnabled &= (~4);
        }
        if(CallBackEnabled==8) gpio_set_irq_enabled_with_callback(PinDef[Option.INT3pin].GPno, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false, &gpio_callback);
        else  if(CallBackEnabled & 8){
            gpio_set_irq_enabled(PinDef[Option.INT3pin].GPno, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
            CallBackEnabled &= (~8);
        }
        if(CallBackEnabled==16) gpio_set_irq_enabled_with_callback(PinDef[Option.INT4pin].GPno, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false, &gpio_callback);
        else  if(CallBackEnabled & 16){
            gpio_set_irq_enabled(PinDef[Option.INT4pin].GPno, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
            CallBackEnabled &= (~16);
        }
    	getargs(&tp,7,",");
        if(argc!=7)error("Syntax");
        unsigned char code;
        if(!(code=codecheck(argv[0])))argv[0]+=2;
        pin1 = getinteger(argv[0]);
        if(!code)pin1=codemap(pin1);
        if(IsInvalidPin(pin1)) error("Invalid pin");
        if(ExtCurrentConfig[pin1] != EXT_NOT_CONFIG)  error("Pin %/| is in use",pin1,pin1);
        if(!(code=codecheck(argv[2])))argv[2]+=2;
        pin2 = getinteger(argv[2]);
        if(!code)pin2=codemap(pin2);
        if(IsInvalidPin(pin2)) error("Invalid pin");
        if(ExtCurrentConfig[pin2] != EXT_NOT_CONFIG)  error("Pin %/| is in use",pin2,pin2);
        if(!(code=codecheck(argv[4])))argv[4]+=2;
        pin3 = getinteger(argv[4]);
        if(!code)pin3=codemap(pin3);
        if(IsInvalidPin(pin3)) error("Invalid pin");
        if(ExtCurrentConfig[pin3] != EXT_NOT_CONFIG)  error("Pin %/| is in use",pin3,pin3);
        if(!(code=codecheck(argv[6])))argv[6]+=2;
        pin4 = getinteger(argv[6]);
        if(!code)pin4=codemap(pin4);
        if(IsInvalidPin(pin4)) error("Invalid pin");
        if(ExtCurrentConfig[pin4] != EXT_NOT_CONFIG)  error("Pin %/| is in use",pin4,pin4);
        if(pin1==pin2 || pin1==pin3 || pin1==pin4 || pin2==pin3 || pin2==pin4 || pin3==pin4)error("Pins must be unique");
        Option.INT1pin=pin1;
        Option.INT2pin=pin2;
        Option.INT3pin=pin3;
        Option.INT4pin=pin4;
        SaveOptions();
        return;
    }
#ifndef PICOMITEVGA
    tp = checkstring(cmdline, "SYSTEM SPI");
    if(tp) {
        int pin1,pin2,pin3;
        if(checkstring(tp, "DISABLE")){
   	    if(CurrentLinePtr) error("Invalid in a program");
         if((Option.SD_CS && Option.SD_CLK_PIN==0) || Option.TOUCH_CS || Option.LCD_CS)error("In use");
            disable_systemspi();
            SaveOptions();
            _excep_code = RESET_COMMAND;
            SoftReset();
            return;                                // this will restart the processor ? only works when not in debug
        }
    	getargs(&tp,5,",");
   	    if(CurrentLinePtr) error("Invalid in a program");
         if(argc!=5)error("Syntax");
        if(Option.SYSTEM_CLK)error("SDcard already configured");
        unsigned char code;
        if(!(code=codecheck(argv[0])))argv[0]+=2;
        pin1 = getinteger(argv[0]);
        if(!code)pin1=codemap(pin1);
        if(IsInvalidPin(pin1)) error("Invalid pin");
        if(ExtCurrentConfig[pin1] != EXT_NOT_CONFIG)  error("Pin %/| is in use",pin1,pin1);
        if(!(code=codecheck(argv[2])))argv[2]+=2;
        pin2 = getinteger(argv[2]);
        if(!code)pin2=codemap(pin2);
        if(IsInvalidPin(pin2)) error("Invalid pin");
        if(ExtCurrentConfig[pin2] != EXT_NOT_CONFIG)  error("Pin %/| is in use",pin2,pin2);
        if(!(code=codecheck(argv[4])))argv[4]+=2;
        pin3 = getinteger(argv[4]);
        if(!code)pin3=codemap(pin3);
        if(IsInvalidPin(pin3)) error("Invalid pin");
        if(ExtCurrentConfig[pin3] != EXT_NOT_CONFIG)  error("Pin %/| is in use",pin3,pin3);
		if(!(PinDef[pin1].mode & SPI0SCK && PinDef[pin2].mode & SPI0TX  && PinDef[pin3].mode & SPI0RX  ) &&
        !(PinDef[pin1].mode & SPI1SCK && PinDef[pin2].mode & SPI1TX  && PinDef[pin3].mode & SPI1RX  ))error("Not valid SPI pins");
        Option.SYSTEM_CLK=pin1;
        Option.SYSTEM_MOSI=pin2;
        Option.SYSTEM_MISO=pin3;
        SaveOptions();
        _excep_code = RESET_COMMAND;
        SoftReset();
        return;
    }
#endif
	tp = checkstring(cmdline, "SDCARD");
    int pin1, pin2, pin3, pin4;
    if(tp) {
        if(checkstring(tp, "DISABLE")){
            FatFSFileSystem=0;
            disable_sd();
            SaveOptions();
            return;                                // this will restart the processor ? only works when not in debug
        }
    	getargs(&tp,7,",");
#ifdef PICOMITEVGA
        if(!(argc==7))error("Syntax");
#else
        if(!(argc==1 || argc==7))error("Syntax");
#endif
   	    if(CurrentLinePtr) error("Invalid in a program");
         if(Option.SD_CS)error("SDcard already configured");
        if(argc==1 && !Option.SYSTEM_CLK)error("System SPI not configured");
        unsigned char code;
        if(!(code=codecheck(argv[0])))argv[0]+=2;
        pin4 = getinteger(argv[0]);
        if(!code)pin4=codemap(pin4);
        if(IsInvalidPin(pin4)) error("Invalid pin");
        if(ExtCurrentConfig[pin4] != EXT_NOT_CONFIG)  error("Pin %/| is in use",pin4,pin4);
        Option.SD_CS=pin4;
        Option.SDspeed=10;
        if(argc>1){
            if(!(code=codecheck(argv[2])))argv[2]+=2;
            pin1 = getinteger(argv[2]);
            if(!code)pin1=codemap(pin1);
            if(IsInvalidPin(pin1)) error("Invalid pin");
            if(ExtCurrentConfig[pin1] != EXT_NOT_CONFIG)  error("Pin %/| is in use",pin1,pin1);
            if(!(code=codecheck(argv[4])))argv[4]+=2;
            pin2 = getinteger(argv[4]);
            if(!code)pin2=codemap(pin2);
            if(IsInvalidPin(pin2)) error("Invalid pin");
            if(ExtCurrentConfig[pin2] != EXT_NOT_CONFIG)  error("Pin %/| is in use",pin2,pin2);
            if(!(code=codecheck(argv[6])))argv[6]+=2;
            pin3 = getinteger(argv[6]);
            if(!code)pin3=codemap(pin3);
            if(IsInvalidPin(pin3)) error("Invalid pin");
            if(ExtCurrentConfig[pin3] != EXT_NOT_CONFIG)  error("Pin %/| is in use",pin3,pin3);
            Option.SD_CLK_PIN=pin1;
            Option.SD_MOSI_PIN=pin2;
            Option.SD_MISO_PIN=pin3;
        }
        SaveOptions();
        _excep_code = RESET_COMMAND;
        SoftReset();
        return;
    }
	tp = checkstring(cmdline, "RESET");
    if(tp) {
   	    if(CurrentLinePtr) error("Invalid in a program");
        if(Option.LIBRARY_FLASH_SIZE==MAX_PROG_SIZE) {
          uint32_t j = FLASH_TARGET_OFFSET + FLASH_ERASE_SIZE + SAVEDVARS_FLASH_SIZE + ((MAXFLASHSLOTS - 1) * MAX_PROG_SIZE);
          uSec(250000);
          disable_interrupts();
          flash_range_erase(j, MAX_PROG_SIZE);
          enable_interrupts();
        }
        ResetOptions();
        _excep_code = RESET_COMMAND;
        SoftReset();
    }
#ifdef PICOMITEWEB
    checkTCPOptions();
#endif     
    error("Invalid Option");
}

void fun_device(void){
    sret = GetTempMemory(STRINGSIZE);                                        // this will last for the life of the command
#ifdef PICOMITEVGA
    strcpy(sret, "PicoMiteVGA");
#endif
#ifdef PICOMITE
    strcpy(sret, "PicoMite");
#endif
#ifdef PICOMITEWEB
    strcpy(sret, "WebMite");
#endif
    CtoM(sret);
    targ = T_STR;
}

uint32_t __get_MSP(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, msp" : "=r" (result) );
  return(result);
}
int ExistsFile(char *p){
    char q[FF_MAX_LFN]={0};
    int retval=0;
    int waste=0, t=FatFSFileSystem+1;
    int localfilesystemsave=FatFSFileSystem;
    t = drivecheck(p,&waste);
    p+=waste;
    getfullfilename(p,q);
    FatFSFileSystem=t-1;
    if(FatFSFileSystem==0){
        struct lfs_info lfsinfo;
        memset(&lfsinfo,0,sizeof(DIR));
        FSerror = lfs_stat(&lfs, q, &lfsinfo);
        if(lfsinfo.type==LFS_TYPE_REG)retval= 1;
    } else {
        DIR djd;
        FILINFO fnod;
        memset(&djd,0,sizeof(DIR));
        memset(&fnod,0,sizeof(FILINFO));
        if(!InitSDCard()) return -1;
        FSerror = f_stat(q, &fnod);
        if(FSerror != FR_OK)iret=0;
        else if(!(fnod.fattrib & AM_DIR))retval=1;
    }
    FatFSFileSystem=localfilesystemsave;
    return retval;
}
int ExistsDir(char *p, char *q, int *filesystem){
    int ireturn=0;
    ireturn=0;
    int localfilesystemsave=FatFSFileSystem;
    int waste=0, t=FatFSFileSystem+1;
    t = drivecheck(p,&waste);
    p+=waste;
    getfullfilename(p,q);
    FatFSFileSystem=t-1;
    *filesystem=FatFSFileSystem;
    if(strcmp(q,"/")==0 || strcmp(q,"/.")==0 || strcmp(q,"/..")==0 ){FatFSFileSystem=localfilesystemsave;ireturn= 1; return ireturn;}
    if(FatFSFileSystem==0){
        struct lfs_info lfsinfo;
        memset(&lfsinfo,0,sizeof(DIR));
        FSerror = lfs_stat(&lfs, q, &lfsinfo);
        if(lfsinfo.type==LFS_TYPE_DIR)ireturn= 1;
    } else {
        DIR djd;
        FILINFO fnod;
        memset(&djd,0,sizeof(DIR));
        memset(&fnod,0,sizeof(FILINFO));
        if(q[strlen(q)-1]=='/')strcat(q,".");
        if(!InitSDCard()) {FatFSFileSystem=localfilesystemsave;ireturn= -1; return ireturn;}
        FSerror = f_stat(q, &fnod);
        if(FSerror != FR_OK)ireturn=0;
        else if((fnod.fattrib & AM_DIR))ireturn=1;
    }
    FatFSFileSystem=localfilesystemsave;
    return ireturn;
}

void fun_info(void){
    unsigned char *tp;
    sret = GetTempMemory(STRINGSIZE);                                  // this will last for the life of the command
    if(checkstring(ep, "AUTORUN")){
        if(Option.Autorun == false)strcpy(sret,"Off");
        else strcpy(sret,"On");
        CtoM(sret);
        targ=T_STR;
        return;
#ifdef PICOMITEWEB
    } else if((tp=checkstring(ep, "TCP REQUEST"))){
        int i=getint(tp,1,MaxPcb)-1;
        iret=TCPstate->inttrig[i];
        targ=T_INT;
        return;
#endif
#ifdef PICOMITEVGA
    } else if((tp=checkstring(ep, "TILE HEIGHT"))){
        iret=ytilecount;
        targ=T_INT;
        return;
#endif
    } else if(checkstring(ep, "BCOLOUR") || checkstring(ep, "BCOLOR")){
            iret=gui_bcolour;
            targ=T_INT;
            return;
    } else if(checkstring(ep, "BOOT COUNT")){
        int boot_count;
        int FatFSFileSystemSave;
        FatFSFileSystem=0;
        int fnbr=FindFreeFileNbr();
        BasicFileOpen("bootcount",fnbr,FA_READ);
        FSerror==lfs_file_read(&lfs, FileTable[fnbr].lfsptr, &boot_count, sizeof(boot_count));	
        if(FSerror>0)FSerror=0;
        ErrorCheck(fnbr);
        FileClose(fnbr);
        FatFSFileSystem=FatFSFileSystemSave;
        iret=boot_count;
        targ=T_INT;
    } else if(*ep=='c' || *ep=='C'){
        if(checkstring(ep, "CALLTABLE")){
            iret = (int64_t)(uint32_t)CallTable;
            targ = T_INT;
            return;
        } else if(checkstring(ep, "CPUSPEED")){
            IntToStr(sret,Option.CPU_Speed*1000,10);
            CtoM(sret);
            targ=T_STR;
            return;
        } else if(checkstring(ep, "CURRENT")){
            if(ProgMemory[0]==1 && ProgMemory[1]==39 && ProgMemory[2]==35){
                strcpy(sret,&ProgMemory[3]);
            } else strcpy(sret,"NONE");
            CtoM(sret);
            targ=T_STR;
            return;
        } else error("Syntax");
    }  else if(*ep=='d' || *ep=='D'){
        if(checkstring(ep, "DEVICE")){
            fun_device();
            return;
        } else if(tp=checkstring(ep, "DRIVE")){
            strcpy(sret,FatFSFileSystem ? "B:":"A:");
            CtoM(sret);
            targ=T_STR;
            return;
        } else if(checkstring(ep, "DISK SIZE")){
            if(FatFSFileSystem){
                if(!InitSDCard()) error((char *)FErrorMsg[20]);					// setup the SD card
                FATFS *fs;
                DWORD fre_clust;
                /* Get volume information and free clusters of drive 1 */
                f_getfree("0:", &fre_clust, &fs);
                /* Get total sectors and free sectors */
                iret= (uint64_t)(fs->n_fatent - 2) * (uint64_t)fs->csize *(uint64_t)FF_MAX_SS;
            } else {
                iret=(Option.FlashSize-RoundUpK4(TOP_OF_SYSTEM_FLASH));
            }
            targ=T_INT;
            return;
        } else error("Syntax");
    } else if(*ep=='e' || *ep=='E'){
        if(checkstring(ep, "ERRNO")){
            iret = MMerrno;
            targ=T_INT;
            return;
        } else if((tp=checkstring(ep, "EXISTS DIR"))){
            char dir[FF_MAX_LFN]={0};
            char *p = getFstring(tp);
            int filesystem;
            targ=T_INT;
            iret=ExistsDir(p,dir,&filesystem);
            return;
        } else if((tp=checkstring(ep, "EXISTS FILE"))){
            char *p = getFstring(tp);
            iret=ExistsFile(p);
            targ=T_INT;
            return;
        } else if(checkstring(ep, "ERRMSG")){
            int i=OptionFileErrorAbort;
            strcpy(sret, MMErrMsg);
            CtoM(sret);
            targ=T_STR;
            OptionFileErrorAbort=i;
            return;
        } else error("Syntax");
    } else if(*ep=='f' || *ep=='F'){
        if(tp=checkstring(ep, "FONT POINTER")){
            iret=(int64_t)((uint32_t)&FontTable[getint(tp,1,FONT_TABLE_SIZE)-1]);
            targ=T_INT;
            return;
        } else if(tp=checkstring(ep, "FONT ADDRESS")){
            iret=(int64_t)((uint32_t)FontTable[getint(tp,1,FONT_TABLE_SIZE)-1]);
            targ=T_INT;
            return;
        } else if(tp=checkstring(ep, "FILESIZE")){
            int i,j;
            DIR djd;
            FILINFO fnod;
            char q[FF_MAX_LFN]={0};
            memset(&djd,0,sizeof(DIR));
            memset(&fnod,0,sizeof(FILINFO));
            int waste=0, t=FatFSFileSystem+1;
            char *p = getFstring(tp);
            targ=T_INT;
            t = drivecheck(p,&waste);
            p+=waste;
            getfullfilename(p,q);
            FatFSFileSystem=t-1;
            iret=-1;
            if(strcmp(q,"/")==0 || strcmp(q,"/.")==0 || strcmp(q,"/..")==0 ){iret= -2; strcpy(MMErrMsg,FErrorMsg[4]); return;}
            if(FatFSFileSystem==0){
                struct lfs_info lfsinfo;
                FSerror = lfs_stat(&lfs, q, &lfsinfo);
                if(lfsinfo.type==LFS_TYPE_DIR){iret= -2; strcpy(MMErrMsg,FErrorMsg[4]); return;}
                if(FSerror){iret= -1; strcpy(MMErrMsg,FErrorMsg[4]); return;}
                int fnbr=FindFreeFileNbr();
                iret=BasicFileOpen(p,fnbr,FA_READ);
                if(iret==false){
                    iret=-1;
                    targ=T_INT;
                    return;
                }
                iret=lfs_file_size(&lfs,FileTable[fnbr].lfsptr);
                FileClose(fnbr);
            } else {
                if(!InitSDCard()) {iret= -1; return;}
                if(q[strlen(q)-1]=='/')strcat(q,".");
                if(strcmp(q,"/")==0){ iret=-2; targ=T_INT; strcpy(MMErrMsg,FErrorMsg[4]); return;}
                FSerror = f_stat(q, &fnod);
                if((fnod.fattrib & AM_DIR)){ iret=-2; targ=T_INT; strcpy(MMErrMsg,FErrorMsg[4]); return;}
                if(FSerror != FR_OK){ iret=-1; targ=T_INT; strcpy(MMErrMsg,FErrorMsg[4]); return;}
                iret=fnod.fsize;
            }
            FatFSFileSystem=FatFSFileSystemSave;
            targ=T_INT;
            return;
        } else if(checkstring(ep, "FREE SPACE")){
            if(FatFSFileSystem){
                if(!InitSDCard()) error((char *)FErrorMsg[20]);					// setup the SD card
                FATFS *fs;
                DWORD fre_clust;
                /* Get volume information and free clusters of drive 1 */
                f_getfree("0:", &fre_clust, &fs);
                /* Get total sectors and free sectors */
                iret = (uint64_t)fre_clust * (uint64_t)fs->csize  *(uint64_t)FF_MAX_SS;
            } else {
                iret=Option.FlashSize-RoundUpK4(TOP_OF_SYSTEM_FLASH)-lfs_fs_size(&lfs)*4096;
            }
            targ=T_INT;
            return;
        } else if(checkstring(ep, "FLASHTOP")){
            iret = (int64_t)(uint32_t)TOP_OF_SYSTEM_FLASH ;
            targ = T_INT;
            return;
        } else if(checkstring(ep, "FONTWIDTH")){
            iret = FontTable[gui_font >> 4][0] * (gui_font & 0b1111);
            targ=T_INT;
            return;
        } else if(checkstring(ep, "FLASH")){
            iret=FlashLoad;
            targ=T_INT;
            return;
        } else if(checkstring(ep, "FCOLOUR") || checkstring(ep, "FCOLOR") ){
            iret=gui_fcolour;
            targ=T_INT;
            return;
        } else if(checkstring(ep, "FONT")){
            iret=(gui_font >> 4)+1;
            targ=T_INT;
            return;
        } else if(checkstring(ep, "FONTHEIGHT")){
            iret = FontTable[gui_font >> 4][1] * (gui_font & 0b1111);
            targ=T_INT;
            return;
        } else error("Syntax");
    } else if(*ep=='h' || *ep=='H'){
        if(checkstring(ep, "HEAP")){
            iret=FreeSpaceOnHeap();
            targ=T_INT;
            return;
        } else if(checkstring(ep, "HPOS")){
            iret = CurrentX;
            targ=T_INT;
            return;
        } else error("Syntax");
    } else if(checkstring(ep,"ID")){  
        strcpy(sret,id_out);
        CtoM(sret);
        targ=T_STR;
        return;
#ifdef PICOMITEWEB
    } else if(checkstring(ep,"IP ADDRESS")){  
        strcpy(sret,ip4addr_ntoa(netif_ip4_addr(netif_list)));
        CtoM(sret);
        targ=T_STR;
        return;
    } else if(checkstring(ep,"MAX CONNECTIONS")){  
        iret=MaxPcb;
        targ=T_INT;
        return;
    } else if(checkstring(ep,"WIFI STATUS")){  
        iret=cyw43_wifi_link_status(&cyw43_state,CYW43_ITF_STA);
        targ=T_INT;
        return;
    } else if(checkstring(ep,"TCPIP STATUS")){  
        iret=cyw43_tcpip_link_status(&cyw43_state,CYW43_ITF_STA);
        targ=T_INT;
        return;
#endif
    } 
    else if(checkstring(ep, "INTERRUPTS")){
    iret=(int64_t)(uint32_t)*((io_rw_32 *) (PPB_BASE + M0PLUS_NVIC_ISER_OFFSET));
    targ=T_INT;
    return;
    }
#ifndef PICOMITEVGA
    else if(checkstring(ep, "LCDPANEL")){
        strcpy(sret,display_details[Option.DISPLAY_TYPE].name);
        CtoM(sret);
        targ=T_STR;
        return;
    } 
#endif
    else if (checkstring(ep, "LINE")) {
        if (!CurrentLinePtr) {
            strcpy(sret, "UNKNOWN");
        } else if (CurrentLinePtr >= ProgMemory + MAX_PROG_SIZE) {
            strcpy(sret, "LIBRARY");
        } else {
            sprintf(sret, "%d", CountLines(CurrentLinePtr));
        }
        CtoM(sret);
        targ=T_STR;
        return;
    }	
    else if(tp=checkstring(ep, "MODIFIED")){
		int i,j;
	    DIR djd;
	    FILINFO fnod;
        sret = GetTempMemory(STRINGSIZE);                                    // this will last for the life of the command
        targ=T_STR; 
		memset(&djd,0,sizeof(DIR));
		memset(&fnod,0,sizeof(FILINFO));
		char *p = getCstring(tp);
        char q[FF_MAX_LFN]={0};
        int waste=0, t=FatFSFileSystem+1;
        t = drivecheck(p,&waste);
        p+=waste;
        getfullfilename(p,q);
        FatFSFileSystem=t-1;
        if(FatFSFileSystem==0){
            int dt;
            FSerror=lfs_getattr(&lfs, q, 'A', &dt,    4);
            if(FSerror!=4) return;
            else {
                WORD *p=(WORD *)&dt;
                fnod.fdate=(WORD)p[1];
                fnod.ftime=(WORD)p[0];
            }
        } else {
			if(!InitSDCard()) {iret= -1; return;}
            FSerror = f_stat(p, &fnod);
            if(FSerror != FR_OK) return;
        }
	    IntToStr(sret , ((fnod.fdate>>9)&0x7F)+1980, 10);
	    sret[4] = '-'; IntToStrPad(sret + 5, (fnod.fdate>>5)&0xF, '0', 2, 10);
	    sret[7] = '-'; IntToStrPad(sret + 8, fnod.fdate&0x1F, '0', 2, 10);
	    sret[10] = ' ';
	    IntToStrPad(sret+11, (fnod.ftime>>11)&0x1F, '0', 2, 10);
	    sret[13] = ':'; IntToStrPad(sret + 14, (fnod.ftime>>5)&0x3F, '0', 2, 10);
	    sret[16] = ':'; IntToStrPad(sret + 17, (fnod.ftime&0x1F)*2, '0', 2, 10);
        FatFSFileSystem=FatFSFileSystemSave;
		CtoM(sret);
	    targ=T_STR;
		return;
	} else if(tp=checkstring(ep, "OPTION")){
        if(checkstring(tp, "AUTORUN")){
			if(Option.Autorun == false)strcpy(sret,"Off");
            else if(Option.Autorun==MAXFLASHSLOTS+1)strcpy(sret,"On");
			else {
                char b[10];
                IntToStr(b,Option.Autorun,10);
                strcpy(sret,b);
            }
            CtoM(sret);
            targ=T_STR;
            return;
		} else if(checkstring(tp, "BASE")){
			if(OptionBase==1)iret=1;
			else iret=0;
			targ=T_INT;
			return;
		} else if(checkstring(tp, "BREAK")){
			iret=BreakKey;
			targ=T_INT;
			return;
		} else if(checkstring(tp, "ANGLE")){
			if(optionangle==1.0)strcpy(sret,"DEGREES");
			else strcpy(sret,"RADIANS");
 		} else if(checkstring(tp, "DEFAULT")){
			if(DefaultType == T_INT)strcpy(sret,"Integer");
			else if(DefaultType == T_NBR)strcpy(sret,"Float");
			else if(DefaultType == T_STR)strcpy(sret,"String");
			else strcpy(sret,"None");
            CtoM(sret);
            targ=T_STR;
            return;
 		} else if(checkstring(tp, "KEYBOARD")){
            strcpy(sret,(char *)KBrdList[(int)Option.KeyboardConfig]);
            CtoM(sret);
            targ=T_STR;
            return;
 		} else if(checkstring(tp, "EXPLICIT")){
			if(OptionExplicit == false)strcpy(sret,"Off");
			else strcpy(sret,"On");
            CtoM(sret);
            targ=T_STR;
            return;
		} else if(checkstring(tp, "FLASH SIZE")){
            uint8_t txbuf[4] = {0x9f};
            uint8_t rxbuf[4] = {0};
            disable_interrupts();
            flash_do_cmd(txbuf, rxbuf, 4);
            enable_interrupts();
            iret= 1 << rxbuf[3];
			targ=T_INT;
			return;
        } else if(checkstring(tp, "HEIGHT")){
            iret = Option.Height;
            targ = T_INT;
            return;
        } else if(checkstring(tp, "WIDTH")){
            iret = Option.Width;
            targ = T_INT;
            return;
#ifdef PICOMITEWEB
		} else if(checkstring(tp, "SSID")){
			strcpy(sret,Option.SSID);
            CtoM(sret);
            targ=T_STR;
            return;
#endif
		} else error("Syntax");
    } else if(*ep=='p' || *ep=='P'){
        if(tp=checkstring(ep, "PINNO")){
            int pin;
            MMFLOAT f;
            long long int i64;
            unsigned char *ss;
            int t=0;
            char code, *ptr;
            char *string=GetTempMemory(STRINGSIZE);
            if(codecheck(tp))evaluate(tp, &f, &i64, &ss, &t, false);
            if(t & T_STR ){
                ptr=getCstring(tp);
                strcpy(string,ptr);
            } else {
                strcpy(string,tp);
            }
            if(!(code=codecheck(string)))string+=2;  
            else error("Syntax");
            pin = getinteger(string);
            if(!code)pin=codemap(pin);
            if(IsInvalidPin(pin))error("Invalid pin");
            iret=pin;
            targ=T_INT;
            return;
        } else if(tp=checkstring(ep, "PIO RX DMA")){
            iret=dma_channel_is_busy(dma_rx_chan);
            targ=T_INT;
            return;
        } else if(tp=checkstring(ep, "PIO TX DMA")){
            iret=dma_channel_is_busy(dma_tx_chan);
            targ=T_INT;
            return;
        } else if(tp=checkstring(ep, "PWM COUNT")){
            int channel=getint(tp,0,7);
            iret=pwm_hw->slice[channel].top;
            targ=T_INT;
            return;
        } else if(tp=checkstring(ep, "PWM DUTY")){
            getargs(&tp,3,",");
            if(argc!=3)error("Syntax");
            int channel=getint(argv[0],0,7);
            int AorB=getint(argv[2],0,1);
            if(AorB)iret=((pwm_hw->slice[channel].cc) >> 16);
            else iret=(pwm_hw->slice[channel].cc & 0xFFFF);
            targ=T_INT;
            return;
        } else if(tp=checkstring(ep, "PIN")){
            int pin;
            char code;
            if(!(code=codecheck(tp)))tp+=2;
            pin = getinteger(tp);
            if(!code)pin=codemap(pin);
            if(IsInvalidPin(pin))strcpy(sret,"Invalid");
            else strcpy(sret,PinFunction[ExtCurrentConfig[pin] & 0xFF]);
            if(ExtCurrentConfig[pin] & EXT_BOOT_RESERVED)strcat(sret, ": Boot Reserved");
            if(ExtCurrentConfig[pin] & EXT_COM_RESERVED)strcat(sret, ": Reserved for function");
            if(ExtCurrentConfig[pin] & EXT_DS18B20_RESERVED)strcat(sret, ": In use for DS18B20");
            CtoM(sret);
            targ=T_STR;
            return;
        } else if(checkstring(ep, "PROGRAM")){
            iret = (int64_t)(uint32_t)ProgMemory;
            targ = T_INT;
            return;
        } else if(checkstring(ep, "PS2")){
            iret = (int64_t)(uint32_t)PS2code;
            targ = T_INT;
            return;
        } else if(checkstring(ep, "PATH")){
            if(ProgMemory[0]==1 && ProgMemory[1]==39 && ProgMemory[2]==35){
                strcpy(sret,&ProgMemory[3]);
                for(int i=strlen(sret)-1;i>0;i--){
                    if(sret[i]!='/')sret[i]=0;
                    else break;
                }
            } else strcpy(sret,"NONE");
            CtoM(sret);
            targ=T_STR;
            return;
        } else error("Syntax");
    } else if(*ep=='s' || *ep=='S'){
        if(checkstring(ep, "SDCARD")){
            int i=OptionFileErrorAbort;
            OptionFileErrorAbort=0;
            if(!InitSDCard())strcpy(sret,"Not present");
            else  strcpy(sret,"Ready");
            CtoM(sret);
            targ=T_STR;
            OptionFileErrorAbort=i;
            return;
        } else if(checkstring(ep, "SPI SPEED")){
            SPISpeedSet(Option.DISPLAY_TYPE);
            if(PinDef[Option.SYSTEM_CLK].mode & SPI0SCK){
                iret=spi_get_baudrate(spi0);
            } else if(PinDef[Option.SYSTEM_CLK].mode & SPI1SCK){
                iret=spi_get_baudrate(spi1);
            } else error("System SPI not configured");
           targ=T_INT;
            return;
        } else if(checkstring(ep, "STACK")){
            iret=(int64_t)((uint32_t)__get_MSP());
            targ=T_INT;
            return;
        } else if(tp=checkstring(ep, "SYSTICK")){
            iret = (int64_t)(uint32_t)systick_hw->cvr;
            targ = T_INT;
            return;
        } else if(checkstring(ep, "SYSTEM HEAP")){
            iret = (int64_t)(uint32_t)getFreeHeap();
            targ=T_INT;
            return;
        } else if(checkstring(ep, "SOUND")){
            switch(CurrentlyPlaying){
            case P_NOTHING:strcpy(sret,"OFF");break;
            case P_PAUSE_TONE:
            case P_PAUSE_WAV:
            case P_PAUSE_SOUND:
            case P_PAUSE_FLAC:strcpy(sret,"PAUSED");break;
            case P_TONE:strcpy(sret,"TONE");break;
            case P_WAV:strcpy(sret,"WAV");break;
            case P_FLAC:strcpy(sret,"FLAC");break;
            case P_SOUND:strcpy(sret,"SOUND");break;
            }
            CtoM(sret);
            targ=T_STR;
            return;
        } else error("Syntax");
    }
#ifndef PICOMITEVGA
    else if(checkstring(ep, "TOUCH")){
        if(Option.TOUCH_CS == false)strcpy(sret,"Disabled");
        else if(Option.TOUCH_XZERO == TOUCH_NOT_CALIBRATED)strcpy(sret,"Not calibrated");
        else strcpy(sret,"Ready");
        CtoM(sret);
        targ=T_STR;
        return;
    } 
#endif
	else if(checkstring(ep, "TRACK")){
		if(CurrentlyPlaying == P_MP3 || CurrentlyPlaying == P_FLAC || CurrentlyPlaying == P_WAV) strcpy(sret,alist[trackplaying].fn);
		else strcpy(sret,"OFF");
        CtoM(sret);
        targ=T_STR;
        return;
    }
    else if(*ep=='v' || *ep=='V'){
        if(checkstring(ep, "VARCNT")){
            iret=(int64_t)((uint32_t)varcnt);
            targ=T_INT;
            return;
        } else if(checkstring(ep, "VERSION")){
            char *p;
            fret = (MMFLOAT)strtol(VERSION, &p, 10);
            fret += (MMFLOAT)strtol(p + 1, &p, 10) / (MMFLOAT)100.0;
            fret += (MMFLOAT)strtol(p + 1, &p, 10) / (MMFLOAT)10000.0;
            fret += (MMFLOAT)strtol(p + 1, &p, 10) / (MMFLOAT)1000000.0;
            targ=T_NBR;
            return;
        } else if(checkstring(ep, "VPOS")){
            iret = CurrentY;
            targ=T_INT;
            return;
        } else error("Syntax");
	} else if(checkstring(ep, "WRITEBUFF")){
        iret=(int64_t)((uint32_t)WriteBuf);
        targ=T_INT;
        return;
    } else error("Syntax");
}


void cmd_watchdog(void) {
    int i;

    if(checkstring(cmdline, "OFF") != NULL) {
        WDTimer = 0;
    } else {
        i = getinteger(cmdline);
        if(i < 1) error("Invalid argument");
        WDTimer = i;
    }
}

void fun_restart(void) {
    iret = WatchdogSet;
    targ = T_INT;
}


void cmd_cpu(void) {
    unsigned char *p;
    if((p = checkstring(cmdline, "RESTART"))) {
        _excep_code = RESET_COMMAND;
//        while(ConsoleTxBufTail != ConsoleTxBufHead);
        uSec(10000);
        SoftReset();                                                // this will restart the processor ? only works when not in debug
    } else if((p = checkstring(cmdline, "SLEEP"))) {
        	int pullup=0;
            MMFLOAT totalseconds;
            getargs(&p, 3, ",");
            totalseconds=getnumber(p);
            if(totalseconds<=0.0)error("Invalid period");
            sleep_us(totalseconds*1000000);

    } else error("Syntax");
}
void cmd_csubinterrupt(void){
    getargs(&cmdline,1,",");
    if(argc != 0){
        if(checkstring(argv[0],"0")){
            CSubInterrupt = NULL;
            CSubComplete=0;  
        } else {
            CSubInterrupt = GetIntAddress(argv[0]); 
            CSubComplete=0;  
            InterruptUsed = true;
        }
    } else CSubComplete=1;  
}
void cmd_cfunction(void) {
    char *p, EndToken;
    EndToken = GetCommandValue("End DefineFont");           // this terminates a DefineFont
    if(cmdtoken == cmdCSUB) EndToken = GetCommandValue("End CSub");                 // this terminates a CSUB
    p = cmdline;
    while(*p != 0xff) {
        if(*p == 0) p++;                                            // if it is at the end of an element skip the zero marker
        if(*p == 0) error("Missing END declaration");               // end of the program
        if(*p == T_NEWLINE) p++;                                    // skip over the newline token
        if(*p == T_LINENBR) p += 3;                                 // skip over the line number
        skipspace(p);
        if(*p == T_LABEL) {
            p += p[1] + 2;                                          // skip over the label
            skipspace(p);                                           // and any following spaces
        }
        if(*p == EndToken) {                                        // found an END token
            nextstmt = p;
            skipelement(nextstmt);
            return;
        }
        p++;
    }
}




// utility function used by cmd_poke() to validate an address
unsigned int GetPokeAddr(unsigned char *p) {
    unsigned int i;
    i = getinteger(p);
//    if(!POKERANGE(i)) error("Address");
    return i;
}


void cmd_poke(void) {
    unsigned char *p, *q;
    void *pp;
    if(p = checkstring(cmdline, "DISPLAY")){
        if(!Option.DISPLAY_TYPE)error("Display not configured");
        if(q=checkstring(p,"HRES")){ 
            HRes=getint(q,0,1920);
            return;
        } else if(q=checkstring(p,"VRES")){
            VRes=getint(q,0,1200);
            return;
#ifndef PICOMITEVGA
        } else {
            getargs(&p,(MAX_ARG_COUNT * 2) - 3,",");
            if(!argc)return;
            if(Option.DISPLAY_TYPE>=SSDPANEL && Option.DISPLAY_TYPE<VIRTUAL){
                WriteCommand(getinteger(argv[0]));
                for(int i = 2; i < argc; i += 2) {
                    WriteData(getinteger(argv[i]));
                }
                return;
            } else if(Option.DISPLAY_TYPE>I2C_PANEL && Option.DISPLAY_TYPE<ST7920){
                spi_write_command(getinteger(argv[0]));
                for(int i = 2; i < argc; i += 2) {
                    spi_write_data(getinteger(argv[i]));
                }
                return;
            } else if(Option.DISPLAY_TYPE<=I2C_PANEL){
                if(argc>1)error("UNsupported command");
                I2C_Send_Command(getinteger(argv[0]));
                return;
            } else 
            error("Display not supported");
#endif
        } error("Syntax");
    } else {
        getargs(&cmdline, 5, ",");
        if((p = checkstring(argv[0], "BYTE"))) {
            if(argc != 3) error("Argument count");
            uint32_t a=GetPokeAddr(p);
            uint8_t *padd=(uint8_t *)(a);
            *padd = getinteger(argv[2]);
            return;
        }
        if((p = checkstring(argv[0], "SHORT"))) {
            if(argc != 3) error("Argument count");
            uint32_t a=GetPokeAddr(p);
            if(a % 2)error("Address not divisible by 2");
            uint16_t *padd=(uint16_t *)(a);
            *padd = getinteger(argv[2]);
            return;
        }

        if((p = checkstring(argv[0], "WORD"))) {
            if(argc != 3) error("Argument count");
            uint32_t a=GetPokeAddr(p);
            if(a % 4)error("Address not divisible by 4");
            uint32_t *padd=(uint32_t *)(a);
            *padd = getinteger(argv[2]);
            return;
        }

        if((p = checkstring(argv[0], "INTEGER"))) {
            if(argc != 3) error("Argument count");
            uint32_t a=GetPokeAddr(p);
            if(a % 8)error("Address not divisible by 8");
            uint64_t *padd=(uint64_t *)(a);
            *padd = getinteger(argv[2]);
            return;
        }
        if((p = checkstring(argv[0], "FLOAT"))) {
            if(argc != 3) error("Argument count");
            uint32_t a=GetPokeAddr(p);
            if(a % 8)error("Address not divisible by 8");
            MMFLOAT *padd=(MMFLOAT *)(a);
            *padd = getnumber(argv[2]);
            return;
        }

        if(argc != 5) error("Argument count");

        if(checkstring(argv[0], "VARTBL")) {
            *((char *)vartbl + (unsigned int)getinteger(argv[2])) = getinteger(argv[4]);
            return;
        }
        if((p = checkstring(argv[0], "VAR"))) {
            pp = findvar(p, V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
            if(vartbl[VarIndex].type & T_CONST) error("Cannot change a constant");
            *((char *)pp + (unsigned int)getinteger(argv[2])) = getinteger(argv[4]);
            return;
        }
        // the default is the old syntax of:   POKE hiaddr, loaddr, byte
        *(char *)(((int)getinteger(argv[0]) << 16) + (int)getinteger(argv[2])) = getinteger(argv[4]);
    }
}


// function to find a CFunction
// only used by fun_peek() below
unsigned int GetCFunAddr(int *ip, int i,unsigned char *offset) {
    while(*ip != 0xffffffff) {
        //if(*ip++ == (unsigned int)(subfun[i]-ProgMemory)) {                      // if we have a match
        if(*ip++ == (unsigned int)(subfun[i]-offset)) {                      // if we have a match
            ip++;                                                   // step over the size word
            i = *ip++;                                              // get the offset
            return (unsigned int)(ip + i);                          // return the entry point
        }
        ip += (*ip + 4) / sizeof(unsigned int);
    }
    return 0;
}




// utility function used by fun_peek() to validate an address
unsigned int GetPeekAddr(unsigned char *p) {
    unsigned int i;
    i = getinteger(p);
//    if(!PEEKRANGE(i)) error("Address");
    return i;
}

#define SPIsend(a) {uint8_t b=a;xmit_byte_multi(&b,1);}
#define SPIsend2(a) {SPIsend(0);SPIsend(a);}

// Will return a byte within the PIC32 virtual memory space.
void fun_peek(void) {
    unsigned char *p;
    void *pp;
    getargs(&ep, 3, ",");
    if((p = checkstring(argv[0], "BP"))){
        if(argc != 1) error("Syntax");
        uint8_t *ptr1 = (uint8_t *)findvar(p, V_FIND  | V_NOFIND_ERR);
        if(!(vartbl[VarIndex].type & T_INT))error("Not integer variable");
        iret = *(unsigned char *)(uint32_t)vartbl[VarIndex].val.i;
        vartbl[VarIndex].val.i++;
        targ = T_INT;
        return;
    }
    if((p = checkstring(argv[0], "WP"))){
        if(argc != 1) error("Syntax");
        uint32_t *ptr1 = (uint32_t *)findvar(p, V_FIND  | V_NOFIND_ERR);
        if(!(vartbl[VarIndex].type & T_INT))error("Not integer variable");
        if(vartbl[VarIndex].val.i & 3)error("Not on word boundary");
        iret = *(unsigned int *)(uint32_t)vartbl[VarIndex].val.i;
        vartbl[VarIndex].val.i+=4;
        targ = T_INT;
        return;
    }
    if((p = checkstring(argv[0], "SP"))){
        if(argc != 1) error("Syntax");
        uint16_t *ptr1 = (uint16_t *)findvar(p, V_FIND  | V_NOFIND_ERR);
        if(!(vartbl[VarIndex].type & T_INT))error("Not integer variable");
        if(vartbl[VarIndex].val.i & 1)error("Not on short boundary");
        iret = *(unsigned short *)(uint32_t)vartbl[VarIndex].val.i;
        vartbl[VarIndex].val.i+=2;
        targ = T_INT;
        return;
    }
    if((p = checkstring(argv[0], "BYTE"))){
        if(argc != 1) error("Syntax");
        iret = *(unsigned char *)GetPeekAddr(p);
        targ = T_INT;
        return;
        }

    if((p = checkstring(argv[0], "VAR"))){
        pp = findvar(p, V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        iret = *((char *)pp + (int)getinteger(argv[2]));
        targ = T_INT;
        return;
    }
    
    if((p = checkstring(argv[0], "VARADDR"))){
        if(argc != 1) error("Syntax");
        pp = findvar(p, V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        iret = (unsigned int)pp;
        targ = T_INT;
        return;
        }

    if((p = checkstring(argv[0], "VARHEADER"))){
        if(argc != 1) error("Syntax");
        pp = findvar(p, V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        iret = (unsigned int)&vartbl[VarIndex].name[0];
        targ = T_INT;
        return;
        }
        
    if((p = checkstring(argv[0], "CFUNADDR"))){
    	int i,j;
        if(argc != 1) error("Syntax");
        i = FindSubFun(p, true);                                    // search for a function first
        if(i == -1) i = FindSubFun(p, false);                       // and if not found try for a subroutine
        if(i == -1 || !(*subfun[i] == cmdCSUB)) error("Invalid argument");
        // search through program flash and the library looking for a match to the function being called
        j = GetCFunAddr((int *)CFunctionFlash, i,ProgMemory);
        if(!j) j = GetCFunAddr((int *)CFunctionLibrary, i,LibMemory);         //Check the library
        if(!j) error("Internal fault 6(sorry)");
        iret = (unsigned int)j;                                     // return the entry point
        targ = T_INT;
        return;
    }

    if((p = checkstring(argv[0], "WORD"))){
        if(argc != 1) error("Syntax");
        iret = *(unsigned int *)(GetPeekAddr(p) & 0b11111111111111111111111111111100);
        targ = T_INT;
        return;
        }
    if((p = checkstring(argv[0], "SHORT"))){
        if(argc != 1) error("Syntax");
        iret = (unsigned long long int) (*(unsigned short *)(GetPeekAddr(p) & 0b11111111111111111111111111111110));
        targ = T_INT;
        return;
        }
    if((p = checkstring(argv[0], "INTEGER"))){
        if(argc != 1) error("Syntax");
        iret = *(uint64_t *)(GetPeekAddr(p) & 0xFFFFFFF8);
        targ = T_INT;
        return;
        }

    if((p = checkstring(argv[0], "FLOAT"))){
        if(argc != 1) error("Syntax");
        fret = *(MMFLOAT *)(GetPeekAddr(p) & 0xFFFFFFF8);
        targ = T_NBR;
        return;
        }

    if(argc != 3) error("Syntax");

    if((checkstring(argv[0], "PROGMEM"))){
        iret = *((char *)ProgMemory + (int)getinteger(argv[2]));
        targ = T_INT;
        return;
    }

    if((checkstring(argv[0], "VARTBL"))){
        iret = *((char *)vartbl + (int)getinteger(argv[2]));
        targ = T_INT;
        return;
    }


    // default action is the old syntax of  b = PEEK(hiaddr, loaddr)
    iret = *(char *)(((int)getinteger(argv[0]) << 16) + (int)getinteger(argv[2]));
    targ = T_INT;
}
void fun_format(void) {
	unsigned char *p, *fmt;
	int inspec;
	getargs(&ep, 3, ",");
	if(argc%2 == 0) error("Invalid syntax");
	if(argc == 3)
		fmt = getCstring(argv[2]);
	else
		fmt = "%g";

	// check the format string for errors that might crash the CPU
	for(inspec = 0, p = fmt; *p; p++) {
		if(*p == '%') {
			inspec++;
			if(inspec > 1) error("Only one format specifier (%) allowed");
			continue;
		}

		if(inspec == 1 && (*p == 'g' || *p == 'G' || *p == 'f' || *p == 'e' || *p == 'E'|| *p == 'l'))
			inspec++;


		if(inspec == 1 && !(IsDigitinline(*p) || *p == '+' || *p == '-' || *p == '.' || *p == ' '))
			error("Illegal character in format specification");
	}
	if(inspec != 2) error("Format specification not found");
	sret = GetTempMemory(STRINGSIZE);									// this will last for the life of the command
	sprintf(sret, fmt, getnumber(argv[0]));
	CtoM(sret);
	targ=T_STR;
}



/***********************************************************************************************
interrupt check

The priority of interrupts (highest to low) is:
Touch (MM+ only)
CFunction Interrupt
ON KEY
I2C Slave Rx
I2C Slave Tx
COM1
COM2
COM3 (MM+ only)
COM4 (MM+ only)
GUI Int Down (MM+ only)
GUI Int Up (MM+ only)
WAV Finished (MM+ only)
IR Receive
I/O Pin Interrupts in order of definition
Tick Interrupts (1 to 4 in that order)

************************************************************************************************/

// check if an interrupt has occured and if so, set the next command to the interrupt routine
// will return true if interrupt detected or false if not
int checkdetailinterrupts(void) {
    int i, v;
    char *intaddr;
    static char rti[2];

    // check for an  ON KEY loc  interrupt
    if(KeyInterrupt != NULL && Keycomplete) {
		Keycomplete=false;
		intaddr = KeyInterrupt;									    // set the next stmt to the interrupt location
		goto GotAnInterrupt;
	}

    if(OnKeyGOSUB && kbhitConsole()) {
        intaddr = OnKeyGOSUB;                                       // set the next stmt to the interrupt location
        goto GotAnInterrupt;
    }
    if(OnPS2GOSUB && PS2int) {
        intaddr = OnPS2GOSUB;                                       // set the next stmt to the interrupt location
        PS2int=false;
        goto GotAnInterrupt;
    }
    if(piointerrupt){  // have any PIO interrupts been set
#ifdef PICOMITE
        for(int pio=0 ;pio<2;pio++){
            PIO pioinuse=(pio==0 ? pio0 :pio1);
            for(int sm=0;sm<4;sm++){
                int TXlevel=((pioinuse->flevel)>>(sm*4)) & 0xf;
                int RXlevel=((pioinuse->flevel)>>(sm*4+4)) & 0xf;
                if(RXlevel && pioRXinterrupts[sm][pio]){ //is there a character in the buffer and has an interrupt been set?
                    intaddr=pioRXinterrupts[sm][pio];
                    goto GotAnInterrupt;
                }
                if(TXlevel && pioTXinterrupts[sm][pio]){
                    int full=(pioinuse->sm->shiftctrl & (1<<30))  ? 8 : 4;
                    if(TXlevel != full && pioTXlast[sm][pio]==full){ // was the buffer full last time and not now and is an interrupt set?
                        intaddr=pioTXinterrupts[sm][pio];
                        pioTXlast[sm][pio]=TXlevel;
                        goto GotAnInterrupt;
                    }
                }
                pioTXlast[sm][pio]=TXlevel;
            }
        }
#else
#ifdef PICOMITEVGA
        PIO pioinuse=pio1;
#else
		PIO pioinuse=pio0;
#endif
        for(int sm=0;sm<4;sm++){
            int TXlevel=((pioinuse->flevel)>>(sm*4)) & 0xf;
            int RXlevel=((pioinuse->flevel)>>(sm*4+4)) & 0xf;
            if(RXlevel && pioRXinterrupts[sm]){ //is there a character in the buffer and has an interrupt been set?
                intaddr=pioRXinterrupts[sm];
                goto GotAnInterrupt;
            }
            if(TXlevel && pioTXinterrupts[sm]){
                int full=(pioinuse->sm->shiftctrl & (1<<30))  ? 8 : 4;
                if(TXlevel != full && pioTXlast[sm]==full){ // was the buffer full last time and not now and is an interrupt set?
                    intaddr=pioTXinterrupts[sm];
                    pioTXlast[sm]=TXlevel;
                    goto GotAnInterrupt;
                }
            }
            pioTXlast[sm]=TXlevel;
        }
#endif
    }
    if(DMAinterruptRX){
        if(!dma_channel_is_busy(dma_rx_chan)){
            PIO pio = (dma_rx_pio ? pio1: pio0);
            intaddr = (unsigned char *)DMAinterruptRX;
            DMAinterruptRX=NULL;
            pio_sm_set_enabled(pio, dma_rx_sm, false);
            goto GotAnInterrupt;
        }
    }
    if(DMAinterruptTX){
        if(!dma_channel_is_busy(dma_tx_chan)){
            PIO pio = (dma_tx_pio ? pio1: pio0);
            if((pio->flevel>>(dma_tx_sm*8) & 0xf)==0){
                intaddr = (unsigned char *)DMAinterruptTX;
                DMAinterruptTX=NULL;
                pio_sm_set_enabled(pio, dma_tx_sm, false);
                goto GotAnInterrupt;
            }
        }
    }

#ifdef PICOMITE
    if(Ctrl!=NULL){
        if(gui_int_down && GuiIntDownVector) {                          // interrupt on pen down
            intaddr = GuiIntDownVector;                                 // get a pointer to the interrupt routine
            gui_int_down = false;
            goto GotAnInterrupt;
        }

        if(gui_int_up && GuiIntUpVector) {
            intaddr = GuiIntUpVector;                                   // get a pointer to the interrupt routine
            gui_int_up = false;
            goto GotAnInterrupt;
        }
    }
#endif
#ifdef PICOMITEVGA
    if (COLLISIONInterrupt != NULL && CollisionFound) {
        CollisionFound = false;
        intaddr = (unsigned char *)COLLISIONInterrupt;									    // set the next stmt to the interrupt location
        goto GotAnInterrupt;
    }
#endif
#ifdef PICOMITEWEB
    if(TCPreceived && TCPreceiveInterrupt){
        intaddr = TCPreceiveInterrupt;                                   // get a pointer to the interrupt routine
        TCPreceived=0;
        goto GotAnInterrupt;
    }
    if(MQTTComplete && MQTTInterrupt != NULL) {
        MQTTComplete = false;
        intaddr = MQTTInterrupt;                                      // set the next stmt to the interrupt location
        goto GotAnInterrupt;
    }
#endif

    if(ADCInterrupt && dmarunning){
        if(!dma_channel_is_busy(dma_chan)){
            __compiler_memory_barrier();
            adc_run(false);
            adc_fifo_drain();
            int k=0;
            for(int i=0;i<ADCmax;i++){
                for(int j=0;j<ADCopen;j++){
                    if(j==0)*a1float++ = (MMFLOAT)ADCbuffer[k++]/4095.0*VCC;
                    if(j==1)*a2float++ = (MMFLOAT)ADCbuffer[k++]/4095.0*VCC;
                    if(j==2)*a3float++ = (MMFLOAT)ADCbuffer[k++]/4095.0*VCC;
                    if(j==3)*a4float++ = (MMFLOAT)ADCbuffer[k++]/4095.0*VCC;
                }
            }
        intaddr = ADCInterrupt;                                   // get a pointer to the interrupt routine
        dmarunning=0;
        FreeMemory((void *)ADCbuffer);
        goto GotAnInterrupt;
        }
    }

//#ifdef INCLUDE_I2C_SLAVE

    if ((I2C_Status & I2C_Status_Slave_Receive_Rdy)) {
        I2C_Status &= ~I2C_Status_Slave_Receive_Rdy;                // clear completed flag
        intaddr = I2C_Slave_Receive_IntLine;                        // set the next stmt to the interrupt location
        goto GotAnInterrupt;
    }
    if ((I2C_Status & I2C_Status_Slave_Send_Rdy)) {
        I2C_Status &= ~I2C_Status_Slave_Send_Rdy;                   // clear completed flag
        intaddr = I2C_Slave_Send_IntLine;                           // set the next stmt to the interrupt location
        goto GotAnInterrupt;
    }
    if ((I2C2_Status & I2C_Status_Slave_Receive_Rdy)) {
        I2C2_Status &= ~I2C_Status_Slave_Receive_Rdy;                // clear completed flag
        intaddr = I2C2_Slave_Receive_IntLine;                        // set the next stmt to the interrupt location
        goto GotAnInterrupt;
    }
    if ((I2C2_Status & I2C_Status_Slave_Send_Rdy)) {
        I2C2_Status &= ~I2C_Status_Slave_Send_Rdy;                   // clear completed flag
        intaddr = I2C2_Slave_Send_IntLine;                           // set the next stmt to the interrupt location
        goto GotAnInterrupt;
    }
//#endif
    if(WAVInterrupt != NULL && WAVcomplete) {
        WAVcomplete=false;
		intaddr = WAVInterrupt;									    // set the next stmt to the interrupt location
		goto GotAnInterrupt;
	}

    // interrupt routines for the serial ports
    if(com1_interrupt != NULL && SerialRxStatus(1) >= com1_ilevel) {// do we need to interrupt?
        intaddr = com1_interrupt;                                   // set the next stmt to the interrupt location
        goto GotAnInterrupt;
    }
    if(com2_interrupt != NULL && SerialRxStatus(2) >= com2_ilevel) {// do we need to interrupt?
        intaddr = com2_interrupt;                                   // set the next stmt to the interrupt location
        goto GotAnInterrupt;
    }
    if(IrGotMsg && IrInterrupt != NULL) {
        IrGotMsg = false;
        intaddr = IrInterrupt;                                      // set the next stmt to the interrupt location
        goto GotAnInterrupt;
    }

    if(KeypadInterrupt != NULL && KeypadCheck()) {
        intaddr = KeypadInterrupt;                                  // set the next stmt to the interrupt location
        goto GotAnInterrupt;
    }

    if(CSubInterrupt != NULL && CSubComplete) {
        intaddr = CSubInterrupt;                                  // set the next stmt to the interrupt location
        CSubComplete=0;
        goto GotAnInterrupt;
    }

    for(i = 0; i < NBRINTERRUPTS; i++) {                            // scan through the interrupt table
        if(inttbl[i].pin != 0) {                                    // if this entry has an interrupt pin set
            v = ExtInp(inttbl[i].pin);                              // get the current value of the pin
            // check if interrupt occured
            if((inttbl[i].lohi == T_HILO && v < inttbl[i].last) || (inttbl[i].lohi == T_LOHI && v > inttbl[i].last) || (inttbl[i].lohi == T_BOTH && v != inttbl[i].last)) {
                intaddr = inttbl[i].intp;                           // set the next stmt to the interrupt location
                inttbl[i].last = v;                                 // save the new pin value
                goto GotAnInterrupt;
            } else
                inttbl[i].last = v;                                 // no interrupt, just update the pin value
        }
    }

    // check if one of the tick interrupts is enabled and if it has occured
    for(i = 0; i < NBRSETTICKS; i++) {
        if(TickInt[i] != NULL && TickTimer[i] > TickPeriod[i]) {
            // reset for the next tick but skip any ticks completely missed
            while(TickTimer[i] > TickPeriod[i]) TickTimer[i] -= TickPeriod[i];
            intaddr = TickInt[i];
            if(intaddr)goto GotAnInterrupt;
        }
    }

    // if no interrupt was found then return having done nothing
    return 0;

    // an interrupt was found if we jumped to here
GotAnInterrupt:
    LocalIndex++;                                                   // IRETURN will decrement this
    if(OptionErrorSkip>0)SaveOptionErrorSkip=OptionErrorSkip;
    else SaveOptionErrorSkip = 0;
    OptionErrorSkip=0;
    strcpy(SaveErrorMessage , MMErrMsg);
    Saveerrno = MMerrno;
    *MMErrMsg = 0;
    MMerrno = 0;
    InterruptReturn = nextstmt;                                     // for when IRETURN is executed
    // if the interrupt is pointing to a SUB token we need to call a subroutine
    if(*intaddr == cmdSUB) {
    	strncpy(CurrentInterruptName, intaddr + 1, MAXVARLEN);
    	rti[0] = cmdIRET;                                           // setup a dummy IRETURN command
        rti[1] = 0;
        if(gosubindex >= MAXGOSUB) error("Too many SUBs for interrupt");
        errorstack[gosubindex] = CurrentLinePtr;
        gosubstack[gosubindex++] = rti;                             // return from the subroutine to the dummy IRETURN command
        LocalIndex++;                                               // return from the subroutine will decrement LocalIndex
        skipelement(intaddr);                                       // point to the body of the subroutine
    }

    nextstmt = intaddr;                                             // the next command will be in the interrupt routine
    return 1;
}
int __not_in_flash_func(check_interrupt)(void) {
#ifdef PICOMITE
    if(Ctrl!=NULL){
        if(!(DelayedDrawKeyboard || DelayedDrawFmtBox || calibrate) )ProcessTouch();
        if(CheckGuiFlag) CheckGui();                                    // This implements a LED flash
    }
#endif
    if(Option.KeyboardConfig)CheckKeyboard();
    if(!InterruptUsed) return 0;                                    // quick exit if there are no interrupts set
    if(InterruptReturn != NULL || CurrentLinePtr == NULL) return 0; // skip if we are in an interrupt or in immediate mode
    return checkdetailinterrupts();
}


// get the address for a MMBasic interrupt
// this will handle a line number, a label or a subroutine
// all areas of MMBasic that can generate an interrupt use this function
unsigned char *GetIntAddress(unsigned char *p) {
    int i;
    if(isnamestart((uint8_t)*p)) {                                           // if it starts with a valid name char
        i = FindSubFun(p, 0);                                       // try to find a matching subroutine
        if(i == -1)
            return findlabel(p);                                    // if a subroutine was NOT found it must be a label
        else
            return subfun[i];                                       // if a subroutine was found, return the address of the sub
    }

    return findline(getinteger(p), true);                           // otherwise try for a line number
}
