/***********************************************************************************************************************
PicoMite MMBasic

custom.c

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
#include <stdio.h>

#include "MMBasic_Includes.h"
#include "Hardware_Includes.h"
#include "hardware/dma.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/structs/dma.h"
#ifdef PICOMITEWEB
#define CJSON_NESTING_LIMIT 100
#include "cJSON.h"
#endif
#define STATIC static

/*************************************************************************************************************************
**************************************************************************************************************************
IMPORTANT:
This module is empty and should be used for your special functions and commands.  In the standard distribution this file  
will never be changed, so your code should be safe here.  You should avoid placing commands and functions in other files as
they may be changed and you would then need to re insert your changes in a new release of the source.

**************************************************************************************************************************
**************************************************************************************************************************/


/********************************************************************************************************************************************
 custom commands and functions
 each function is responsible for decoding a command
 all function names are in the form cmd_xxxx() (for a basic command) or fun_xxxx() (for a basic function) so, if you want to search for the
 function responsible for the NAME command look for cmd_name

 There are 4 items of information that are setup before the command is run.
 All these are globals.

 int cmdtoken	This is the token number of the command (some commands can handle multiple
				statement types and this helps them differentiate)

 unsigned char *cmdline	This is the command line terminated with a zero unsigned char and trimmed of leading
				spaces.  It may exist anywhere in memory (or even ROM).

 unsigned char *nextstmt	This is a pointer to the next statement to be executed.  The only thing a
				command can do with it is save it or change it to some other location.

 unsigned char *CurrentLinePtr  This is read only and is set to NULL if the command is in immediate mode.

 The only actions a command can do to change the program flow is to change nextstmt or
 execute longjmp(mark, 1) if it wants to abort the program.

 ********************************************************************************************************************************************/
/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/claim.h"
#define PIO_NUM(pio) ((pio) == pio0 ? 0 : 1)
#define CLKMIN ((Option.CPU_Speed*125)>>13)
#define CLKMAX (Option.CPU_Speed *1000)

#include "hardware/pio.h"
#include "hardware/pio_instructions.h"
#ifdef PICOMITE
char *pioRXinterrupts[4][2]={0};
char *pioTXinterrupts[4][2]={0};
uint8_t pioTXlast[4][2]={0};
#else
char *pioRXinterrupts[4]={0};
char *pioTXinterrupts[4]={0};
uint8_t pioTXlast[4]={0};
#endif
char *DMAinterruptRX=NULL;
char *DMAinterruptTX=NULL;
uint32_t dma_rx_chan;
uint32_t dma_tx_chan;
int dma_tx_pio;
int dma_tx_sm;
int dma_rx_pio;
int dma_rx_sm;
#ifdef PICOMITEWEB
volatile int TCPreceived=0;
char *TCPreceiveInterrupt=NULL;
#endif
#define MAXLABEL 16
static int sidepins=0,sideopt=0,sidepinsdir=0, PIOlinenumber=0, PIOstart=0;
static int delaypossible=5;
static int checksideanddelay=0;
int dirOK=2;
static int *instructions;
static char *labelsfound, *labelsneeded;
int piointerrupt=0;
static inline uint32_t pio_sm_calc_wrap(uint wrap_target, uint wrap) {
    uint32_t calc=0;
    valid_params_if(PIO, wrap < PIO_INSTRUCTION_COUNT);
    valid_params_if(PIO, wrap_target < PIO_INSTRUCTION_COUNT);
    return  (calc & ~(PIO_SM0_EXECCTRL_WRAP_TOP_BITS | PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS)) |
            (wrap_target << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB) |
            (wrap << PIO_SM0_EXECCTRL_WRAP_TOP_LSB);
}
int calcsideanddelay(char *p, int sidepins, int maxdelaybits){
        int data=0;
        char *pp;
        if((pp=fstrstr(p,"side "))){
                pp+=5;
                char *ppp=pp;
                while(*ppp>='0' && *ppp<='9' && *ppp){ppp++;}
                if(*ppp)*ppp=',';
                data=(getint(pp,0,(1<<(sidepins-sideopt)))<<(8+maxdelaybits));
                if(sideopt)data|=0x1000;
        }
        if((pp=fstrstr(p,"["))){
                pp++;
                char *s=strstr(pp,"]");
                *s=' ';
                data|=(getint(pp,0,(1<<maxdelaybits)-1))<<8;
        }
        return data;
}
int getirqnum(char *p){
        int data=0;
        char *pp=p;
        int rel=0;
        skipspace(pp);
        char *ppp=pp;
        char save;
        while(*ppp>='0' && *ppp<='9' && *ppp){ppp++;}
        if(*ppp){
                save=*ppp;
                *ppp=',';
        }
        data=getint(pp,0,7);
        if(*ppp==',')*ppp=save;
        if((pp=fstrstr(p," rel")) && (pp[4]==0 || pp[4]==' ' || pp[4]==';'))rel=1;
        if(rel){
                data|=0x10;
        }
        return data;
}
int checkblock(char *p){
        int data=0;
        char *pp;
        if((pp=fstrstr(p,"IFFULL"))){
                if(!(pp[6]==' ' || pp[6]==0))error("Syntax");
                data=0b1000000;
        }
        if((pp=fstrstr(p,"BLOCK"))){
                if(!(pp[5]==' ' || pp[5]==0))error("Syntax");
                data=0b100000;
        }
        if((pp=fstrstr(p,"NOBLOCK"))){
                if(!(pp[7]==' ' || pp[7]==0))error("Syntax");
        }
        return data;
}
void cmd_pio(void){
    unsigned char *tp;
    tp = checkstring(cmdline, "EXECUTE");
    if(tp){
        int i;
        getargs(&tp, (MAX_ARG_COUNT * 2) - 1, (unsigned char *)",");
        if((argc & 0x01) == 0) error("Syntax");
        if(argc<5)error("Syntax");
#ifdef PICOMITEVGA
        PIO pio = (getint(argv[0],1,1) ? pio1: pio0);
#endif
#ifdef PICOMITE
        PIO pio = (getint(argv[0],0,1) ? pio1: pio0);
#endif
#ifdef PICOMITEWEB
        PIO pio = (getint(argv[0],0,0) ? pio1: pio0);
#endif
        int sm=getint(argv[2],0,3);
        for(i = 4; i < argc; i += 2) {
            pio_sm_exec(pio, sm, getint(argv[i],0,65535));
        }
        return;
    }
    tp = checkstring(cmdline, "WRITE");
    if(tp){
        int i=6;
        getargs(&tp, (MAX_ARG_COUNT * 2) - 1, (unsigned char *)",");
        if((argc & 0x01) == 0) error("Syntax");
        if(argc<5)error("Syntax");
#ifdef PICOMITEVGA
        PIO pio = (getint(argv[0],1,1) ? pio1: pio0);
#endif
#ifdef PICOMITE
        PIO pio = (getint(argv[0],0,1) ? pio1: pio0);
#endif
#ifdef PICOMITEWEB
        PIO pio = (getint(argv[0],0,0) ? pio1: pio0);
#endif
        int sm=getint(argv[2],0,3);
        int count=getint(argv[4],0,MAX_ARG_COUNT-3);
        while(count--) {
            pio_sm_put_blocking(pio, sm, getint(argv[i],0,0xFFFFFFFF));
            i+=2;
        }
        return;
    }
    tp = checkstring(cmdline, "DMA RX");
    if(tp){
        getargs(&tp, 13, (unsigned char *)",");
        if(checkstring(argv[0],"OFF")){
                if(dma_channel_is_busy(dma_rx_chan)){
                        dma_channel_abort(dma_rx_chan);
                } else error("Not active");
                return;
        }
        if(DMAinterruptRX || dma_channel_is_busy(dma_rx_chan)) error("DMA active");
        if(argc<7)error("Syntax");
#ifdef PICOMITEVGA
        int i=  getint(argv[0],1,1);
#endif
#ifdef PICOMITE
        int i=  getint(argv[0],0,1);
#endif
#ifdef PICOMITEWEB
        int i=  getint(argv[0],0,0);
#endif
        PIO pio = (i ? pio1: pio0);
        int sm=getint(argv[2],0,3);
        dma_rx_pio=i;
        dma_rx_sm=sm;
        int nbr=getint(argv[4],0,0xFFFFFFFF);
        uint32_t *a1int=NULL;
	void *ptr1 = NULL;
        int toarraysize;
        ptr1 = findvar(argv[6], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(vartbl[VarIndex].type &  T_INT) {
                if(vartbl[VarIndex].dims[1] != 0) error("Target must be an 1D integer array");
                if(vartbl[VarIndex].dims[0] <= 0) {		// Not an array
                        error("Target must be an 1D integer array");
                }
                toarraysize=vartbl[VarIndex].dims[0]-OptionBase+1;
                a1int = (uint32_t *)ptr1;
                if((uint32_t)ptr1!=(uint32_t)vartbl[VarIndex].val.s)error("Syntax");
        } else error("Target must be an 1D integer array");
        if(argc>=9 && *argv[10]){
                DMAinterruptRX=GetIntAddress(argv[8]);
                InterruptUsed=true;
        }
        int dmasize=DMA_SIZE_32;
        if(argc==11){
                dmasize=getinteger(argv[10]);
                if(!(dmasize==8 || dmasize==16 || dmasize==32))error("Invalid transfer size");
                if(dmasize==8)dmasize=DMA_SIZE_8;
                else if(dmasize==16)dmasize=DMA_SIZE_16;
                else if(dmasize==32)dmasize=DMA_SIZE_32;
        }
        dma_rx_chan = PIO_RX_DMA;

        dma_channel_config c = dma_channel_get_default_config(dma_rx_chan);
        channel_config_set_read_increment(&c, false);
        channel_config_set_transfer_data_size(&c, dmasize);
        channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));
        if(argc==13){
                int size=getinteger(argv[12]);
                if(!(size==1 || size==2 || size==4 || size==8 || size==16 || size==32 || size==64 || size==128 || size==256 || size==512 || size==1024 || size== 2048 || size==4096 || size==8192 || size==16384 || size==32768))error("Not power of 2");
                if(size!=1){
                        int i=0,j=size;
                        if((uint32_t)a1int & (j-1))error("Data alignment error");
                        while(j>>=1)i++;
                        i+=dmasize;
                        if((1<<i)>(toarraysize*8))error("Array size");
                        channel_config_set_ring(&c,true,i);
                        channel_config_set_write_increment(&c, true);
                } else channel_config_set_write_increment(&c, false);
        } else {
                if((nbr<<dmasize)>(toarraysize*8))error("Array size");
                channel_config_set_write_increment(&c, true);
        } 
        dma_channel_configure(dma_rx_chan, &c,
                a1int,        // Destination pointer
                &pio->rxf[sm],      // Source pointer
                nbr, // Number of transfers
                true                // Start immediately
        );
        pio_sm_restart(pio, sm);
        pio_sm_set_enabled(pio, sm, true);
        return;
    }
    tp = checkstring(cmdline, "DMA TX");
    if(tp){
        getargs(&tp, 13, (unsigned char *)",");
        if(checkstring(argv[0],"OFF")){
                if(dma_channel_is_busy(dma_tx_chan)){
                        dma_channel_abort(dma_tx_chan);
                } else error("Not active");
                return;
        }
        if(DMAinterruptTX || dma_channel_is_busy(dma_tx_chan)) error("DMA active");
        if(argc<7)error("Syntax");
#ifdef PICOMITEVGA
        int i=  getint(argv[0],1,1);
#endif
#ifdef PICOMITE
        int i=  getint(argv[0],0,1);
#endif
#ifdef PICOMITEWEB
        int i=  getint(argv[0],0,0);
#endif
        PIO pio = (i ? pio1: pio0);
        int sm=getint(argv[2],0,3);
        dma_tx_pio=i;
        dma_tx_sm=sm;
        int nbr=getint(argv[4],0,0xFFFFFFFF);
        uint32_t *a1int=NULL;
	void *ptr1 = NULL;
        int toarraysize;
        ptr1 = findvar(argv[6], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(vartbl[VarIndex].type &  T_INT) {
                if(vartbl[VarIndex].dims[1] != 0) error("Target must be an 1D integer array");
                if(vartbl[VarIndex].dims[0] <= 0) {		// Not an array
                        error("Target must be an 1D integer array");
                }
                toarraysize=vartbl[VarIndex].dims[0]-OptionBase+1;
                a1int = (uint32_t *)ptr1;
                if((uint32_t)ptr1!=(uint32_t)vartbl[VarIndex].val.s)error("Syntax");
        } else error("Target must be an 1D integer array");
        if(argc>=9 && *argv[8]){
                DMAinterruptTX=GetIntAddress(argv[8]);
                InterruptUsed=true;
        }
        int dmasize=DMA_SIZE_32;
        if(argc>=11 && *argv[10]){
                dmasize=getinteger(argv[10]);
                if(!(dmasize==8 || dmasize==16 || dmasize==32))error("Invalid transfer size");
                if(dmasize==8)dmasize=DMA_SIZE_8;
                else if(dmasize==16)dmasize=DMA_SIZE_16;
                else if(dmasize==32)dmasize=DMA_SIZE_32;
        }
        dma_tx_chan = PIO_TX_DMA;
        dma_channel_config c = dma_channel_get_default_config(dma_tx_chan);
        channel_config_set_write_increment(&c, false);
        channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));
        channel_config_set_transfer_data_size(&c, dmasize);
        if(argc==13){
                int size=getinteger(argv[12]);
                if(!(size==1 || size==2 || size==4 || size==8 || size==16 || size==32 || size==64 || size==128 || size==256 || size==512 || size==1024 || size== 2048 || size==4096 || size==8192 || size==16384 || size==32768))error("Not power of 2");
                if(size!=1){
                        int i=0,j=size;
                        if((uint32_t)a1int & (j-1))error("Data alignment error");
                        while(j>>=1)i++;
                        i+=dmasize;
                        if((1<<i)>(toarraysize*8))error("Array size");
                        channel_config_set_read_increment(&c, true);
                        channel_config_set_ring(&c,false,i);
                } else channel_config_set_read_increment(&c, false);
        } else {
                if((nbr<<dmasize)>(toarraysize*8))error("Array size");
                channel_config_set_read_increment(&c, true);
        } 
        dma_channel_configure(dma_tx_chan,
                &c,
                &pio->txf[sm],      // Destination pointer
                a1int,        // Source pointer
                nbr, // Number of transfers
                true                // Start immediately
        );
        pio_sm_restart(pio, sm);
        pio_sm_set_enabled(pio, sm, true);
        return;
    }
    tp = checkstring(cmdline, "INTERRUPT");
    if(tp){
        unsigned char *p;
        unsigned int nbr, *d;
        long long int *dd;
        int i;
        getargs(&tp, 7, (unsigned char *)",");
        if((argc & 0x01) == 0) error("Syntax");
        if(argc<5)error("Syntax");
#ifndef PICOMITE
#ifdef PICOMITEWEB
        i=getint(argv[0],0,0);
#else
        i=getint(argv[0],1,1);
#endif
        PIO pio = (i ? pio1: pio0);
        int sm=getint(argv[2],0,3);
        if(*argv[4]){
                if(checkstring(argv[4],"0"))pioRXinterrupts[sm]=NULL;
                else pioRXinterrupts[sm]=GetIntAddress(argv[4]);
        }
        if(argc==7){
                if(checkstring(argv[6],"0"))pioTXinterrupts[sm]=NULL;
                else pioTXinterrupts[sm]=GetIntAddress(argv[6]);
        }
        piointerrupt=0;
        for(int i=0;i<4;i++){
                if(pioRXinterrupts[i] || pioTXinterrupts[i]){
                        piointerrupt=1;
                        InterruptUsed=1;
                }
        }
#else
        i=getint(argv[0],0,1);
        PIO pio = (i ? pio1: pio0);
        int sm=getint(argv[2],0,3);
        if(*argv[4]){
                if(checkstring(argv[4],"0"))pioRXinterrupts[sm][i]=NULL;
                else pioRXinterrupts[sm][i]=GetIntAddress(argv[4]);
        }
        if(argc==7){
                if(checkstring(argv[6],"0"))pioTXinterrupts[sm][i]=NULL;
                else pioTXinterrupts[sm][i]=GetIntAddress(argv[6]);
        }
        piointerrupt=0;
        for(int i=0;i<4;i++){
                for(int j=0 ;j<2;j++){
                        if(pioRXinterrupts[i][j] || pioTXinterrupts[i][j]){
                                piointerrupt=1;
                                InterruptUsed=1;
                        }
                }
        }
#endif
        return;
    }
    tp = checkstring(cmdline, "READ");
    if(tp){
        unsigned char *p;
        unsigned int nbr, *d;
        long long int *dd;
        int i;
        getargs(&tp, (MAX_ARG_COUNT * 2) - 1, (unsigned char *)",");
        if((argc & 0x01) == 0) error("Syntax");
        if(argc<5)error("Syntax");
#ifdef PICOMITEVGA
        PIO pio = (getint(argv[0],1,1) ? pio1: pio0);
#endif
#ifdef PICOMITE
        PIO pio = (getint(argv[0],0,1) ? pio1: pio0);
#endif
#ifdef PICOMITEWEB
        PIO pio = (getint(argv[0],0,0) ? pio1: pio0);
#endif
        int sm=getint(argv[2],0,3);
        nbr = getinteger(argv[4]);
        dd = findvar(argv[6], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(((vartbl[VarIndex].type & T_INT) && vartbl[VarIndex].dims[0] > 0 && vartbl[VarIndex].dims[1] == 0))
        {		// integer array
            if( (((long long int *)dd - vartbl[VarIndex].val.ia) + nbr) > (vartbl[VarIndex].dims[0] + 1 - OptionBase) )
                error("Insufficient array size");
        }  else  if ((vartbl[VarIndex].type & T_INT) && vartbl[VarIndex].dims[0] == 0 && nbr==1){
            // single variable
        }  else error("Invalid variable");
        
        while(nbr--) {
       	    *dd = pio_sm_get(pio, sm);
            if(pio->fdebug & (1<<(sm + 16)))*dd=-1;
       	    if(nbr)dd++;
        }
        return;
    }
    tp = checkstring(cmdline, "PROGRAM LINE");
    if(tp){
        getargs(&tp,5,",");
        if(argc!=5)error("Syntax");
#ifdef PICOMITEVGA
        PIO pio = (getint(argv[0],1,1) ? pio1: pio0);
#endif
#ifdef PICOMITE
        PIO pio = (getint(argv[0],0,1) ? pio1: pio0);
#endif
#ifdef PICOMITEWEB
        PIO pio = (getint(argv[0],0,0) ? pio1: pio0);
#endif
        int slot=getint(argv[2],0,31);
        int instruction=getint(argv[4],0,0xFFFF);
        pio->instr_mem[slot]=instruction;
        return;
    }
    tp = checkstring(cmdline, "ASSEMBLE");
    if(tp){
        getargs(&tp,3,",");
        if(argc!=3)error("Syntax");
#ifdef PICOMITEVGA
        PIO pio = (getint(argv[0],1,1) ? pio1: pio0);
#endif
#ifdef PICOMITE
        PIO pio = (getint(argv[0],0,1) ? pio1: pio0);
#endif
#ifdef PICOMITEWEB
        PIO pio = (getint(argv[0],0,0) ? pio1: pio0);
#endif
        unsigned int ins=0;
        char *ss=getCstring(argv[2]);
        char *comment=strchr(ss,';');
        if(comment)*comment=0;
        skipspace(ss);
        if(*ss==0)return;
        if(!strncasecmp(ss,".PROGRAM ",9)){
                if(dirOK!=2)error("Program already started");
                sidepins=0,sideopt=0,sidepinsdir=0;
                delaypossible=5;
                checksideanddelay=0;
                dirOK=1;
                PIOlinenumber=0;
                instructions = (int *)GetMemory(sizeof(int)*32);
                for(int i=0;i<32;i++)instructions[i]=-1;
                labelsfound = GetMemory(32 * MAXLABEL);
                labelsneeded = GetMemory(32 * MAXLABEL);
                PIOstart=0;
                return;
        }
        if(dirOK!=2){
                char *p;
                if(strchr(ss,':') && !fstrstr(ss,"::")){
                        p=strchr(ss,':');
                        skipspace(ss);
                        *p=0;
                        if(((uint32_t)p-(uint32_t)ss) > (MAXLABEL-1))error("Label too long");
                        for(int j=PIOstart;j<PIOlinenumber;j++){
                                if(strcasecmp(ss,&labelsfound[j*MAXLABEL])==0) {
                                        error("Duplicate label");
                                }
                        }
                        strcpy(&labelsfound[MAXLABEL*PIOlinenumber],ss);
                        return;
                } else if(*ss!='.'){
                        if(PIOlinenumber>31)error("Program too large");
                        if(!strncasecmp(ss,"JMP ",4)){
                                int dup=0;checksideanddelay=1;
                                ss+=3;
                                dirOK=0;
                                skipspace(ss);
                                if(strncasecmp(ss,"!X",2)==0 && (ss[2]==' ' || ss[2]==',')) {
                                        ins|=0x20;
                                        dup=1;
                                        ss+=2;
                                } 
                                if(strncasecmp(ss,"X--",3)==0 && (ss[3]==' ' || ss[3]==',')) {
                                        if(dup)error("Syntax");
                                        ins|=0x40;
                                        dup=1;
                                        ss+=3;
                                }
                                if(strncasecmp(ss,"!Y",2)==0 && (ss[2]==' ' || ss[2]==',')) {
                                        if(dup)error("Syntax");
                                        ins|=0x60;
                                        dup=1;
                                        ss+=2;
                                }
                                if(strncasecmp(ss,"Y--",3)==0 && (ss[3]==' ' || ss[3]==',')) {
                                        if(dup)error("Syntax");
                                        ins|=0x80;
                                        dup=1;
                                        ss+=3;
                                }
                                if(strncasecmp(ss,"X!=Y",4)==0 && (ss[4]==' ' || ss[4]==',')) {
                                        if(dup)error("Syntax");
                                        ins|=0xA0;
                                        dup=1;
                                        ss+=4;
                                }
                                if(strncasecmp(ss,"PIN",3)==0 && (ss[3]==' ' || ss[3]==',')) {
                                        if(dup)error("Syntax");
                                        ins|=0xC0;
                                        dup=1;
                                        ss+=3;
                                }
                                if(strncasecmp(ss,"!OSRE",5)==0 && (ss[5]==' ' || ss[5]==',')) {
                                        if(dup)error("Syntax");
                                        ins|=0xC0;
                                        dup=1;
                                        ss+=5;
                                }
                                char save;
                                skipspace(ss);
                                if(*ss==','){
                                        ss++;
                                        skipspace(ss);
                                }
                                if(*ss>='0' && *ss<='9'){
                                        char *ppp=ss;
                                        while(*ppp>='0' && *ppp<='9' && *ppp){ppp++;}
                                        if(*ppp){
                                                save=*ppp;
                                                *ppp=',';
                                        }

                                        ins|=getint(ss,0,31);
                                        ins|=0x10000;
                                        if(*ppp==',')*ppp=save;
                                } else {
                                        char *ppp=ss;
                                        if(!isnamestart(*ppp))error("Syntax");
                                        while(isnamechar(*ppp)){ppp++;}
                                        if(*ppp){
                                                save=*ppp;
                                                *ppp=0;
                                        }
                                        strcpy(&labelsneeded[PIOlinenumber*MAXLABEL],ss);
                                        *ppp=save;
                                }
                        } else if(!strncasecmp(ss,"WAIT ",5)){
                                dirOK=0;
                                ss+=4;
                                ins=0x2000;checksideanddelay=1;
                                skipspace(ss);
                                if(!(*ss=='1' || *ss=='0'))error("Syntax");
                                if(*ss=='1')ins |=0x80;
                                ss++;
                                skipspace(ss);
                                if(*ss==','){
                                        ss++;
                                        skipspace(ss);
                                }
                                int rel=2;
                                if(strncasecmp(ss,"GPIO",4)==0 && (ss[4]==' ' || ss[4]==',')){
                                        ss+=4;
                                } else if(strncasecmp(ss,"PIN",3)==0 && (ss[3]==' ' || ss[3]==',')){
                                        ss+=3;
                                        ins |=0b100000;
                                } else if(strncasecmp(ss,"IRQ",3)==0 && (ss[3]==' ' || ss[3]==',')){
                                        char *pp;
                                        ss+=3;
                                        rel=0;
                                        ins |=0b1000000;
                                        if((pp=fstrstr(ss," rel")) && (pp[4]==0 || pp[4]==' ' || pp[4]==';'))rel=1;
                                } else error("syntax");
                                skipspace(ss);
                                if(*ss==','){
                                        ss++;
                                        skipspace(ss);
                                }
                                char save;
                                char *ppp=ss;
                                while(*ppp>='0' && *ppp<='9' && *ppp){ppp++;}
                                if(*ppp){
                                        save=*ppp;
                                        *ppp=',';
                                }
                                int bits=getint(ss,0,rel==2? 31 : 7);
                                if(rel==1) bits |=0x10;
                                ins |=bits;
                       } else if(!strncasecmp(ss,"IN ",3)){
                                dirOK=0;
                                ss+=3;
                                ins=0x4000;checksideanddelay=1;
                                skipspace(ss);
                                if(strncasecmp(ss,"PINS",4)==0 && (ss[4]==' ' || ss[4]==',') ){
                                        ss+=4;
                                } else if(strncasecmp(ss,"X",1)==0 && (ss[1]==' ' || ss[1]==',') ){
                                        ss++;
                                        ins|=0b100000;
                                } else if(strncasecmp(ss,"Y",1)==0 && (ss[1]==' ' || ss[1]==',') ){
                                        ss++;
                                        ins|=0b1000000;
                                } else if(strncasecmp(ss,"NULL",4)==0 && (ss[4]==' ' || ss[4]==',') ){
                                        ss+=4;
                                        ins|=0b1100000;
                                } else if(strncasecmp(ss,"ISR",3)==0 && (ss[3]==' ' || ss[3]==',') ){
                                        ss+=3;
                                        ins|=0b11000000;
                                } else if(strncasecmp(ss,"OSR",3)==0 && (ss[3]==' ' || ss[3]==',') ){
                                        ss+=3;
                                        ins|=0b11100000;
                                } else error("Syntax");
                                skipspace(ss);
                                if(*ss!=',')error("Syntax");
                                ss++;
                                char save;
                                skipspace(ss);
                                char *ppp=ss;
                                while(*ppp>='0' && *ppp<='9' && *ppp){ppp++;}
                                if(*ppp){
                                        save=*ppp;
                                        *ppp=',';
                                }
                                int bits=getint(ss,1,32);
                                if(bits==32)bits=0;
                                ins|=bits;
                                if(*ppp==',')*ppp=save;
                        } else if(!strncasecmp(ss,"OUT ",4)){
                                dirOK=0;
                                ss+=3;
                                ins=0x6000;checksideanddelay=1;
                                skipspace(ss);
                                if(strncasecmp(ss,"PINS",4)==0 && (ss[4]==' ' || ss[4]==',') ){
                                        ss+=4;
                                } else if(strncasecmp(ss,"X",1)==0 && (ss[1]==' ' || ss[1]==',') ){
                                        ss++;
                                        ins|=0b100000;
                                } else if(strncasecmp(ss,"Y",1)==0 && (ss[1]==' ' || ss[1]==',') ){
                                        ss++;
                                        ins|=0b1000000;
                                } else if(strncasecmp(ss,"NULL",4)==0 && (ss[4]==' ' || ss[4]==',') ){
                                        ss+=4;
                                        ins|=0b1100000;
                                } else if(strncasecmp(ss,"PINDIRS",7)==0 && (ss[7]==' ' || ss[7]==',') ){
                                        ss+=7;
                                        ins|=0b10000000;
                                } else if(strncasecmp(ss,"PC",2)==0 && (ss[2]==' ' || ss[2]==',') ){
                                        ss+=2;
                                        ins|=0b11000000;
                                } else if(strncasecmp(ss,"EXEC",4)==0 && (ss[4]==' ' || ss[4]==',') ){
                                        ss+=4;
                                        ins|=0b11100000;
                                } else error("Syntax");
                                skipspace(ss);
                                if(*ss!=',')error("Syntax");
                                ss++;
                                char save;
                                skipspace(ss);
                                char *ppp=ss;
                                while(*ppp>='0' && *ppp<='9' && *ppp){ppp++;}
                                if(*ppp){
                                        save=*ppp;
                                        *ppp=',';
                                }
                                int bits=getint(ss,1,32);
                                if(bits==32)bits=0;
                                ins|=bits;
                                if(*ppp==',')*ppp=save;
                        } else if(!strncasecmp(ss,"PUSH",4) && (ss[4]==0 || ss[4]==' ')){
                                dirOK=0;
                                ss+=4;
                                ins=0x8000;checksideanddelay=1;
                                ins |= checkblock(ss);
                        } else if(!strncasecmp(ss,"PULL ",4) && (ss[4]==0 || ss[4]==' ')){
                                dirOK=0;
                                ss+=4;
                                ins=0x8080;checksideanddelay=1;
                                ins |= checkblock(ss);
                        } else if(!strncasecmp(ss,"MOV ",4)){
                                dirOK=0;
                                ss+=3;
                                ins=0xA000;checksideanddelay=1;
                                skipspace(ss);
                                if(strncasecmp(ss,"PINS",4)==0 && (ss[4]==' ' || ss[4]==',') ){
                                        ss+=4;
                                } else if(strncasecmp(ss,"X",1)==0 && (ss[1]==' ' || ss[1]==',') ){
                                        ss++;
                                        ins|=0b100000;
                                } else if(strncasecmp(ss,"Y",1)==0 && (ss[1]==' ' || ss[1]==',') ){
                                        ss++;
                                        ins|=0b1000000;
                                } else if(strncasecmp(ss,"EXEC",4)==0 && (ss[4]==' ' || ss[4]==',') ){
                                        ss+=4;
                                        ins|=0b10000000;
                                } else if(strncasecmp(ss,"PC",2)==0 && (ss[2]==' ' || ss[2]==',') ){
                                        ss+=2;
                                        ins|=0b10100000;
                                } else if(strncasecmp(ss,"ISR",3)==0 && (ss[3]==' ' || ss[3]==',') ){
                                        ss+=4;
                                        ins|=0b11000000;
                                } else if(strncasecmp(ss,"OSR",3)==0 && (ss[3]==' ' || ss[3]==',') ){
                                        ss+=4;
                                        ins|=0b11100000;
                                 } else error("Syntax");
                                skipspace(ss);
                                if(*ss!=',')error("Syntax");
                                ss++;
                                skipspace(ss);
                                if(*ss=='~' || *ss=='!'){
                                        ins |=8;
                                        ss++;
                                }
                                if(*ss==':' && ss[1]==':'){
                                        ins |=0x10;
                                        ss+=2;
                                }
                                skipspace(ss);
                                if(strncasecmp(ss,"PINS",4)==0 && (ss[4]==' ' || ss[4]==',') ){
                                        ss+=4;
                                } else if(strncasecmp(ss,"X",1)==0 && (ss[1]==' ' || ss[1]==',') ){
                                        ss++;
                                        ins|=0b1;
                                } else if(strncasecmp(ss,"Y",1)==0 && (ss[1]==' ' || ss[1]==',') ){
                                        ss++;
                                        ins|=0b10;
                                } else if(strncasecmp(ss,"NULL",4)==0 && (ss[4]==' ' || ss[4]==',') ){
                                        ss+=4;
                                        ins|=0b11;
                                } else if(strncasecmp(ss,"STATUS",6)==0 && (ss[6]==' ' || ss[6]==',') ){
                                        ss+=6;
                                        ins|=0b101;
                                } else if(strncasecmp(ss,"ISR",3)==0 && (ss[3]==' ' || ss[3]==',') ){
                                        ss+=4;
                                        ins|=0b110;
                                } else if(strncasecmp(ss,"OSR",3)==0 && (ss[3]==' ' || ss[3]==',') ){
                                        ss+=4;
                                        ins|=0b111;
                                 } else error("Syntax");

                        } else if(!strncasecmp(ss,"NOP",3) && (ss[3]==0 || ss[3]==' ')){
                                dirOK=0;
                                ss+=3;
                                ins=0xA000;
                                ins |=0b01000010;checksideanddelay=1;
                        } else if(!strncasecmp(ss,"IRQ SET ",8)){
                                dirOK=0;
                                ss+=7;
                                ins=0xC000;checksideanddelay=1;
                                ins |= getirqnum(ss);
                        } else if(!strncasecmp(ss,"IRQ WAIT ",9)){
                                dirOK=0;
                                ss+=8;
                                ins=0xC020;checksideanddelay=1;
                                ins |= getirqnum(ss);
                        } else if(!strncasecmp(ss,"IRQ CLEAR ",10)){
                                dirOK=0;
                                ss+=9;
                                ins=0xC040;checksideanddelay=1;
                                ins |= getirqnum(ss);
                        } else if(!strncasecmp(ss,"IRQ NOWAIT ",11)){
                                dirOK=0;
                                ss+=10;
                                ins=0xC000;checksideanddelay=1;
                                ins |= getirqnum(ss);
                        } else if(!strncasecmp(ss,"IRQ ",4)){
                                dirOK=0;
                                ss+=3;
                                ins=0xC000;checksideanddelay=1;
                                ins |= getirqnum(ss);
                        } else if(!strncasecmp(ss,"SET ",4)){
                                dirOK=0;
                                ss+=3;
                                ins=0xE000;checksideanddelay=1;
                                skipspace(ss);
                                if(strncasecmp(ss,"PINS",4)==0 && (ss[4]==' ' || ss[4]==',') ){
                                        ss+=4;
                                } else if(strncasecmp(ss,"X",1)==0 && (ss[1]==' ' || ss[1]==',') ){
                                        ss++;
                                        ins|=0b100000;
                                } else if(strncasecmp(ss,"Y",1)==0 && (ss[1]==' ' || ss[1]==',') ){
                                        ss++;
                                        ins|=0b1000000;
                                } else if(strncasecmp(ss,"PINDIRS",7)==0 && (ss[7]==' ' || ss[7]==',') ){
                                        ss+=7;
                                        ins|=0b10000000;
                                } else error("Syntax");
                                skipspace(ss);
                                if(*ss!=',')error("Syntax");
                                ss++;
                                char save;
                                skipspace(ss);
                                char *ppp=ss;
                                while(*ppp>='0' && *ppp<='9' && *ppp){ppp++;}
                                if(*ppp){
                                        save=*ppp;
                                        *ppp=',';
                                }
                                ins|=getint(ss,0,31);
                                if(*ppp==',')*ppp=save;
                        } else error("PIO instruction not found");
                        if(checksideanddelay==1)ins|=calcsideanddelay(ss, sidepins, delaypossible);
                        instructions[PIOlinenumber]=ins;
                        PIOlinenumber++;
                        return;
                } else {
                        if(!strncasecmp(ss,".LINE ",5)){
                                if(!dirOK)error("Directive must appear before instructions");
                                ss+=5;
                                PIOlinenumber=getint(ss,0,31);
                                PIOstart=PIOlinenumber;
                                return;
                        } else if(!strncasecmp(ss,".END PROGRAM",12)){ //sort out the jmps
                                if(dirOK==2)error("Program not started");
                                int totallines=0;
                                dirOK=2;
                                for(int i=PIOstart; i<32;i++){
                                        if(instructions[i]!=-1){
                                                totallines++;
                                        }
                                }

                                for(int i=PIOstart; i<totallines;i++){
                                        if(!(instructions[i] & 0x1E000)){ // jmp instructions needs resolving
                                                int notfound=1;
                                                for(int j=PIOstart;j<totallines;j++){
                                                        if(strcasecmp(&labelsneeded[i*MAXLABEL],&labelsfound[j*MAXLABEL])==0) {
                                                                instructions[i] |=j; 
                                                                notfound=0;
                                                        }
                                                }
                                                if(notfound)error("label not found at line %",i);
                                        }
                                        if(instructions[i]&0x10000)instructions[i]&=0xFFFF;
                                }
                                ss+=12;
                                skipspace(ss);
                                if(!strncasecmp(ss,"LIST",4) && (ss[4]==0 || ss[4]==' '))
                                for(int i=PIOstart;i<totallines;i++){
                                        pio->instr_mem[i]=instructions[i];
                                        char c[10]={0};
                                        PInt(i);
                                        MMPrintString(": ");
                                        IntToStr(c,instructions[i]+0x10000,16);
                                        MMPrintString(&c[1]);
                                        PRet();
                                }
                                FreeMemory((char *)instructions);
                                FreeMemory(labelsneeded);
                                FreeMemory(labelsfound);
                                return;
                        } else if(!strncasecmp(ss,".SIDE_SET ",10)){
                                if(!dirOK)error("Directive must appear before instructions");
                                ss+=10;
                                sidepins=*ss-'0';
                                if(sidepins<1 || sidepins>5)error("Invalid side_set pin count");
                                ss++;
                                delaypossible=5-sidepins;
                                ins=0xE000;
                                skipspace(ss);
                                if(!strncasecmp(ss,"opt",3) && (ss[3]==0 || ss[3]==' ')){
                                        sideopt=1;
                                        delaypossible=4-sidepins;
                                        if(sideopt && sidepins==5)error("Only 4 side set pins allowed with opt set");
                                        ss+=3;
                                        skipspace(ss);
                                }
                                if(!strncasecmp(ss,"pindirs",7) && (ss[7]==0 || ss[7]==' '))sidepinsdir=1;
                                return;
                        } else error("PIO directive not found");
                }
        } else error(".program must be first statement");
    }
    
    tp = checkstring(cmdline, "CLEAR");
    if(tp){
        getargs(&tp,1,",");
#ifdef PICOMITEVGA
        PIO pio = (getint(argv[0],1,1) ? pio1: pio0);
#endif
#ifdef PICOMITE
        PIO pio = (getint(argv[0],0,1) ? pio1: pio0);
#endif
#ifdef PICOMITEWEB
        PIO pio = (getint(argv[0],0,0) ? pio1: pio0);
#endif
        for(int sm=0;sm<4;sm++){
            hw_clear_bits(&pio->ctrl, 1 << (PIO_CTRL_SM_ENABLE_LSB + sm));
            pio->sm[sm].pinctrl=(5<<26);
            pio->sm[sm].execctrl=(0x1f<<12);
            pio->sm[sm].shiftctrl=(3<<18);
            pio_sm_clear_fifos(pio,sm);
        }
        pio_clear_instruction_memory(pio);
        return;
    }
    tp = checkstring(cmdline, "MAKE RING BUFFER");
    if(tp){
        getargs(&tp,3,",");
        if(argc<3)error("Syntax");
        int size=getinteger(argv[2]);
        if(!(size==256 || size==512 || size==1024 || size== 2048 || size==4096 || size==8192 || size==16384 || size==32768))error("Not power of 2");
        findvar(argv[0], V_FIND | V_NOFIND_ERR);
        if ((vartbl[VarIndex].type & T_INT) && vartbl[VarIndex].dims[0] == 0 && vartbl[VarIndex].level==0){
                vartbl[VarIndex].val.s =(char *)GetAlignedMemory(size);
                vartbl[VarIndex].size=255;
                vartbl[VarIndex].dims[0] = size/8-1+OptionBase;
        }  else error("Invalid variable");
        return;
    }

    tp = checkstring(cmdline, "PROGRAM");
    if(tp){
        struct pio_program program;
        getargs(&tp,3,",");
        if(argc!=3)error("Syntax");
        void *prt1;
        program.length=32;
        program.origin=0;
#ifdef PICOMITEVGA
        PIO pio = (getint(argv[0],1,1) ? pio1: pio0);
#endif
#ifdef PICOMITE
        PIO pio = (getint(argv[0],0,1) ? pio1: pio0);
#endif
#ifdef PICOMITEWEB
        PIO pio = (getint(argv[0],0,0) ? pio1: pio0);
#endif
	    void *ptr1 = findvar(argv[2], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
        if(vartbl[VarIndex].type & T_INT) {
            if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
            if(vartbl[VarIndex].dims[0] <= 0) {		// Not an array
                error("Argument 2 must be integer array");
            }
            if((vartbl[VarIndex].dims[0] - OptionBase)!=7)error("Array size");
            program.instructions = (const uint16_t *)ptr1;
        } else error("Argument 2 must be integer array");
        for(int sm=0;sm<4;sm++)hw_clear_bits(&pio->ctrl, 1 << (PIO_CTRL_SM_ENABLE_LSB + sm));
        pio_clear_instruction_memory(pio);
        int offset=pio_add_program(pio, &program);
        return;
    }
    tp = checkstring(cmdline, "START");
    if(tp){
        getargs(&tp,3,",");
        if(argc!=3)error("Syntax");
#ifdef PICOMITEVGA
        PIO pio = (getint(argv[0],1,1) ? pio1: pio0);
#endif
#ifdef PICOMITE
        PIO pio = (getint(argv[0],0,1) ? pio1: pio0);
#endif
#ifdef PICOMITEWEB
        PIO pio = (getint(argv[0],0,0) ? pio1: pio0);
#endif
        int sm=getint(argv[2],0,3);
        pio_sm_clear_fifos(pio,sm);
        pio_sm_restart(pio, sm);
        pio_sm_set_enabled(pio, sm, true);
        return;
    }
    tp = checkstring(cmdline, "STOP");
    if(tp){
        getargs(&tp,3,",");
        if(argc!=3)error("Syntax");
#ifdef PICOMITEVGA
        PIO pio = (getint(argv[0],1,1) ? pio1: pio0);
#endif
#ifdef PICOMITE
        PIO pio = (getint(argv[0],0,1) ? pio1: pio0);
#endif
#ifdef PICOMITEWEB
        PIO pio = (getint(argv[0],0,0) ? pio1: pio0);
#endif
        int sm=getint(argv[2],0,3);
        pio_sm_set_enabled(pio, sm, false);
        return;
    }
    tp = checkstring(cmdline, "INIT MACHINE");
    if(tp){
        int start=0;
        getargs(&tp,13,",");
        if(argc<5)error("Syntax");
        pio_sm_config mypio=pio_get_default_sm_config();
#ifdef PICOMITEVGA
        PIO pio = (getint(argv[0],1,1) ? pio1: pio0);
#endif
#ifdef PICOMITE
        PIO pio = (getint(argv[0],0,1) ? pio1: pio0);
#endif
#ifdef PICOMITEWEB
        PIO pio = (getint(argv[0],0,0) ? pio1: pio0);
#endif
        int sm=getint(argv[2],0,3);
        float clock=getnumber(argv[4]);
        if(clock<CLKMIN || clock> CLKMAX)error("Clock must be in range % to %",CLKMIN,CLKMAX);
        clock=(float)Option.CPU_Speed*1000.0/clock;
        if(argc>5 && *argv[6])mypio.pinctrl = getint(argv[6],0,0xFFFFFFFF);
        if(argc>7 && *argv[8])mypio.execctrl= getint(argv[8],0,0xFFFFFFFF);
        if(argc>9 && *argv[10])mypio.shiftctrl= getint(argv[10],0,0xFFFFFFFF);
        if(argc>11) start=getint(argv[12],0,31);
        mypio.clkdiv = (uint32_t) (clock * (1 << 16));
        pio_sm_set_config(pio, sm, &mypio);
        pio_sm_init(pio, sm, start, &mypio);
        pio_sm_clear_fifos(pio,sm);
        return;
    }
    error("Syntax");
}
void fun_pio(void){
    unsigned char *tp;
    tp = checkstring(ep, "PINCTRL");
    if(tp){
        getargs(&tp,13,",");
        if(argc<3)error("Syntax");
        iret=(getint(argv[0],0,5)<<29); // no of side set pins
        if(argc>1 && *argv[2])iret|=(getint(argv[2],0,5)<<26); // no of set pins
        if(argc>3 && *argv[4])iret|=(getint(argv[4],0,29)<<20); // no of OUT pins
        if(argc>5 && *argv[6]){
            if(!toupper(*argv[6])=='G')error("Syntax");
            argv[6]++;
            if(!toupper(*argv[6])=='P')error("Syntax");
            argv[6]++;
            iret|=(getint(argv[6],0,29)<<15); // IN base
        }
        if(argc>7 && *argv[8]){
            if(!toupper(*argv[8])=='G')error("Syntax");
            argv[8]++;
            if(!toupper(*argv[8])=='P')error("Syntax");
            argv[8]++;
            iret|=(getint(argv[8],0,29)<<10); // SIDE SET base
        }
        if(argc>9 && *argv[10]){
            if(!toupper(*argv[10])=='G')error("Syntax");
            argv[10]++;
            if(!toupper(*argv[10])=='P')error("Syntax");
            argv[10]++;
            iret|=(getint(argv[10],0,29)<<5); // SET base
        }
        if(argc==13){
            if(!toupper(*argv[12])=='G')error("Syntax");
            argv[12]++;
            if(!toupper(*argv[12])=='P')error("Syntax");
            argv[12]++;
            iret|=getint(argv[12],0,29); //OUT base
        }
        targ=T_INT;
        return;
    }
    tp = checkstring(ep, "EXECCTRL");
    if(tp){
        getargs(&tp,9,",");
        if(!(argc==5 || argc==7 || argc==9))error("Syntax");
        if(!toupper(*argv[0])=='G')error("Syntax");
        argv[0]++;
        if(!toupper(*argv[0])=='P')error("Syntax");
        argv[0]++;
        iret=(getint(argv[0],0,29)<<24); // jmp pin
        iret |= pio_sm_calc_wrap(getint(argv[2],0,31), getint(argv[4],0,31));
        if(argc>=7 && *argv[6])iret|=(getint(argv[6],0,1)<<29); //SIDE_PINDIR
        if(argc==9)iret|=(getint(argv[8],0,1)<<30); // SIDE_EN
        targ=T_INT;
        return;
    }
    tp = checkstring(ep, "SHIFTCTRL");
    if(tp){
        getargs(&tp,15,",");
        if(argc<1)error("Syntax");
        iret=(getint(argv[0],0,31)<<20); // push threshold
        iret|=(getint(argv[2],0,31)<<25); // pull threshold
        if(argc>3 && *argv[4])iret|=(getint(argv[4],0,1)<<16); // autopush
        if(argc>5 && *argv[6])iret|=(getint(argv[6],0,1)<<17); // autopull
        if(argc>7 && *argv[8])iret|=(getint(argv[8],0,1)<<18); // IN_SHIFTDIR
        if(argc>9 && *argv[10])iret|=(getint(argv[10],0,1)<<19); // OUT_SHIFTDIR
        if(argc>11 && *argv[12])iret|=(getint(argv[12],0,1)<<30); // FJOIN_RX
        if(argc>13 && *argv[14])iret|=(getint(argv[14],0,1)<<31); // FJOIN_TX
        targ=T_INT;
        return;
    }
    tp = checkstring(ep, "FSTAT");
    if(tp){
        getargs(&tp,1,",");
#ifdef PICOMITEVGA
        PIO pio = (getint(argv[0],1,1) ? pio1: pio0);
#endif
#ifdef PICOMITE
        PIO pio = (getint(argv[0],0,1) ? pio1: pio0);
#endif
#ifdef PICOMITEWEB
        PIO pio = (getint(argv[0],0,0) ? pio1: pio0);
#endif
        iret=pio->fstat; // jmp pin
        targ=T_INT;
        return;
    }
    tp = checkstring(ep, "FDEBUG");
    if(tp){
        getargs(&tp,1,",");
#ifdef PICOMITEVGA
        PIO pio = (getint(argv[0],1,1) ? pio1: pio0);
#endif
#ifdef PICOMITE
        PIO pio = (getint(argv[0],0,1) ? pio1: pio0);
#endif
#ifdef PICOMITEWEB
        PIO pio = (getint(argv[0],0,0) ? pio1: pio0);
#endif
        iret=pio->fdebug; // jmp pin
        targ=T_INT;
        return;
    }
    tp = checkstring(ep, "FLEVEL");
    if(tp){
        getargs(&tp,5,",");
#ifdef PICOMITEVGA
        PIO pio = (getint(argv[0],1,1) ? pio1: pio0);
#endif
#ifdef PICOMITE
        PIO pio = (getint(argv[0],0,1) ? pio1: pio0);
#endif
#ifdef PICOMITEWEB
        PIO pio = (getint(argv[0],0,0) ? pio1: pio0);
#endif
        if(argc==1)iret=pio->flevel; // jmp pin
        else {
                if(argc!=5)error("Syntax");
                int sm=getint(argv[2],0,3)*8;
                if(checkstring(argv[4],"TX"))iret=((pio->flevel)>>sm) & 0xf;
                else if(checkstring(argv[4],"RX"))iret=((pio->flevel)>>(sm+4)) & 0xf;
                else error("Syntax");
        }
        targ=T_INT;
        return;
    }
    tp = checkstring(ep, "DMA RX POINTER");
    if(tp){
        iret=dma_hw->ch[dma_rx_chan].write_addr;
        targ=T_INT;
        return;
    }
    tp = checkstring(ep, "DMA TX POINTER");
    if(tp){
        iret=dma_hw->ch[dma_tx_chan].read_addr;
        targ=T_INT;
        return;
    }
    error("Syntax");


}

#ifdef PICOMITEWEB
typedef struct NTP_T_ {
    ip_addr_t ntp_server_address;
    bool dns_request_sent;
    struct udp_pcb *ntp_pcb;
    absolute_time_t ntp_test_time;
    alarm_id_t ntp_resend_alarm;
} NTP_T;
NTP_T *NTPstate=NULL;
#define NTP_SERVER "pool.ntp.org"
#define NTP_MSG_LEN 48
#define NTP_PORT 123
#define NTP_DELTA 2208988800 // seconds between 1 Jan 1900 and 1 Jan 1970
#define NTP_TEST_TIME (30 * 1000)
#define NTP_RESEND_TIME (10 * 1000)
volatile int NTPdone=0;
volatile time_t timeadjust=0;
// Called with results of operation
static void ntp_result(NTP_T* state, int status, time_t *result) {
    if (status == 0 && result) {
        *result=*result+timeadjust;
        struct tm *utc = gmtime(result);
        char buff[STRINGSIZE];
        sprintf(buff,"got ntp response: %02d/%02d/%04d %02d:%02d:%02d\r\n", utc->tm_mday, utc->tm_mon + 1, utc->tm_year + 1900,
               utc->tm_hour, utc->tm_min, utc->tm_sec);
        MMPrintString(buff);
        hour = utc->tm_hour;
        minute = utc->tm_min;
        second = utc->tm_sec;
        day_of_week=utc->tm_wday;
        if(day_of_week==0)day_of_week=7;
        year = utc->tm_year + 1900;
	month = utc->tm_mon + 1;
	day = utc->tm_mday;
        uSec(200);
        NTPdone=1;
    }
}

static int64_t ntp_failed_handler(alarm_id_t id, void *user_data);

// Make an NTP request
static void ntp_request(NTP_T *state) {
    // cyw43_arch_lwip_begin/end should be used around calls into lwIP to ensure correct locking.
    // You can omit them if you are in a callback from lwIP. Note that when using pico_cyw_arch_poll
    // these calls are a no-op and can be omitted, but it is a good practice to use them in
    // case you switch the cyw43_arch type later.
    cyw43_arch_lwip_begin();
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, NTP_MSG_LEN, PBUF_RAM);
    uint8_t *req = (uint8_t *) p->payload;
    memset(req, 0, NTP_MSG_LEN);
    req[0] = 0x1b;
    udp_sendto(state->ntp_pcb, p, &state->ntp_server_address, NTP_PORT);
    pbuf_free(p);
    cyw43_arch_lwip_end();
}

static int64_t ntp_failed_handler(alarm_id_t id, void *user_data)
{
    NTP_T* state = (NTP_T*)user_data;
    free(state);
    error("ntp request failed");
    return 0;
}

// Call back with a DNS result
static void ntp_dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg) {
    NTP_T *state = (NTP_T*)arg;
    if (ipaddr) {
        state->ntp_server_address = *ipaddr;
        char buff[STRINGSIZE];
        sprintf(buff,"ntp address %s\r\n", ip4addr_ntoa(&state->ntp_server_address));
        MMPrintString(buff);
        ntp_request(state);
    } else {
        free(state);
        error("ntp dns request failed");
    }
}

// NTP data received
static void ntp_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    NTP_T *state = (NTP_T*)arg;
    uint8_t mode = pbuf_get_at(p, 0) & 0x7;
    uint8_t stratum = pbuf_get_at(p, 1);

    // Check the result
    if (ip_addr_cmp(addr, &state->ntp_server_address) && port == NTP_PORT && p->tot_len == NTP_MSG_LEN &&
        mode == 0x4 && stratum != 0) {
        uint8_t seconds_buf[4] = {0};
        pbuf_copy_partial(p, seconds_buf, sizeof(seconds_buf), 40);
        uint32_t seconds_since_1900 = seconds_buf[0] << 24 | seconds_buf[1] << 16 | seconds_buf[2] << 8 | seconds_buf[3];
        uint32_t seconds_since_1970 = seconds_since_1900 - NTP_DELTA;
        time_t epoch = seconds_since_1970;
        ntp_result(state, 0, &epoch);
    } else {
        pbuf_free(p);
        free(state);
        error("invalid ntp response");
    }
    pbuf_free(p);
}

// Perform initialisation
static NTP_T* ntp_init(void) {
    NTPstate = (NTP_T *)calloc(1,sizeof(NTP_T));
    NTP_T *state=NTPstate;
    if (!state) {
        error("failed to allocate state\n");
        return NULL;
    }
    state->ntp_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (!state->ntp_pcb) {
        error("failed to create pcb\n");
        free((void *)state);
        return NULL;
    }
    udp_recv(state->ntp_pcb, ntp_recv, state);
    return state;
}
void GetNTPTime(void){
        NTP_T *state = ntp_init();
        if (!state) error("Can't create NTP structure");
        cyw43_arch_lwip_begin();
        int err = dns_gethostbyname(NTP_SERVER, &state->ntp_server_address, ntp_dns_found, state);

        cyw43_arch_lwip_end();
        if (err == ERR_OK) {
                char buff[STRINGSIZE];
                sprintf(buff,"ntp address %s\r\n", ip4addr_ntoa(&state->ntp_server_address));
                MMPrintString(buff);
                ntp_request(state); // Cached result
        } else if (err != ERR_INPROGRESS) { // ERR_INPROGRESS means expect a callback
                free(state);
                error("dns request failed");
        }
}

 
static int scan_result(void *env, const cyw43_ev_scan_result_t *result) {
    if (result) {
        Timer4=5000;
        char buff[STRINGSIZE];
        sprintf(buff,"ssid: %-32s rssi: %4d chan: %3d mac: %02x:%02x:%02x:%02x:%02x:%02x sec: %u\r\n",
            result->ssid, result->rssi, result->channel,
            result->bssid[0], result->bssid[1], result->bssid[2], result->bssid[3], result->bssid[4], result->bssid[5],
            result->auth_mode);
        MMPrintString(buff);
    } 
    return 0;
}
int TCP_PORT ;
//#define DEBUG_printf printf
#define DEBUG_printf
const char httpheadersfail[]="HTTP/1.0 404\r\n\r\n";
void checksent(void *arg, int fn);
TCP_SERVER_T *TCPstate=NULL;
jmp_buf recover;
static TCP_SERVER_T* tcp_server_init(void) {
    if(!TCPstate) {
        TCPstate = (TCP_SERVER_T*)calloc(1,sizeof(TCP_SERVER_T));
        memset(TCPstate,0,sizeof(TCP_SERVER_T));
    }
    if (!TCPstate) {
        DEBUG_printf("failed to allocate state\r\n");
        return NULL;
    }
    for(int i=0;i<MaxPcb;i++){
        TCPstate->client_pcb[i]=NULL;
    }
    return TCPstate;
}

err_t tcp_server_close(void *arg) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    err_t err = ERR_OK;
    if (state->client_pcb[state->write_pcb] != 0) {
        tcp_arg(state->client_pcb[state->write_pcb], NULL);
        tcp_sent(state->client_pcb[state->write_pcb], NULL);
        tcp_recv(state->client_pcb[state->write_pcb], NULL);
        tcp_err(state->client_pcb[state->write_pcb], NULL);
        tcp_poll(state->client_pcb[state->write_pcb], NULL,0);
        err = tcp_close(state->client_pcb[state->write_pcb]);
        if (err != ERR_OK) {
            tcp_abort(state->client_pcb[state->write_pcb]);
            error("close failed %, calling abort", err);
            err = ERR_ABRT;
        }
        DEBUG_printf("Close success %x\r\n",(uint32_t)state->client_pcb[state->write_pcb]);
        state->recv_len[state->write_pcb]=0;
        state->client_pcb[state->write_pcb]=NULL;
        FreeMemorySafe((void **)&state->buffer_recv[state->write_pcb]);
    }
    return err;
}

static err_t tcp_server_result(void *arg, int status) {
        return ERR_OK;
}

static err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
//    DEBUG_printf("tcp_server_sent %u\n", len);
    state->sent_len[state->write_pcb] = len;
    return ERR_OK;
}

err_t tcp_server_send_data(void *arg, struct tcp_pcb *tpcb)
{
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    cyw43_arch_lwip_check();

    state->sent_len[state->write_pcb] = 0;
    DEBUG_printf("Writing %d bytes to client %x\r\n",state->to_send[state->write_pcb], (uint32_t)tpcb);
    // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
    // can use this method to cause an assertion in debug mode, if this method is called when
    // cyw43_arch_lwip_begin IS needed
    err_t err = tcp_write(tpcb, state->buffer_sent, state->to_send[state->write_pcb],  0);
    if (err != ERR_OK) {
//        error("Failed to write data %",err);
    }

//    err=tcp_output(tpcb);
///    if (err != ERR_OK) {
//        error("Failed to output data %", err);
//    }
    return ERR_OK;
}

err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
        TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
        if (!p) {
                return ERR_OK;
        }
        // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
        // can use this method to cause an assertion in debug mode, if this method is called when
        // cyw43_arch_lwip_begin IS needed
        cyw43_arch_lwip_check();
        if (p->tot_len > 0) {
                TCPreceived=1;
                int i=0;
                while(state->client_pcb[i]!=tpcb && i<=MaxPcb)i++;
                if(i==MaxPcb)error("Internal TCP receive error");
    	        if(!CurrentLinePtr){  // deal with requests when we don't want them
                        pbuf_free(p);
                        DEBUG_printf("Sending 404 on pcb %d, rbuff address is %x ",i, (uint32_t)state->buffer_recv[i]);
                        state->write_pcb=i;
                        state->to_send[state->write_pcb]=strlen(httpheadersfail);
                        state->buffer_sent=(char *)httpheadersfail;
                        if(state->client_pcb[state->write_pcb]){
                                tcp_server_send_data(state, state->client_pcb[state->write_pcb]);
                                checksent(state,0);
                                tcp_server_close(state) ;
                        }
                } else {
                        state->buffer_recv[i]=GetMemory(p->tot_len);
                        state->inttrig[i]=1;
                        DEBUG_printf("tcp_server_recv %x on pcb %d ",(uint32_t)tpcb, i);
                        state->recv_len[i] = pbuf_copy_partial(p, state->buffer_recv[i] , p->tot_len, 0);
                        
                        pbuf_free(p);
                        DEBUG_printf("%d err %d, %s\r\n", state->recv_len[i], err, state->buffer_recv[i]);
                }
        }
        return ERR_OK;
}

/*static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb) {
    DEBUG_printf("tcp_server_poll_fn\n");
    return tcp_server_result(arg, -1); // no response is an error?
}*/

static void tcp_server_err(void *arg, err_t err) {
    if (err != ERR_ABRT) {
        DEBUG_printf("tcp_client_err_fn %d\r\n", err);
        tcp_server_close(arg);
        longjmp(recover, 1);   
    }
}

/*static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb) {
        TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
        cyw43_arch_lwip_check();
        int i=0;
        while(state->client_pcb[i]!=tpcb && i<=MaxPcb)i++;
        if(i==MaxPcb)error("Internal TCP receive error");
        state->write_pcb=i;
        printf("tcp_server_poll_fn\r\n");
        return tcp_server_close(arg); // no activity so close the connection
}*/


static err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    int i;
    if (err != ERR_OK || client_pcb == NULL) {
        DEBUG_printf("Failure in accept\r\n");
        tcp_server_result(arg, err);
        return ERR_VAL;
    }
    cyw43_arch_lwip_check();
    cyw43_arch_lwip_begin();
    for(i=0;i<=MaxPcb;i++){
        if(i==MaxPcb)error("No free connections");
        if(state->client_pcb[i]==NULL){
                state->client_pcb[i] = client_pcb;
                break;
        }
    }
    cyw43_arch_lwip_end();
    DEBUG_printf("Client connected %x on pcb %d\r\n",(uint32_t)client_pcb,i);
    tcp_arg(client_pcb, state);
    tcp_sent(client_pcb, tcp_server_sent);
    tcp_recv(client_pcb, tcp_server_recv);
//    tcp_poll(client_pcb, tcp_server_poll, POLL_TIME_S * 2);
    tcp_err(client_pcb, tcp_server_err);

    return ERR_OK;
}
static bool tcp_server_open(void *arg) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    MMPrintString("Starting server at ");
    MMPrintString(ip4addr_ntoa(netif_ip4_addr(netif_list)));
    MMPrintString(" on port ");
    PInt(TCP_PORT);PRet();

    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        DEBUG_printf("failed to create pcb\r\n");
        return false;
    }

    err_t err = tcp_bind(pcb, NULL, TCP_PORT);
    if (err) {
        DEBUG_printf("failed to bind to port %d\n");
        return false;
    }

    state->server_pcb = tcp_listen_with_backlog(pcb, MaxPcb);
    if (!state->server_pcb) {
        DEBUG_printf("failed to listen\r\n");
        if (pcb) {
            tcp_close(pcb);
        }
        return false;
    }

    tcp_arg(state->server_pcb, state);
    tcp_accept(state->server_pcb, tcp_server_accept);

    return true;
}
void checksent(void *arg, int fn){
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    Timer4=1000;

    while(!(state->sent_len[state->write_pcb]==state->to_send[state->write_pcb]) || Timer4==0){ {if(startupcomplete)cyw43_arch_poll();}}
    if(Timer4==0){
        if(fn)ForceFileClose(fn);
        tcp_server_close(state) ;
        error("LWIP send data timeout");
    }
    {if(startupcomplete)cyw43_arch_poll();}
}
const char httpheaders[]="HTTP/1.1 200 OK\r\nServer:CPi\r\nConnection:close\r\nContent-type:";
const char httptail[]="\r\nContent-Length:";
const char httpend[]="\r\n\r\n";
const char httphtml[]="text/html\r\nContent-Length:";

void cleanserver(void){
    if(!TCPstate)return;
    TCP_SERVER_T *state = (TCP_SERVER_T*)TCPstate;
        for(int i=0 ; i<MaxPcb ; i++){
                state->write_pcb=i;
                if(state->client_pcb[i])tcp_server_close(state);
        }
}
void cmd_transmit(unsigned char *cmd){
    unsigned char *tp;
    int tlen;
    tp=checkstring(cmd, "CODE");
    if(tp){
    	getargs(&tp, 3, ",");
        if(argc != 3)error("Argument count");
        TCP_SERVER_T *state = (TCP_SERVER_T*)TCPstate;
        char httpheaders[]="HTTP/1.0 404\r\n\r\n";
        state->write_pcb = getint(argv[0],1,MaxPcb)-1;
        tlen=getint(argv[2],100,999);
        IntToStr(&httpheaders[9],tlen,10);
        httpheaders[12]='\r';
        state->to_send[state->write_pcb]=strlen(httpheaders);
        state->buffer_sent=httpheaders;
        if(setjmp(recover) != 0)return;
        if(state->client_pcb[state->write_pcb]){
                tcp_server_send_data(state, state->client_pcb[state->write_pcb]);
                checksent(state,0);
                tcp_server_close(state) ;
        }
        {if(startupcomplete)cyw43_arch_poll();}
        return;
    } 

    if((tp=checkstring(cmd, "FILE"))){
        int32_t fn;
        char *fname;
        char *ctype;
        char *outstr=GetTempMemory(STRINGSIZE);
        char p[10]={0};
        char *pBuf=GetTempMemory(TCP_MSS);
        int FileSize, n_read;
    	getargs(&tp, 5, ",");
        if(argc != 5)error("Argument count");
        TCP_SERVER_T *state = (TCP_SERVER_T*)TCPstate;
        state->write_pcb = getint(argv[0],1,MaxPcb)-1;
        fname=getCstring(argv[2]);
        ctype=getCstring(argv[4]);
        strcpy(outstr,httpheaders);
        strcat(outstr,ctype);
        strcat(outstr,httptail);
        if(*fname == 0) error("Cannot find file");
        if (ExistsFile(fname)) {
                fn = FindFreeFileNbr();
                if (!BasicFileOpen(fname, fn, FA_READ))
                return;
                if(FatFSFileSystem)  FileSize = f_size(FileTable[fn].fptr);
                else FileSize = lfs_file_size(&lfs,FileTable[fn].lfsptr);
                IntToStr(p,FileSize,10);
                strcat(outstr,p);
                strcat(outstr,httpend);
                int i=0;
                state->to_send[state->write_pcb]=strlen(outstr);
                state->buffer_sent=outstr;
                if(setjmp(recover) != 0)return;
                tcp_server_send_data(state, state->client_pcb[state->write_pcb]);
                checksent(state,fn);
                while(1) {
                        if(
                           ((filesource[fn]==FLASHFILE) && (lfs_file_tell(&lfs,FileTable[fn].lfsptr)==lfs_file_size(&lfs,FileTable[fn].lfsptr)))
                           || ((filesource[fn]!=FLASHFILE) && f_eof(FileTable[fn].fptr))
                        ) break;
                        if(filesource[fn]!=FLASHFILE)  f_read(FileTable[fn].fptr, pBuf, TCP_MSS, &n_read);
                        else n_read=lfs_file_read(&lfs, FileTable[fn].lfsptr, pBuf, TCP_MSS);
                        state->to_send[state->write_pcb]=n_read;
                        state->buffer_sent=pBuf;
                        tcp_server_send_data(state, state->client_pcb[state->write_pcb]);
                        checksent(state,fn);
                }
                {if(startupcomplete)cyw43_arch_poll();}
                FileClose(fn);
        } else {
                state->to_send[state->write_pcb]=strlen(httpheadersfail);
                state->buffer_sent=(char *)httpheadersfail;
                tcp_server_send_data(state, state->client_pcb[state->write_pcb]);
                checksent(state,0);
        }
        {if(startupcomplete)cyw43_arch_poll();}
        tcp_server_close(state) ;
        return;
    }
    if((tp=checkstring(cmd, "PAGE"))){
	MMFLOAT f;
        int32_t fn;
        int64_t i64;
        unsigned char *s;
        int i;
        int t;
        char *fname;
        char c;
        char valbuf[STRINGSIZE];
        char vartest[MAXVARLEN];
        int vartestp;
	void *ptr = NULL;
        int FileSize, n_read;
        int savepointer=0;
        char outstr[255]={0};
        char p[10]={0};
        strcat(outstr,httpheaders);
        strcat(outstr,httphtml);
    	getargs(&tp, 3, ",");
        if(argc!=3)error("Argument count");
        TCP_SERVER_T *state = (TCP_SERVER_T*)TCPstate;
        state->write_pcb = getint(argv[0],1,MaxPcb)-1;
        fname=getCstring(argv[2]);
        if(*fname == 0) error("Cannot find file");
        if (ExistsFile(fname)) {
                fn = FindFreeFileNbr();
                if (!BasicFileOpen(fname, fn, FA_READ)) return;
                if(filesource[fn]!=FLASHFILE) FileSize = f_size(FileTable[fn].fptr);
                else FileSize = lfs_file_size(&lfs,FileTable[fn].lfsptr);
                char *SocketOut=GetMemory(FileSize+256);
//                strcpy(SocketOut,"<!doctype html>\r\n");
                int SocketOutPointer=0;
                while(1) {
                        if(FileEOF(fn)) break;
                        c=FileGetChar(fn);
                        if(c==26)continue; //deal with xmodem padding
                        if(SocketOutPointer>FileSize+256)break;
                        if(c=='{'){ //start of variable definition
                                vartestp=0;
                                while(c!='}'){
                                        c=FileGetChar(fn);
                                        if(vartestp==0 && c=='{') break;
                                        if(c!='}')vartest[vartestp++]=c;
                                }
                                if(c=='{')SocketOut[SocketOutPointer++]=c; 
                                else {
                                        vartest[vartestp]=0;
                                        unsigned char *s, *st, *temp_tknbuf;
                                        temp_tknbuf = GetMemory(STRINGSIZE);
                                        strcpy((char *)temp_tknbuf, (char *)tknbuf);                                    // first save the current token buffer in case we are in immediate mode
                                        // we have to fool the tokeniser into thinking that it is processing a program line entered at the console
                                        st = GetMemory(STRINGSIZE);
                                        inpbuf[0] = 'r'; inpbuf[1] = '=';                               // place a dummy assignment in the input buffer to keep the tokeniser happy
                                        strcpy((char *)inpbuf + 2, (char *)vartest);
                                        tokenise(true);                                                 // and tokenise it (the result is in tknbuf)
                                        strcpy((char *)st, (char *)(tknbuf + 3));
                                        t = T_NOTYPE;
                                        int os=OptionExplicit;
                                        OptionExplicit = false;
                                        evaluate(st, &f, &i64, &s, &t, false);
                                        OptionExplicit = os;
                                        if(t & T_NBR) {
                                                FloatToStr(valbuf, f, 0, STR_AUTO_PRECISION, ' ');   // set the string value to be saved
                                                for(i=0;i<strlen(valbuf);i++)SocketOut[SocketOutPointer++]=valbuf[i];
                                        } else if(t & T_INT) {
                                                IntToStr(valbuf, i64, 10); // if positive output a space instead of the sign
                                                for(i=0;i<strlen(valbuf);i++)SocketOut[SocketOutPointer++]=valbuf[i];
                                        } else if(t & T_STR) {
                                                for(i=1;i<=s[0];i++)SocketOut[SocketOutPointer++]=s[i];
                                        } 
                                        strcpy((char *)tknbuf, (char *)temp_tknbuf);// restore the saved token buffer
                                        FreeMemory(temp_tknbuf) ;
                                        FreeMemory(st);
                                }
                        } else 
                        SocketOut[SocketOutPointer++]=c;
                }
                FileClose(fn);
                SocketOut[SocketOutPointer++]=10;
                SocketOut[SocketOutPointer++]=13;
                SocketOut[SocketOutPointer++]=10;
                SocketOut[SocketOutPointer++]=13;
                SocketOut[SocketOutPointer]=0;
                IntToStr(p,strlen(SocketOut),10);
                strcat(outstr,p);
                strcat(outstr,httpend);
//
                if(setjmp(recover) != 0)return;
                state->to_send[state->write_pcb]=strlen(outstr);
                state->buffer_sent=outstr;
                tcp_server_send_data(state, state->client_pcb[state->write_pcb]);
                DEBUG_printf("sending to pcb %d\r\n",state->write_pcb);
                checksent(state,0);
//
                state->to_send[state->write_pcb]=strlen(SocketOut);
                state->buffer_sent=SocketOut;
                int bufflen=strlen(SocketOut);
                while(bufflen>TCP_MSS){
                        state->to_send[state->write_pcb]=TCP_MSS;
                        tcp_server_send_data(state, state->client_pcb[state->write_pcb]);
                        DEBUG_printf("sending to pcb %d\r\n",state->write_pcb);
                        bufflen-=TCP_MSS;
                        state->buffer_sent+=TCP_MSS;
                        checksent(state,0);
                }
                state->to_send[state->write_pcb]=bufflen;
                tcp_server_send_data(state, state->client_pcb[state->write_pcb]);
                DEBUG_printf("sending to pcb %d\r\n",state->write_pcb);
                checksent(state,0);
//
                FreeMemory(SocketOut);
        } else {
                state->to_send[state->write_pcb]=strlen(httpheadersfail);
                state->buffer_sent=(char *)httpheadersfail;
                tcp_server_send_data(state, state->client_pcb[state->write_pcb]);

                checksent(state,0);
        }
        tcp_server_close(state) ;
        {if(startupcomplete)cyw43_arch_poll();}
        return;
    }
    error("Invalid option");
}
void open_tcp_server(int full){
        if(!Option.TCP_PORT)return;
        tcp_server_init();
        if(!full)return;
        TCP_PORT=Option.TCP_PORT;
        if (!TCPstate) {
                MMPrintString("Failed to create TCP server\r\n");
        }
        if (!tcp_server_open(TCPstate)) {
                MMPrintString("Failed to create TCP server\r\n");
        }
        return;
}

typedef struct TCP_CLIENT_T_ {
    struct tcp_pcb *tcp_pcb;
    ip_addr_t remote_addr;
    uint8_t *buffer;
    int buffer_len;
    volatile int sent_len;
    volatile bool complete;
    volatile bool connected;
    int BUF_SIZE;
    int TCP_PORT;
} TCP_CLIENT_T;
TCP_CLIENT_T *TCP_CLIENT=NULL;
// Perform initialisation
static TCP_CLIENT_T* tcp_client_init(void) {
    TCP_CLIENT_T *state = calloc(1, sizeof(TCP_CLIENT_T));
    if (!state) {
        DEBUG_printf("failed to allocate state\n");
        return NULL;
    }
//  ip4addr_aton(TEST_TCP_SERVER_IP, &state->remote_addr);
    return state;
}
// Call back with a DNS result
static void tcp_dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T*)arg;
    if (ipaddr) {
        state->remote_addr = *ipaddr;
        char buff[STRINGSIZE];
        sprintf(buff,"tcp address %s\r\n", ip4addr_ntoa(ipaddr));
        MMPrintString(buff);
        state->complete=1;
//        ntp_request(state);
    } else {
        free(state);
        error("tcp dns request failed");
    }
}
static err_t tcp_client_close(void *arg) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T*)arg;
    err_t err = ERR_OK;
    if (state->tcp_pcb != NULL) {
        tcp_arg(state->tcp_pcb, NULL);
        tcp_poll(state->tcp_pcb, NULL, 0);
        tcp_sent(state->tcp_pcb, NULL);
        tcp_recv(state->tcp_pcb, NULL);
        tcp_err(state->tcp_pcb, NULL);
        err = tcp_close(state->tcp_pcb);
        if (err != ERR_OK) {
            DEBUG_printf("close failed %d, calling abort\n", err);
            tcp_abort(state->tcp_pcb);
            err = ERR_ABRT;
        }
        state->tcp_pcb = NULL;
    }
    return err;
}
static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
        TCP_CLIENT_T *state = (TCP_CLIENT_T*)arg;
        DEBUG_printf("tcp_client_sent %u\r\n", len);
        return ERR_OK;
}
static void tcp_client_err(void *arg, err_t err) {
    if (err != ERR_ABRT) {
        error("TCP client");
    }
}
err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T*)arg;
    if (!p) {
        return ERR_OK;
    }
    // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
    // can use this method to cause an assertion in debug mode, if this method is called when
    // cyw43_arch_lwip_begin IS needed
    cyw43_arch_lwip_check();
    if (p->tot_len > 0) {
        DEBUG_printf("recv %d err %d\r\n", p->tot_len, err);
        // Receive the buffer
        const uint16_t buffer_left = state->BUF_SIZE - state->buffer_len;
        state->buffer_len += pbuf_copy_partial(p, state->buffer + state->buffer_len,
                                               p->tot_len > buffer_left ? buffer_left : p->tot_len, 0);
        tcp_recved(tpcb, p->tot_len);
        cyw43_arch_lwip_begin();
        uint64_t *x=(uint64_t *)state->buffer;
        x--;
        *x=state->buffer_len;
        cyw43_arch_lwip_end();
    }
    pbuf_free(p);
    return ERR_OK;
}
static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T*)arg;
    if (err != ERR_OK) {
        error("connect failed %", err);
    }
    MMPrintString("Connected\r\n");
    state->connected = true;
    return ERR_OK;
}


static bool tcp_client_open(void *arg) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T*)arg;
    char buff[STRINGSIZE];
    sprintf("Connecting to %s port %u\r\n", ip4addr_ntoa(&state->remote_addr), state->TCP_PORT);
    MMPrintString(buff);
    state->tcp_pcb = tcp_new_ip_type(IP_GET_TYPE(&state->remote_addr));
    if (!state->tcp_pcb) {
        error("failed to create pcb");
        return false;
    }

    tcp_arg(state->tcp_pcb, state);
//    tcp_poll(state->tcp_pcb, tcp_client_poll, POLL_TIME_S * 2);
    tcp_sent(state->tcp_pcb, tcp_client_sent);
    tcp_recv(state->tcp_pcb, tcp_client_recv);
    tcp_err(state->tcp_pcb, tcp_client_err);

    state->buffer_len = 0;

    // cyw43_arch_lwip_begin/end should be used around calls into lwIP to ensure correct locking.
    // You can omit them if you are in a callback from lwIP. Note that when using pico_cyw_arch_poll
    // these calls are a no-op and can be omitted, but it is a good practice to use them in
    // case you switch the cyw43_arch type later.
    cyw43_arch_lwip_begin();
    err_t err = tcp_connect(state->tcp_pcb, &state->remote_addr, state->TCP_PORT, tcp_client_connected);
    cyw43_arch_lwip_end();

    return err == ERR_OK;
}

void cmd_web(void){
        if(!WIFIconnected)error("WIFI not connected");
        unsigned char *tp;
        tp=checkstring(cmdline, "NTP");
        if(tp){
            getargs(&tp,1,",");
            if (argc == 1){
                MMFLOAT adjust = getnumber(argv[0]);
                if (adjust < -12.0 || adjust > 14.0) error("Invalid Time Offset");
                timeadjust=(time_t)(adjust*3600.0);
            } else timeadjust=0;
            GetNTPTime(); 
            Timer4=5000;
            while(!NTPdone){
                {if(startupcomplete)cyw43_arch_poll();}
                if(!Timer4){
                        udp_remove(NTPstate->ntp_pcb);
                        memset(NTPstate,0,sizeof(NTPstate));
                        free(NTPstate);
                        error("NTP timeout");
                }
            }
            NTPdone=0;
            udp_remove(NTPstate->ntp_pcb);
            memset(NTPstate,0,sizeof(NTPstate));
            free(NTPstate);
            return;   
        }
        tp=checkstring(cmdline, "SCAN");
        if(tp){
               cyw43_wifi_scan_options_t scan_options = {0};
                int err = cyw43_wifi_scan(&cyw43_state, &scan_options, NULL, scan_result);
                if (err == 0) {
                    MMPrintString("\nPerforming wifi scan\n");
                } else {
                    char buff[STRINGSIZE];
                    sprintf(buff,"Failed to start scan: %d\n", err);
                    MMPrintString(buff);
                }
                Timer4=500;
            while (Timer4){{if(startupcomplete)cyw43_arch_poll();}}
            return;   
        }
        tp=checkstring(cmdline, "TCP INTERRUPT");
        if(tp){
                getargs(&tp, 1, ",");
                if(argc!=1)error("Syntax");
                TCPreceiveInterrupt=GetIntAddress(argv[0]);
                InterruptUsed=true;
                TCPreceived=0;
                return;
        }
        tp=checkstring(cmdline, "TCP CLOSE");
        if(tp){
                TCP_SERVER_T *state = TCPstate;
                getargs(&tp, 1, ",");
                if(argc!=1)error("Syntax");
                state->write_pcb = getint(argv[0],1,MaxPcb)-1;
                tcp_server_close(state) ;
                return;
        }
        tp=checkstring(cmdline, "TRANSMIT");
        if(tp){
                cmd_transmit(tp);
        
                return;
        }
        tp=checkstring(cmdline, "OPEN TCP CLIENT");
        if(tp){
                char *IP=GetMemory(STRINGSIZE);
                int timeout=2000;
                getargs(&tp,3,",");
                if(argc!=3)error("Syntax");
                IP=getCstring(argv[0]);
                int port=getint(argv[2],1,65535);
                TCP_CLIENT_T *state = tcp_client_init();
                TCP_CLIENT=state;
                state->TCP_PORT=port;
                if(!isalpha(*IP)){
                        if(!ip4addr_aton(IP, &state->remote_addr))error("Invalid address format");
                } else {
                        cyw43_arch_lwip_begin();
                        int err = dns_gethostbyname(IP, &state->remote_addr, tcp_dns_found, state);
                        cyw43_arch_lwip_end();
                        Timer4=timeout;
                        while(!state->complete && Timer4 && !(err==ERR_OK)){{if(startupcomplete)cyw43_arch_poll();}};
                        if(!Timer4)error("Failed to convert web address");
                        state->complete=0;
                }
                if (!tcp_client_open(state)) {
                        error("Failed to open client");
                }

                Timer4=timeout;
                while(!state->connected && Timer4){{if(startupcomplete)cyw43_arch_poll();}}
                if(!Timer4)error("No response from client");
                return;
        }

        tp=checkstring(cmdline, "TCP CLIENT REQUEST");
        if(tp){
                void *ptr1 = NULL;
                int64_t *dest=NULL;
                uint8_t *q=NULL;
                int size, timeout=5000;
                TCP_CLIENT_T *state = TCP_CLIENT;
                getargs(&tp,5,",");
                if(!state)error("No connection");
                if(!state->connected)error("No connection");
                if(argc<3)error("Syntax");
                char *request=getCstring(argv[0]);
                ptr1 = findvar(argv[2], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
                if(vartbl[VarIndex].type & T_INT) {
                        if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
                        if(vartbl[VarIndex].dims[0] <= 0) {      // Not an array
                                error("Argument 2 must be integer array");
                        }
                        size=(vartbl[VarIndex].dims[0] - OptionBase)*8;
                        dest = (long long int *)ptr1;
                        dest[0]=0;
                        q=(uint8_t *)&dest[1];
                } else error("Argument 2 must be integer array");
                if(argc==5)timeout=getint(argv[4],1,0xFFFFFF);
                state->BUF_SIZE=size;
                state->buffer=q;
                state->buffer_len=0;
                err_t err = tcp_write(state->tcp_pcb, request, strlen(request), 0);
                if(err)error("write failed %",err);
                Timer4=timeout;
                while(!state->buffer_len && Timer4){{if(startupcomplete)cyw43_arch_poll();}}
                if(!Timer4)error("No response from server");
                return;
        }
        tp=checkstring(cmdline, "CLOSE TCP CLIENT");
        if(tp){
                TCP_CLIENT_T *state = TCP_CLIENT;
                if(!state)error("No connection");
                tcp_client_close(state) ;
                free(state);
                TCP_CLIENT=NULL;
                return;
        }

        tp=checkstring(cmdline, "TCP READ");
        if(tp){
                void *ptr1 = NULL;
                int64_t *dest=NULL;
                uint8_t *q=NULL;
                int size;
                TCP_SERVER_T *state = TCPstate;
                getargs(&tp, 3, ",");
                if(argc!=3)error("Syntax");
                int pcb=getint(argv[0],1,MaxPcb)-1;
                ptr1 = findvar(argv[2], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
                if(vartbl[VarIndex].type & T_INT) {
                        if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
                        if(vartbl[VarIndex].dims[0] <= 0) {      // Not an array
                                error("Argument 2 must be integer array");
                        }
                        size=(vartbl[VarIndex].dims[0] - OptionBase +1)*8;
                        dest = (long long int *)ptr1;
                        dest[0]=0;
                        q=(uint8_t *)&dest[1];
                } else error("Argument 2 must be integer array");
                if(!state->inttrig[pcb]){
                        memset(ptr1,0,size);
                        return;
                }
                if(size-8<state->recv_len[pcb])error("array too small");
                cyw43_arch_lwip_begin();
                memcpy(q,state->buffer_recv[pcb],state->recv_len[pcb]);
                dest[0]=  state->recv_len[pcb];             
                memset(state->buffer_recv[pcb],0,state->recv_len[pcb]);
                state->recv_len[pcb]==0; 
                state->inttrig[pcb]=0;
                cyw43_arch_lwip_end();
                return;
        }
        tp=checkstring(cmdline, "TCP SEND");
        if(tp){
                TCP_SERVER_T *state = (TCP_SERVER_T*)TCPstate;
                 void *ptr1 = NULL;
                int64_t *dest=NULL;
                uint8_t *q=NULL;
                getargs(&tp, 3, ",");
                if(!TCPstate)error("Server not open");
                if(argc != 3)error("Argument count");
                state->write_pcb = getint(argv[0],1,MaxPcb)-1;
                ptr1 = findvar(argv[2], V_FIND | V_EMPTY_OK | V_NOFIND_ERR);
                if(vartbl[VarIndex].type & T_INT) {
                if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
                if(vartbl[VarIndex].dims[0] <= 0) {      // Not an array
                        error("Argument 1 must be integer array");
                }
                dest = (long long int *)ptr1;
                q=(uint8_t *)&dest[1];
                } else error("Argument must be integer array");
                int j=(vartbl[VarIndex].dims[0] - OptionBase);
                state->buffer_sent=q;
                state->to_send[state->write_pcb]=dest[0];
                if(state->client_pcb[state->write_pcb])tcp_server_send_data(state, state->client_pcb[state->write_pcb]);
                cyw43_arch_lwip_check();
           return;   
        }

        error("Syntax");
}
void checkTCPOptions(void){
    unsigned char *tp = checkstring(cmdline, "TCP SERVER");
    if(tp) {
   	if(CurrentLinePtr) error("Invalid in a program");
        getargs(&cmdline, 1, ",");
        if(argc!=1)error("Syntax");
        Option.TCP_PORT=getint(argv[0],1,65535);
        return;
        _excep_code = RESET_COMMAND;
        SoftReset();
    }
}
void fun_json(void){
    char *json_string=NULL;
    const cJSON *root = NULL;
    void *ptr1 = NULL;
    char *p;
    sret=GetTempMemory(STRINGSIZE);
    int64_t *dest=NULL;
    MMFLOAT tempd;
    int i,j,k,mode,index;
    char field[32],num[6];
    getargs(&ep, 3, ",");
    char *a=GetTempMemory(STRINGSIZE);
    ptr1 = findvar(argv[0], V_FIND | V_EMPTY_OK);
    if(vartbl[VarIndex].type & T_INT) {
    if(vartbl[VarIndex].dims[1] != 0) error("Invalid variable");
    if(vartbl[VarIndex].dims[0] <= 0) {		// Not an array
        error("Argument 1 must be integer array");
    }
    dest = (long long int *)ptr1;
    json_string=(char *)&dest[1];
    } else error("Argument 1 must be integer array");
    cJSON * parse = cJSON_Parse(json_string);
    if(parse==NULL)error("Invalid JSON data");
    root=parse;
    p=getCstring(argv[2]);
    int len = strlen(p);
    memset(field,0,32);
    memset(num,0,6);
    i=0;j=0;k=0;mode=0;
    while(i<len){
        if(p[i]=='['){ //start of index
            mode=1;
            field[j]=0;
            root = cJSON_GetObjectItemCaseSensitive(root, field);
            memset(field,0,32);
            j=0;
        }
        if(p[i]==']'){
            num[k]=0;
            index=atoi(num);
            root = cJSON_GetArrayItem(root, index);
            memset(num,0,6);
            k=0;
        }
        if(p[i]=='.'){ //new field separator
            if(mode==0){
                field[j]=0;
                root = cJSON_GetObjectItemCaseSensitive(root, field);
             memset(field,0,32);
                j=0;
            } else { //step past the dot after a close bracket
                mode=0;
            }
        } else  {
            if(mode==0)field[j++]=p[i];
            else if(p[i]!='[')num[k++]=p[i];
        }
        i++;
    }
    root = cJSON_GetObjectItem(root, field);

    if (cJSON_IsObject(root)){
        cJSON_Delete(parse);
        error("Not an item");
        return;
    }
    if (cJSON_IsInvalid(root)){
        cJSON_Delete(parse);
        error("Not an item");
        return;
    }
    if (cJSON_IsNumber(root))
    {
        tempd = root->valuedouble;

        if((MMFLOAT)((int64_t)tempd)==tempd) IntToStr(a,(int64_t)tempd,10);
        else FloatToStr(a, tempd, 0, STR_AUTO_PRECISION, ' ');   // set the string value to be saved
        cJSON_Delete(parse);
        sret=a;
        sret=CtoM(sret);
        targ=T_STR;
        return;
    }
    if (cJSON_IsBool(root)){
        int64_t tempint;
        tempint=root->valueint;
        cJSON_Delete(parse);
        if(tempint)strcpy(sret,"true");
        else strcpy(sret,"false");
        sret=CtoM(sret);
        targ=T_STR;
        return;
    }
    if (cJSON_IsString(root)){
        strcpy(a,root->valuestring);
        cJSON_Delete(parse);
        sret=a;
        sret=CtoM(sret);
        targ=T_STR;
        return;
    }
    sret=a;
}

#endif