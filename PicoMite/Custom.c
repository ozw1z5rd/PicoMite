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
                                if((*ss>='0' && *ss<='9') || *ss=='&' ){
                                        char *ppp=ss;
                                        if(*ss=='&'){
                                                if(!(toupper(ss[1])=='B' || toupper(ss[1])=='H' || toupper(ss[1])=='O')) error("Syntax");
                                                ppp+=2;
                                        } 
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
                                if((*ss>='0' && *ss<='9') || *ss=='&' ){
                                        char *ppp=ss;
                                        if(*ss=='&'){
                                                if(!(toupper(ss[1])=='B' || toupper(ss[1])=='H' || toupper(ss[1])=='O')) error("Syntax");
                                                ppp+=2;
                                        } 
                                        while(*ppp>='0' && *ppp<='9' && *ppp){ppp++;}
                                        if(*ppp){
                                                save=*ppp;
                                                *ppp=',';
                                        }
                                } else error("Syntax");
                                int bits=getint(ss,0,rel==2? 31 : 7);
                                if(*ppp==',')*ppp=save;
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
                                if((*ss>='0' && *ss<='9') || *ss=='&' ){
                                        char *ppp=ss;
                                        if(*ss=='&'){
                                                if(!(toupper(ss[1])=='B' || toupper(ss[1])=='H' || toupper(ss[1])=='O')) error("Syntax");
                                                ppp+=2;
                                        } 
                                        while(*ppp>='0' && *ppp<='9' && *ppp){ppp++;}
                                        if(*ppp){
                                                save=*ppp;
                                                *ppp=',';
                                        }
                                } else error("Syntax");
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
                                if((*ss>='0' && *ss<='9') || *ss=='&' ){
                                        char *ppp=ss;
                                        if(*ss=='&'){
                                                if(!(toupper(ss[1])=='B' || toupper(ss[1])=='H' || toupper(ss[1])=='O')) error("Syntax");
                                                ppp+=2;
                                        } 
                                        while(*ppp>='0' && *ppp<='9' && *ppp){ppp++;}
                                        if(*ppp){
                                                save=*ppp;
                                                *ppp=',';
                                        }
                                } else error("Syntax");
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
                                } else if(strncasecmp(ss,"X",1)==0 && (ss[1]==0 || ss[1]==' ' || ss[1]==',') ){
                                        ss++;
                                        ins|=0b1;
                                } else if(strncasecmp(ss,"Y",1)==0 && (ss[1]==0 || ss[1]==' ' || ss[1]==',') ){
                                        ss++;
                                        ins|=0b10;
                                } else if(strncasecmp(ss,"NULL",4)==0 && (ss[4]==0 || ss[4]==' ' || ss[4]==',') ){
                                        ss+=4;
                                        ins|=0b11;
                                } else if(strncasecmp(ss,"STATUS",6)==0 && (ss[6]==0 || ss[6]==' ' || ss[6]==',') ){
                                        ss+=6;
                                        ins|=0b101;
                                } else if(strncasecmp(ss,"ISR",3)==0 && (ss[3]==0 || ss[3]==' ' || ss[3]==',') ){
                                        ss+=3;
                                        ins|=0b110;
                                } else if(strncasecmp(ss,"OSR",3)==0 && (ss[3]==0 || ss[3]==' ' || ss[3]==',') ){
                                        ss+=3;
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
                                if((*ss>='0' && *ss<='9') || *ss=='&' ){
                                        char *ppp=ss;
                                        if(*ss=='&'){
                                                if(!(toupper(ss[1])=='B' || toupper(ss[1])=='H' || toupper(ss[1])=='O')) error("Syntax");
                                                ppp+=2;
                                        } 
                                        while(*ppp>='0' && *ppp<='9' && *ppp){ppp++;}
                                        if(*ppp){
                                                save=*ppp;
                                                *ppp=',';
                                        }
                                } else error("Syntax");
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
                                for(int i=PIOstart; i<PIOstart+totallines;i++){
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
                                for(int i=PIOstart;i<PIOstart+totallines;i++){
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
 
static int scan_result(void *env, const cyw43_ev_scan_result_t *result) {
    if (result) {
        Timer4=5000;
        char buff[STRINGSIZE]={0};
        sprintf(buff,"ssid: %-32s rssi: %4d chan: %3d mac: %02x:%02x:%02x:%02x:%02x:%02x sec: %u\r\n",
            result->ssid, result->rssi, result->channel,
            result->bssid[0], result->bssid[1], result->bssid[2], result->bssid[3], result->bssid[4], result->bssid[5],
            result->auth_mode);
        MMPrintString(buff);
    } 
    return 0;
}
void cmd_web(void){
        unsigned char *tp;
        tp=checkstring(cmdline, "CONNECT");
        if(tp){
            if(cyw43_wifi_link_status(&cyw43_state,CYW43_ITF_STA)<0){
                WebConnect();
            }
            return;   
        }
        if(!(WIFIconnected &&  cyw43_tcpip_link_status(&cyw43_state,CYW43_ITF_STA)==CYW43_LINK_UP))error("WIFI not connected");
        tp=checkstring(cmdline, "NTP");
        if(tp){
            cmd_ntp(tp);
            return;   
        }
        tp=checkstring(cmdline, "NTP");
        if(tp){
            cmd_ntp(tp);
            return;   
        }
        tp=checkstring(cmdline, "SCAN");
        if(tp){
               cyw43_wifi_scan_options_t scan_options = {0};
                int err = cyw43_wifi_scan(&cyw43_state, &scan_options, NULL, scan_result);
                if (err == 0) {
                    MMPrintString("\nPerforming wifi scan\n");
                } else {
                    char buff[STRINGSIZE]={0};
                    sprintf(buff,"Failed to start scan: %d\n", err);
                    MMPrintString(buff);
                }
                Timer4=500;
            while (Timer4){{if(startupcomplete)cyw43_arch_poll();}}
            return;   
        }
        if(cmd_mqtt())return;
        if(cmd_tcpclient())return;
        if(cmd_tcpserver())return;
        if(cmd_tls())return;
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
    cJSON_InitHooks(NULL);
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
    cJSON_Delete(parse);
    targ=T_STR;
    sret=a;
}

#endif