### PICOMITE HGC
<img src="https://github.com/ozw1z5rd/PicoMite/blob/main/IMAGES/SS3.png?raw=true" width="500" />

At the moment the branch main is a copy of the original repository. The reworked source code are in different branches: you should switch to `hcg20` if you want to build the code for the PICO PI or `hcg20-wifi` for the PICO PI W ( with wifi support ). 

The code reworked and stored in this repository ( not the original one ) compiles with no issue with the pico-sdk@45984e2

```
 $ cd pico-sdk
 $ git checkout 45984e2
```

 A numbe of bugs have been fixed, the procedure has been tested on a Raspberry PI. These source code are part of the NEXT project from Home Computer Group. 

PicoMite is the porting of MMBasic to PICO PI, it is a powerful BASIC intepreter with full PICO PI's hardware support.  Reading the manual is not an option. Visit https://geoffg.net/picomite.html for prebuild binaries, manuals and general information.

To follow the NEXT project visiti the https://homecomputer.group website, the PicoMite is covered in the Issue #20 ( available in September 2024 ), in this issue there are all the instruction to compile and uplod using the SWD and OpenOCD.
 
<img src="https://github.com/ozw1z5rd/PicoMite/blob/main/IMAGES/SS1.png?raw=true" width="700" />

Have fun!

**********************************

PicoMite MMBasic


<COPYRIGHT HOLDERS>  Geoff Graham, Peter Mather
Copyright (c) 2021, <COPYRIGHT HOLDERS> All rights reserved.
    
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met: 
1.	Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2.	Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer
    in the documentation and/or other materials provided with the distribution.
3.	The name MMBasic be used when referring to the interpreter in any documentation and promotional material and the original copyright message be displayed 
    on the console at startup (additional copyright messages may be added).
4.	All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed 
    by Geoff Graham, Peter Mather.
5.	Neither the name of the <copyright holder> nor the names of its contributors may be used to endorse or promote products derived from this software 
    without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY Geoff Graham, Peter Mather AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDERS> BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 

************************************************************************************************************************
