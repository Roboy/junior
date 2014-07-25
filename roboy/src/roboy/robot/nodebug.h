/*Copyright (c) 2014, University of Zurich, Department of Informatics, Artificial Intelligence Laboratory
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its contributors 
   may be used to endorse or promote products derived from this software without 
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdlib.h>

#ifndef NODEBUG
#define NODEBUG

#ifdef TRACE
#undef TRACE
#endif

#ifdef TRACE_GREEN
#undef TRACE_GREEN
#endif

#ifdef TRACE_RED
#undef TRACE_RED
#endif

#ifdef TRACE_BLUE
#undef TRACE_BLUE
#endif

#ifdef TRACE_CYAN
#undef TRACE_CYAN
#endif

#ifdef TRACE_PURP
#undef TRACE_PURP
#endif

#ifdef TRACE_GREEN_BOLD
#undef TRACE_GREEN_BOLD
#endif

#ifdef TRACE_RED_BOLD
#undef TRACE_RED_BOLD
#endif

#ifdef TRACE_BLUE_BOLD
#undef TRACE_BLUE_BOLD
#endif

#ifdef TRACE_CYAN_BOLD
#undef TRACE_CYAN_BOLD
#endif

#ifdef TRACE_PURP_BOLD
#undef TRACE_PURP_BOLD
#endif



#ifdef TRACE_GREEN_LN
#undef TRACE_GREEN_LN
#endif

#ifdef TRACE_RED_LN
#undef TRACE_RED_LN
#endif

#ifdef TRACE_BLUE_LN
#undef TRACE_BLUE_LN
#endif

#ifdef TRACE_CYAN_LN
#undef TRACE_CYAN_LN
#endif

#ifdef TRACE_PURP_LN
#undef TRACE_PURP_LN
#endif

#ifdef TRACE_GREEN_BOLD_LN
#undef TRACE_GREEN_BOLD_LN
#endif

#ifdef TRACE_RED_BOLD_LN
#undef TRACE_RED_BOLD_LN
#endif

#ifdef TRACE_BLUE_BOLD_LN
#undef TRACE_BLUE_BOLD_LN
#endif

#ifdef TRACE_CYAN_BOLD_LN
#undef TRACE_CYAN_BOLD_LN
#endif

#ifdef TRACE_PURP_BOLD_LN
#undef TRACE_PURP_BOLD_LN
#endif

#ifdef BREAK_POINT
#undef BREAK_POINT
#endif

#ifdef DEBUG
#undef DEBUG
#endif

#define TRACE(a...) while(0) 
	
#define TRACE_GREEN(a...) while(0) 
#define TRACE_RED(a...) do{{printf("\033[0;31m");printf(a);printf("\033[0;37m");}}while(0) 
#define TRACE_BLUE(a...) while(0) 
#define TRACE_CYAN(a...) while(0) 
#define TRACE_PURP(a...) while(0) 

#define TRACE_GREEN_BOLD(a...) while(0) 
#define TRACE_RED_BOLD(a...) do{{printf("\033[1;31m");printf(a);printf("\033[0;37m");}}while(0) 
#define TRACE_BLUE_BOLD(a...) while(0) 
#define TRACE_CYAN_BOLD(a...) while(0) 
#define TRACE_PURP_BOLD(a...) while(0) 

#define TRACE_GREEN_BOLD_LN(a...) while(0) 
#define TRACE_RED_BOLD_LN(a...) do{{TRACE_RED_BOLD(a);printf("\033[0;37m\n");}}while(0) 
#define TRACE_BLUE_BOLD_LN(a...) while(0)  
#define TRACE_CYAN_BOLD_LN(a...) while(0)  
#define TRACE_PURP_BOLD_LN(a...) while(0)

#define TRACE_GREEN_LN(a...) while(0) 
#define TRACE_RED_LN(a...) do{{TRACE_RED(a);TRACE("\n");printf("\033[0;37m\n");}}while(0) 
#define TRACE_BLUE_LN(a...) while(0) 
#define TRACE_CYAN_LN(a...) while(0) 
#define TRACE_PURP_LN(a...) while(0)  


#define BREAK_POINT(a...) while(0)

#endif
