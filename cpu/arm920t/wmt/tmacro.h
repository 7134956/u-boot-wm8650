/*++ 
Copyright (c) 2010 WonderMedia Technologies, Inc.

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details. You
should have received a copy of the GNU General Public License along with this
program. If not, see http://www.gnu.org/licenses/>.

WonderMedia Technologies, Inc.
10F, 529, Chung-Cheng Road, Hsin-Tien, Taipei 231, R.O.C.
--*/


#ifndef __TMACRO_H__
#define __TMACRO_H__


#if !defined(__TTYPE_H__)
#include "ttype.h"
#endif


/****** Common helper macros ***********************************************/

#if !defined(LONIBBLE)
#define LONIBBLE(b)         ((BYTE)(b) & 0x0F)
#endif
#if !defined(HINIBBLE)
#define HINIBBLE(b)         ((BYTE)(((WORD)(b) >> 4) & 0x0F))
#endif

#if !defined(LOBYTE)
#define LOBYTE(w)           ((BYTE)(w))
#endif
#if !defined(HIBYTE)
#define HIBYTE(w)           ((BYTE)(((WORD)(w) >> 8) & 0xFF))
#endif

#if !defined(LOWORD)
#define LOWORD(d)           ((WORD)(d))
#endif
#if !defined(HIWORD)
#define HIWORD(d)           ((WORD)((((DWORD)(d)) >> 16) & 0xFFFF))
#endif

#define LODWORD(q)          ((q).u.dwLowDword)
#define HIDWORD(q)          ((q).u.dwHighDword)

#define MAKEBYTE(ln, hn)    ((BYTE)(((BYTE)(ln) & 0x0F) | ((BYTE)(hn) << 4)))
#if !defined(MAKEWORD)
#define MAKEWORD(lb, hb)    ((WORD)(((BYTE)(lb)) | (((WORD)((BYTE)(hb))) << 8)))
#endif
#if !defined(MAKEDWORD)
#define MAKEDWORD(lw, hw)   ((DWORD)(((WORD)(lw)) | (((DWORD)((WORD)(hw))) << 16)))
#endif
#define MAKEQWORD(ld, hd, pq) {pq->u.dwLowDword = ld; pq->u.dwHighDword = hd;}

#if !defined(MAKELONG)
#define MAKELONG(low, high) ((LONG)(((WORD)(low)) | (((DWORD)((WORD)(high))) << 16)))
#endif

#if !defined(REVDWORD)
#define REVDWORD(d) (MAKEDWORD(MAKEWORD(HIBYTE(HIWORD(d)),LOBYTE(HIWORD(d))),MAKEWORD(HIBYTE(LOWORD(d)),LOBYTE(LOWORD(d)))))
#endif

/****** Misc macros ********************************************************/
#define RANDOM(_x_)    (UINT)(((UINT)rand()*(_x_)) / (UINT)65536)

#endif // __TMACRO_H__

