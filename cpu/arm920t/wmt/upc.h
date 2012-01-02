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


#ifndef __UPC_H__
#define __UPC_H__

#if !defined(__TTYPE_H__)
#include "ttype.h"
#endif


/*---------------------  Export Definitions -------------------------*/

/*---------------------  Export Classes  ----------------------------*/

/*---------------------  Export Variables  --------------------------*/

/*---------------------  Export Functions  --------------------------*/
#if defined(__cplusplus)
extern "C" {                            /* Assume C declarations for C++ */
#endif // __cplusplus


// VIA Device do Memory-Mapped IO when in 32-bit environment
#define VNSvInPortB(dwIOAddress, pbyData)       \
{                                               \
    volatile BYTE* pbyAddr = ((PBYTE)(dwIOAddress));     \
    *(pbyData) = *pbyAddr;                      \
}

#define VNSvInPortW(dwIOAddress, pwData)        \
{                                               \
    volatile WORD* pwAddr = ((PWORD)(dwIOAddress));      \
    *(pwData) = *pwAddr;                        \
}

#define VNSvInPortD(dwIOAddress, pdwData)       \
{                                               \
    volatile DWORD* pdwAddr = ((PDWORD)(dwIOAddress));   \
    *(pdwData) = *pdwAddr;                      \
}

#define VNSvOutPortB(dwIOAddress, byData)       \
{                                               \
    volatile PBYTE pbyTmp = (PBYTE)(dwIOAddress);        \
    *pbyTmp = (byData);                         \
}

#define VNSvOutPortW(dwIOAddress, wData)        \
{                                               \
    volatile PWORD pwTmp = (PWORD)(dwIOAddress);         \
    *pwTmp = (wData);                           \
}

#define VNSvOutPortD(dwIOAddress, dwData)       \
{                                               \
    volatile PDWORD pdwTmp = (PDWORD)(dwIOAddress);      \
    *pdwTmp = (dwData);                         \
}

#define PCAvDelayByIO(wDelayUnit) {             \
    BYTE    byTemp;                             \
    ULONG   ii;                                 \
                                                \
    for (ii = 0; ii < (wDelayUnit); ii++)       \
        VNSvInPortB(0x61, &byTemp);             \
}

#if defined(__cplusplus)
}                                       /* End of extern "C" { */
#endif // __cplusplus

#endif // __UPC_H__

