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
#ifndef BLOCK_MEM_FS
#define BLOCK_MEM_FS

#include <ide.h>
//#include <part.h>

#define BLOCK_SIZE 512
#define RAMDISK_ADDRESS 0x1000000
#define BLOCK_NUM	32*0x100000 / BLOCK_SIZE

#define CONFIG_MEM


//extern block_dev_desc_t block_mem;

unsigned long mem_block_read(int dev,
								unsigned long start,
								lbaint_t blkcnt,
								unsigned long *buffer);

#endif
