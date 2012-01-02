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

#include <common.h>
#include <config.h>
#include <asm/byteorder.h>
#include <part.h>

#include "mem.h"


block_dev_desc_t block_mem_fat = {
	if_type: IF_TYPE_MEM,			/* type of the interface */
	dev: 0,	  						/* device number */
	part_type: PART_TYPE_DOS,  		/* partition type */
	//target: ,						/* target SCSI ID */
	//lun:,							/* target LUN */
	type: DEV_TYPE_HARDDISK,		/* device type */
	removable: 1,					/* removable device */
	lba: BLOCK_NUM ,				/* number of blocks */
	blksz: BLOCK_SIZE,		/* block size */
	vendor: "VIA", 	/* IDE model, SCSI Vendor */
	product: "VT8500",	/* IDE Serial no, SCSI product */
	revision: "019",	/* firmware revision */
	block_read: mem_block_read	
};

block_dev_desc_t block_mem_ext2 = {
	if_type: IF_TYPE_MEM,			/* type of the interface */
	dev: 0,	  						/* device number */
	//target: ,						/* target SCSI ID */
	//lun:,							/* target LUN */
	type: DEV_TYPE_HARDDISK,		/* device type */
	removable: 1,					/* removable device */
	lba: BLOCK_NUM ,				/* number of blocks */
	blksz: BLOCK_SIZE,		/* block size */
	vendor: "VIA", 	/* IDE model, SCSI Vendor */
	product: "VT8500",	/* IDE Serial no, SCSI product */
	revision: "019",	/* firmware revision */
	block_read: mem_block_read	
};


unsigned long 
mem_block_read(int dev,
				unsigned long start,
				lbaint_t blkcnt,
				unsigned long *buffer)
{
	unsigned long *src = (unsigned long *)(RAMDISK_ADDRESS + start * BLOCK_SIZE);
	unsigned long * dest = buffer;
	unsigned int count = blkcnt * BLOCK_SIZE;

	if(!blkcnt)
		return 0;
	//printf("src : %p, dest : %p, count %d\n", src, dest, count);
	
	memcpy(dest, src, count);
	return blkcnt;
	
}


















































