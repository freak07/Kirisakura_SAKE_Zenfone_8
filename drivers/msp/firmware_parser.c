/* --COPYRIGHT--,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
#include "firmware_parser.h"
#include <linux/file.h>
#include <linux/fs.h>


#include "firmware.c"

tMSPMemorySegment MSPMemory[] = {
{0,0, NULL, NULL}, 
{0,0, NULL, NULL}, 
{0,0, NULL, NULL},
{0,0, NULL, NULL},
{0,0, NULL, NULL},
{0,0, NULL, NULL}
};

uint8_t* firmwareSegment[] = {NULL, NULL, NULL, NULL, NULL, NULL};

#define DEFAULT_ONE_LINE_MAX_SIZE  135000	//15000
#define DEFAULT_ONE_MAX_SEGMENT  65535		//5000
#define MAX_CALIBRATION_LINE 60
//Patch for 4/5 segment.
static int Zen7MAX_MSP_SEGMENT = 0;
//Get last fw version password.
extern uint8_t bslPassword[32];	

static void process_firmware_item(char* buf, ssize_t bufLen) {
uint16_t address;
uint8_t content;
ssize_t offset=0;
int8_t segmentIndex=-1;
uint16_t segmentCount=0;
int parseItems=0;
		
	//Patch for 4/5 segment.
	int MAX_MSP_SEGMENT;
	unsigned char SegCnt = 0;
	for(offset=0; offset<bufLen; offset++){
		if(buf[offset]=='@'){
			SegCnt++;
		}		
	}
	pr_err("[MCU] %s SegCnt:%d", __func__, SegCnt);
	MAX_MSP_SEGMENT = SegCnt;
	Zen7MAX_MSP_SEGMENT = MAX_MSP_SEGMENT;

	//const int MAX_MSP_SEGMENT= sizeof(firmwareSegment)/sizeof(firmwareSegment[0]);
	for (offset=0;offset<MAX_MSP_SEGMENT;offset++) {
		firmwareSegment[offset]=kzalloc(sizeof(uint8_t)*DEFAULT_ONE_MAX_SEGMENT, GFP_KERNEL);
	}

	//process mybuf
	for (offset=0;offset<bufLen;) {
		if (buf[offset]==0xd || buf[offset]==0xa) {
			offset++;
			continue;
		}

		
	   if (buf[offset]=='@') {
			parseItems = sscanf(buf+offset, "@%4x",&address);
			if (parseItems) {
				pr_err("[MCU] get Address=0x%X, item=%d",address, parseItems);
				offset+=5;
				segmentIndex++;
				MSPMemory[segmentIndex].ui32MemoryStartAddr =address;
				if (segmentIndex>0) {
					MSPMemory[segmentIndex-1].ui32MemoryLength =segmentCount;
					MSPMemory[segmentIndex-1].ui8Buffer=firmwareSegment[segmentIndex-1];
					MSPMemory[segmentIndex-1].pNextSegment=(void*) &MSPMemory[segmentIndex];
					segmentCount=0;
				}
				continue;
			} 
	   	}
	 
		parseItems = sscanf(buf+offset, "%2x ",&content);
		
		if (parseItems) {
			firmwareSegment[segmentIndex][segmentCount]=content;
			segmentCount++;
			offset+=3;
			continue;
		} 

		if (buf[offset]=='q') {
			if (segmentIndex>=0) {
				MSPMemory[segmentIndex].ui32MemoryLength =segmentCount;
				MSPMemory[segmentIndex].ui8Buffer=firmwareSegment[segmentIndex];
				segmentCount=0;
			}
			break;
		}
		
		{
			char debug[50];
			memset(debug, 0, 50);
			memcpy(debug, buf+offset-10, 20);
			offset++;
			pr_err("[MCU] invalid offset=%d, content= { %s }",offset, debug);
		}
	}

	for (offset=0;offset<MAX_MSP_SEGMENT;offset++)
	{	
		pr_err("[MCU] MSPMemory @%4X, size = %d, = { 0x%X, 0x%X, ... 0x%X, 0x%X }",
			MSPMemory[offset].ui32MemoryStartAddr, 
			MSPMemory[offset].ui32MemoryLength, 
			MSPMemory[offset].ui8Buffer[0],
			MSPMemory[offset].ui8Buffer[1],
			MSPMemory[offset].ui8Buffer[MSPMemory[offset].ui32MemoryLength-2],
			MSPMemory[offset].ui8Buffer[MSPMemory[offset].ui32MemoryLength-1]
		);

		//Get last fw's password.
		if(MSPMemory[offset].ui32MemoryStartAddr == 0xFFDA){
			if(MSPMemory[offset].ui32MemoryLength != 38){
				pr_err("[MCU] get password lenght error.");
			}else{				
				//memcpy(bslPassword, &(MSPMemory[offset].ui8Buffer[6]), 32);
				pr_err("[MCU] get password: { 0x%X, 0x%X, ... 0x%X, 0x%X }",	\
					bslPassword[0], bslPassword[1], bslPassword[30], bslPassword[31]);
			}
		}
	}

}

bool read_kernel_file(const char* fileName, void (*process)(char*, ssize_t) ) {
	struct file *file;
	ssize_t n=0;
	loff_t file_offset = 0;
	uint8_t* mybuf=NULL;
	pr_info("[MCU] read_kernel_file++");
	mybuf = kzalloc(sizeof(uint8_t)*DEFAULT_ONE_LINE_MAX_SIZE, GFP_KERNEL);
	file = filp_open(fileName, O_RDONLY | O_LARGEFILE, 0);
	if (IS_ERR(file)) {
		pr_err("[MCU]: Can not open file(%s) with errno = %ld!\n", fileName, -PTR_ERR(file));
		return 0;
	}

	n = kernel_read(file, mybuf, DEFAULT_ONE_LINE_MAX_SIZE, &file_offset);
	if (n < DEFAULT_ONE_LINE_MAX_SIZE) {
		pr_info("[MCU]:Done  exit since EOF from file(%s) file size:%d\n",fileName, n);
		// END_OF_FILE;
		(*process)(mybuf, n);
		goto EOF;
	} else {
		pr_err("[MCU]: Can not open file(%s), file size too large = %d!\n", n);
	}
	//IN_PROGRESS
EOF:
	fput(file);
	kfree(mybuf);
	pr_info("[MCU] read_kernel_file--");
	return n < DEFAULT_ONE_LINE_MAX_SIZE;
}

static const char mcu_firmware_file[] = {"/vendor/firmware/mcu_firmware.txt"};
static const char mcu_firmware_file_9970[] = {"/vendor/firmware/mcu_firmware_9970.txt"};
tMSPMemorySegment* read_firmware_file() {
	 if(g_ASUS_hwID <= HW_REV_ER){
		 pr_info("[MCU] angle sensor is 9970, %d\n.", g_ASUS_hwID);
		 if (read_kernel_file(mcu_firmware_file_9970,  process_firmware_item)) {
			 return &MSPMemory[0];
		 }		
	 }else{
		 if (read_kernel_file(mcu_firmware_file,  process_firmware_item)) {
			 return &MSPMemory[0];
		 }	
	 } 
	return NULL;
}


tMSPMemorySegment* MSP430BSL_parseTextFile()
{
int i;
const int MAX_MSP_SEGMENT= sizeof(MemorySegment_Addr)/sizeof(MemorySegment_Addr[0]);
for (i=0;i<MAX_MSP_SEGMENT;i++)
{	
	MSPMemory[i].ui32MemoryStartAddr=MemorySegment_Addr[i];
	MSPMemory[i].ui32MemoryLength=MemorySegment_Size[i];
	MSPMemory[i].ui8Buffer=MemorySegment_Ptr[i];

	if (i==MAX_MSP_SEGMENT-1) {
		MSPMemory[i].pNextSegment=NULL;
	} else {	
		MSPMemory[i].pNextSegment=(void*) &MSPMemory[i+1];
	}
//	pr_err("MSPMemory[%d].pNextSegment assigned to 0x%p", i,MSPMemory[i].pNextSegment);
}
    return &MSPMemory[0];
}

void MSP430BSL_cleanUpPointer()
{
	//const int MAX_MSP_SEGMENT= sizeof(firmwareSegment)/sizeof(firmwareSegment[0]);
	const int MAX_MSP_SEGMENT= Zen7MAX_MSP_SEGMENT;
	int i=0;
	for (i=0;i<MAX_MSP_SEGMENT;i++) {
		kfree(firmwareSegment[i]);
	}
}

void MSP430BSL_parsePasswordFile(const uint8_t* passwordfile, uint8_t* password)
{
//empty
}


