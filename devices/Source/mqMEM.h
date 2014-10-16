/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

// Memory manager

#ifndef _MQMEM_H
#define _MQMEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>

typedef struct
{
    void * pHead;
    void * pTail;
}Queue_t;

void mqInit(void);
void *mqAlloc(size_t xWantedSize);
void mqFree(void *pv);
size_t MEM_GetFreeHeapSize(void);

//Queue_t * MEM_Create_Queue(void);
bool mqEnqueue(Queue_t * pQueue, void * pBuf);
void * mqDequeue(Queue_t * pQueue);

#ifdef __cplusplus
}
#endif

#endif  //  _MQMEM_H