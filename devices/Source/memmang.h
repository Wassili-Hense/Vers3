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

#ifndef _MEMMANG_H
#define _MEMMANG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>

typedef struct
{
    void * pHead;
    void * pTail;
}Queue_t;

void MEM_Init(void);
void *MEM_Malloc(size_t xWantedSize);
void MEM_Free(void *pv);
size_t MEM_GetFreeHeapSize(void);

//Queue_t * MEM_Create_Queue(void);
bool MEM_Enqueue(Queue_t * pQueue, void * pBuf);
void * MEM_Dequeue(Queue_t * pQueue);

#ifdef __cplusplus
}
#endif

#endif  //  _MEMMANG_H