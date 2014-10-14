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

#include <stdlib.h>

void MEM_Init(void);
void *MEM_Malloc(size_t xWantedSize);
void MEM_Free(void *pv);
size_t MEM_GetFreeHeapSize(void);

#endif  //  _MEMMANG_H