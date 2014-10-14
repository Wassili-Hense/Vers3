#include "config.h"

// AVR Architecture specifics.
#define portPOINTER_SIZE_TYPE       uintptr_t
#define portBYTE_ALIGNMENT          1
#define configTOTAL_HEAP_SIZE       1200    // ATMega328P


// Local defines
typedef struct S_BLOCK_LINK
{
    struct S_BLOCK_LINK *pNextFreeBlock;
    size_t BlockSize;
} BlockLink_t;

#if (portBYTE_ALIGNMENT == 8)
    #define portBYTE_ALIGNMENT_MASK     (0x0007U)
#elif (portBYTE_ALIGNMENT == 4)
    #define portBYTE_ALIGNMENT_MASK     (0x0003)
#elif (portBYTE_ALIGNMENT == 2)
    #define portBYTE_ALIGNMENT_MASK     (0x0001)
#elif (portBYTE_ALIGNMENT == 1)
    #define portBYTE_ALIGNMENT_MASK     (0x0000)
#else
    #error "Invalid portBYTE_ALIGNMENT definition"
#endif

#define configADJUSTED_HEAP_SIZE    (configTOTAL_HEAP_SIZE - portBYTE_ALIGNMENT)
#define heapSTRUCT_SIZE ((uint16_t)((sizeof(BlockLink_t) + (portBYTE_ALIGNMENT - 1)) & ~portBYTE_ALIGNMENT_MASK))

// Allocate the memory for the heap.
static uint8_t mem_heap[configTOTAL_HEAP_SIZE];
// Create a couple of list links to mark the start and end of the list.
static BlockLink_t mem_start, mem_end;

/* Keeps track of the number of free bytes remaining, but says nothing about
fragmentation. */
static size_t xFreeBytesRemaining = configADJUSTED_HEAP_SIZE;

void MEM_Init(void)
{
    BlockLink_t * pFirstFreeBlock;
    uint8_t     * pucAlignedHeap;

    /* Ensure the heap starts on a correctly aligned boundary. */
    pucAlignedHeap = (uint8_t *)(((portPOINTER_SIZE_TYPE)&mem_heap[portBYTE_ALIGNMENT]) & ((portPOINTER_SIZE_TYPE)~portBYTE_ALIGNMENT_MASK));

    /* mem_start is used to hold a pointer to the first item in the list of free
    blocks.  The void cast is used to prevent compiler warnings. */
    mem_start.pNextFreeBlock = (void *)pucAlignedHeap;
    mem_start.BlockSize = (size_t)0;

    /* mem_end is used to mark the end of the list of free blocks. */
    mem_end.BlockSize = configADJUSTED_HEAP_SIZE;
    mem_end.pNextFreeBlock = NULL;

    /* To start with there is a single free block that is sized to take up the
    entire heap space. */
    pFirstFreeBlock = (void *)pucAlignedHeap;
    pFirstFreeBlock->BlockSize = configADJUSTED_HEAP_SIZE;
    pFirstFreeBlock->pNextFreeBlock = &mem_end;
}
/*-----------------------------------------------------------*/

void *MEM_Malloc(size_t xWantedSize)
{
    BlockLink_t *pxBlock, *pxPreviousBlock, *pxNewBlockLink;
    void *pvReturn = NULL;

    /* The wanted size is increased so it can contain a BlockLink_t
    structure in addition to the requested amount of bytes. */
    if(xWantedSize > 0)
    {
        xWantedSize += heapSTRUCT_SIZE;

        /* Ensure that blocks are always aligned to the required number of bytes. */
        if((xWantedSize & portBYTE_ALIGNMENT_MASK) != 0)
        {
            /* Byte alignment required. */
            xWantedSize += (portBYTE_ALIGNMENT - (xWantedSize & portBYTE_ALIGNMENT_MASK));
        }
    }

    if((xWantedSize > 0) && (xWantedSize < configADJUSTED_HEAP_SIZE))
    {
        /* Blocks are stored in byte order - traverse the list from the start
        (smallest) block until one of adequate size is found. */
        pxPreviousBlock = &mem_start;
        pxBlock = mem_start.pNextFreeBlock;
        while( ( pxBlock->BlockSize < xWantedSize ) && ( pxBlock->pNextFreeBlock != NULL ) )
        {
            pxPreviousBlock = pxBlock;
            pxBlock = pxBlock->pNextFreeBlock;
        }

        /* If we found the end marker then a block of adequate size was not found. */
        if(pxBlock != &mem_end)
        {
            /* Return the memory space - jumping over the BlockLink_t structure
                at its start. */
            pvReturn = (void *)(((uint8_t *)pxPreviousBlock->pNextFreeBlock) + heapSTRUCT_SIZE);

            /* This block is being returned for use so must be taken out of the
                list of free blocks. */
            pxPreviousBlock->pNextFreeBlock = pxBlock->pNextFreeBlock;

            /* If the block is larger than required it can be split into two. */
            if((pxBlock->BlockSize - xWantedSize) > ((size_t)(heapSTRUCT_SIZE * 2)))
            {
                /* This block is to be split into two.  Create a new block
                    following the number of bytes requested. The void cast is
                    used to prevent byte alignment warnings from the compiler. */
                pxNewBlockLink = (void *)(((uint8_t *)pxBlock) + xWantedSize);

                // Calculate the sizes of two blocks split from the single block.
                pxNewBlockLink->BlockSize = pxBlock->BlockSize - xWantedSize;
                pxBlock->BlockSize = xWantedSize;

                // Insert the new block into the list of free blocks.
                {
                BlockLink_t *pxIterator;
                size_t BlockSize = pxNewBlockLink->BlockSize;
                for(pxIterator = &mem_start;
                    pxIterator->pNextFreeBlock->BlockSize < BlockSize;
                    pxIterator = pxIterator->pNextFreeBlock);

                pxNewBlockLink->pNextFreeBlock = pxIterator->pNextFreeBlock;
                pxIterator->pNextFreeBlock = pxNewBlockLink;
                }
            }
            xFreeBytesRemaining -= pxBlock->BlockSize;
        }
    }
    return pvReturn;
}

void MEM_Free(void *pv)
{
    uint8_t *puc = (uint8_t *)pv;
    BlockLink_t *pxLink;

    if(pv != NULL)
    {
        /* The memory being freed will have an BlockLink_t structure immediately
        before it. */
        puc -= heapSTRUCT_SIZE;

        /* This unexpected casting is to keep some compilers from issuing
        byte alignment warnings. */
        pxLink = (void *)puc;

        // Add this block to the list of free blocks.
        BlockLink_t *pxIterator;
        size_t BlockSize = pxLink->BlockSize;
        for(pxIterator = &mem_start;
            pxIterator->pNextFreeBlock->BlockSize < BlockSize;
            pxIterator = pxIterator->pNextFreeBlock);
        pxLink->pNextFreeBlock = pxIterator->pNextFreeBlock;
        pxIterator->pNextFreeBlock = pxLink;

        xFreeBytesRemaining += pxLink->BlockSize;
    }
}
/*-----------------------------------------------------------*/

size_t MEM_GetFreeHeapSize(void)
{
    return xFreeBytesRemaining;
}