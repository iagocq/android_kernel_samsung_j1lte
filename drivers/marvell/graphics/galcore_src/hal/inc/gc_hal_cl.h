/****************************************************************************
*
*    Copyright (c) 2005 - 2012 by Vivante Corp.
*    
*    This program is free software; you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation; either version 2 of the license, or
*    (at your option) any later version.
*    
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*    GNU General Public License for more details.
*    
*    You should have received a copy of the GNU General Public License
*    along with this program; if not write to the Free Software
*    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*
*
*****************************************************************************/




#ifndef __gc_hal_user_cl_h_
#define __gc_hal_user_cl_h_


#ifdef __cplusplus
extern "C" {
#endif

#define     USE_NEW_MEMORY_ALLOCATION   0

/******************************************************************************\
****************************** Object Declarations *****************************
\******************************************************************************/

/* gcoCL_DEVICE_INFO object. */
typedef struct _gcoCL_DEVICE_INFO
{
    gctUINT             maxComputeUnits;
    gctUINT             maxWorkItemDimensions;
    gctUINT             maxWorkItemSizes[3];
    gctUINT             maxWorkGroupSize;
    gctUINT64           maxGlobalWorkSize ;
    gctUINT             clockFrequency;

    gctUINT             addrBits;
    gctUINT64           maxMemAllocSize;
    gctUINT64           globalMemSize;
    gctUINT64           localMemSize;
    gctUINT             localMemType;               /* cl_device_local_mem_type */
    gctUINT             globalMemCacheType;         /* cl_device_mem_cache_type */
    gctUINT             globalMemCachelineSize;
    gctUINT64           globalMemCacheSize;
    gctUINT             maxConstantArgs;
    gctUINT64           maxConstantBufferSize;
    gctUINT             maxParameterSize;
    gctUINT             memBaseAddrAlign;
    gctUINT             minDataTypeAlignSize;

    gctBOOL             imageSupport;
    gctUINT             maxReadImageArgs;
    gctUINT             maxWriteImageArgs;
    gctUINT             vectorWidthChar;
    gctUINT             vectorWidthShort;
    gctUINT             vectorWidthInt;
    gctUINT             vectorWidthLong;
    gctUINT             vectorWidthFloat;
    gctUINT             vectorWidthDouble;
    gctUINT             vectorWidthHalf;
    gctUINT             image2DMaxWidth;
    gctUINT             image2DMaxHeight;
    gctUINT             image3DMaxWidth;
    gctUINT             image3DMaxHeight;
    gctUINT             image3DMaxDepth;
    gctUINT             maxSamplers;

    gctUINT64           queueProperties;        /* cl_command_queue_properties */
    gctBOOL             hostUnifiedMemory;
    gctBOOL             errorCorrectionSupport;
    gctUINT64           singleFpConfig;         /* cl_device_fp_config */
    gctUINT64           doubleFpConfig;         /* cl_device_fp_config */
    gctUINT             profilingTimingRes;
    gctBOOL             endianLittle;
    gctBOOL             deviceAvail;
    gctBOOL             compilerAvail;
    gctUINT64           execCapability;         /* cl_device_exec_capabilities */
    gctBOOL             atomicSupport;
    gctBOOL             psCL;
} gcoCL_DEVICE_INFO;

typedef gcoCL_DEVICE_INFO *  gcoCL_DEVICE_INFO_PTR;


/*******************************************************************************
**
**  gcoCL_InitializeHardware
**
**  Initialize hardware.  This is required for each thread.
**
**  INPUT:
**
**      Nothing
**
**  OUTPUT:
**
**      Nothing
*/
gceSTATUS
gcoCL_InitializeHardware(
    );

gceSTATUS
gcoCL_InitializeEnv(
    );

/**********************************************************************
**
**  gcoCL_SetHardware
**
**  Set the gcoHARDWARE object for current thread.
**
**  INPUT:
**
**      Nothing
**
**  OUTPUT:
**
**      Nothing
*/
gceSTATUS
gcoCL_SetHardware(
    );

/*******************************************************************************
**
**  gcoCL_AllocateMemory
**
**  Allocate contiguous memory from the kernel.
**
**  INPUT:
**
**      gctSIZE_T * Bytes
**          Pointer to the number of bytes to allocate.
**
**  OUTPUT:
**
**      gctSIZE_T * Bytes
**          Pointer to a variable that will receive the aligned number of bytes
**          allocated.
**
**      gctPHYS_ADDR * Physical
**          Pointer to a variable that will receive the physical addresses of
**          the allocated memory.
**
**      gctPOINTER * Logical
**          Pointer to a variable that will receive the logical address of the
**          allocation.
**
**      gcsSURF_NODE_PTR  * Node
**          Pointer to a variable that will receive the gcsSURF_NODE structure
**          pointer that describes the video memory to lock.
*/
gceSTATUS
gcoCL_AllocateMemory(
    IN OUT gctSIZE_T *      Bytes,
    OUT gctPHYS_ADDR *      Physical,
    OUT gctPOINTER *        Logical,
    OUT gcsSURF_NODE_PTR *  Node
    );

/*******************************************************************************
**
**  gcoCL_FreeMemory
**
**  Free contiguous memeory to the kernel.
**
**  INPUT:
**
**      gctPHYS_ADDR Physical
**          The physical addresses of the allocated pages.
**
**      gctPOINTER Logical
**          The logical address of the allocation.
**
**      gctSIZE_T Bytes
**          Number of bytes allocated.
**
**      gcsSURF_NODE_PTR  Node
**          Pointer to a gcsSURF_NODE structure
**          that describes the video memory to unlock.
**
**  OUTPUT:
**
**      Nothing
*/
gceSTATUS
gcoCL_FreeMemory(
    IN gctPHYS_ADDR         Physical,
    IN gctPOINTER           Logical,
    IN gctSIZE_T            Bytes,
    IN gcsSURF_NODE_PTR     Node
    );

/*******************************************************************************
**
**  gcoCL_FlushMemory
**
**  Flush memory to the kernel.
**
**  INPUT:
**
**      gcsSURF_NODE_PTR  Node
**          Pointer to a gcsSURF_NODE structure
**          that describes the video memory to flush.
**
**      gctPOINTER Logical
**          The logical address of the allocation.
**
**      gctSIZE_T Bytes
**          Number of bytes allocated.
**
**  OUTPUT:
**
**      Nothing
*/
gceSTATUS
gcoCL_FlushMemory(
    IN gcsSURF_NODE_PTR     Node,
    IN gctPOINTER           Logical,
    IN gctSIZE_T            Bytes
    );

/*******************************************************************************
**
**  gcoCL_InvalidateMemoryCache
**
**  Invalidate memory cache in CPU.
**
**  INPUT:
**
**      gcsSURF_NODE_PTR  Node
**          Pointer to a gcsSURF_NODE structure
**          that describes the video memory to flush.
**
**      gctPOINTER Logical
**          The logical address of the allocation.
**
**      gctSIZE_T Bytes
**          Number of bytes allocated.
**
**  OUTPUT:
**
**      Nothing
*/
gceSTATUS
gcoCL_InvalidateMemoryCache(
    IN gcsSURF_NODE_PTR     Node,
    IN gctPOINTER           Logical,
    IN gctSIZE_T            Bytes
    );

/*******************************************************************************
**
**  gcoCL_ShareMemoryWithStream
**
**  Share memory with a stream.
**
**  INPUT:
**
**      gcoSTREAM Stream
**          Pointer to the stream object.
**
**  OUTPUT:
**
**      gctSIZE_T * Bytes
**          Pointer to a variable that will receive the aligned number of bytes
**          allocated.
**
**      gctPHYS_ADDR * Physical
**          Pointer to a variable that will receive the physical addresses of
**          the allocated memory.
**
**      gctPOINTER * Logical
**          Pointer to a variable that will receive the logical address of the
**          allocation.
**
**      gcsSURF_NODE_PTR  * Node
**          Pointer to a variable that will receive the gcsSURF_NODE structure
**          pointer that describes the video memory to lock.
*/
gceSTATUS
gcoCL_ShareMemoryWithStream(
    IN gcoSTREAM            Stream,
    OUT gctSIZE_T *         Bytes,
    OUT gctPHYS_ADDR *      Physical,
    OUT gctPOINTER *        Logical,
    OUT gcsSURF_NODE_PTR *  Node
    );

/*******************************************************************************
**
**  gcoCL_ShareMemoryWithBufObj
**
**  Share memory with a BufObj.
**
**  INPUT:
**
**      gcoBUFOBJ Stream
**          Pointer to the stream object.
**
**  OUTPUT:
**
**      gctSIZE_T * Bytes
**          Pointer to a variable that will receive the aligned number of bytes
**          allocated.
**
**      gctPHYS_ADDR * Physical
**          Pointer to a variable that will receive the physical addresses of
**          the allocated memory.
**
**      gctPOINTER * Logical
**          Pointer to a variable that will receive the logical address of the
**          allocation.
**
**      gcsSURF_NODE_PTR  * Node
**          Pointer to a variable that will receive the gcsSURF_NODE structure
**          pointer that describes the video memory to lock.
*/
gceSTATUS
gcoCL_ShareMemoryWithBufObj(
    IN gcoBUFOBJ            BufObj,
    OUT gctSIZE_T *         Bytes,
    OUT gctPHYS_ADDR *      Physical,
    OUT gctPOINTER *        Logical,
    OUT gcsSURF_NODE_PTR *  Node
    );

/*******************************************************************************
**
**  gcoCL_UnshareMemory
**
**  Unshare memory with a stream.
**
**  INPUT:
**
**      gcsSURF_NODE_PTR  Node
**          Pointer to a  gcsSURF_NODE structure
*/
gceSTATUS
gcoCL_UnshareMemory(
    IN gcsSURF_NODE_PTR     Node
    );

/*******************************************************************************
**
**  gcoCL_FlushSurface
**
**  Flush surface to the kernel.
**
**  INPUT:
**
**      gcoSURF           Surface
**          gcoSURF structure
**          that describes the surface to flush.
**
**  OUTPUT:
**
**      Nothing
*/
gceSTATUS
gcoCL_FlushSurface(
    IN gcoSURF              Surface
    );

gceSTATUS
gcoCL_LockSurface(
    IN gcoSURF Surface,
    OUT gctUINT32 * Address,
    OUT gctPOINTER * Memory
    );

gceSTATUS
gcoCL_UnlockSurface(
    IN gcoSURF Surface,
    IN gctPOINTER Memory
    );

/*******************************************************************************
**
**  gcoCL_CreateTexture
**
**  Create texture for image.
**
**  INPUT:
**
**      gctUINT Width
**          Width of the image.
**
**      gctUINT Heighth
**          Heighth of the image.
**
**      gctUINT Depth
**          Depth of the image.
**
**      gctCONST_POINTER Memory
**          Pointer to the data of the input image.
**
**      gctUINT Stride
**          Size of one row.
**
**      gctUINT Slice
**          Size of one plane.
**
**      gceSURF_FORMAT FORMAT
**          Format of the image.
**
**      gceENDIAN_HINT EndianHint
**          Endian needed to handle the image data.
**
**  OUTPUT:
**
**      gcoTEXTURE * Texture
**          Pointer to a variable that will receive the gcoTEXTURE structure.
**
**      gcoSURF * Surface
**          Pointer to a variable that will receive the gcoSURF structure.
**
**      gctPHYS_ADDR * Physical
**          Pointer to a variable that will receive the physical addresses of
**          the allocated memory.
**
**      gctPOINTER * Logical
**          Pointer to a variable that will receive the logical address of the
**          allocation.
*/
gceSTATUS
gcoCL_CreateTexture(
    IN gctBOOL              Maped,
    IN gctUINT              Width,
    IN gctUINT              Height,
    IN gctUINT              Depth,
    IN gctCONST_POINTER     Memory,
    IN gctUINT              Stride,
    IN gctUINT              Slice,
    IN gceSURF_FORMAT       Format,
    IN gceENDIAN_HINT       EndianHint,
    OUT gcoTEXTURE *        Texture,
    OUT gcoSURF *           Surface,
    OUT gctPHYS_ADDR *      Physical,
    OUT gctPOINTER *        Logical,
    OUT gctSIZE_T *         RowPitch,
    OUT gctPOINTER*         MapInfo
    );

/*******************************************************************************
**
**  gcoCL_DestroyTexture
**
**  Destroy an gcoTEXTURE object.
**
**  INPUT:
**
**      gcoTEXTURE Texture
**          Pointer to an gcoTEXTURE object.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gcoCL_DestroyTexture(
    IN gcoTEXTURE Texture
    );

/*******************************************************************************
**
**  gcoCL_QueryDeviceInfo
**
**  Query the OpenCL capabilities of the device.
**
**  INPUT:
**
**      Nothing
**
**  OUTPUT:
**
**      gcoCL_DEVICE_INFO_PTR DeviceInfo
**          Pointer to the device information
*/
gceSTATUS
gcoCL_QueryDeviceInfo(
    OUT gcoCL_DEVICE_INFO_PTR   DeviceInfo
    );

/*******************************************************************************
**
**  gcoCL_Commit
**
**  Commit the current command buffer to hardware and optionally wait until the
**  hardware is finished.
**
**  INPUT:
**
**      gctBOOL Stall
**          gcvTRUE if the thread needs to wait until the hardware has finished
**          executing the committed command buffer.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gcoCL_Commit(
    IN gctBOOL Stall
    );

gceSTATUS
gcoCL_Flush(
    IN gctBOOL      Stall
    );

gceSTATUS
gcoCL_FlushHWCache();
/*******************************************************************************
**
**  gcoCL_CreateSignal
**
**  Create a new signal.
**
**  INPUT:
**
**      gctBOOL ManualReset
**          If set to gcvTRUE, gcoOS_Signal with gcvFALSE must be called in
**          order to set the signal to nonsignaled state.
**          If set to gcvFALSE, the signal will automatically be set to
**          nonsignaled state by gcoOS_WaitSignal function.
**
**  OUTPUT:
**
**      gctSIGNAL * Signal
**          Pointer to a variable receiving the created gctSIGNAL.
*/
gceSTATUS
gcoCL_CreateSignal(
    IN gctBOOL ManualReset,
    OUT gctSIGNAL * Signal
    );

/*******************************************************************************
**
**  gcoCL_DestroySignal
**
**  Destroy a signal.
**
**  INPUT:
**
**      gctSIGNAL Signal
**          Pointer to the gctSIGNAL.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gcoCL_DestroySignal(
    IN gctSIGNAL Signal
    );

gceSTATUS
gcoCL_SubmitSignal(
    IN gctSIGNAL    Signal,
    IN gctHANDLE    Process
    );

/*******************************************************************************
**
**  gcoCL_WaitSignal
**
**  Wait for a signal to become signaled.
**
**  INPUT:
**
**      gctSIGNAL Signal
**          Pointer to the gctSIGNAL.
**
**      gctUINT32 Wait
**          Number of milliseconds to wait.
**          Pass the value of gcvINFINITE for an infinite wait.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gcoCL_WaitSignal(
    IN gctSIGNAL Signal,
    IN gctUINT32 Wait
    );

/*******************************************************************************
**                                gcoCL_LoadKernel
********************************************************************************
**
**  Load a pre-compiled and pre-linked kernel program into the hardware.
**
**  INPUT:
**
**      gctSIZE_T StateBufferSize
**          The number of bytes in the 'StateBuffer'.
**
**      gctPOINTER StateBuffer
**          Pointer to the states that make up the shader program.
**
**      gcsHINT_PTR Hints
**          Pointer to a gcsHINT structure that contains information required
**          when loading the shader states.
*/
gceSTATUS
gcoCL_LoadKernel(
    IN gctSIZE_T StateBufferSize,
    IN gctPOINTER StateBuffer,
    IN gcsHINT_PTR Hints
    );

gceSTATUS
gcoCL_InvokeThreadWalker(
    IN gcsTHREAD_WALKER_INFO_PTR Info,
    IN gctBOOL Dirty
    );

#ifdef __cplusplus
}
#endif

#endif /* __gc_hal_user_cl_h_ */
