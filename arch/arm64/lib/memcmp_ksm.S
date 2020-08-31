/* 2014-11-25 woosik.youm@samsung.com */

#include <linux/linkage.h>
#include <linux/const.h>
#include <asm/assembler.h>
#include <asm/page.h>

#define CACHE_LINE_SIZE 64

#define src1		x0
#define src2		x1
#define limit		x2
#define temp		x3
#define	data1		x4
#define data2		x5
#define result		x0

#define vdata1		v0
#define vdata2		v1
#define vdata3		v2
#define vdata4		v3
#define vresult		v4


ENTRY(page_memcmp)

.loop: // cache line size align
	prfm	pldl1strm, [src1, #(CACHE_LINE_SIZE * 1)]
	prfm	pldl1strm, [src2, #(CACHE_LINE_SIZE * 1)]

	ld1	{vdata1.2d, vdata2.2d}, [src1], #32
	ld1	{vdata3.2d, vdata4.2d}, [src2], #32
	
	sub	vresult.2d, vdata1.2d, vdata3.2d
	mov	temp, vresult.2d[0]
	cbnz	temp, .end
	mov	temp, vresult.2d[1]
	cbnz	temp, .end	

	sub	vresult.2d, vdata2.2d, vdata4.2d
	mov	temp, vresult.2d[0]
	cbnz	temp, .end
	mov	temp, vresult.2d[1]
	cbnz	temp, .end	

	ld1	{vdata1.2d, vdata2.2d}, [src1], #32
	ld1	{vdata3.2d, vdata4.2d}, [src2], #32
	
	sub	vresult.2d, vdata1.2d, vdata3.2d
	mov	temp, vresult.2d[0]
	cbnz	temp, .end
	mov	temp, vresult.2d[1]
	cbnz	temp, .end	

	sub	vresult.2d, vdata2.2d, vdata4.2d
	mov	temp, vresult.2d[0]
	cbnz	temp, .end
	mov	temp, vresult.2d[1]
	cbnz	temp, .end	
	
	sub	limit, limit, #64
	cbnz	limit, .loop
	
	mov	result, #0
	ret	

.end : 
	tst	temp, #1<<63
	csneg	result, limit, limit, ne
	ret

END(page_memcmp)





