#ifndef __MMPX_DT_H
#define __MMPX_DT_H

extern void mmp_clk_of_init(void);
extern void __init pxa1908_reserve_sec_debug_mem(void);
extern void __init pxa1908_reserve_seclogmem(void);
extern void __init pxa1908_reserve_fbmem(void);
extern void __init pxa1908_reserve_sec_ftrace_mem(void);
#endif
