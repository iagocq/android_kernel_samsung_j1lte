#ifndef __MACH_CLK_PLL_HELANX_H
#define __MACH_CLK_PLL_HELANX_H

/*
 * struct kvco_range -store kvco and vrng for different frequency range
 * @vco_min:	min frequency of vco
 * @vco_max:	max frequency of vco
 * @kvco:	kvco val for relevant frequency range
 * @vrng:	vrng val for relevant frequency range
 */
struct kvco_range {
	int vco_min;
	int vco_max;
	u8 kvco;
	u8 vrng;
};

struct div_map {
	unsigned int div;
	unsigned int hw_val;
};

struct intpi_range {
	int vco_min;
	int vco_max;
	u8 value;
};

enum ssc_mode {
	CENTER_SPREAD = 0x0,
	DOWN_SPREAD = 0x1,
};

struct ssc_params {
	enum ssc_mode ssc_mode;
	int base;
	unsigned int amplitude;
	int desired_mod_freq;

	void __iomem *ssc_ctrl;
	void __iomem *ssc_cfg;
};

struct mmp_vco_freq_table {
	unsigned long output_rate;
	u16 refd;
	u16 fbd;
	u8 kvco;
	u8 vcovnrg;
};

struct mmp_vco_params {
	unsigned long vco_min;
	unsigned long vco_max;
	void __iomem *cr_reg;
	void __iomem *pll_swcr;
	void __iomem *lock_reg;
	u32 lock_enable_bit;
	unsigned long default_rate;

	struct mmp_vco_freq_table *freq_table;
	int freq_table_size;

	bool ssc_enabled;
	struct ssc_params *ssc_params;
};

struct mmp_pll_params {
	void __iomem *pll_swcr;
	unsigned long default_rate;
};

struct clk_vco {
	struct clk_hw hw;
	spinlock_t *lock;
	u32 flags;
	struct mmp_vco_params *params;
};

struct clk_pll {
	struct clk_hw hw;
	const char *parent;
	spinlock_t *lock;
	u32 flags;
	struct mmp_pll_params *params;
};

#define to_clk_vco(vco_hw) container_of(vco_hw, struct clk_vco, hw)
#define to_clk_pll(pll_hw) container_of(pll_hw, struct clk_pll, hw)

/**
 * VCO Flags:
 */
#define HELANX_PLL2CR_V1		BIT(0)	/* only for old pll2 config */
#define HELANX_PLL_SSC_FEAT		BIT(1)
#define HELANX_PLL_SSC_AON		BIT(2)
#define HELANX_PLL_28NM			BIT(3)
#define HELANX_PLL_40NM			BIT(4)

extern struct clk *helanx_clk_register_vco(const char *name,
					   const char *parent_name,
					   unsigned long flags, u32 vco_flags,
					   spinlock_t *lock,
					   struct mmp_vco_params *params);

/**
 * PLL Flags:
 */

#define HELANX_PLLOUT		BIT(0)	/* identify PLL and PLLP */
#define HELANX_PLLOUTP		BIT(1)

extern struct clk *helanx_clk_register_pll(const char *name,
					   const char *parent_name,
					   unsigned long flags, u32 vco_flags,
					   spinlock_t *lock,
					   struct mmp_pll_params *params);

#endif
