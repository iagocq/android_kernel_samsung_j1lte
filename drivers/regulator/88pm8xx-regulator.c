#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/88pm80x.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/regulator/of_regulator.h>
#include "88pm8xx-regulator.h"

/* Ranges are sorted in ascending order. */
/* 88pm800/88pm822 buck1, 0.6V->1.5875V; 1.6V->1.8V */
const struct regulator_linear_range buck_volt_range1[] = {
	REGULATOR_LINEAR_RANGE(600000, 0, 0x4f, 12500),
	REGULATOR_LINEAR_RANGE(1600000, 0x50, 0x54, 50000),
};

/* BUCK 2~4 have same ranges. */
/* 88pm800 buck2~5 and 88pm822 buck2~4, 0.6V->1.5875V; 1.6V->3.3V */
const struct regulator_linear_range buck_volt_range2[] = {
	REGULATOR_LINEAR_RANGE(600000, 0, 0x4f, 12500),
	REGULATOR_LINEAR_RANGE(1600000, 0x50, 0x72, 50000),
};

/* 88pm822 buck5: 0.6V->1.5875V; 1.6V->3.95V */
const struct regulator_linear_range buck_volt_range3[] = {
	REGULATOR_LINEAR_RANGE(600000, 0, 0x4f, 12500),
	REGULATOR_LINEAR_RANGE(1600000, 0x50, 0x80, 50000),
};

/* 88pm800 ldo1; 88pm86x ldo19 */
const unsigned int ldo_volt_table1[] = {
	600000,  650000,  700000,  750000,  800000,  850000,  900000,  950000,
	1000000, 1050000, 1100000, 1150000, 1200000, 1300000, 1400000, 1500000,
};

/* 88pm800 ldo2; 88pm86x ldo20 */
const unsigned int ldo_volt_table2[] = {
	1700000, 1800000, 1900000, 2000000, 2100000, 2500000, 2700000, 2800000,
};

/* 88pm800 ldo 3~17*/
const unsigned int ldo_volt_table3[] = {
	1200000, 1250000, 1700000, 1800000, 1850000, 1900000, 2500000, 2600000,
	2700000, 2750000, 2800000, 2850000, 2900000, 3000000, 3100000, 3300000,
};

/* 88pm800 18~19 */
const unsigned int ldo_volt_table4[] = {
	1700000, 1800000, 1900000, 2500000, 2800000, 2900000, 3100000, 3300000,
};

/* 88pm822 ldo1 and ldo2; 88pm86x ldo 1~3*/
const unsigned int ldo_volt_table5[] = {
	1700000, 1800000, 1900000, 2500000, 2800000, 2900000, 3100000, 3300000,
};

/* 88pm822 ldo 3~11; 88pm86x ldo 4~18*/
const unsigned int ldo_volt_table6[] = {
	1200000, 1250000, 1700000, 1800000, 1850000, 1900000, 2500000, 2600000,
	2700000, 2750000, 2800000, 2850000, 2900000, 3000000, 3100000, 3300000,
};

/* 88pm822 ldo12*/
const unsigned int ldo_volt_table7[] = {
	600000,  650000,  700000,  750000,  800000,  850000,  900000,  950000,
	1000000, 1050000, 1100000, 1150000, 1200000, 1300000, 1400000, 1500000,
};

/* 88pm822 ldo13 */
const unsigned int ldo_volt_table8[] = {
	1700000, 1800000, 1900000, 2500000, 2800000, 2900000, 3100000, 3300000,
};

/* 88pm822 ldo14 */
const unsigned int ldo_volt_table9[] = {
	1700000, 1800000, 1900000, 2000000, 2100000, 2500000, 2700000, 2800000,
};

const unsigned int voutsw_table[] = {
};

/* The array is indexed by id(PM800_ID_XXX) */
struct pm800_regulator_info pm800_regulator_info[] = {
	PM800_BUCK(BUCK1, BUCK_ENA, 0, 3000000, 0, BUCK_SLP1, buck_volt_range1, 0x55),
	PM800_BUCK(BUCK2, BUCK_ENA, 1, 1200000, 2, BUCK_SLP1, buck_volt_range2, 0x73),
	PM800_BUCK(BUCK3, BUCK_ENA, 2, 1200000, 4, BUCK_SLP1, buck_volt_range2, 0x73),
	PM800_BUCK(BUCK4, BUCK_ENA, 3, 1200000, 6, BUCK_SLP1, buck_volt_range2, 0x73),
	PM800_BUCK(BUCK5, BUCK_ENA, 4, 1200000, 0, BUCK_SLP2, buck_volt_range2, 0x73),

	PM800_LDO(LDO1, LDO_ENA1_1, 0, 200000, 0, LDO_SLP1, ldo_volt_table1),
	PM800_LDO(LDO2, LDO_ENA1_1, 1, 10000, 2, LDO_SLP1, ldo_volt_table2),
	PM800_LDO(LDO3, LDO_ENA1_1, 2, 300000, 4, LDO_SLP1, ldo_volt_table3),
	PM800_LDO(LDO4, LDO_ENA1_1, 3, 300000, 6, LDO_SLP1, ldo_volt_table3),
	PM800_LDO(LDO5, LDO_ENA1_1, 4, 300000, 0, LDO_SLP2, ldo_volt_table3),
	PM800_LDO(LDO6, LDO_ENA1_1, 5, 300000, 2, LDO_SLP2, ldo_volt_table3),
	PM800_LDO(LDO7, LDO_ENA1_1, 6, 300000, 4, LDO_SLP2, ldo_volt_table3),
	PM800_LDO(LDO8, LDO_ENA1_1, 7, 300000, 6, LDO_SLP2, ldo_volt_table3),
	PM800_LDO(LDO9, LDO_ENA1_2, 0, 300000, 0, LDO_SLP3, ldo_volt_table3),
	PM800_LDO(LDO10, LDO_ENA1_2, 1, 300000, 2, LDO_SLP3, ldo_volt_table3),
	PM800_LDO(LDO11, LDO_ENA1_2, 2, 300000, 4, LDO_SLP3, ldo_volt_table3),
	PM800_LDO(LDO12, LDO_ENA1_2, 3, 300000, 6, LDO_SLP3, ldo_volt_table3),
	PM800_LDO(LDO13, LDO_ENA1_2, 4, 300000, 0, LDO_SLP4, ldo_volt_table3),
	PM800_LDO(LDO14, LDO_ENA1_2, 5, 300000, 2, LDO_SLP4, ldo_volt_table3),
	PM800_LDO(LDO15, LDO_ENA1_2, 6, 300000, 4, LDO_SLP4, ldo_volt_table3),
	PM800_LDO(LDO16, LDO_ENA1_2, 7, 300000, 6, LDO_SLP4, ldo_volt_table3),
	PM800_LDO(LDO17, LDO_ENA1_3, 0, 300000, 0, LDO_SLP5, ldo_volt_table3),
	PM800_LDO(LDO18, LDO_ENA1_3, 1, 200000, 2, LDO_SLP5, ldo_volt_table4),
	PM800_LDO(LDO19, LDO_ENA1_3, 2, 200000, 4, LDO_SLP5, ldo_volt_table4),
};

struct pm800_regulator_info pm822_regulator_info[] = {
	PM800_BUCK(BUCK1, BUCK_ENA, 0, 3500000, 0, BUCK_SLP1, buck_volt_range1, 0x55),
	PM800_BUCK(BUCK2, BUCK_ENA, 1, 750000, 2, BUCK_SLP1, buck_volt_range2, 0x73),
	PM800_BUCK(BUCK3, BUCK_ENA, 2, 1500000, 4, BUCK_SLP1, buck_volt_range2, 0x73),
	PM800_BUCK(BUCK4, BUCK_ENA, 3, 750000, 6, BUCK_SLP1, buck_volt_range2, 0x73),
	PM800_BUCK(BUCK5, BUCK_ENA, 4, 1500000, 0, BUCK_SLP2, buck_volt_range3, 0x81),
	PM800_BUCK(BUCK6, BUCK_ENA, 5, 1500000, 2, BUCK_SLP2, buck_volt_range2, 0x73),
	PM800_BUCK(BUCK1B, BUCK_ENA, 6, 3500000, 4, BUCK_SLP2, buck_volt_range2, 0x73),

	PM800_LDO(LDO1, LDO_ENA1_1, 0, 100000, 0, LDO_SLP1, ldo_volt_table5),
	PM800_LDO(LDO2, LDO_ENA1_1, 1, 100000, 2, LDO_SLP1, ldo_volt_table5),
	PM800_LDO(LDO3, LDO_ENA1_1, 2, 400000, 4, LDO_SLP1, ldo_volt_table6),
	PM800_LDO(LDO4, LDO_ENA1_1, 3, 400000, 6, LDO_SLP1, ldo_volt_table6),
	PM800_LDO(LDO5, LDO_ENA1_1, 4, 200000, 0, LDO_SLP2, ldo_volt_table6),
	PM800_LDO(LDO6, LDO_ENA1_1, 5, 200000, 2, LDO_SLP2, ldo_volt_table6),
	PM800_LDO(LDO7, LDO_ENA1_1, 6, 100000, 4, LDO_SLP2, ldo_volt_table6),
	PM800_LDO(LDO8, LDO_ENA1_1, 7, 100000, 6, LDO_SLP2, ldo_volt_table6),
	PM800_LDO(LDO9, LDO_ENA1_2, 0, 200000, 0, LDO_SLP3, ldo_volt_table6),
	PM800_LDO(LDO10, LDO_ENA1_2, 1, 400000, 2, LDO_SLP3, ldo_volt_table6),
	PM800_LDO(LDO11, LDO_ENA1_2, 2, 200000, 4, LDO_SLP3, ldo_volt_table6),
	PM800_LDO(LDO12, LDO_ENA1_2, 3, 400000, 6, LDO_SLP3, ldo_volt_table7),
	PM800_LDO(LDO13, LDO_ENA1_2, 4, 180000, 0, LDO_SLP4, ldo_volt_table8),
	PM800_LDO(LDO14, LDO_ENA1_2, 5, 8000, 2, LDO_SLP4, ldo_volt_table9),
	PM800_LDO(VOUTSW, MISC_EN1, 4, 0, 0, LDO_SLP4, voutsw_table),
};

struct pm800_regulator_info pm86x_regulator_info[] = {
	PM800_BUCK(BUCK1A, BUCK_ENA, 0, 3000000, 0, BUCK_SLP1, buck_volt_range1, 0x55),
	PM800_BUCK(BUCK2, BUCK_ENA, 1, 750000, 2, BUCK_SLP1, buck_volt_range2, 0x73),
	PM800_BUCK(BUCK3, BUCK_ENA, 2, 1500000, 4, BUCK_SLP1, buck_volt_range2, 0x73),
	PM800_BUCK(BUCK4, BUCK_ENA, 3, 750000, 6, BUCK_SLP1, buck_volt_range2, 0x73),
	PM800_BUCK(BUCK5, BUCK_ENA, 4, 1500000, 0, BUCK_SLP2, buck_volt_range3, 0x81),
	PM800_BUCK(BUCK6, BUCK_ENA, 5, 800000, 2, BUCK_SLP2, buck_volt_range2, 0x73),
	PM800_BUCK(BUCK1B, BUCK_ENA, 6, 3000000, 4, BUCK_SLP2, buck_volt_range2, 0x73),

	PM800_LDO(LDO1, LDO_ENA1_1, 0, 100000, 0, LDO_SLP1, ldo_volt_table5),
	PM800_LDO(LDO2, LDO_ENA1_1, 1, 100000, 2, LDO_SLP1, ldo_volt_table5),
	PM800_LDO(LDO3, LDO_ENA1_1, 2, 100000, 4, LDO_SLP1, ldo_volt_table5),
	PM800_LDO(LDO4, LDO_ENA1_1, 3, 400000, 6, LDO_SLP1, ldo_volt_table6),
	PM800_LDO(LDO5, LDO_ENA1_1, 4, 400000, 0, LDO_SLP2, ldo_volt_table6),
	PM800_LDO(LDO6, LDO_ENA1_1, 5, 400000, 2, LDO_SLP2, ldo_volt_table6),
	PM800_LDO(LDO7, LDO_ENA1_1, 6, 400000, 4, LDO_SLP2, ldo_volt_table6),
	PM800_LDO(LDO8, LDO_ENA1_1, 7, 400000, 6, LDO_SLP2, ldo_volt_table6),
	PM800_LDO(LDO9, LDO_ENA1_2, 0, 400000, 0, LDO_SLP3, ldo_volt_table6),
	PM800_LDO(LDO10, LDO_ENA1_2, 1, 200000, 2, LDO_SLP3, ldo_volt_table6),
	PM800_LDO(LDO11, LDO_ENA1_2, 2, 200000, 4, LDO_SLP3, ldo_volt_table6),
	PM800_LDO(LDO12, LDO_ENA1_2, 3, 200000, 6, LDO_SLP3, ldo_volt_table6),
	PM800_LDO(LDO13, LDO_ENA1_2, 4, 200000, 0, LDO_SLP4, ldo_volt_table6),
	PM800_LDO(LDO14, LDO_ENA1_2, 5, 200000, 2, LDO_SLP4, ldo_volt_table6),
	PM800_LDO(LDO15, LDO_ENA1_2, 6, 200000, 4, LDO_SLP4, ldo_volt_table6),
	PM800_LDO(LDO16, LDO_ENA1_2, 7, 200000, 6, LDO_SLP4, ldo_volt_table6),
	PM800_LDO(LDO17, LDO_ENA1_3, 0, 200000, 0, LDO_SLP5, ldo_volt_table6),
	PM800_LDO(LDO18, LDO_ENA1_3, 1, 200000, 2, LDO_SLP5, ldo_volt_table6),
	PM800_LDO(LDO19, LDO_ENA1_3, 2, 400000, 4, LDO_SLP5, ldo_volt_table1),
	PM800_LDO(LDO20, LDO_ENA1_3, 3, 10000, 6, LDO_SLP5, ldo_volt_table2),
};

struct of_regulator_match pm800_regulator_matches[] = {
	PM800_REGULATOR_OF_MATCH(BUCK1),
	PM800_REGULATOR_OF_MATCH(BUCK2),
	PM800_REGULATOR_OF_MATCH(BUCK3),
	PM800_REGULATOR_OF_MATCH(BUCK4),
	PM800_REGULATOR_OF_MATCH(BUCK5),
	PM800_REGULATOR_OF_MATCH(LDO1),
	PM800_REGULATOR_OF_MATCH(LDO2),
	PM800_REGULATOR_OF_MATCH(LDO3),
	PM800_REGULATOR_OF_MATCH(LDO4),
	PM800_REGULATOR_OF_MATCH(LDO5),
	PM800_REGULATOR_OF_MATCH(LDO6),
	PM800_REGULATOR_OF_MATCH(LDO7),
	PM800_REGULATOR_OF_MATCH(LDO8),
	PM800_REGULATOR_OF_MATCH(LDO9),
	PM800_REGULATOR_OF_MATCH(LDO10),
	PM800_REGULATOR_OF_MATCH(LDO11),
	PM800_REGULATOR_OF_MATCH(LDO12),
	PM800_REGULATOR_OF_MATCH(LDO13),
	PM800_REGULATOR_OF_MATCH(LDO14),
	PM800_REGULATOR_OF_MATCH(LDO15),
	PM800_REGULATOR_OF_MATCH(LDO16),
	PM800_REGULATOR_OF_MATCH(LDO17),
	PM800_REGULATOR_OF_MATCH(LDO18),
	PM800_REGULATOR_OF_MATCH(LDO19),
};

struct of_regulator_match pm822_regulator_matches[] = {
	PM822_REGULATOR_OF_MATCH(BUCK1),
	PM822_REGULATOR_OF_MATCH(BUCK2),
	PM822_REGULATOR_OF_MATCH(BUCK3),
	PM822_REGULATOR_OF_MATCH(BUCK4),
	PM822_REGULATOR_OF_MATCH(BUCK5),
	PM822_REGULATOR_OF_MATCH(LDO1),
	PM822_REGULATOR_OF_MATCH(LDO2),
	PM822_REGULATOR_OF_MATCH(LDO3),
	PM822_REGULATOR_OF_MATCH(LDO4),
	PM822_REGULATOR_OF_MATCH(LDO5),
	PM822_REGULATOR_OF_MATCH(LDO6),
	PM822_REGULATOR_OF_MATCH(LDO7),
	PM822_REGULATOR_OF_MATCH(LDO8),
	PM822_REGULATOR_OF_MATCH(LDO9),
	PM822_REGULATOR_OF_MATCH(LDO10),
	PM822_REGULATOR_OF_MATCH(LDO11),
	PM822_REGULATOR_OF_MATCH(LDO12),
	PM822_REGULATOR_OF_MATCH(LDO13),
	PM822_REGULATOR_OF_MATCH(LDO14),
	PM822_REGULATOR_OF_MATCH(VOUTSW),
};

struct of_regulator_match pm86x_regulator_matches[] = {
	PM86X_REGULATOR_OF_MATCH(BUCK1A),
	PM86X_REGULATOR_OF_MATCH(BUCK2),
	PM86X_REGULATOR_OF_MATCH(BUCK3),
	PM86X_REGULATOR_OF_MATCH(BUCK4),
	PM86X_REGULATOR_OF_MATCH(BUCK5),
	PM86X_REGULATOR_OF_MATCH(BUCK6),
	PM86X_REGULATOR_OF_MATCH(BUCK1B),
	PM86X_REGULATOR_OF_MATCH(LDO1),
	PM86X_REGULATOR_OF_MATCH(LDO2),
	PM86X_REGULATOR_OF_MATCH(LDO3),
	PM86X_REGULATOR_OF_MATCH(LDO4),
	PM86X_REGULATOR_OF_MATCH(LDO5),
	PM86X_REGULATOR_OF_MATCH(LDO6),
	PM86X_REGULATOR_OF_MATCH(LDO7),
	PM86X_REGULATOR_OF_MATCH(LDO8),
	PM86X_REGULATOR_OF_MATCH(LDO9),
	PM86X_REGULATOR_OF_MATCH(LDO10),
	PM86X_REGULATOR_OF_MATCH(LDO11),
	PM86X_REGULATOR_OF_MATCH(LDO12),
	PM86X_REGULATOR_OF_MATCH(LDO13),
	PM86X_REGULATOR_OF_MATCH(LDO14),
	PM86X_REGULATOR_OF_MATCH(LDO15),
	PM86X_REGULATOR_OF_MATCH(LDO16),
	PM86X_REGULATOR_OF_MATCH(LDO17),
	PM86X_REGULATOR_OF_MATCH(LDO18),
	PM86X_REGULATOR_OF_MATCH(LDO19),
	PM86X_REGULATOR_OF_MATCH(LDO20),
};