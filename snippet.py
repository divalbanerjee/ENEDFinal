diff --git a/board/lego/ev3/legoev3.c b/board/lego/ev3/legoev3.c
index cbd579a..73363b99 100644
--- a/board/lego/ev3/legoev3.c
+++ b/board/lego/ev3/legoev3.c
@@ -24,6 +24,8 @@
 #include <hwconfig.h>
 #include <asm/mach-types.h>
 #include <asm/setup.h>
+#include <mach/da850_lowlevel.h>
+#include <mach/pll_defs.h>

 #ifdef CONFIG_MMC_DAVINCI
 #include <mmc.h>
@@ -58,6 +60,114 @@ const struct pinmux_resource pinmuxes[] = {

 const int pinmuxes_size = ARRAY_SIZE(pinmuxes);

+/*
+ * PLL configuration
+ */
+#define CONFIG_SYS_DV_CLKMODE 0
+#define CONFIG_SYS_DA850_PLL0_POSTDIV 1
+
+static void da850_waitloop(unsigned long loopcnt)
+{
+ unsigned long i;
+
+ for (i = 0; i < loopcnt; i++)
+ asm(" NOP");
+}
+
+static int da850_pll_init(unsigned long pllmult)
+{
+ /* Unlock PLL registers. */
+ clrbits_le32(&davinci_syscfg_regs->cfgchip0, PLL_MASTER_LOCK);
+
+ /*
+ * Set PLLENSRC '0',bit 5, PLL Enable(PLLEN) selection is controlled
+ * through MMR
+ */
+ clrbits_le32(&davinci_pllc0_regs->pllctl, PLLCTL_PLLENSRC);
+ /* PLLCTL.EXTCLKSRC bit 9 should be left at 0 for Freon */
+ clrbits_le32(&davinci_pllc0_regs->pllctl, PLLCTL_EXTCLKSRC);
+
+ /* Set PLLEN=0 => PLL BYPASS MODE */
+ clrbits_le32(&davinci_pllc0_regs->pllctl, PLLCTL_PLLEN);
+
+ da850_waitloop(150);
+
+ /*
+ * Select the Clock Mode bit 8 as External Clock or On Chip
+ * Oscilator
+ */
+ dv_maskbits(&davinci_pllc0_regs->pllctl, ~PLLCTL_RES_9);
+ setbits_le32(&davinci_pllc0_regs->pllctl,
+ (CONFIG_SYS_DV_CLKMODE << PLLCTL_CLOCK_MODE_SHIFT));
+
+ /* Clear PLLRST bit to reset the PLL */
+ clrbits_le32(&davinci_pllc0_regs->pllctl, PLLCTL_PLLRST);
+
+ /* Disable the PLL output */
+ setbits_le32(&davinci_pllc0_regs->pllctl, PLLCTL_PLLDIS);
+
+ /* PLL initialization sequence */
+ /*
+ * Power up the PLL- PWRDN bit set to 0 to bring the PLL out of
+ * power down bit
+ */
+ clrbits_le32(&davinci_pllc0_regs->pllctl, PLLCTL_PLLPWRDN);
+
+ /* Enable the PLL from Disable Mode PLLDIS bit to 0 */
+ clrbits_le32(&davinci_pllc0_regs->pllctl, PLLCTL_PLLDIS);
+
+ /* Program the required multiplier value in PLLM */
+ writel(pllmult, &davinci_pllc0_regs->pllm);
+
+ /* program the postdiv */
+ writel((PLL_POSTDEN | CONFIG_SYS_DA850_PLL0_POSTDIV),
+ &davinci_pllc0_regs->postdiv);
+
+ /*
+ * Check for the GOSTAT bit in PLLSTAT to clear to 0 to indicate that
+ * no GO operation is currently in progress
+ */
+ while ((readl(&davinci_pllc0_regs->pllstat) & PLLCMD_GOSTAT) == PLLCMD_GOSTAT)
+ continue;
+
+ /*
+ * Set the GOSET bit in PLLCMD to 1 to initiate a new divider
+ * transition.
+ */
+ setbits_le32(&davinci_pllc0_regs->pllcmd, PLLCMD_GOSTAT);
+
+ /*
+ * Wait for the GOSTAT bit in PLLSTAT to clear to 0
+ * (completion of phase alignment).
+ */
+ while ((readl(&davinci_pllc0_regs->pllstat) & PLLCMD_GOSTAT) == PLLCMD_GOSTAT)
+ continue;
+
+ /* Wait for PLL to reset properly. See PLL spec for PLL reset time */
+ da850_waitloop(200);
+
+ /* Set the PLLRST bit in PLLCTL to 1 to bring the PLL out of reset */
+ setbits_le32(&davinci_pllc0_regs->pllctl, PLLCTL_PLLRST);
+
+ /* Wait for PLL to lock. See PLL spec for PLL lock time */
+ da850_waitloop(2400);
+
+ /*
+ * Set the PLLEN bit in PLLCTL to 1 to remove the PLL from bypass
+ * mode
+ */
+ setbits_le32(&davinci_pllc0_regs->pllctl, PLLCTL_PLLEN);
+
+ /*
+ * clear EMIFA and EMIFB clock source settings, let them
+ * run off SYSCLK
+ */
+ dv_maskbits(&davinci_syscfg_regs->cfgchip3,
+ ~(PLL_SCSCFG3_DIV45PENA | PLL_SCSCFG3_EMA_CLKSRC));
+
+ return 0;
+}
+
 const struct lpsc_resource lpsc[] = {
 { DAVINCI_LPSC_SPI0 }, /* Serial Flash */
 { DAVINCI_LPSC_UART1 }, /* console */
@@ -68,6 +178,8 @@ const int lpsc_size = ARRAY_SIZE(lpsc);

 int board_early_init_f(void)
 {
+ da850_pll_init(30);
+
 /* enable the console UART */
 writel((DAVINCI_UART_PWREMU_MGMT_FREE | DAVINCI_UART_PWREMU_MGMT_URRST |
 DAVINCI_UART_PWREMU_MGMT_UTRST),
