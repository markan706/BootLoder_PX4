/*
 * STM32F7 board support for the bootloader.
 *
 */

#include "hw_config.h"

#include <stdlib.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/pwr.h>

#include "bl.h"
#include "uart.h"

#if !defined(USART6)
#  define USART6    USART6_BASE
#endif
#if !defined(UART6)
#  define UART7    UART7_BASE
#endif
#if !defined(USART8)
#  define UART8    UART8_BASE
#endif

/* flash parameters that we should not really know */
static struct {
	uint32_t	sector_number;
	uint32_t	size;
} flash_sectors[] = {

	/* Physical FLASH sector 0 is reserved for bootloader and is not
	 * the table below.
	 * N sectors may aslo be reserved for the app fw in which case
	 * the zero based define BOARD_FIRST_FLASH_SECTOR_TO_ERASE must
	 * be defined to begin the erase above of the reserved sectors.
	 * The default value of BOARD_FIRST_FLASH_SECTOR_TO_ERASE is 0
	 * and begins flash erase operations at phsical sector 1 the 0th entry
	 * in the table below.
	 * A value of 1 for BOARD_FIRST_FLASH_SECTOR_TO_ERASE would reserve
	 * the 0th entry and begin erasing a index 1 the third physical sector
	 * on the device.
	 *
	 * When BOARD_FIRST_FLASH_SECTOR_TO_ERASE is defined APP_RESERVATION_SIZE
	 * must also be defined to remove that additonal reserved FLASH space
	 * from the BOARD_FLASH_SIZE. See APP_SIZE_MAX below.
	 */

	{0x01, 32 * 1024},
	{0x02, 32 * 1024},
	{0x03, 32 * 1024},
	{0x04, 128 * 1024},
	{0x05, 256 * 1024},
	{0x06, 256 * 1024},
	{0x07, 256 * 1024},
	{0x08, 256 * 1024},
	{0x09, 256 * 1024},
	{0x0a, 256 * 1024},
	{0x0b, 256 * 1024},
};
#define BOOTLOADER_RESERVATION_SIZE	(32 * 1024)

#define OTP_BASE			0x1ff0f000
#define OTP_SIZE			1024
#define UDID_START			0x1ff0f420

// address of MCU IDCODE
#define DBGMCU_IDCODE		0xE0042000
#define STM32_UNKNOWN	0
#define STM32F74x_75x	0x449
#define STM32F76x_77x	0x451

#define REVID_MASK	0xFFFF0000
#define DEVID_MASK	0xFFF

/* magic numbers from reference manual */

typedef enum mcu_rev_e {
	MCU_REV_STM32F7_REV_A = 0x1000,
	MCU_REV_STM32F7_REV_Z = 0x1001,
} mcu_rev_e;

typedef struct mcu_des_t {
	uint16_t mcuid;
	const char *desc;
	char  rev;
} mcu_des_t;

// The default CPU ID  of STM32_UNKNOWN is 0 and is in offset 0
// Before a rev is known it is set to ?
// There for new silicon will result in STM32F4..,?
mcu_des_t mcu_descriptions[] = {
	{ STM32_UNKNOWN,	"STM32F??????",		'?'},
	{ STM32F74x_75x, 	"STM32F7[4|5]x",	'?'},
	{ STM32F76x_77x, 	"STM32F7[6|7]x",	'?'},
};

typedef struct mcu_rev_t {
	mcu_rev_e revid;
	char  rev;
} mcu_rev_t;

/*
 * This table is used in 2 ways. One to look look up the revision
 * of a given chip. Two to see it a revsion is in the group of "Bad"
 * silicon.
 *
 * Therefore when adding entries for good silicon rev, they must be inserted
 * at the beginning of the table. The value of FIRST_BAD_SILICON_OFFSET will
 * also need to be increased to that of the value of the first bad silicon offset.
 *
 */
const mcu_rev_t silicon_revs[] = {
	{MCU_REV_STM32F7_REV_A, 'A'}, /* Revision A */
	{MCU_REV_STM32F7_REV_Z, 'Z'}, /* Revision Z */
};

#define APP_SIZE_MAX			(BOARD_FLASH_SIZE - (BOOTLOADER_RESERVATION_SIZE + APP_RESERVATION_SIZE))

/* context passed to cinit */
#if INTERFACE_USART
# define BOARD_INTERFACE_CONFIG_USART	(void *)BOARD_USART
#endif
#if INTERFACE_USB
# define BOARD_INTERFACE_CONFIG_USB  	NULL
#endif

/* board definition */
struct boardinfo board_info = {
	.board_type	= BOARD_TYPE,
	.board_rev	= 0,
	.fw_size	= 0,

	.systick_mhz	= 168,
};

static void board_init(void);

#define BOOT_RTC_SIGNATURE          0xb007b007
#define POWER_DOWN_RTC_SIGNATURE    0xdeaddead // Written by app fw to not re-power on.
#define BOOT_RTC_REG                MMIO32(RTC_BASE + 0x50)

/* standard clocking for all F7 boards: 216MHz w/ overdrive
 *
 * f_voc = f_osc * (PLLN / PLLM) = OSC_FREQ * PLLN / OSC_FREQ = PLLN = 432
 * f_pll = f_voc / PLLP = PLLN / PLLP = 216
 * f_usb_sdmmc = f_voc / PLLQ = 48
 */
static const struct rcc_clock_scale clock_setup = {
	/* 216MHz */
	.plln = 432,
	.pllp = 2,
	.pllq = 9,
	.flash_waitstates = 7,
	.hpre = RCC_CFGR_HPRE_DIV_NONE,
	.ppre1 = RCC_CFGR_PPRE_DIV_4,
	.ppre2 = RCC_CFGR_PPRE_DIV_2,
	.vos_scale = PWR_SCALE1, /** <= 180MHz w/o overdrive, <= 216MHz w/ overdrive */
	.overdrive = 1,
	.apb1_frequency = 54000000,
	.apb2_frequency = 108000000,
};

static uint32_t
board_get_rtc_signature()
{
	// bymark Real-time clock (RTC), 使能RTC备份寄存器
	/* enable the backup registers */
	PWR_CR1 |= PWR_CR1_DBP;
	RCC_BDCR |= RCC_BDCR_RTCEN;

	uint32_t result = BOOT_RTC_REG;  // bymark 获取RTC备份寄存器的值

	/* disable the backup registers */
	RCC_BDCR &= RCC_BDCR_RTCEN;
	PWR_CR1 &= ~PWR_CR1_DBP;

	return result;
}

static void
board_set_rtc_signature(uint32_t sig)
{
	/* enable the backup registers */
	PWR_CR1 |= PWR_CR1_DBP;
	RCC_BDCR |= RCC_BDCR_RTCEN;

	BOOT_RTC_REG = sig;

	/* disable the backup registers */
	RCC_BDCR &= RCC_BDCR_RTCEN;
	PWR_CR1 &= ~PWR_CR1_DBP;
}

static bool
board_test_force_pin()
{
#if defined(BOARD_FORCE_BL_PIN_IN) && defined(BOARD_FORCE_BL_PIN_OUT)
	/* two pins strapped together */
	volatile unsigned samples = 0;
	volatile unsigned vote = 0;

	for (volatile unsigned cycles = 0; cycles < 10; cycles++) {
		gpio_set(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN_OUT);

		for (unsigned count = 0; count < 20; count++) {
			if (gpio_get(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN_IN) != 0) {
				vote++;
			}

			samples++;
		}

		gpio_clear(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN_OUT);

		for (unsigned count = 0; count < 20; count++) {
			if (gpio_get(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN_IN) == 0) {
				vote++;
			}

			samples++;
		}
	}

	/* the idea here is to reject wire-to-wire coupling, so require > 90% agreement */
	if ((vote * 100) > (samples * 90)) {
		return true;
	}

#endif
#if defined(BOARD_FORCE_BL_PIN)
	/* single pin pulled up or down */
	volatile unsigned samples = 0;
	volatile unsigned vote = 0;

	for (samples = 0; samples < 200; samples++) {
		if ((gpio_get(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN) ? 1 : 0) == BOARD_FORCE_BL_STATE) { // bymark 强制bootloader引脚为高
			vote++;
		}
	}

	/* reject a little noise */
	if ((vote * 100) > (samples * 90)) {
		return true;
	}

#endif
	return false;
}

#if INTERFACE_USART
static bool
board_test_usart_receiving_break()
{
	 // bymark PX4_FMU_V5是定义了SERIAL_BREAK_DETECT_DISABLED为1，即关闭串口检测break_booting的功能
#if !defined(SERIAL_BREAK_DETECT_DISABLED)
	/* (re)start the SysTick timer system */
	systick_interrupt_disable(); // Kill the interrupt if it is still active
	systick_counter_disable(); // Stop the timer
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);

	/* Set the timer period to be half the bit rate
	 *
	 * Baud rate = 115200, therefore bit period = 8.68us
	 * Half the bit rate = 4.34us
	 * Set period to 4.34 microseconds (timer_period = timer_tick / timer_reset_frequency = 168MHz / (1/4.34us) = 729.12 ~= 729)
	 */
	systick_set_reload(729);  /* 4.3us tick, magic number */
	systick_counter_enable(); // Start the timer

	uint8_t cnt_consecutive_low = 0;
	uint8_t cnt = 0;

	/* Loop for 3 transmission byte cycles and count the low and high bits. Sampled at a rate to be able to count each bit twice.
	 *
	 * One transmission byte is 10 bits (8 bytes of data + 1 start bit + 1 stop bit)
	 * We sample at every half bit time, therefore 20 samples per transmission byte,
	 * therefore 60 samples for 3 transmission bytes
	 */
	while (cnt < 60) {
		// Only read pin when SysTick timer is true
		if (systick_get_countflag() == 1) {
			if (gpio_get(BOARD_PORT_USART, BOARD_PIN_RX) == 0) {
				cnt_consecutive_low++;	// Increment the consecutive low counter

			} else {
				cnt_consecutive_low = 0; // Reset the consecutive low counter
			}

			cnt++;
		}

		// If 9 consecutive low bits were received break out of the loop
		if (cnt_consecutive_low >= 18) {
			break;
		}

	}

	systick_counter_disable(); // Stop the timer

	/*
	 * If a break is detected, return true, else false
	 *
	 * Break is detected if line was low for 9 consecutive bits.
	 */
	if (cnt_consecutive_low >= 18) {
		return true;
	}

#endif // !defined(SERIAL_BREAK_DETECT_DISABLED)

	return false;
}
#endif

static void
board_init(void)
{
	/* fix up the max firmware size, we have to read memory to get this */
	board_info.fw_size = APP_SIZE_MAX; // bymark 设置固件大小的最大值 

// bymark 默认情况下PX4_FMU_V5 没有定义BOARD_POWER_PIN_OUT
#if defined(BOARD_POWER_PIN_OUT)
	/* Configure the Power pins */
	rcc_peripheral_enable_clock(&BOARD_POWER_CLOCK_REGISTER, BOARD_POWER_CLOCK_BIT);
	gpio_mode_setup(BOARD_POWER_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, BOARD_POWER_PIN_OUT);
	gpio_set_output_options(BOARD_POWER_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, BOARD_POWER_PIN_OUT);
	BOARD_POWER_ON(BOARD_POWER_PORT, BOARD_POWER_PIN_OUT);
#endif

// bymark 采用usb通信接口 VBUS线是HOST/HUB向USB设备供电的电源线, 即平常USB设备的+5V
#if INTERFACE_USB
	/* enable Port A GPIO9 to sample VBUS */
	// bymark 端口A的时钟使能
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_GPIOAEN);
#  if defined(USE_VBUS_PULL_DOWN)
	// bymark 对GPIOA9进行引脚属性配置
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO9);
#  endif
#endif

// bymark 采用串口作为通信接口
#if INTERFACE_USART
	/* configure USART pins */
	// bymark 串口使用了Port D下面的Pin, 所以使能端口D的时钟
	rcc_peripheral_enable_clock(&BOARD_USART_PIN_CLOCK_REGISTER, BOARD_USART_PIN_CLOCK_BIT);

	/* Setup GPIO pins for USART transmit. */
	// bymark 配置PD5和PD6的引脚模式属性
	gpio_mode_setup(BOARD_PORT_USART, GPIO_MODE_AF, GPIO_PUPD_PULLUP, BOARD_PIN_TX | BOARD_PIN_RX);
	/* Setup USART TX & RX pins as alternate function. */
	// bymark 设置引脚PD5和PD6的复用功能
	gpio_set_af(BOARD_PORT_USART, BOARD_PORT_USART_AF, BOARD_PIN_TX);
	gpio_set_af(BOARD_PORT_USART, BOARD_PORT_USART_AF, BOARD_PIN_RX);

	/* configure USART clock */
	// bymark 使能串口的时钟
	rcc_peripheral_enable_clock(&BOARD_USART_CLOCK_REGISTER, BOARD_USART_CLOCK_BIT);
#endif

// bymark 默认情况下PX4_FMU_V5 没有定义BOARD_FORCE_BL_PIN_IN和BOARD_FORCE_BL_PIN_OUT
#if defined(BOARD_FORCE_BL_PIN_IN) && defined(BOARD_FORCE_BL_PIN_OUT)
	/* configure the force BL pins */
	rcc_peripheral_enable_clock(&BOARD_FORCE_BL_CLOCK_REGISTER, BOARD_FORCE_BL_CLOCK_BIT);
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_INPUT, BOARD_FORCE_BL_PULL, BOARD_FORCE_BL_PIN_IN);
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, BOARD_FORCE_BL_PIN_OUT);
	gpio_set_output_options(BOARD_FORCE_BL_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, BOARD_FORCE_BL_PIN_OUT);
#endif

// bymark 默认情况下PX4_FMU_V5 没有定义BOARD_FORCE_BL_PIN
#if defined(BOARD_FORCE_BL_PIN)
	/* configure the force BL pins */
	rcc_peripheral_enable_clock(&BOARD_FORCE_BL_CLOCK_REGISTER, BOARD_FORCE_BL_CLOCK_BIT);
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_INPUT, BOARD_FORCE_BL_PULL, BOARD_FORCE_BL_PIN);
#endif

#if defined(BOARD_CLOCK_LEDS)
	/* initialise LEDs */
	// bymark 默认情况下，led是接到端口C下面的Pin上，端口C的时钟使能
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, BOARD_CLOCK_LEDS);
	// bymark 配置引脚PC6和PC7的模式
	gpio_mode_setup(
		BOARD_PORT_LEDS,
		GPIO_MODE_OUTPUT,
		GPIO_PUPD_NONE,
		BOARD_PIN_LED_BOOTLOADER | BOARD_PIN_LED_ACTIVITY);
	// bymark 配置引脚PC6和PC7输出属性
	gpio_set_output_options(
		BOARD_PORT_LEDS,
		GPIO_OTYPE_PP,
		GPIO_OSPEED_2MHZ,
		BOARD_PIN_LED_BOOTLOADER | BOARD_PIN_LED_ACTIVITY);
	// bymark 打开led(低导通：当PC6或PC7输出为低时，led导通)
	BOARD_LED_ON(
		BOARD_PORT_LEDS,
		BOARD_PIN_LED_BOOTLOADER | BOARD_PIN_LED_ACTIVITY);
#endif
	/* enable the power controller clock */
	// bymark pwr控制器时钟使能
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_PWREN);
}

void
board_deinit(void)
{

#if INTERFACE_USART
	/* deinitialise GPIO pins for USART transmit. */
	gpio_mode_setup(BOARD_PORT_USART, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_PIN_TX | BOARD_PIN_RX);

	/* disable USART peripheral clock */
	rcc_peripheral_disable_clock(&BOARD_USART_CLOCK_REGISTER, BOARD_USART_CLOCK_BIT);
#endif

#if defined(BOARD_FORCE_BL_PIN_IN) && defined(BOARD_FORCE_BL_PIN_OUT)
	/* deinitialise the force BL pins */
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_FORCE_BL_PIN_OUT);
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_FORCE_BL_PIN_IN);
#endif

#if defined(BOARD_FORCE_BL_PIN)
	/* deinitialise the force BL pin */
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_FORCE_BL_PIN);
#endif

#if defined(BOARD_POWER_PIN_OUT) && defined(BOARD_POWER_PIN_RELEASE)
	/* deinitialize the POWER pin - with the assumption the hold up time of
	 * the voltage being bleed off by an inupt pin impedance will allow
	 * enough time to boot the app
	 */
	gpio_mode_setup(BOARD_POWER_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_POWER_PIN);
#endif

#if defined(BOARD_CLOCK_LEDS)
	/* deinitialise LEDs */
	gpio_mode_setup(
		BOARD_PORT_LEDS,
		GPIO_MODE_INPUT,
		GPIO_PUPD_NONE,
		BOARD_PIN_LED_BOOTLOADER | BOARD_PIN_LED_ACTIVITY);
#endif

	/* disable the power controller clock */
	rcc_peripheral_disable_clock(&RCC_APB1ENR, RCC_APB1ENR_PWREN);

	/* disable the AHB peripheral clocks */
	RCC_AHB1ENR = 0x00100000; // XXX Magic reset number from STM32F4x reference manual
}

/**
  * @brief  Initializes the RCC clock configuration.
  *
  * @param  clock_setup : The clock configuration to set
  */
static inline void
clock_init(void)
{
	rcc_clock_setup_hse(&clock_setup, OSC_FREQ);
}

/**
  * @brief  Resets the RCC clock configuration to the default reset state.
  * @note   The default reset state of the clock configuration is given below:
  *            - HSI ON and used as system clock source
  *            - HSE, PLL and PLLI2S OFF
  *            - AHB, APB1 and APB2 prescaler set to 1.
  *            - CSS, MCO1 and MCO2 OFF
  *            - All interrupts disabled
  * @note   This function doesn't modify the configuration of the
  *            - Peripheral clocks
  *            - LSI, LSE and RTC clocks
  */
void
clock_deinit(void)
{
	/* Enable internal high-speed oscillator. */
	rcc_osc_on(RCC_HSI);
	rcc_wait_for_osc_ready(RCC_HSI);

	/* Reset the RCC_CFGR register */
	RCC_CFGR = 0x000000;

	/* Stop the HSE, CSS, PLL, PLLI2S, PLLSAI */
	rcc_osc_off(RCC_HSE);
	rcc_osc_off(RCC_PLL);
	rcc_css_disable();

	/* Reset the RCC_PLLCFGR register */
	RCC_PLLCFGR = 0x24003010; // XXX Magic reset number from STM32F4xx reference manual

	/* Reset the HSEBYP bit */
	rcc_osc_bypass_disable(RCC_HSE);

	/* Reset the CIR register */
	RCC_CIR = 0x000000;
}

uint32_t
flash_func_sector_size(unsigned sector)
{
	if (sector < BOARD_FLASH_SECTORS) {
		return flash_sectors[sector].size;
	}

	return 0;
}

void
flash_func_erase_sector(unsigned sector)
{
	if (sector >= BOARD_FLASH_SECTORS || sector < BOARD_FIRST_FLASH_SECTOR_TO_ERASE) {
		return;
	}

	/* Caculate the logical base address of the sector
	 * flash_func_read_word will add APP_LOAD_ADDRESS
	 */
	uint32_t address = 0;

	for (unsigned i = BOARD_FIRST_FLASH_SECTOR_TO_ERASE; i < sector; i++) {
		address += flash_func_sector_size(i);
	}

	/* blank-check the sector */
	unsigned size = flash_func_sector_size(sector);
	bool blank = true;

	for (unsigned i = 0; i < size; i += sizeof(uint32_t)) {
		if (flash_func_read_word(address + i) != 0xffffffff) {
			blank = false;
			break;
		}
	}

	/* erase the sector if it failed the blank check */
	if (!blank) {
		flash_erase_sector(flash_sectors[sector].sector_number, FLASH_CR_PROGRAM_X32);
	}
}

void
flash_func_write_word(uint32_t address, uint32_t word)
{
	address += APP_LOAD_ADDRESS;

	/* Ensure that all flash operations are complete. */

	flash_wait_for_last_operation();

	/* Program the 32bits. */

	FLASH_CR &= ~(FLASH_CR_PROGRAM_MASK << FLASH_CR_PROGRAM_SHIFT);
	FLASH_CR |= FLASH_CR_PROGRAM_X32 << FLASH_CR_PROGRAM_SHIFT;

	/* Enable writes to flash. */

	FLASH_CR |= FLASH_CR_PG;

	/* Program the word. */

	MMIO32(address)   = word;

	/* Use DSB to complete write etal above. So that wait is not skipped */

	__asm__ volatile("DSB \n");

	flash_wait_for_last_operation();

	/* Disable writes to flash. */

	FLASH_CR &= ~FLASH_CR_PG;
}

uint32_t
flash_func_read_word(uint32_t address)
{
	if (address & 3) {
		return 0;
	}

	return *(uint32_t *)(address + APP_LOAD_ADDRESS);
}

uint32_t
flash_func_read_otp(uint32_t address)
{
	if (address & 3) {
		return 0;
	}

	if (address > OTP_SIZE) {
		return 0;
	}

	return *(uint32_t *)(address + OTP_BASE);
}

uint32_t get_mcu_id(void)
{
	return *(uint32_t *)DBGMCU_IDCODE;
}

int get_mcu_desc(int max, uint8_t *revstr)
{
	uint32_t idcode = (*(uint32_t *)DBGMCU_IDCODE);
	int32_t mcuid = idcode & DEVID_MASK;
	mcu_rev_e revid = (idcode & REVID_MASK) >> 16;

	mcu_des_t des = mcu_descriptions[STM32_UNKNOWN];

	for (int i = 0; i < arraySize(mcu_descriptions); i++) {
		if (mcuid == mcu_descriptions[i].mcuid) {
			des = mcu_descriptions[i];
			break;
		}
	}

	for (int i = 0; i < arraySize(silicon_revs); i++) {
		if (silicon_revs[i].revid == revid) {
			des.rev = silicon_revs[i].rev;
		}
	}

	uint8_t *endp = &revstr[max - 1];
	uint8_t *strp = revstr;

	while (strp < endp && *des.desc) {
		*strp++ = *des.desc++;
	}

	if (strp < endp) {
		*strp++ = ',';
	}

	if (strp < endp) {
		*strp++ = des.rev;
	}

	return  strp - revstr;
}


int check_silicon(void)
{
	return 0;
}

uint32_t
flash_func_read_sn(uint32_t address)
{
	// read a byte out from unique chip ID area
	// it's 12 bytes, or 3 words.
	return *(uint32_t *)(address + UDID_START);
}

void
led_on(unsigned led)
{
	switch (led) {
	case LED_ACTIVITY:
#if defined(BOARD_PIN_LED_ACTIVITY)
		BOARD_LED_ON(BOARD_PORT_LEDS, BOARD_PIN_LED_ACTIVITY);
#endif
		break;

	case LED_BOOTLOADER:
#if defined(BOARD_PIN_LED_BOOTLOADER)
		BOARD_LED_ON(BOARD_PORT_LEDS, BOARD_PIN_LED_BOOTLOADER);
#endif
		break;
	}
}

void
led_off(unsigned led)
{
	switch (led) {
	case LED_ACTIVITY:
#if defined(BOARD_PIN_LED_ACTIVITY)
		BOARD_LED_OFF(BOARD_PORT_LEDS, BOARD_PIN_LED_ACTIVITY);
#endif
		break;

	case LED_BOOTLOADER:
#if defined(BOARD_PIN_LED_BOOTLOADER)
		BOARD_LED_OFF(BOARD_PORT_LEDS, BOARD_PIN_LED_BOOTLOADER);
#endif
		break;
	}
}

void
led_toggle(unsigned led)
{
	switch (led) {
	case LED_ACTIVITY:
#if defined(BOARD_PIN_LED_ACTIVITY)
		gpio_toggle(BOARD_PORT_LEDS, BOARD_PIN_LED_ACTIVITY);
#endif
		break;

	case LED_BOOTLOADER:
#if defined(BOARD_PIN_LED_BOOTLOADER)
		gpio_toggle(BOARD_PORT_LEDS, BOARD_PIN_LED_BOOTLOADER);
#endif
		break;
	}
}

/* we should know this, but we don't */
#ifndef SCB_CPACR
# define SCB_CPACR (*((volatile uint32_t *) (((0xE000E000UL) + 0x0D00UL) + 0x088)))
#endif

int
main(void)
{
	// bymark 在进入bootloader之前,尝试boot到操作系统（app_fw）
	bool try_boot = true;			/* try booting before we drop to the bootloader */  
	// bymark 在bootloader里面等待timeout这么久，然后离开，timeout默认是5000ms=5毫秒
	unsigned timeout = BOOTLOADER_DELAY;	/* if nonzero, drop out of the bootloader after this time */ 

	/* Enable the FPU before we hit any FP instructions */   // bymark floating point unit(FPU) 使能浮点运算单元
	SCB_CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 Full Access and set CP11 Full Access */

// bymark 默认情况下PX4_FMU_V5 没有宏定义 BOARD_POWER_PIN_OUT
#if defined(BOARD_POWER_PIN_OUT)

	/* Here we check for the app setting the POWER_DOWN_RTC_SIGNATURE
	 * in this case, we reset the signature and wait to die
	 */
	if (board_get_rtc_signature() == POWER_DOWN_RTC_SIGNATURE) {
		board_set_rtc_signature(0);

		while (1);
	}

#endif

	/* do board-specific initialisation */
	board_init(); 	// bymark USB, USART,LED的引脚配置和时钟使能，然后开启LED

	/* configure the clock for bootloader activity */
	clock_init();	//bymark 根据晶振频率初始化RCC时钟配置

	/*
	 * Check the force-bootloader register; if we find the signature there, don't
	 * try booting. 检查强制bootloader寄存器（即RTC备份寄存器RTC_BKPxR），如果找到该标志位，则不去尝试boot到飞控系统
	 */
	if (board_get_rtc_signature() == BOOT_RTC_SIGNATURE) {  // bymark 获取RTC备份寄存器RTC_BKPxR的值，来决定是否会boot到bootloader中

		/*
		 * Don't even try to boot before dropping to the bootloader.
		 */
		try_boot = false; // bymark 不去尝试boot到飞控系统

		/*
		 * Don't drop out of the bootloader until something has been uploaded.
		 */
		timeout = 0; // bymark 一直在bootloader中等待，直到升级完成

		/*
		 * Clear the signature so that if someone resets us while we're
		 * in the bootloader we'll try to boot next time.
		 */
		board_set_rtc_signature(0); // bymark 对RTC备份寄存器RTC_BKPxR清零
	}

#ifdef BOOT_DELAY_ADDRESS
	{
		/*
		  if a boot delay signature is present then delay the boot
		  by at least that amount of time in seconds. This allows
		  for an opportunity for a companion computer to load a
		  new firmware, while still booting fast by sending a BOOT
		  command
		 */
		// bymark 如果设置了boot_delay标志，就要延时n秒后再boot（n是设定值），这样做的目的是为了companion computer有机会去加载新的固件
		uint32_t sig1 = flash_func_read_word(BOOT_DELAY_ADDRESS);  // bymark 获取boot_delay标志
		uint32_t sig2 = flash_func_read_word(BOOT_DELAY_ADDRESS + 4); // bymark 获取boot_delay标志

		if (sig2 == BOOT_DELAY_SIGNATURE2 &&
		    (sig1 & 0xFFFFFF00) == (BOOT_DELAY_SIGNATURE1 & 0xFFFFFF00)) { // bymark 宏BOOT_DELAY_SIGNATURE1为0x92c2ecff(32bits),其中低8位是对应的boot_delay
			unsigned boot_delay = sig1 & 0xFF; // bymark 获取boot的延时时间boot_delay毫秒

			if (boot_delay <= BOOT_DELAY_MAX) { // bymark 最大的boot延时时间是30毫秒
				try_boot = false; // bymark 不去尝试boot到飞控系统

				if (timeout < boot_delay * 1000) {  // bymark boot_delay ---> timeout(在bootloader中的停留时长，单位是ms)
					timeout = boot_delay * 1000;
				}
			}
		}
	}
#endif

	/*
	 * Check if the force-bootloader pins are strapped; if strapped,
	 * don't try booting.
	 */
	// bymark 检测强制bootloader的引脚，默认情况下，PX4_FMU_V5没有定义强制bootloader的引脚
	if (board_test_force_pin()) {
		try_boot = false;
	}

#if INTERFACE_USB

	/*
	 * Check for USB connection - if present, don't try to boot, but set a timeout after
	 * which we will fall out of the bootloader. 
	 *
	 * If the force-bootloader pins are tied, we will stay here until they are removed and
	 * we then time out.
	 */

	// bymark 默认情况下，PX4_FMU_V5没有定义BOARD_USB_VBUS_SENSE_DISABLED，即如果采用USB连接，就使用USB供电，VUSB是电源线
#if defined(BOARD_USB_VBUS_SENSE_DISABLED)
	try_boot = false;
#else
	// bymark VUSB对应的引脚是GPIOA9, 当有USB连接，则GPIOA9不为零。有USB连接，则不去尝试boot到飞控系统
	if (gpio_get(GPIOA, GPIO9) != 0) {

		/* don't try booting before we set up the bootloader */
		try_boot = false;
	}

#endif
#endif

#if INTERFACE_USART

	/*
	 * Check for if the USART port RX line is receiving a break command, or is being held low. If yes,
	 * don't try to boot, but set a timeout after
	 * which we will fall out of the bootloader.
	 *
	 * If the force-bootloader pins are tied, we will stay here until they are removed and
	 * we then time out.
	 */
	// 检测串口的RX引脚是否收到break指令，或者保持为低，如果是，则不去尝试boot到飞控系统
	if (board_test_usart_receiving_break()) { // bymark 默认情况下，PX4_FMU_V5定义了SERIAL_BREAK_DETECT_DISABLED为1，即关闭串口检测break指令
		try_boot = false;
	}

#endif

	/* Try to boot the app if we think we should just go straight there */
	if (try_boot) {

		/* set the boot-to-bootloader flag so that if boot fails on reset we will stop here */
		// bymark  默认情况下，PX4_FMU_V5没有定义BOARD_BOOT_FAIL_DETECT
#ifdef BOARD_BOOT_FAIL_DETECT
		board_set_rtc_signature(BOOT_RTC_SIGNATURE);
#endif

		/* try to boot immediately */
		jump_to_app(); // bymark 会根据flash中飞控固件的前两个字来确定是否要引导到飞控操作系统

		// If it failed to boot, reset the boot signature and stay in bootloader
		// bymark 尝试boot到飞控操作系统失败后，设置强制bootloader寄存器的标志
		board_set_rtc_signature(BOOT_RTC_SIGNATURE);

		/* booting failed, stay in the bootloader forever */
		// bymark 尝试boot到飞控操作系统失败后，设置timeout为0，以便一直呆在bootloader中
		timeout = 0;
	}


	/* start the interface */
#if INTERFACE_USART
	cinit(BOARD_INTERFACE_CONFIG_USART, USART); // bymark 配置串口的波特率（默认115200），数据位(8bits)，停止位(1bit)，奇偶校验位(none)，流控制(none)，然后使能串口
#endif
#if INTERFACE_USB
	cinit(BOARD_INTERFACE_CONFIG_USB, USB);
#endif


#if 0
	// MCO1/02
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO8);
	gpio_set_af(GPIOA, GPIO_AF0, GPIO8);
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
	gpio_set_af(GPIOC, GPIO_AF0, GPIO9);
#endif


	while (1) {
		/* run the bootloader, come back after an app is uploaded or we time out */
		// bymark 进入bootloader，在飞控固件烧写完成或者超时后return
		bootloader(timeout);

		/* if the force-bootloader pins are strapped, just loop back */
		// bymark 检测强制bootloader的引脚，默认情况下，PX4_FMU_V5没有定义强制bootloader的引脚
		if (board_test_force_pin()) {
			continue;
		}

#if INTERFACE_USART

		/* if the USART port RX line is still receiving a break, just loop back */
		// bymark 默认情况下，PX4_FMU_V5定义了SERIAL_BREAK_DETECT_DISABLED为1，即关闭串口检测break指令
		if (board_test_usart_receiving_break()) {
			continue;
		}

#endif

		/* set the boot-to-bootloader flag so that if boot fails on reset we will stop here */
		// bymark  默认情况下，PX4_FMU_V5没有定义BOARD_BOOT_FAIL_DETECT
#ifdef BOARD_BOOT_FAIL_DETECT
		board_set_rtc_signature(BOOT_RTC_SIGNATURE);
#endif

		/* look to see if we can boot the app */
		jump_to_app();

		/* launching the app failed - stay in the bootloader forever */
		timeout = 0;
	}
}
