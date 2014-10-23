/***************************************************************************//**
 * @file
 * @brief Public definitions for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 2.0.1
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012-2014 Telecom Design S.A., http://www.telecomdesign.fr</b>
 ******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Telecom Design SA has no
 * obligation to support this Software. Telecom Design SA is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Telecom Design SA will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
  ******************************************************************************/

#ifndef __TD_MODULE_H
#define __TD_MODULE_H

#include "td_config_ext.h"

#ifdef __cplusplus
extern "C" {
#endif

	/***************************************************************************//**
	 * @addtogroup Compiler Compiler-specific Definitions
	 * @brief Compiler Compiler-specific Definitions
	 * @{
	 ******************************************************************************/

#if defined(__ICCARM__)

	/** Macro for packing structure depending on the used compiler */
#define __PACKED                __packed

	/** Macro for aligning structure depending on the used compiler */
#define __ALIGN(i)              __align(i)

	/** Macro for RAM function declaration depending on the used compiler */
#define __RAMFUNCTION           __ramfunc

	/** __WEAK is already defined in compiler include */

#elif defined(__GNUC__)

	/** Macro for packing structure depending on the used compiler */
#define __PACKED                __attribute__((packed))

	/** Macro for aligning structure depending on the used compiler */
#define __ALIGN(i)              __attribute__((aligned(i)))


	/** Macro for RAM function declaration depending on the used compiler */
#define __RAMFUNCTION           __attribute__((section(".data.ram @ ")))

	/** __WEAK for defining link symbols only if they are not already defined */
#define __WEAK					__attribute__((weak))

#endif

	/** @} (end addtogroup Compiler) */

	/***************************************************************************//**
	 * @addtogroup Revision
	 * @brief TDxxxx RF module revision
	 * @{
	 ******************************************************************************/

	/** Module P/N in decimal format */
#define IN1201                  1201

	/** Module P/N in string format */
#define	MODULE_PN               "IN1201"

	/** Module revision index */
#define REVISION_D              4	///< Module TD1202 Revision D
#define REVISION_E              5	///< Module TD1202 Revision E
#define REVISION_F              6	///< Module TD1202/TD1208 Revision F
#define REVISION_VBC4           7	///< VBC4, all revisions
#define REVISION_TD1204         8	///< Module TD1204, all revisions
#define REVISION_TD1205         9	///< Module TD1205, all revisions
#define REVISION_TD1202			10	///< Module TD1202, all revisions
#define REVISION_TD1208			11	///< Module TD1208, all revisions
#define REVISION_CUSTOM         0xFF///< All other implementation that must define all their config before including td_config.h. See td_config.h doc

	/* This define enables to build a very small subset of the TD_CORE library
	 * and externally controlled system timer
	 */
//#define LIB_TDCORE_TINY

	/** @} (end addtogroup Revision) */

	/***************************************************************************//**
	 * @addtogroup Pinout TDxxxx RF Module Pinout
	 * @brief TDxxxx RF module pinout
	 * @{
	 ******************************************************************************/

	/***************************************************************************//**
	 * @addtogroup Port_A Port A
	 * @brief TDxxxx RF module port A
	 * @{
	 ******************************************************************************/

	/**I/O definitions */
#define	SDA_PORT                gpioPortA           /**< I2C Data port */
#define	SDA_BIT                 0					/**< I2C Data bit */
#define	SDA_MASK                (1 << SDA_BIT)		/**< I2C Data mask */
#define	SDA_PIN                 1					/**< I2C Data pin */

#define	SCL_PORT                gpioPortA           /**< I2C Clock port */
#define	SCL_BIT                 1					/**< I2C Clock bit */
#define	SCL_MASK                (1 << SCL_BIT)		/**< I2C Clock mask */
#define SCL_PIN                 2					/**< I2C Clock pin */

#define	NIRQ_RF_PORT            gpioPortA           /**< RF IRQ port */
#define	NIRQ_RF_BIT             2					/**< RF IRQ bit */
#define	NIRQ_RF_MASK            (1 << NIRQ_RF_BIT)	/**< RF IRQ mask */
#define NIRQ_RF_PIN             3					/**< RF IRQ pin */

	/** @} (end addtogroup Port_A) */

	/***************************************************************************//**
	 * @addtogroup Port_B Port B
	 * @brief TDxxxx RF module port B
	 * @{
	 ******************************************************************************/

#define	LFXTAL_P_PORT           gpioPortB           /**< Positive 32768 Hz Crystal port */
#define	LFXTAL_P_BIT            7					/**< Positive 32768 Hz Crystal bit */
#define	LFXTAL_P_MASK           (1 << LFXTAL_P_BIT)	/**< Positive 32768 Hz Crystal mask */
#define	LFXTAL_P_PIN            7					/**< Positive 32768 Hz Crystal pin */

#define	LFXTAL_N_PORT           gpioPortB           /**< Negative 32768 Hz Crystal port */
#define	LFXTAL_N_BIT            8					/**< Negative 32768 Hz Crystal bit */
#define	LFXTAL_N_MASK           (1 << LFXTAL_N_BIT)	/**< Negative 32768 Hz Crystal mask */
#define	LFXTAL_N_PIN            8					/**< Negative 32768 Hz Crystal pin */

#define	DAC0_PORT               gpioPortB           /**< Digital to Analog Converter 0 port */
#define	DAC0_BIT                11					/**< Digital to Analog Converter 0 bit */
#define	DAC0_MASK               (1 << DAC0_BIT)		/**< Digital to Analog Converter 0 mask */
#define	DAC0_PIN                10					/**< Digital to Analog Converter 0 pin */

#define	USR0_PORT               gpioPortB           /**< User GPIO 0 port */
#define	USR0_BIT                13					/**< User GPIO 0 bit */
#define	USR0_MASK               (1 << USR0_BIT)		/**< User GPIO 0 mask */
#define	USR0_PIN                12					/**< User GPIO 0 pin */

#define	GP2_PORT                gpioPortB           /**< RF GPIO 2 port */
#define	GP2_BIT                 14					/**< RF GPIO 2 bit */
#define	GP2_MASK                (1 << GP2_BIT)		/**< RF GPIO 2 mask */
#define	GP2_PIN                 13					/**< RF GPIO 2 pin */

	/** @} (end addtogroup Port_B) */

	/***************************************************************************//**
	 * @addtogroup Port_C Port C
	 * @brief TDxxxx RF module port C
	 * @{
	 ******************************************************************************/

#define	USR2_PORT               gpioPortC           /**< User GPIO 2 port */
#define	USR2_BIT                0					/**< User GPIO 2 bit */
#define	USR2_MASK               (1 << USR2_BIT)		/**< User GPIO 2 mask */
#define	USR2_PIN                5					/**< User GPIO 2 pin */

#define	USR3_PORT               gpioPortC           /**< User GPIO 3 port */
#define	USR3_BIT                1					/**< User GPIO 3 bit */
#define	USR3_MASK               (1 << USR3_BIT)		/**< User GPIO 31 mask */
#define	USR3_PIN                6					/**< User GPIO 3 pin */

#define	USR4_PORT               gpioPortC           /**< User GPIO 4 port */
#define	USR4_BIT                14					/**< User GPIO 4 bit */
#define	USR4_MASK               (1 << USR4_BIT)		/**< User GPIO 4 mask */
#define	USR4_PIN                23					/**< User GPIO 4 pin */

#define	USR1_PORT               gpioPortC           /**< User GPIO 1 port */
#define	USR1_BIT                15					/**< User GPIO 1 bit */
#define	USR1_MASK               (1 << USR1_BIT)		/**< User GPIO 1 mask */
#define	USR1_PIN                24					/**< User GPIO 1 pin */

	/** @} (end addtogroup Port_C) */

	/***************************************************************************//**
	 * @addtogroup Port_D Port D
	 * @brief TDxxxx RF module port D
	 * @{
	 ******************************************************************************/

#define TX_PORT                 gpioPortD           /**< RS232 TX (Out) port */
#define TX_BIT                  4					/**< RS232 TX (Out) bit */
#define TX_MASK                 (1 << TX_BIT)		/**< RS232 TX (Out) mask */
#define TX_PIN                  16					/**< RS232 TX (Out) pin */

#define RX_PORT                 gpioPortD           /**< RS232 RX (In) port */
#define RX_BIT                  5					/**< RS232 RX (In) bit */
#define RX_MASK                 (1 << RX_BIT)		/**< RS232 RX (In) mask */
#define RX_PIN                  17					/**< RS232 RX (In) pin */

#define ADC0_PORT               gpioPortD           /**< Analog to Digital Converter 0 port */
#define ADC0_BIT                6					/**< Analog to Digital Converter 0 bit */
#define ADC0_MASK               (1 << ADC0_BIT)		/**< Analog to Digital Converter 0 mask */
#define ADC0_PIN                18					/**< Analog to Digital Converter 0 pin */

#define TIM2_PORT               gpioPortD           /**< Timer 2 port */
#define TIM2_BIT                7					/**< Timer 2 bit */
#define TIM2_MASK               (1 << TIM2_BIT)		/**< Timer 2 mask */
#define TIM2_PIN                19					/**< Timer 2 pin */

	/** @} (end addtogroup Port_D) */

	/***************************************************************************//**
	 * @addtogroup Port_E Port E
	 * @brief TDxxxx RF module port E
	 * @{
	 ******************************************************************************/

#define SDI_RF_PORT             gpioPortE           /**< RF SPI MOSI (Out) port */
#define SDI_RF_BIT              10					/**< RF SPI MOSI (Out) bit */
#define SDI_RF_MASK             (1 << SDI_RF_BIT)	/**< RF SPI MOSI (Out) mask */
#define SDI_RF_PIN              29					/**< RF SPI MOSI (Out) pin */

#define SDO_RF_PORT             gpioPortE           /**< RF SPI MISO (In) port */
#define SDO_RF_BIT              11					/**< RF SPI MISO (In) bit */
#define SDO_RF_MASK             (1 << SDO_RF_BIT)	/**< RF SPI MISO (In) mask */
#define SDO_RF_PIN              30					/**< RF SPI MISO (In) pin */

#define SCLK_RF_PORT            gpioPortE           /**< RF SPI Clock (Out) port */
#define SCLK_RF_BIT             12					/**< RF SPI Clock (Out) bit */
#define SCLK_RF_MASK            (1 << SCLK_RF_BIT)	/**< RF SPI Clock (Out) mask */
#define SCLK_RF_PIN             31					/**< RF SPI Clock (Out) pin */

#define NSEL_RF_PORT            gpioPortE           /**< RF Chip Select (Out) port */
#define NSEL_RF_BIT             13					/**< RF Chip Select (Out) bit */
#define NSEL_RF_MASK            (1 << NSEL_RF_BIT)	/**< RF Chip Select (Out) mask */
#define NSEL_RF_PIN             32					/**< RF Chip Select (Out) pin */

	/** @} (end addtogroup Port_E) */

	/***************************************************************************//**
	 * @addtogroup Port_F Port F
	 * @brief TDxxxx RF module port_F
	 * @{
	 ******************************************************************************/

#define DB3_PORT                gpioPortF           /**< JTAG SWDCLK Clock (In) port */
#define DB3_BIT                 0					/**< JTAG SWDCLK Clock (In) bit */
#define DB3_MASK                (1 << DB3_BIT)		/**< JTAG SWDCLK Clock (In) mask */
#define DB3_PIN                 25					/**< JTAG SWDCLK Clock (In) pin */

#define DB2_PORT                gpioPortF           /**< JTAG SWDIO Data I/O port */
#define DB2_BIT                 1					/**< JTAG SWDIO Data I/O bit */
#define DB2_MASK                (1 << DB2_BIT)		/**< JTAG SWDIO Data I/O mask */
#define DB2_PIN                 26					/**< JTAG SWDIO Data I/O pin */

	/** @} (end addtogroup Port_F) */

	/***************************************************************************//**
	 * @addtogroup RF_Interface RF Interface
	 * @brief TDxxxx RF module RF chip interface
	 * @{
	 ******************************************************************************/

	/** RF Chip interface */
#define DATA_RF_PORT            GP1_PORT			/**< RF Data port */
#define DATA_RF_BIT             GP1_BIT				/**< RF Data bit */
#define DATA_RF_MASK            GP1_MASK			/**< RF Data mask */
#define DATA_RF_PIN             GP1_PIN				/**< RF Data pin */

#define CLOCK_RF_PORT           GP2_PORT			/**< RF clock port */
#define CLOCK_RF_BIT            GP2_BIT				/**< RF clock bit */
#define CLOCK_RF_MASK           GP2_MASK			/**< RF clock mask */
#define CLOCK_RF_PIN            GP2_PIN				/**< RF clock pin */

#define IRQ0_RADIO              NIRQ_RF_MASK		/**< RF IRQ0 mask */
#define IRQ0_PORT               NIRQ_RF_PORT		/**< RF IRQ0 port */
#define IRQ0_BIT                NIRQ_RF_BIT			/**< RF IRQ0 bit */

#define IRQ1_RADIO              GP2_MASK			/**< RF IRQ1 mask */
#define IRQ1_PORT               GP2_PORT			/**< RF IRQ1 port */
#define IRQ1_BIT                GP2_BIT				/**< RF IRQ1 bit */

	/** @} (end addtogroup RF_Interface) */
	/** @} (end addtogroup Pinout) */

#ifdef __cplusplus
}
#endif

#endif // __TD_MODULE_H
