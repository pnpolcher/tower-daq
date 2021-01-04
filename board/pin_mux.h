/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

/*! @name PORTE27 (coord J4), GPIO7
  @{ */
#define BOARD_INITPINS_GPIO7_GPIO GPIOE /*!<@brief GPIO device name: GPIOE */
#define BOARD_INITPINS_GPIO7_PORT PORTE /*!<@brief PORT device name: PORTE */
#define BOARD_INITPINS_GPIO7_PIN 27U    /*!<@brief PORTE pin index: 27 */
                                        /* @} */

/*! @name PORTE5 (coord E2), GPIO8
  @{ */
#define BOARD_INITPINS_GPIO8_GPIO GPIOE /*!<@brief GPIO device name: GPIOE */
#define BOARD_INITPINS_GPIO8_PORT PORTE /*!<@brief PORT device name: PORTE */
#define BOARD_INITPINS_GPIO8_PIN 5U     /*!<@brief PORTE pin index: 5 */
                                        /* @} */

/*! @name PORTC19 (coord B5), GPIO9
  @{ */
#define BOARD_INITPINS_GPIO9_GPIO GPIOC /*!<@brief GPIO device name: GPIOC */
#define BOARD_INITPINS_GPIO9_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_INITPINS_GPIO9_PIN 19U    /*!<@brief PORTC pin index: 19 */
                                        /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
