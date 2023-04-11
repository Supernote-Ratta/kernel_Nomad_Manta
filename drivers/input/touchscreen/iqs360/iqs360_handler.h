/******************************************************************************
*                                                                             *
*                                                                             *
*                                Copyright by                                 *
*                                                                             *
*                              Azoteq (Pty) Ltd                               *
*                          Republic of South Africa                           *
*                                                                             *
*                           Tel: +27(0)21 863 0033                            *
*                          E-mail: info@azoteq.com                            *
*                                                                             *
*=============================================================================*
* @file 	IQS360_handler.h  					      *
* @brief 	IQS360 handler - handles the IQS360 setup and read            *
*               Interface
* @author 	AJ van der Merwe - Azoteq PTY Ltd                             *
* @version 	V1.0.0                                                        *
* @date 	22/9/2014                                                     *
*******************************************************************************/

#ifndef IQS360_HANDLER_H
#define	IQS360_HANDLER_H

/**
 * @brief   Handles the IQS360 Setup of the Object, as well as physical
 * @param   None
 * @retval  None
 */
void IQS360_setup(void);

/**
 * @brief   Reads from the IQS360 and refreshes the Object with lates data -
 *          this data is stored in the IQS360 object
 * @param   None
 * @retval  None
 */
void IQS360_refresh_data(void);

/**
 * @brief   Handles the IQS360 events, based on the latest object
 * @param   None
 * @retval  None
 */
void IQS360_event_handler(struct i2c_client *client);



#endif	/* IQS360_HANDLER_H */

