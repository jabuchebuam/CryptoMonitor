/*
 * PollingRoutines.h
 *
 *  Created on: 22 de ago de 2022
 *      Author: Jonathan
 */

#ifndef APPLICATION_USER_POLLINGROUTINES_H_
#define APPLICATION_USER_POLLINGROUTINES_H_
#ifdef __cplusplus
extern "C"{
#endif

#define UART_BUF_SIZE	16

void PollingInit();
void PollingRoutine();

#ifdef __cplusplus
}
#endif
#endif /* APPLICATION_USER_POLLINGROUTINES_H_ */
