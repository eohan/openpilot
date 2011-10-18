/*
 * protocol.h
 *
 *  Created on: Oct 17, 2011
 *      Author: msmith
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

/*
 * Trivial I2C protocol for IO.
 */

#define IOP_SET_PWM			'S'
#define IOP_PWM_CHANNELS	8

struct iop_set_pwm {
	uint16_t		values[IOP_PWM_CHANNELS];
} __attribute__((packed));

struct iop_command {
	uint8_t		opcode;
	uint8_t		pad[3];
	union {
		struct iop_set_pwm		pwm;
	} d;
} __attribute__((packed)) ;

#endif /* PROTOCOL_H_ */
