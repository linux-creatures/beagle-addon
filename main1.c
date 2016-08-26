/*
 * HC-SR04 remoteproc/rpmsg project based on TI's lab05 example.
 *
 * Copyright (C) 2015 Dimitar Dimitrov <dinuxbg@gmail.com>
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <pru_cfg.h>
#include <pru_ctrl.h>
#include <pru_iep.h>
#include <pru_intc.h>
#include <rsc_types.h>
#include <pru_virtqueue.h>
#include <pru_rpmsg.h>
#include <sys_mailbox.h>
#include "resource_table_1.h"

/* PRU1 is mailbox module user 2 */
#define MB_USER						2
/* Mbox0 - mail_u2_irq (mailbox interrupt for PRU1) is Int Number 59 */
#define MB_INT_NUMBER				59

/* Host-1 Interrupt sets bit 31 in register R31 */
#define HOST_INT					0x80000000

/* The mailboxes used for RPMsg are defined in the Linux device tree
 * PRU0 uses mailboxes 2 (From ARM) and 3 (To ARM)
 * PRU1 uses mailboxes 4 (From ARM) and 5 (To ARM)
 */
#define MB_TO_ARM_HOST				5
#define MB_FROM_ARM_HOST			4

#define CHAN_NAME					"rpmsg-pru"

#define CHAN_DESC					"Channel 31"
#define CHAN_PORT					31

/*
 * Used to make sure the Linux drivers are ready for RPMsg communication
 * Found at linux-x.y.z/include/uapi/linux/virtio_config.h
 */
#define VIRTIO_CONFIG_S_DRIVER_OK	4

char payload[RPMSG_BUF_SIZE];
volatile register uint32_t __R31;

#define PRU_OCP_RATE_HZ		(200 * 1000 * 1000)

#define TRIG_PULSE_US		10

#define GPIO1_BASE		0x4804C000

#define GPIO1_OE		(*(volatile uint32_t *)(GPIO1_BASE + 0x134))
#define GPIO1_DATAIN		(*(volatile uint32_t *)(GPIO1_BASE + 0x138))
#define GPIO1_CLEARDATAOUT	(*(volatile uint32_t *)(GPIO1_BASE + 0x190))
#define GPIO1_SETDATAOUT	(*(volatile uint32_t *)(GPIO1_BASE + 0x194))

#define TRIG_BIT		12
#define ECHO_BIT		13



/* A utility function to reverse a string  */
void reverse(char str[], int length)
{
    char temp;
    int start = 0;
    int end = length -1;
    while (start < end)
    {
	temp = str[start];
	str[start] = str[end];
	str[end] = temp;
        start++;
        end--;
    }
}
 
// Implementation of itoa()
char* itoa(int num, char* str, int base)
{
    int i = 0;
    bool isNegative = false;
 
    /* Handle 0 explicitely, otherwise empty string is printed for 0 */
    if (num == 0)
    {
        str[i++] = '0';
        str[i] = '\0';
        return str;
    }
 
    // In standard itoa(), negative numbers are handled only with 
    // base 10. Otherwise numbers are considered unsigned.
    if (num < 0 && base == 10)
    {
        isNegative = true;
        num = -num;
    }
 
    // Process individual digits
    while (num != 0)
    {
        int rem = num % base;
        str[i++] = (rem > 9)? (rem-10) + 'a' : rem + '0';
        num = num/base;
    }
 
    // If number is negative, append '-'
    if (isNegative)
        str[i++] = '-';
 
    str[i] = '\0'; // Append string terminator
 
    // Reverse the string
    reverse(str, i);
 
    return str;
}

void hc_sr04_init(void)
{

	/*
	 * Don't bother with PRU GPIOs. Our timing requirements allow
	 * us to use the "slow" system GPIOs.
	 */
	GPIO1_OE &= ~(1u << TRIG_BIT);	/* output */
	GPIO1_OE |= (1u << ECHO_BIT);	/* input */
}

int hc_sr04_measure_pulse(void)
{
	bool echo, timeout;
	
	/* pulse the trigger for 10us */
	GPIO1_SETDATAOUT = 1u << TRIG_BIT;
	__delay_cycles(TRIG_PULSE_US * (PRU_OCP_RATE_HZ / 1000000));
	GPIO1_CLEARDATAOUT = 1u << TRIG_BIT;

	/* Enable counter */
	PRU1_CTRL.CYCLE = 0;
	PRU1_CTRL.CTRL_bit.CTR_EN = 1;

	/* wait for ECHO to get high */
	do {
		echo = !!(GPIO1_DATAIN & (1u << ECHO_BIT));
		timeout = PRU1_CTRL.CYCLE > PRU_OCP_RATE_HZ;
	} while (!echo && !timeout);

	PRU1_CTRL.CTRL_bit.CTR_EN = 0;

	if (timeout)
		return -1;

	/* Restart the counter */
	PRU1_CTRL.CYCLE = 0;
	PRU1_CTRL.CTRL_bit.CTR_EN = 1;

	/* measure the "high" pulse length */
	do {
		echo = !!(GPIO1_DATAIN & (1u << ECHO_BIT));
		timeout = PRU1_CTRL.CYCLE > PRU_OCP_RATE_HZ;
	} while (echo && !timeout);

	PRU1_CTRL.CTRL_bit.CTR_EN = 0;

	if (timeout)
		return -1;

	uint64_t cycles = PRU1_CTRL.CYCLE;

	return cycles / ((uint64_t)PRU_OCP_RATE_HZ / 1000000);
}


static int measure_distance_mm(void)
{
	int t_us = hc_sr04_measure_pulse();
	int d_mm;

	/*
	 * Print the distance received from the sonar
	 * At 20 degrees in dry air the speed of sound is 3422 mm/sec
	 * so it takes 2.912 us to make 1 mm, i.e. 5.844 us for a
	 * roundtrip of 1 mm.
	 */
	d_mm = (t_us * 1000) / 5844;
	if (t_us < 0)
		d_mm = -1;

	return d_mm;
}

static void handle_mailbox_interrupt(struct pru_rpmsg_transport *transport)
{
	int16_t st;
	uint16_t src, dst, len;

	/* Clear the mailbox interrupt */
	CT_MBX.IRQ[MB_USER].STATUS_CLR |= 1 << (MB_FROM_ARM_HOST * 2);

	/* Clear the event status, event MB_INT_NUMBER
	 * corresponds to the mailbox interrupt */
	CT_INTC.SICR_bit.STS_CLR_IDX = MB_INT_NUMBER;

	/* read all of the current messages in the mailbox */
	while (CT_MBX.MSGSTATUS_bit[MB_FROM_ARM_HOST].NBOFMSG > 0) {
		/* Check to see if the message corresponds
		 * to a receive event for the PRU */
		if (CT_MBX.MESSAGE[MB_FROM_ARM_HOST] == 1){
			/* Receive the message */
			st = pru_rpmsg_receive(transport, &src, &dst,
						payload, &len);
			if (st == PRU_RPMSG_SUCCESS) {
				int d_mm = measure_distance_mm();

				/* there is no room in IRAM for iprintf */
				itoa(d_mm, payload, 10);

				pru_rpmsg_send(transport, dst, src,
						payload, strlen(payload) + 1);
			}
		}
	}
}

int main(void)
{
	struct pru_rpmsg_transport transport;
	volatile uint8_t *status;

	/* allow OCP master port access by the PRU so the PRU
	 * can read external memories */
	CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

	/* clear the status of event MB_INT_NUMBER (the mailbox event)
	 * and enable the mailbox event */
	CT_INTC.SICR_bit.STS_CLR_IDX = MB_INT_NUMBER;
	CT_MBX.IRQ[MB_USER].ENABLE_SET |= 1 << (MB_FROM_ARM_HOST * 2);

	/* Make sure the Linux drivers are ready for RPMsg communication */
	status = &resourceTable.rpmsg_vdev.status;
	while (!(*status & VIRTIO_CONFIG_S_DRIVER_OK))
		;

	/* Initialize pru_virtqueue corresponding
	 * to vring0 (PRU to ARM Host direction) */
	pru_virtqueue_init(&transport.virtqueue0,
			&resourceTable.rpmsg_vring0,
			&CT_MBX.MESSAGE[MB_TO_ARM_HOST],
			&CT_MBX.MESSAGE[MB_FROM_ARM_HOST]);

	/* Initialize pru_virtqueue corresponding
	 * to vring1 (ARM Host to PRU direction) */
	pru_virtqueue_init(&transport.virtqueue1,
			&resourceTable.rpmsg_vring1,
			&CT_MBX.MESSAGE[MB_TO_ARM_HOST],
			&CT_MBX.MESSAGE[MB_FROM_ARM_HOST]);

	/* Create the RPMsg channel between the PRU and ARM user space using the transport structure. */
	while (pru_rpmsg_channel(RPMSG_NS_CREATE, &transport, CHAN_NAME, CHAN_DESC, CHAN_PORT) != PRU_RPMSG_SUCCESS)
		;

	hc_sr04_init();

	while (1) {
		/* Check bit 31 of register R31 to see
		 * if the mailbox interrupt has occurred */
		if (__R31 & HOST_INT) {
			handle_mailbox_interrupt(&transport);
		}
	}

	return 0;
}
