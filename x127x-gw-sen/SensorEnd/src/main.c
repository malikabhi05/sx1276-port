/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <misc/printk.h>
#include <stdio.h>
#include <ipm.h>
#include <ipm/ipm_quark_se.h>
#include <string.h>
#include <init.h>
#include "mraa.h"

QUARK_SE_IPM_DEFINE(req_queue, 0, QUARK_SE_IPM_OUTBOUND);
//QUARK_SE_IPM_DEFINE(resp_queue, 1, QUARK_SE_IPM_OUTBOUND);

#define SLEEPTIME 1100
#define STACKSIZE 2000

#define REQ_FIBER_PRE 4

char thread_stacks[2][STACKSIZE];
int local_buf[3];

//static int value = 0;
volatile int c = 0;
int xxx = 0;

/*
void retrieveData(void *context, uint32_t id, volatile void *data) {
	//char *datac = (char *)data;
	c = 1;
	value = *(int *)data;
	//printf("INSIDE ARC\n");
	printk("****%d******\n", value);
	printk("*****************************************\n");
}
*/

void req_thread(void *arg1, void *arg2, void *arg3) {
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	struct device *ipm = device_get_binding("req_queue");

	while(1) {
        k_sleep(5000);
        local_buf[0] = xxx++;
        local_buf[1] = 22;
        local_buf[2] = xxx++;
        ipm_send(ipm, 1, 0, local_buf, 12);
	}
}

void main(void)
{
	printf("Hello World! %s\n", CONFIG_ARCH);
	k_thread_spawn(&thread_stacks[0][0], STACKSIZE, req_thread, 0, 0, 0, K_PRIO_COOP(REQ_FIBER_PRE), 0, 0);
	//struct device *ipm;

	//ipm = device_get_binding("req_queue");
	//ipm_register_callback(ipm, retrieveData, NULL);
    //ipm_set_enabled(ipm, 1);

    while(1) {
    	k_sleep(1000);
    }
}
