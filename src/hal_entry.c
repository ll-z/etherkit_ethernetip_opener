/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2025-11-25     starzhuu   first version
 */

#include <rtthread.h>
#include "hal_data.h"
#include <rtdevice.h>
#include <board.h>
#include <opener_api.h>

#define IO_OUTPUT0    BSP_IO_PORT_12_PIN_4
#define IO_OUTPUT1    BSP_IO_PORT_22_PIN_3
#define IO_OUTPUT2    BSP_IO_PORT_22_PIN_2
#define IO_OUTPUT3    BSP_IO_PORT_17_PIN_4

#define IO_INPUT0    BSP_IO_PORT_04_PIN_1
#define IO_INPUT1    BSP_IO_PORT_16_PIN_7
#define IO_INPUT2    BSP_IO_PORT_17_PIN_3
#define IO_INPUT3    BSP_IO_PORT_18_PIN_6

extern uint8_t g_assembly_data064[];
extern uint8_t g_assembly_data096[];



static void input_thread_entry(void *param)
{
    const uint8_t mask = 0x0F;
    while (1)
    {

        uint8_t bits = 0;
        bsp_io_level_t lvl;

        R_IOPORT_PinRead(&g_ioport_ctrl, IO_INPUT0, &lvl);  bits |= (lvl != 0) << 0;
        R_IOPORT_PinRead(&g_ioport_ctrl, IO_INPUT1, &lvl);  bits |= (lvl != 0) << 1;
        R_IOPORT_PinRead(&g_ioport_ctrl, IO_INPUT2, &lvl);  bits |= (lvl != 0) << 2;
        R_IOPORT_PinRead(&g_ioport_ctrl, IO_INPUT3, &lvl);  bits |= (lvl != 0) << 3;

        g_assembly_data064[0] = bits & mask;
        rt_thread_delay(10);
    }
}

static void output_thread_entry(void *param)
{
    bool status =false;
    while (1)
    {
        uint8_t bits = g_assembly_data096[0];   /* 原子读 */

        rt_pin_write(IO_OUTPUT0, (bits >> 0) & 1u);
        rt_pin_write(IO_OUTPUT1, (bits >> 1) & 1u);
        rt_pin_write(IO_OUTPUT2, (bits >> 2) & 1u);
        rt_pin_write(IO_OUTPUT3, (bits >> 3) & 1u);

        rt_thread_delay(10);
    }
}

void hal_entry(void)
{
    rt_kprintf("\nHello RT-Thread!\n");
    rt_kprintf("==================================================\n");
    rt_kprintf("This example project is an Ethernet/IP routine!\n");
    rt_kprintf("==================================================\n");

    rt_thread_t input_thr = rt_thread_create("input_thr",input_thread_entry,RT_NULL,512,10,20);
    rt_thread_t output_thr = rt_thread_create("output_thr",output_thread_entry,RT_NULL,512,10,20);
    if (input_thr != RT_NULL)
        rt_thread_startup(input_thr);
    if (output_thr != RT_NULL)
        rt_thread_startup(output_thr);


    while (1)
    {
        rt_thread_mdelay(1000);

    }
}
