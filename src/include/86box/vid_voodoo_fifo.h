/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Voodoo Graphics, 2, Banshee, 3 emulation.
 *
 *
 *
 * Authors: Sarah Walker, <https://pcem-emulator.co.uk/>
 *          leilei
 *
 *          Copyright 2008-2020 Sarah Walker.
 */

#ifndef VIDEO_VOODOO_FIFO_H
#define VIDEO_VOODOO_FIFO_H

void voodoo_queue_command(voodoo_t *voodoo, uint32_t addr_type, uint32_t val);
void voodoo_flush(voodoo_t *voodoo);
void voodoo_execute_commands(voodoo_set_t *set, voodoo_t *voodoo);
void voodoo_wait_for_swap_complete(voodoo_t *voodoo);
void voodoo_execute_command(voodoo_t *voodoo);

#endif /*VIDEO_VOODOO_FIFO_H*/
