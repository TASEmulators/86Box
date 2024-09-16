/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          3DFX Voodoo emulation.
 *
 *
 *
 * Authors: Sarah Walker, <https://pcem-emulator.co.uk/>
 *
 *          Copyright 2008-2020 Sarah Walker.
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <wchar.h>
#include <math.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include "cpu.h"
#include <86box/machine.h>
#include <86box/device.h>
#include <86box/mem.h>
#include <86box/timer.h>
#include <86box/device.h>
#include <86box/plat.h>
#include <86box/thread.h>
#include <86box/video.h>
#include <86box/vid_svga.h>
#include <86box/vid_voodoo_common.h>
#include <86box/vid_voodoo_banshee_blitter.h>
#include <86box/vid_voodoo_fb.h>
#include <86box/vid_voodoo_fifo.h>
#include <86box/vid_voodoo_reg.h>
#include <86box/vid_voodoo_regs.h>
#include <86box/vid_voodoo_render.h>
#include <86box/vid_voodoo_texture.h>

#ifdef ENABLE_VOODOO_FIFO_LOG
int voodoo_fifo_do_log = ENABLE_VOODOO_FIFO_LOG;

static void
voodoo_fifo_log(const char *fmt, ...)
{
    va_list ap;

    if (voodoo_fifo_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define voodoo_fifo_log(fmt, ...)
#endif

void
voodoo_queue_command(voodoo_t *voodoo, uint32_t addr_type, uint32_t val)
{
    fifo_entry_t *fifo = &voodoo->fifo[voodoo->fifo_write_idx & FIFO_MASK];

    if (FIFO_FULL) {
        voodoo_execute_command(voodoo);
    }

    fifo->val       = val;
    fifo->addr_type = addr_type;

    voodoo->fifo_write_idx++;
    voodoo->cmd_status &= ~(1 << 24);

    if (FIFO_ENTRIES > 0xe000)
        voodoo_execute_command(voodoo);
}

void
voodoo_flush(voodoo_t *voodoo)
{
    voodoo->flush = 1;
    voodoo_execute_command(voodoo);
    voodoo->flush = 0;
}

void
voodoo_execute_commands(voodoo_set_t *set, voodoo_t *voodoo)
{
    voodoo_execute_command(voodoo);
    if (SLI_ENABLED && voodoo->type != VOODOO_2 && set->voodoos[0] == voodoo)
        voodoo_execute_command(set->voodoos[1]);
}

void
voodoo_wait_for_swap_complete(voodoo_t *voodoo)
{
    if (voodoo->swap_pending) {
        if (voodoo->swap_pending && voodoo->flush) {
            /*Main thread is waiting for FIFO to empty, so skip vsync wait and just swap*/
            memset(voodoo->dirty_line, 1, sizeof(voodoo->dirty_line));
            voodoo->front_offset = voodoo->params.front_offset;
            if (voodoo->swap_count > 0)
                voodoo->swap_count--;
            voodoo->swap_pending = 0;
        }
    }
}

static uint32_t
cmdfifo_get(voodoo_t *voodoo)
{
    uint32_t val;

    val = *(uint32_t *) &voodoo->fb_mem[voodoo->cmdfifo_rp & voodoo->fb_mask];

    if (!voodoo->cmdfifo_in_sub)
        voodoo->cmdfifo_depth_rd++;
    voodoo->cmdfifo_rp += 4;

    //        voodoo_fifo_log("  CMDFIFO get %08x\n", val);
    return val;
}

static inline float
cmdfifo_get_f(voodoo_t *voodoo)
{
    union {
        uint32_t i;
        float    f;
    } tempif;

    tempif.i = cmdfifo_get(voodoo);
    return tempif.f;
}

enum {
    CMDFIFO3_PC_MASK_RGB   = (1 << 10),
    CMDFIFO3_PC_MASK_ALPHA = (1 << 11),
    CMDFIFO3_PC_MASK_Z     = (1 << 12),
    CMDFIFO3_PC_MASK_Wb    = (1 << 13),
    CMDFIFO3_PC_MASK_W0    = (1 << 14),
    CMDFIFO3_PC_MASK_S0_T0 = (1 << 15),
    CMDFIFO3_PC_MASK_W1    = (1 << 16),
    CMDFIFO3_PC_MASK_S1_T1 = (1 << 17),

    CMDFIFO3_PC = (1 << 28)
};

static uint32_t guess_command_size(uint32_t header)
{
    uint32_t cmd_size = 0;
    uint32_t mask;
    int num;
    int num_verticies;

    switch (header & 7)
    {
        case 0:
        return 0;

        case 1:
        return header >> 16;

        case 2:
        mask = (header >> 3);
        while (mask)
        {
            if (mask & 1)
            {
                cmd_size++;
            }
            mask >>= 1;
        }
        break;
        
        case 3:
        num = (header >> 29) & 7;
        mask = header;//(header >> 10) & 0xff;
        num_verticies = (header >> 6) & 0xf;

        while (num_verticies--)
        {
            cmd_size += 2;
            if (mask & CMDFIFO3_PC_MASK_RGB)
            {
                if (header & CMDFIFO3_PC)
                    cmd_size++;
                else
                    cmd_size += 3;
            }
            if ((mask & CMDFIFO3_PC_MASK_ALPHA) && !(header & CMDFIFO3_PC))
                cmd_size++;
            if (mask & CMDFIFO3_PC_MASK_Z)
                cmd_size++;
            if (mask & CMDFIFO3_PC_MASK_Wb)
                cmd_size++;
            if (mask & CMDFIFO3_PC_MASK_W0)
                cmd_size++;
            if (mask & CMDFIFO3_PC_MASK_S0_T0)
                cmd_size += 2;
            if (mask & CMDFIFO3_PC_MASK_W1)
                cmd_size++;
            if (mask & CMDFIFO3_PC_MASK_S1_T1)
                cmd_size += 2;
        }
        cmd_size += num;
        break;

        case 4:
        num = (header >> 29) & 7;
        mask = (header >> 15) & 0x3fff;
        while (mask)
        {
            if (mask & 1)
                cmd_size++;

            mask >>= 1;
        }
        cmd_size += num;
        break;

        case 5:
        num = (header >> 3) & 0x7ffff;
        cmd_size++;
        if (!num)
            num = 1;

        switch (header >> 30)
        {
            case 0: /*Linear framebuffer (Banshee)*/
            case 1: /*Planar YUV*/
            case 2: /*Framebuffer*/
            case 3: /*Texture*/
            cmd_size += num;
            break;
        }
        break;
    }
    return cmd_size;    
}

void voodoo_execute_command(voodoo_t *voodoo)
{
    voodoo->voodoo_busy = 1;
    while (!FIFO_EMPTY) {
        uint64_t      start_time = plat_timer_read();
        uint64_t      end_time;
        fifo_entry_t *fifo = &voodoo->fifo[voodoo->fifo_read_idx & FIFO_MASK];

        switch (fifo->addr_type & FIFO_TYPE) {
            case FIFO_WRITEL_REG:
                while ((fifo->addr_type & FIFO_TYPE) == FIFO_WRITEL_REG) {
                    voodoo_reg_writel(fifo->addr_type & FIFO_ADDR, fifo->val, voodoo);
                    fifo->addr_type = FIFO_INVALID;
                    voodoo->fifo_read_idx++;
                    if (FIFO_EMPTY)
                        break;
                    fifo = &voodoo->fifo[voodoo->fifo_read_idx & FIFO_MASK];
                }
                break;
            case FIFO_WRITEW_FB:
                voodoo_wait_for_render_thread_idle(voodoo);
                while ((fifo->addr_type & FIFO_TYPE) == FIFO_WRITEW_FB) {
                    voodoo_fb_writew(fifo->addr_type & FIFO_ADDR, fifo->val, voodoo);
                    fifo->addr_type = FIFO_INVALID;
                    voodoo->fifo_read_idx++;
                    if (FIFO_EMPTY)
                        break;
                    fifo = &voodoo->fifo[voodoo->fifo_read_idx & FIFO_MASK];
                }
                break;
            case FIFO_WRITEL_FB:
                voodoo_wait_for_render_thread_idle(voodoo);
                while ((fifo->addr_type & FIFO_TYPE) == FIFO_WRITEL_FB) {
                    voodoo_fb_writel(fifo->addr_type & FIFO_ADDR, fifo->val, voodoo);
                    fifo->addr_type = FIFO_INVALID;
                    voodoo->fifo_read_idx++;
                    if (FIFO_EMPTY)
                        break;
                    fifo = &voodoo->fifo[voodoo->fifo_read_idx & FIFO_MASK];
                }
                break;
            case FIFO_WRITEL_TEX:
                while ((fifo->addr_type & FIFO_TYPE) == FIFO_WRITEL_TEX) {
                    if (!(fifo->addr_type & 0x400000))
                        voodoo_tex_writel(fifo->addr_type & FIFO_ADDR, fifo->val, voodoo);
                    fifo->addr_type = FIFO_INVALID;
                    voodoo->fifo_read_idx++;
                    if (FIFO_EMPTY)
                        break;
                    fifo = &voodoo->fifo[voodoo->fifo_read_idx & FIFO_MASK];
                }
                break;
            case FIFO_WRITEL_2DREG:
                while ((fifo->addr_type & FIFO_TYPE) == FIFO_WRITEL_2DREG) {
                    voodoo_2d_reg_writel(voodoo, fifo->addr_type & FIFO_ADDR, fifo->val);
                    fifo->addr_type = FIFO_INVALID;
                    voodoo->fifo_read_idx++;
                    if (FIFO_EMPTY)
                        break;
                    fifo = &voodoo->fifo[voodoo->fifo_read_idx & FIFO_MASK];
                }
                break;

            default:
                fatal("Unknown fifo entry %08x\n", fifo->addr_type);
        }

        end_time = plat_timer_read();
        voodoo->time += end_time - start_time;
    }

    voodoo->cmd_status |= (1 << 24);

    while (voodoo->cmdfifo_enabled && (voodoo->cmdfifo_depth_rd != voodoo->cmdfifo_depth_wr || voodoo->cmdfifo_in_sub)) {
        uint32_t header = cmdfifo_get(voodoo);
        uint32_t cmd_size = guess_command_size(header);
        
        /* Check that we have enough values to perform the command */
        if ((voodoo->cmdfifo_depth_wr - voodoo->cmdfifo_depth_rd) < cmd_size)
        {
            /* Push back the header */
            if (!voodoo->cmdfifo_in_sub)
                voodoo->cmdfifo_depth_rd--;
            voodoo->cmdfifo_rp -= 4;
            break;
        }
        
        uint64_t start_time = plat_timer_read();
        uint64_t end_time;
        uint32_t addr;
        uint32_t mask;
        int      smode;
        int      num;
        int      num_verticies;
        int      v_num;

#if 0
        voodoo_fifo_log(" CMDFIFO header %08x at %08x\n", header, voodoo->cmdfifo_rp);
#endif

        voodoo->cmd_status &= ~7;
        voodoo->cmd_status |= (header & 7);
        voodoo->cmd_status |= (1 << 11);
        switch (header & 7) {
            case 0:
#if 0
                voodoo_fifo_log("CMDFIFO0\n");
#endif
                voodoo->cmd_status = (voodoo->cmd_status & 0xffff8fff) | (((header >> 3) & 7) << 12);
                switch ((header >> 3) & 7) {
                    case 0: /*NOP*/
                        break;

                    case 1: /*JSR*/
#if 0
                        voodoo_fifo_log("JSR %08x\n", (header >> 4) & 0xfffffc);
#endif
                        voodoo->cmdfifo_ret_addr = voodoo->cmdfifo_rp;
                        voodoo->cmdfifo_rp       = (header >> 4) & 0xfffffc;
                        voodoo->cmdfifo_in_sub   = 1;
                        break;

                    case 2: /*RET*/
                        voodoo->cmdfifo_rp     = voodoo->cmdfifo_ret_addr;
                        voodoo->cmdfifo_in_sub = 0;
                        break;

                    case 3: /*JMP local frame buffer*/
                        voodoo->cmdfifo_rp = (header >> 4) & 0xfffffc;
#if 0
                        voodoo_fifo_log("JMP to %08x %04x\n", voodoo->cmdfifo_rp, header);
#endif
                        break;

                    default:
                        fatal("Bad CMDFIFO0 %08x\n", header);
                }
                voodoo->cmd_status = (voodoo->cmd_status & ~(1 << 27)) | (voodoo->cmdfifo_in_sub << 27);
                break;

            case 1:
                num  = header >> 16;
                addr = (header & 0x7ff8) >> 1;
#if 0
                voodoo_fifo_log("CMDFIFO1 addr=%08x\n",addr);
#endif
                while (num--) {
                    uint32_t val = cmdfifo_get(voodoo);
                    if ((addr & (1 << 13)) && voodoo->type >= VOODOO_BANSHEE) {
#if 0
                        if (voodoo->type != VOODOO_BANSHEE)
                            fatal("CMDFIFO1: Not Banshee\n");
#endif

#if 0
                        voodoo_fifo_log("CMDFIFO1: write %08x %08x\n", addr, val);
#endif
                        voodoo_2d_reg_writel(voodoo, addr, val);
                    } else {
                        if ((addr & 0x3ff) == SST_triangleCMD || (addr & 0x3ff) == SST_ftriangleCMD || (addr & 0x3ff) == SST_fastfillCMD || (addr & 0x3ff) == SST_nopCMD)
                            voodoo->cmd_written_fifo++;

                        if (voodoo->type >= VOODOO_BANSHEE && (addr & 0x3ff) == SST_swapbufferCMD)
                            voodoo->cmd_written_fifo++;
                        voodoo_reg_writel(addr, val, voodoo);
                    }

                    if (header & (1 << 15))
                        addr += 4;
                }
                break;

            case 2:
                if (voodoo->type < VOODOO_2)
                    fatal("CMDFIFO2: Not Voodoo 2\n");
                mask = (header >> 3);
                addr = 8;
                while (mask) {
                    if (mask & 1) {
                        uint32_t val = cmdfifo_get(voodoo);

                        voodoo_2d_reg_writel(voodoo, addr, val);
                    }

                    addr += 4;
                    mask >>= 1;
                }
                break;

            case 3:
                num   = (header >> 29) & 7;
                mask  = header; //(header >> 10) & 0xff;
                smode = (header >> 22) & 0xf;
                voodoo_reg_writel(SST_sSetupMode, ((header >> 10) & 0xff) | (smode << 16), voodoo);
                num_verticies = (header >> 6) & 0xf;
                v_num         = 0;
                if (((header >> 3) & 7) == 2)
                    v_num = 1;
#if 0
                voodoo_fifo_log("CMDFIFO3: num=%i verts=%i mask=%02x\n", num, num_verticies, (header >> 10) & 0xff);
                voodoo_fifo_log("CMDFIFO3 %02x %i\n", (header >> 10), (header >> 3) & 7);
#endif

                while (num_verticies--) {
                    voodoo->verts[3].sVx = cmdfifo_get_f(voodoo);
                    voodoo->verts[3].sVy = cmdfifo_get_f(voodoo);
                    if (mask & CMDFIFO3_PC_MASK_RGB) {
                        if (header & CMDFIFO3_PC) {
                            uint32_t val            = cmdfifo_get(voodoo);
                            voodoo->verts[3].sBlue  = (float) (val & 0xff);
                            voodoo->verts[3].sGreen = (float) ((val >> 8) & 0xff);
                            voodoo->verts[3].sRed   = (float) ((val >> 16) & 0xff);
                            voodoo->verts[3].sAlpha = (float) ((val >> 24) & 0xff);
                        } else {
                            voodoo->verts[3].sRed   = cmdfifo_get_f(voodoo);
                            voodoo->verts[3].sGreen = cmdfifo_get_f(voodoo);
                            voodoo->verts[3].sBlue  = cmdfifo_get_f(voodoo);
                        }
                    }
                    if ((mask & CMDFIFO3_PC_MASK_ALPHA) && !(header & CMDFIFO3_PC))
                        voodoo->verts[3].sAlpha = cmdfifo_get_f(voodoo);
                    if (mask & CMDFIFO3_PC_MASK_Z)
                        voodoo->verts[3].sVz = cmdfifo_get_f(voodoo);
                    if (mask & CMDFIFO3_PC_MASK_Wb)
                        voodoo->verts[3].sWb = cmdfifo_get_f(voodoo);
                    if (mask & CMDFIFO3_PC_MASK_W0)
                        voodoo->verts[3].sW0 = cmdfifo_get_f(voodoo);
                    if (mask & CMDFIFO3_PC_MASK_S0_T0) {
                        voodoo->verts[3].sS0 = cmdfifo_get_f(voodoo);
                        voodoo->verts[3].sT0 = cmdfifo_get_f(voodoo);
                    }
                    if (mask & CMDFIFO3_PC_MASK_W1)
                        voodoo->verts[3].sW1 = cmdfifo_get_f(voodoo);
                    if (mask & CMDFIFO3_PC_MASK_S1_T1) {
                        voodoo->verts[3].sS1 = cmdfifo_get_f(voodoo);
                        voodoo->verts[3].sT1 = cmdfifo_get_f(voodoo);
                    }
                    if (v_num)
                        voodoo_reg_writel(SST_sDrawTriCMD, 0, voodoo);
                    else
                        voodoo_reg_writel(SST_sBeginTriCMD, 0, voodoo);
                    v_num++;
                    if (v_num == 3 && ((header >> 3) & 7) == 0)
                        v_num = 0;
                }
                while (num--)
                    cmdfifo_get(voodoo);
                break;

            case 4:
                num  = (header >> 29) & 7;
                mask = (header >> 15) & 0x3fff;
                addr = (header & 0x7ff8) >> 1;
#if 0
                voodoo_fifo_log("CMDFIFO4 addr=%08x\n",addr);
#endif
                while (mask) {
                    if (mask & 1) {
                        uint32_t val = cmdfifo_get(voodoo);

                        if ((addr & (1 << 13)) && voodoo->type >= VOODOO_BANSHEE) {
                            if (voodoo->type < VOODOO_BANSHEE)
                                fatal("CMDFIFO1: Not Banshee\n");
#if 0
                            voodoo_fifo_log("CMDFIFO1: write %08x %08x\n", addr, val);
#endif

                            voodoo_2d_reg_writel(voodoo, addr, val);
                        } else {
                            if ((addr & 0x3ff) == SST_triangleCMD || (addr & 0x3ff) == SST_ftriangleCMD || (addr & 0x3ff) == SST_fastfillCMD || (addr & 0x3ff) == SST_nopCMD)
                                voodoo->cmd_written_fifo++;

                            if (voodoo->type >= VOODOO_BANSHEE && (addr & 0x3ff) == SST_swapbufferCMD)
                                voodoo->cmd_written_fifo++;
                            voodoo_reg_writel(addr, val, voodoo);
                        }
                    }

                    addr += 4;
                    mask >>= 1;
                }
                while (num--)
                    cmdfifo_get(voodoo);
                break;

            case 5:
#if 0
                if (header & 0x3fc00000)
                    fatal("CMDFIFO packet 5 has byte disables set %08x\n", header);
#endif
                num  = (header >> 3) & 0x7ffff;
                addr = cmdfifo_get(voodoo) & 0xffffff;
                if (!num)
                    num = 1;
#if 0
                voodoo_fifo_log("CMDFIFO5 addr=%08x num=%i\n", addr, num);
#endif
                switch (header >> 30) {
                    case 0: /*Linear framebuffer (Banshee)*/
                    case 1: /*Planar YUV*/
                        if (voodoo->texture_present[0][(addr & voodoo->texture_mask) >> TEX_DIRTY_SHIFT]) {
#if 0
                            voodoo_fifo_log("texture_present at %08x %i\n", addr, (addr & voodoo->texture_mask) >> TEX_DIRTY_SHIFT);
#endif
                            flush_texture_cache(voodoo, addr & voodoo->texture_mask, 0);
                        }
                        if (voodoo->texture_present[1][(addr & voodoo->texture_mask) >> TEX_DIRTY_SHIFT]) {
#if 0
                            voodoo_fifo_log("texture_present at %08x %i\n", addr, (addr & voodoo->texture_mask) >> TEX_DIRTY_SHIFT);
#endif
                            flush_texture_cache(voodoo, addr & voodoo->texture_mask, 1);
                        }
                        while (num--) {
                            uint32_t val = cmdfifo_get(voodoo);
                            if (addr <= voodoo->fb_mask)
                                *(uint32_t *) &voodoo->fb_mem[addr] = val;
                            addr += 4;
                        }
                        break;
                    case 2: /*Framebuffer*/
                        while (num--) {
                            uint32_t val = cmdfifo_get(voodoo);
                            voodoo_fb_writel(addr, val, voodoo);
                            addr += 4;
                        }
                        break;
                    case 3: /*Texture*/
                        while (num--) {
                            uint32_t val = cmdfifo_get(voodoo);
                            voodoo_tex_writel(addr, val, voodoo);
                            addr += 4;
                        }
                        break;

                    default:
                        fatal("CMDFIFO packet 5 bad space %08x %08x\n", header, voodoo->cmdfifo_rp);
                }
                break;

            default:
                fatal("Bad CMDFIFO packet %08x %08x\n", header, voodoo->cmdfifo_rp);
        }

        end_time = plat_timer_read();
        voodoo->time += end_time - start_time;
    }
    voodoo->voodoo_busy = 0;
}
