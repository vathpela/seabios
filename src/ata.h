// Low level ATA disk definitions
//
// Copyright (C) 2008  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2002  MandrakeSoft S.A.
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#ifndef __ATA_H
#define __ATA_H

#include "types.h" // u16
#include "atabits.h"

struct ata_pio_command {
    u16 segment;
    u16 offset;
    u8 biosid;

    u8 feature;
    u8 sector_count;
    u8 lba_low;
    u8 lba_mid;
    u8 lba_high;
    u8 device;
    u8 command;

    u8 sector_count2;
    u8 lba_low2;
    u8 lba_mid2;
    u8 lba_high2;
};

// Function definitions
void ata_reset(u16 device);
int ata_transfer(struct ata_pio_command *cmd);
int ata_cmd_packet(u16 device, u8 *cmdbuf, u8 cmdlen
                   , u16 header, u32 length, u16 bufseg, u16 bufoff);
int cdrom_read(u16 device, u32 lba, u32 count
               , u16 segment, u16 offset, u16 skip);
void ata_detect();

static inline int
ata_cmd_data(u16 biosid, u16 command, u32 lba, u16 count
             , u16 segment, u16 offset)
{
    u8 slave   = biosid % 2;

    struct ata_pio_command cmd;
    cmd.biosid = biosid;
    cmd.segment = segment;
    cmd.offset = offset;

    if (count >= (1<<8) || lba + count >= (1<<28)) {
        cmd.sector_count2 = count >> 8;
        cmd.lba_low2 = lba >> 24;
        cmd.lba_mid2 = 0;
        cmd.lba_high2 = 0;

        command |= 0x04;
        lba &= 0xffffff;
    }

    cmd.feature = 0;
    cmd.sector_count = count;
    cmd.lba_low = lba;
    cmd.lba_mid = lba >> 8;
    cmd.lba_high = lba >> 16;
    cmd.device = ((slave ? ATA_CB_DH_DEV1 : ATA_CB_DH_DEV0)
                  | ((lba >> 24) & 0xf) | ATA_CB_DH_LBA);
    cmd.command = command;
    return ata_transfer(&cmd);
}

static inline int
ata_cmd_data_chs(u16 biosid, u16 command, u16 cyl, u16 head, u16 sect, u16 count
                 , u16 segment, u16 offset)
{
    u8 slave   = biosid % 2;

    struct ata_pio_command cmd;
    cmd.biosid = biosid;
    cmd.segment = segment;
    cmd.offset = offset;

    cmd.sector_count = count & 0xff;
    cmd.feature = 0;
    cmd.lba_low = sect;
    cmd.lba_mid = cyl;
    cmd.lba_high = cyl >> 8;
    cmd.device = (slave ? ATA_CB_DH_DEV1 : ATA_CB_DH_DEV0) | (head & 0xff);
    cmd.command = command;
    return ata_transfer(&cmd);
}

#endif /* __ATA_H */
