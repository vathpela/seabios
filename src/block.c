// Disk setup and access
//
// Copyright (C) 2008,2009  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2002  MandrakeSoft S.A.
//
// This file may be distributed under the terms of the GNU LGPLv3 license.

#include "biosvar.h" // GET_GLOBAL
#include "block.h" // process_op
#include "hw/ata.h" // process_ata_op
#include "hw/ahci.h" // process_ahci_op
#include "hw/blockcmd.h" // cdb_*
#include "hw/esp-scsi.h" // esp_scsi_process_op
#include "hw/lsi-scsi.h" // lsi_scsi_process_op
#include "hw/megasas.h" // megasas_process_op
#include "hw/pci.h" // pci_bdf_to_bus
#include "hw/pvscsi.h" // pvscsi_process_op
#include "hw/rtc.h" // rtc_read
#include "hw/usb-msc.h" // usb_process_op
#include "hw/usb-uas.h" // uas_process_op
#include "hw/virtio-blk.h" // process_virtio_blk_op
#include "hw/virtio-scsi.h" // virtio_scsi_process_op
#include "malloc.h" // malloc_low
#include "output.h" // dprintf
#include "stacks.h" // stack_hop
#include "std/disk.h" // struct dpte_s
#include "string.h" // checksum
#include "util.h" // process_floppy_op

u8 FloppyCount VARFSEG;
u8 CDCount;
struct drive_s *IDMap[3][BUILD_MAX_EXTDRIVE] VARFSEG;
u8 *bounce_buf_fl VARFSEG;

struct drive_s *
getDrive(u8 exttype, u8 extdriveoffset)
{
    if (extdriveoffset >= ARRAY_SIZE(IDMap[0]))
        return NULL;
    return GET_GLOBAL(IDMap[exttype][extdriveoffset]);
}

int getDriveId(u8 exttype, struct drive_s *drive)
{
    ASSERT32FLAT();
    int i;
    for (i = 0; i < ARRAY_SIZE(IDMap[0]); i++)
        if (getDrive(exttype, i) == drive)
            return i;
    return -1;
}

int create_bounce_buf(void)
{
    if (bounce_buf_fl)
        return 0;

    u8 *buf = malloc_low(CDROM_SECTOR_SIZE);
    if (!buf) {
        warn_noalloc();
        return -1;
    }
    bounce_buf_fl = buf;
    return 0;
}

/****************************************************************
 * Disk geometry translation
 ****************************************************************/

static u8
get_translation(struct drive_s *drive)
{
    u8 type = drive->type;
    if (CONFIG_QEMU && type == DTYPE_ATA) {
        // Emulators pass in the translation info via nvram.
        u8 translation = rtc_read(CMOS_BIOS_DISKTRANSFLAG + drive->cntl_id/4);
        translation >>= 2 * (drive->cntl_id % 4);
        translation &= 0x03;
        return translation;
    }

    // Otherwise use a heuristic to determine translation type.
    u16 heads = drive->pchs.head;
    u16 cylinders = drive->pchs.cylinder;
    u16 spt = drive->pchs.sector;
    u64 sectors = drive->sectors;
    u64 psectors = (u64)heads * cylinders * spt;
    if (!heads || !cylinders || !spt || psectors > sectors)
        // pchs doesn't look valid - use LBA.
        return TRANSLATION_LBA;

    if (cylinders <= 1024 && heads <= 16 && spt <= 63)
        return TRANSLATION_NONE;
    if (cylinders * heads <= 131072)
        return TRANSLATION_LARGE;
    return TRANSLATION_LBA;
}

static void
setup_translation(struct drive_s *drive)
{
    u8 translation = get_translation(drive);
    drive->translation = translation;

    u16 heads = drive->pchs.head ;
    u16 cylinders = drive->pchs.cylinder;
    u16 spt = drive->pchs.sector;
    u64 sectors = drive->sectors;
    const char *desc = NULL;

    switch (translation) {
    default:
    case TRANSLATION_NONE:
        desc = "none";
        break;
    case TRANSLATION_LBA:
        desc = "lba";
        spt = 63;
        if (sectors > 63*255*1024) {
            heads = 255;
            cylinders = 1024;
            break;
        }
        u32 sect = (u32)sectors / 63;
        heads = sect / 1024;
        if (heads>128)
            heads = 255;
        else if (heads>64)
            heads = 128;
        else if (heads>32)
            heads = 64;
        else if (heads>16)
            heads = 32;
        else
            heads = 16;
        cylinders = sect / heads;
        break;
    case TRANSLATION_RECHS:
        desc = "r-echs";
        // Take care not to overflow
        if (heads==16) {
            if (cylinders>61439)
                cylinders=61439;
            heads=15;
            cylinders = (u16)((u32)(cylinders)*16/15);
        }
        // then go through the large bitshift process
    case TRANSLATION_LARGE:
        if (translation == TRANSLATION_LARGE)
            desc = "large";
        while (cylinders > 1024) {
            cylinders >>= 1;
            heads <<= 1;

            // If we max out the head count
            if (heads > 127)
                break;
        }
        break;
    }
    // clip to 1024 cylinders in lchs
    if (cylinders > 1024)
        cylinders = 1024;
    dprintf(1, "drive %p: PCHS=%u/%d/%d translation=%s LCHS=%d/%d/%d s=%d\n"
            , drive
            , drive->pchs.cylinder, drive->pchs.head, drive->pchs.sector
            , desc
            , cylinders, heads, spt
            , (u32)sectors);

    drive->lchs.head = heads;
    drive->lchs.cylinder = cylinders;
    drive->lchs.sector = spt;
}


/****************************************************************
 * Drive mapping
 ****************************************************************/

// Fill in Fixed Disk Parameter Table (located in ebda).
static void
fill_fdpt(struct drive_s *drive, int hdid)
{
    if (hdid > 1)
        return;

    u16 nlc = drive->lchs.cylinder;
    u16 nlh = drive->lchs.head;
    u16 nls = drive->lchs.sector;

    u16 npc = drive->pchs.cylinder;
    u16 nph = drive->pchs.head;
    u16 nps = drive->pchs.sector;

    struct fdpt_s *fdpt = &get_ebda_ptr()->fdpt[hdid];
    fdpt->precompensation = 0xffff;
    fdpt->drive_control_byte = 0xc0 | ((nph > 8) << 3);
    fdpt->landing_zone = npc;
    fdpt->cylinders = nlc;
    fdpt->heads = nlh;
    fdpt->sectors = nls;

    if (nlc != npc || nlh != nph || nls != nps) {
        // Logical mapping present - use extended structure.

        // complies with Phoenix style Translated Fixed Disk Parameter
        // Table (FDPT)
        fdpt->phys_cylinders = npc;
        fdpt->phys_heads = nph;
        fdpt->phys_sectors = nps;
        fdpt->a0h_signature = 0xa0;

        // Checksum structure.
        fdpt->checksum -= checksum(fdpt, sizeof(*fdpt));
    }

    if (hdid == 0)
        SET_IVT(0x41, SEGOFF(get_ebda_seg(), offsetof(
                                 struct extended_bios_data_area_s, fdpt[0])));
    else
        SET_IVT(0x46, SEGOFF(get_ebda_seg(), offsetof(
                                 struct extended_bios_data_area_s, fdpt[1])));
}

// Find spot to add a drive
static void
add_drive(struct drive_s **idmap, u8 *count, struct drive_s *drive)
{
    if (*count >= ARRAY_SIZE(IDMap[0])) {
        warn_noalloc();
        return;
    }
    idmap[*count] = drive;
    *count = *count + 1;
}

// Map a hard drive
void
map_hd_drive(struct drive_s *drive)
{
    ASSERT32FLAT();
    struct bios_data_area_s *bda = MAKE_FLATPTR(SEG_BDA, 0);
    int hdid = bda->hdcount;
    dprintf(3, "Mapping hd drive %p to %d\n", drive, hdid);
    add_drive(IDMap[EXTTYPE_HD], &bda->hdcount, drive);

    // Setup disk geometry translation.
    setup_translation(drive);

    // Fill "fdpt" structure.
    fill_fdpt(drive, hdid);
}

// Map a cd
void
map_cd_drive(struct drive_s *drive)
{
    ASSERT32FLAT();
    dprintf(3, "Mapping cd drive %p\n", drive);
    add_drive(IDMap[EXTTYPE_CD], &CDCount, drive);
}

// Map a floppy
void
map_floppy_drive(struct drive_s *drive)
{
    ASSERT32FLAT();
    dprintf(3, "Mapping floppy drive %p\n", drive);
    add_drive(IDMap[EXTTYPE_FLOPPY], &FloppyCount, drive);

    // Update equipment word bits for floppy
    if (FloppyCount == 1) {
        // 1 drive, ready for boot
        set_equipment_flags(0x41, 0x01);
        SET_BDA(floppy_harddisk_info, 0x07);
    } else if (FloppyCount >= 2) {
        // 2 drives, ready for boot
        set_equipment_flags(0x41, 0x41);
        SET_BDA(floppy_harddisk_info, 0x77);
    }
}

/****************************************************************
 * Extended Disk Drive (EDD) get drive parameters
 ****************************************************************/
// Fill in EDD info
int VISIBLE32FLAT
fill_edd_generic(struct segoff_s edd, struct drive_s *drive_gf
                 , struct dpte_s *dpte, struct segoff_s dpte_so
                 , struct int13dpt_s *dpt)
{
    ASSERT32FLAT();
    dprintf(1, "%s: type 0x%02x\n", __func__, GET_GLOBALFLAT(drive_gf->type));
    u16 seg = edd.seg;
    struct int13dpt_s *param_far = (void*)(edd.offset+0);
    u16 size = GET_FARVAR(seg, param_far->size);
    u16 t13 = size >= 74;
    u8 sum;
    dprintf(1, "%s: edd.seg: 0x%04x\n", __func__, seg);
    dprintf(1, "%s: size: %d\n", __func__, size);
    dprintf(1, "%s: dpte: 0x%08x dpte_so.seg: 0x%08x dpte_so.offset: 0x%08x\n"
            , __func__, (u32)dpte, dpte_so.seg, dpte_so.offset);

    // Buffer is too small
    if (size < 26)
        return DISK_RET_EPARAM;

    // EDD 1.x

    u8  type    = GET_GLOBALFLAT(drive_gf->type);
    u16 npc     = GET_GLOBALFLAT(drive_gf->pchs.cylinder);
    u16 nph     = GET_GLOBALFLAT(drive_gf->pchs.head);
    u16 nps     = GET_GLOBALFLAT(drive_gf->pchs.sector);
    u64 lba     = GET_GLOBALFLAT(drive_gf->sectors);
    u16 blksize = GET_GLOBALFLAT(drive_gf->blksize);

    dprintf(DEBUG_HDL_13, "disk_1348 size=%d t=%d chs=%d,%d,%d lba=%d bs=%d\n"
            , size, type, npc, nph, nps, (u32)lba, blksize);

    dpt->size = 26;
    if (lba == (u64)-1) {
        // 0x74 = removable, media change, lockable, max values
        dpt->infos = 0x74;
        dpt->cylinders = 0xffffffff;
        dpt->heads = 0xffffffff;
        dpt->spt = 0xffffffff;
    } else {
        if (lba > (u64)nps*nph*0x3fff) {
            dpt->infos = 0x00; // geometry is invalid
            dpt->cylinders = 0x3fff;
        } else {
            dpt->infos = 0x02; // geometry is valid
            dpt->cylinders = (u32)npc;
        }
        dpt->heads = (u32)nph;
        dpt->spt = (u32)nps;
    }
    dpt->sector_count = lba;
    dpt->blksize = blksize;

    if (size < 30 || !dpte) {
        dprintf(1, "%s:%d size is %d and dpte is 0x%08x\n"
                , __func__, __LINE__, size, (u32)dpte);

#if 1
        u8 *src=(u8 *)dpt, *dest=(u8 *)param_far;
        for (int i=0; i < dpt->size; i++)
            SET_FARVAR(seg, dest[i], src[i]);
#else
        memcpy_far(seg, param_far, FLATPTR_TO_SEG(dpt)
                   , (void*)FLATPTR_TO_OFFSET(dpt), dpt->size);
#endif
        return DISK_RET_SUCCESS;
    }

    // EDD 2.x

    dpt->size = 30;
    dpt->dpte.segoff = dpte_so.segoff;

    if (size < (t13 ? 74 : 66) || !memcmp(dpt->host_bus, "\0\0\0\0", 4)) {
        dprintf(1, "%s:%d size is %d and host_bus is \"%c%c%c%c\"\n"
                , __func__, __LINE__, size, dpt->host_bus[0], dpt->host_bus[1]
                , dpt->host_bus[2], dpt->host_bus[3]);
        SET_LOW(dpte->revision, 0x21);
        sum = checksum_far(SEG_LOW, dpte, 15);
        SET_LOW(dpte->checksum, -sum);

#if 1
        u8 *src=(u8 *)dpt, *dest=(u8 *)param_far;
        for (int i=0; i < dpt->size; i++)
            SET_FARVAR(seg, dest[i], src[i]);
#else
        memcpy_far(seg, param_far, FLATPTR_TO_SEG(dpt)
                   , (void*)FLATPTR_TO_OFFSET(dpt), dpt->size);
#endif
        return DISK_RET_SUCCESS;
    }

    // EDD 3.x
    dpt->size = t13 ? 74 : 66;
    dpt->key = 0xbedd;
    dpt->dpi_length = t13 ? 44 : 36;
    dpt->reserved1 = 0;
    dpt->reserved2 = 0;

    SET_LOW(dpte->revision, 0x30);
    sum = checksum_far(SEG_LOW, dpte, 15);
    SET_LOW(dpte->checksum, -sum);

    // Phoenix v3 spec (pre t13) did not define the PCI channel field
    if (!t13)
        dpt->iface_path &= 0x00ffffff;

    if (t13) {
        void *start = (void *)&dpt->key;
        int size =
            offsetof(struct int13dpt_s, device_path.t13.generic.checksum)
            - offsetof(struct int13dpt_s, key);
#if 1
        sum = checksum(start, size);
        dprintf(1, "%s:%d: sum[%p:%p(+%d)] is 0x%x -> 0x%x\n"
                , __func__, __LINE__
                , start, start+size, size, sum, -sum);
	dpt->device_path.t13.generic.checksum = -sum;
#elif 0
        sum = checksum_far(edd.seg, (void *)param_far+30, 43);
        SET_FARVAR(seg, param_far->device_path.t13.generic.checksum
                   , -sum;
#endif
    } else {
        void *start = (void *)&dpt->key;
        int size = offsetof(struct int13dpt_s
                            , device_path.phoenix.checksum);
        sum = checksum(start, size);
        dpt->device_path.phoenix.checksum = (u8)((~sum)+1);
    }

#if 1
    u8 *src=(u8 *)dpt, *dest=(u8 *)param_far;
    for (int i=0; i < dpt->size; i++)
        SET_FARVAR(seg, dest[i], src[i]);
#elif 0
    memcpy_far(seg, param_far, FLATPTR_TO_SEG(dpt)
               , (void*)FLATPTR_TO_OFFSET(dpt), dpt->size);
#endif
    return DISK_RET_SUCCESS;
}

// Build an EDD "iface_path" field for a PCI device
static u32
edd_pci_path(u16 bdf, u8 channel)
{
    return (pci_bdf_to_bus(bdf) | (pci_bdf_to_dev(bdf) << 8)
            | (pci_bdf_to_fn(bdf) << 16) | ((u32)channel << 24));
}

struct dpte_s dpte VARLOW = { 0, };

int VISIBLE32FLAT
fill_edd_ahci(struct segoff_s edd, struct drive_s *drive_gf
              , struct segoff_s dpte_so)
{
    ASSERT32FLAT();
    dprintf(1, "%s: type 0x%02x\n", __func__, GET_GLOBALFLAT(drive_gf->type));
    dprintf(1, "%s: edd.seg: 0x%04x\n", __func__, edd.seg);
    if (!CONFIG_AHCI)
        return DISK_RET_EPARAM;

    struct ahci_port_s *port_gf = container_of(
        drive_gf, struct ahci_port_s, drive);
    struct ahci_ctrl_s *ctrl_gf = GET_GLOBALFLAT(port_gf->ctrl);
    u16 options = 0;
    struct int13dpt_s dpt = { 0, };

    memcpy(dpt.host_bus, "PCI ", 4);
    memcpy(dpt.iface_type, "SATA    ", 8);
    dpt.iface_path = edd_pci_path(GET_GLOBALFLAT(ctrl_gf->pci_bdf), 0xff);
    dpt.device_path.t13.generic.reserved0 = 0;
    dpt.device_path.t13.generic.reserved1 = 0;
    dpt.device_path.t13.generic.reserved2 = 0;
    dpt.device_path.t13.sata.port = GET_GLOBALFLAT(port_gf->pnr);
    dpt.device_path.t13.sata.pmp = 0;

    if (GET_GLOBALFLAT(port_gf->atapi)) {
        // ATAPI
        options |= 1<<5; // removable device
        options |= 1<<6; // atapi device
    }

    u8 translation = GET_GLOBALFLAT(drive_gf->translation);
    if (translation != TRANSLATION_NONE) {
        options |= 1<<3; // CHS translation
        if (translation == TRANSLATION_LBA)
            options |= 1<<9;
        if (translation == TRANSLATION_RECHS)
            options |= 3<<9;
    }

    options |= 1<<4; // lba translation
    options |= 1<<7; // 32-bit transfers

    // Fill in dpte
    SET_LOW(dpte.iobase1, GET_GLOBALFLAT(ctrl_gf->iobase));
    SET_LOW(dpte.irq, GET_GLOBALFLAT(ctrl_gf->irq));
    SET_LOW(dpte.blkcount, 1);
    SET_LOW(dpte.dma, 1);
    SET_LOW(dpte.options, options);

    return fill_edd_generic(edd, drive_gf, &dpte, dpte_so, &dpt);
}

// EDD info for ATA and ATAPI drives
int VISIBLE32FLAT
fill_edd_ata(struct segoff_s *edd, struct drive_s *drive_gf
              , struct segoff_s dpte_so)
{
    ASSERT32FLAT();
    if (!CONFIG_ATA)
        return DISK_RET_EPARAM;

    struct atadrive_s *adrive_gf = container_of(
        drive_gf, struct atadrive_s, drive);
    struct ata_channel_s *chan_gf = GET_GLOBALFLAT(adrive_gf->chan_gf);
    u8 slave = GET_GLOBALFLAT(adrive_gf->slave);
    u16 iobase2 = GET_GLOBALFLAT(chan_gf->iobase2);
    u8 irq = GET_GLOBALFLAT(chan_gf->irq);
    u16 iobase1 = GET_GLOBALFLAT(chan_gf->iobase1);
    int bdf = GET_GLOBALFLAT(drive_gf->cntl_id);
    u8 channel = GET_GLOBALFLAT(chan_gf->chanid);
    u16 options = 0;
    struct int13dpt_s dpt = { 0, };

    dpt.device_path.t13.generic.reserved0 = 0;
    dpt.device_path.t13.generic.reserved1 = 0;
    dpt.device_path.t13.generic.reserved2 = 0;
    dpt.device_path.t13.ata.device = channel;

    if (GET_GLOBALFLAT(drive_gf->type) == DTYPE_ATA_ATAPI) {
        // ATAPI
        options |= 1<<5; // removable device
        options |= 1<<6; // atapi device
    }

    u8 translation = GET_GLOBALFLAT(drive_gf->translation);
    if (translation != TRANSLATION_NONE) {
        options |= 1<<3; // CHS translation
        if (translation == TRANSLATION_LBA)
            options |= 1<<9;
        if (translation == TRANSLATION_RECHS)
            options |= 3<<9;
    }

    memcpy(dpt.iface_type, "ATA     ", 8);
    if (bdf == -1) {
        memcpy(dpt.host_bus, "ISA ", 4);
        dpt.iface_path = iobase1;
    } else {
        memcpy(dpt.host_bus, "PCI ", 4);
        dpt.iface_path = edd_pci_path(bdf, slave);
    }

    options |= 1<<4; // lba translation
    if (CONFIG_ATA_PIO32)
        options |= 1<<7; // 32-bit transfers

    // Fill in dpte
    SET_LOW(dpte.iobase1, iobase1);
    SET_LOW(dpte.iobase2, iobase2 + ATA_CB_DC);
    SET_LOW(dpte.prefix, ((slave ? ATA_CB_DH_DEV1 : ATA_CB_DH_DEV0)
                                 | ATA_CB_DH_LBA));
    SET_LOW(dpte.unused, 0xcb);
    SET_LOW(dpte.irq, irq);
    SET_LOW(dpte.blkcount, 1);
    SET_LOW(dpte.options, options);

    return fill_edd_generic(*edd, drive_gf, &dpte, dpte_so, &dpt);
}

struct scsi_params {
    u32 iobase;
    u16 target;
    u32 lun;
    struct segoff_s dpte_so;
};

int VISIBLE32FLAT
fill_edd_scsi(struct segoff_s *edd, struct drive_s *drive_gf
              , struct scsi_params *params)
{
    ASSERT32FLAT();
    dprintf(1, "%s: params->dpte_so.seg: 0x%x params->dpte_so.offset: 0x%x\n"
            , __func__, params->dpte_so.seg, params->dpte_so.offset);
    dprintf(1, "%s: iobase/target/lun: 0x%x 0x%x 0x%x\n", __func__
            , params->iobase, params->target, params->lun);

    u16 options = 0;
    u8 translation = GET_GLOBALFLAT(drive_gf->translation);
    struct int13dpt_s dpt = { 0, };

    if (translation != TRANSLATION_NONE) {
        options |= 1<<3; // CHS translation
        if (translation == TRANSLATION_LBA)
            options |= 1<<9;
        if (translation == TRANSLATION_RECHS)
            options |= 3<<9;
    }

    options |= 1<<4; // lba translation
    options |= 1<<7; // 32-bit transfer mode

    memcpy(dpt.host_bus, "PCI ", 4);
    memcpy(dpt.iface_type, "SCSI    ", 8);
    dpt.iface_path = edd_pci_path(GET_GLOBALFLAT(drive_gf->cntl_id), 0xff);
    dpt.device_path.t13.generic.reserved0 = 0;
    dpt.device_path.t13.generic.reserved1 = 0;
    dpt.device_path.t13.generic.reserved2 = 0;
    dpt.device_path.t13.scsi.id = params->target;
    dpt.device_path.t13.scsi.lun = params->lun;

    // Fill in dpte
    SET_LOW(dpte.iobase1, params->iobase);
    SET_LOW(dpte.blkcount, 1);
    SET_LOW(dpte.options, options);

    return fill_edd_generic(*edd, drive_gf, &dpte, params->dpte_so, &dpt);
}

static int
fill_edd_both(struct segoff_s edd, struct drive_s *drive_gf
              , struct segoff_s dpte_so)
{
    extern void _cfunc32flat_fill_edd_32(void);
    extern void _cfunc32flat_fill_edd_scsi(void);
    extern void _cfunc32flat_fill_edd_ata(void);

    struct scsi_params params = { .dpte_so = dpte_so };
    int ret;

    dprintf(1, "%s: dpte_so.seg: 0x%x dpte_so.offset: 0x%x\n", __func__
            , dpte_so.seg, dpte_so.offset);
    dprintf(1, "%s: params.dpte_so.seg: 0x%x params.dpte_so.offset: 0x%x\n"
            , __func__, params.dpte_so.seg, params.dpte_so.offset);
    switch (GET_GLOBALFLAT(drive_gf->type)) {
    case DTYPE_ATA_ATAPI:
        if (MODESEGMENT)
            return call32_params(_cfunc32flat_fill_edd_ata
                                 , (u32)MAKE_FLATPTR(GET_SEG(SS), &edd)
                                 , (u32)drive_gf
                                 , dpte_so.segoff
                                 , DISK_RET_EPARAM);
        else
            return fill_edd_ata(&edd, drive_gf, dpte_so);
    case DTYPE_LSI_SCSI:
        ret = lsi_scsi_get_device_parameters(drive_gf, &params.iobase
                                             , &params.target, &params.lun);
        if (ret != DISK_RET_SUCCESS)
            return ret;
        if (MODESEGMENT)
            return call32_params(_cfunc32flat_fill_edd_scsi
                                 , (u32)MAKE_FLATPTR(GET_SEG(SS), &edd)
                                 , (u32)drive_gf
                                 , (u32)MAKE_FLATPTR(GET_SEG(SS), &params)
                                 , DISK_RET_EPARAM);
        else
            return fill_edd_scsi(&edd, drive_gf, &params);
    case DTYPE_ESP_SCSI:
        ret = esp_scsi_get_device_parameters(drive_gf, &params.iobase
                                             , &params.target, &params.lun);
        if (ret != DISK_RET_SUCCESS)
            return ret;
        if (MODESEGMENT)
            return call32_params(_cfunc32flat_fill_edd_scsi
                                 , (u32)MAKE_FLATPTR(GET_SEG(SS), &edd)
                                 , (u32)drive_gf
                                 , (u32)MAKE_FLATPTR(GET_SEG(SS), &params)
                                 , DISK_RET_EPARAM);
        else
            return fill_edd_scsi(&edd, drive_gf, &params);
    case DTYPE_MEGASAS:
        ret = megasas_get_device_parameters(drive_gf, &params.iobase
                                            , &params.target, &params.lun);
        if (ret != DISK_RET_SUCCESS)
            return ret;
        if (MODESEGMENT)
            return call32_params(_cfunc32flat_fill_edd_scsi
                                 , (u32)MAKE_FLATPTR(GET_SEG(SS), &edd)
                                 , (u32)drive_gf
                                 , (u32)MAKE_FLATPTR(GET_SEG(SS), &params)
                                 , DISK_RET_EPARAM);
        else
            return fill_edd_scsi(&edd, drive_gf, &params);
    default:
    case DTYPE_AHCI:
    case DTYPE_AHCI_ATAPI:
    case DTYPE_VIRTIO_BLK:
    case DTYPE_VIRTIO_SCSI:
    case DTYPE_PVSCSI:
        // if we're in 32bit mode here, then fill_edd_32 is our caller.
        if (!MODESEGMENT)
            return DISK_RET_EPARAM;
        // In 16bit mode and driver not found - try in 32bit mode
        return call32_params(_cfunc32flat_fill_edd_32
                             , (u32)MAKE_FLATPTR(GET_SEG(SS), &edd)
                             , (u32)drive_gf
                             , (u32)dpte_so.segoff
                             , DISK_RET_EPARAM);
    }
}

#if 0
static u32
get_dpte_so(u32 dtpe)
{
    struct segoff_s dpte_so = SEGOFF(SEG_LOW, dpte);
    return dpte_so.segoff;
}
#endif

int VISIBLE32FLAT
fill_edd_32(struct segoff_s *edd, struct drive_s *drive_gf
            , struct segoff_s dpte_so)
{
    ASSERT32FLAT();
    dprintf(1, "%s: dpte_so.seg: 0x%x dpte_so.offset: 0x%x\n", __func__
            , dpte_so.seg, dpte_so.offset);
    int ret;
    struct scsi_params params = { .dpte_so = dpte_so };
#if 0
    struct segoff_s dpte_so;
    struct bregs br;

    memset(&br, 0, sizeof(br));
    br.flags = F_IF;
    br.code = get_dpte_so;
    br.eax = &dpte;
    farcall16(&br);

    dpte_so.segoff = farcall16(&br);
    params.dpte_so = dpte_so;
#endif

    switch (drive_gf->type) {
    case DTYPE_AHCI:
    case DTYPE_AHCI_ATAPI:
        return fill_edd_ahci(*edd, drive_gf, dpte_so);
    case DTYPE_VIRTIO_BLK:
        ret = virtio_blk_get_device_parameters(drive_gf, &params.iobase
                                               , &params.target, &params.lun);
        if (ret != DISK_RET_SUCCESS)
            return ret;

        return fill_edd_scsi(edd, drive_gf, &params);
    case DTYPE_VIRTIO_SCSI:
        ret = virtio_scsi_get_device_parameters(drive_gf, &params.iobase
                                                , &params.target, &params.lun);
        if (ret != DISK_RET_SUCCESS)
            return ret;
        return fill_edd_scsi(edd, drive_gf, &params);
    case DTYPE_PVSCSI:
        ret = pvscsi_get_device_parameters(drive_gf, &params.iobase
                                           , &params.target, &params.lun);
        if (ret != DISK_RET_SUCCESS)
            return ret;
        return fill_edd_scsi(edd, drive_gf, &params);
    default:
        return fill_edd_both(*edd, drive_gf, dpte_so);
    }
}

static int
fill_edd_16(struct segoff_s edd, struct drive_s *drive_gf
            , struct segoff_s dpte_so)
{
    ASSERT16();
    dprintf(1, "%s: dpte_so.seg: 0x%x dpte_so.offset: 0x%x\n", __func__
            , dpte_so.seg, dpte_so.offset);

    switch (GET_GLOBALFLAT(drive_gf->type)) {
    case DTYPE_ATA: ;
        extern void _cfunc32flat_fill_edd_ata(void);
        return call32_params(_cfunc32flat_fill_edd_ata
                             , (u32)MAKE_FLATPTR(GET_SEG(SS), &edd)
                             , (u32)drive_gf, dpte_so.segoff, DISK_RET_EPARAM);
    default:
        return fill_edd_both(edd, drive_gf, dpte_so);
    }
}

// Fill Extended Disk Drive (EDD) "Get drive parameters" info for a drive
int
fill_edd(struct segoff_s edd, struct drive_s *drive_gf)
{
    dprintf(1, "%s: type 0x%02x edd.segoff: 0x%x\n"
            , __func__, GET_GLOBALFLAT(drive_gf->type), edd.segoff);
    int ret;
    if (MODESEGMENT)
        ret = fill_edd_16(edd, drive_gf, SEGOFF(SEG_LOW, (u32)&dpte));
    else
        ret = fill_edd_32(&edd, drive_gf, SEGOFF(SEG_LOW, (u32)&dpte));
    return ret;
}

/****************************************************************
 * Disk driver dispatch
 ****************************************************************/

// Fallback handler for command requests not implemented by drivers
int
default_process_op(struct disk_op_s *op)
{
    switch (op->command) {
    case CMD_FORMAT:
    case CMD_RESET:
    case CMD_ISREADY:
    case CMD_VERIFY:
    case CMD_SEEK:
        // Return success if the driver doesn't implement these commands
        return DISK_RET_SUCCESS;
    default:
        return DISK_RET_EPARAM;
    }
}

// Command dispatch for disk drivers that run in both 16bit and 32bit mode
static int
process_op_both(struct disk_op_s *op)
{
    switch (GET_GLOBALFLAT(op->drive_gf->type)) {
    case DTYPE_ATA_ATAPI:
        return ata_atapi_process_op(op);
    case DTYPE_USB:
        return usb_process_op(op);
    case DTYPE_UAS:
        return uas_process_op(op);
    case DTYPE_LSI_SCSI:
        return lsi_scsi_process_op(op);
    case DTYPE_ESP_SCSI:
        return esp_scsi_process_op(op);
    case DTYPE_MEGASAS:
        return megasas_process_op(op);
    default:
        if (!MODESEGMENT)
            return DISK_RET_EPARAM;
        // In 16bit mode and driver not found - try in 32bit mode
        extern void _cfunc32flat_process_op_32(void);
        return call32(_cfunc32flat_process_op_32
                      , (u32)MAKE_FLATPTR(GET_SEG(SS), op), DISK_RET_EPARAM);
    }
}

// Command dispatch for disk drivers that only run in 32bit mode
int VISIBLE32FLAT
process_op_32(struct disk_op_s *op)
{
    ASSERT32FLAT();
    switch (op->drive_gf->type) {
    case DTYPE_VIRTIO_BLK:
        return virtio_blk_process_op(op);
    case DTYPE_AHCI:
        return ahci_process_op(op);
    case DTYPE_AHCI_ATAPI:
        return ahci_atapi_process_op(op);
    case DTYPE_SDCARD:
        return sdcard_process_op(op);
    case DTYPE_USB_32:
        return usb_process_op(op);
    case DTYPE_UAS_32:
        return uas_process_op(op);
    case DTYPE_VIRTIO_SCSI:
        return virtio_scsi_process_op(op);
    case DTYPE_PVSCSI:
        return pvscsi_process_op(op);
    default:
        return process_op_both(op);
    }
}

// Command dispatch for disk drivers that only run in 16bit mode
static int
process_op_16(struct disk_op_s *op)
{
    ASSERT16();
    switch (GET_GLOBALFLAT(op->drive_gf->type)) {
    case DTYPE_FLOPPY:
        return floppy_process_op(op);
    case DTYPE_ATA:
        return ata_process_op(op);
    case DTYPE_RAMDISK:
        return ramdisk_process_op(op);
    case DTYPE_CDEMU:
        return cdemu_process_op(op);
    default:
        return process_op_both(op);
    }
}

// Execute a disk_op_s request.
int
process_op(struct disk_op_s *op)
{
    int ret, origcount = op->count;
    if (origcount * GET_GLOBALFLAT(op->drive_gf->blksize) > 64*1024) {
        op->count = 0;
        return DISK_RET_EBOUNDARY;
    }
    if (MODESEGMENT)
        ret = process_op_16(op);
    else
        ret = process_op_32(op);
    if (ret && op->count == origcount)
        // If the count hasn't changed on error, assume no data transferred.
        op->count = 0;
    return ret;
}

// Execute a "disk_op_s" request - this runs on the extra stack.
static int
__send_disk_op(struct disk_op_s *op_far, u16 op_seg)
{
    struct disk_op_s dop;
    memcpy_far(GET_SEG(SS), &dop
               , op_seg, op_far
               , sizeof(dop));

    dprintf(DEBUG_HDL_13, "disk_op d=%p lba=%d buf=%p count=%d cmd=%d\n"
            , dop.drive_gf, (u32)dop.lba, dop.buf_fl
            , dop.count, dop.command);

    int status = process_op(&dop);

    // Update count with total sectors transferred.
    SET_FARVAR(op_seg, op_far->count, dop.count);

    return status;
}

// Execute a "disk_op_s" request by jumping to the extra 16bit stack.
int
send_disk_op(struct disk_op_s *op)
{
    ASSERT16();
    if (! CONFIG_DRIVES)
        return -1;

    return stack_hop((u32)op, GET_SEG(SS), __send_disk_op);
}
