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
        u8 ataid = drive->cntl_id;
        u8 channel = ataid / 2;
        u8 translation = rtc_read(CMOS_BIOS_DISKTRANSFLAG + channel/2);
        translation >>= 2 * (ataid % 4);
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

// flags for bus_iface field in fill_generic_edd()
#define EDD_ISA        0x01
#define EDD_PCI        0x02
#define EDD_BUS_MASK   0x0f
#define EDD_ATA        0x10
#define EDD_SCSI       0x20
#define EDD_SATA       0x40
#define EDD_IFACE_MASK 0xf0

// Fill in EDD info
static int
fill_generic_edd(struct segoff_s edd, struct drive_s *drive_gf
                 , struct dpte_s *dpte, u8 bus_iface, u32 iface_path
                 , union device_path_u dp)
{
    u16 seg = edd.seg;
    struct int13dpt_s *param_far = (void*)(edd.offset+0);
    u16 size = GET_FARVAR(seg, param_far->size);
    u16 t13 = size == 74;
    u32 dpte_so = dpte ? SEGOFF(SEG_LOW, (u32)dpte).segoff : 0;
    u8 sum;

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

    SET_FARVAR(seg, param_far->size, 26);
    if (lba == (u64)-1) {
        // 0x74 = removable, media change, lockable, max values
        SET_FARVAR(seg, param_far->infos, 0x74);
        SET_FARVAR(seg, param_far->cylinders, 0xffffffff);
        SET_FARVAR(seg, param_far->heads, 0xffffffff);
        SET_FARVAR(seg, param_far->spt, 0xffffffff);
    } else {
        if (lba > (u64)nps*nph*0x3fff) {
            SET_FARVAR(seg, param_far->infos, 0x00); // geometry is invalid
            SET_FARVAR(seg, param_far->cylinders, 0x3fff);
        } else {
            SET_FARVAR(seg, param_far->infos, 0x02); // geometry is valid
            SET_FARVAR(seg, param_far->cylinders, (u32)npc);
        }
        SET_FARVAR(seg, param_far->heads, (u32)nph);
        SET_FARVAR(seg, param_far->spt, (u32)nps);
    }
    SET_FARVAR(seg, param_far->sector_count, lba);
    SET_FARVAR(seg, param_far->blksize, blksize);

    if (size < 30 || !dpte_so)
        return DISK_RET_SUCCESS;

    // EDD 2.x

    SET_FARVAR(seg, param_far->size, 30);
    SET_FARVAR(seg, param_far->dpte.segoff, dpte_so);

    if (size < (t13 ? 74 : 66) || !bus_iface) {
        SET_LOW(dpte->revision, 0x21);
        sum = checksum_far(SEG_LOW, dpte, 15);
        SET_LOW(dpte->checksum, -sum);

        return DISK_RET_SUCCESS;
    }

    // EDD 3.x
    SET_FARVAR(seg, param_far->size, t13 ? 74 : 66);
    SET_FARVAR(seg, param_far->key, 0xbedd);
    SET_FARVAR(seg, param_far->dpi_length, t13 ? 44 : 36);
    SET_FARVAR(seg, param_far->reserved1, 0);
    SET_FARVAR(seg, param_far->reserved2, 0);

    SET_LOW(dpte->revision, 0x30);
    sum = checksum_far(SEG_LOW, dpte, 15);
    SET_LOW(dpte->checksum, -sum);

    const char *host_bus = "ISA ";
    if ((bus_iface & EDD_BUS_MASK) == EDD_PCI) {
        host_bus = "PCI ";
        if (!t13)
            // Phoenix v3 spec (pre t13) did not define the PCI channel field
            iface_path &= 0x00ffffff;
    }
    memcpy_far(seg, param_far->host_bus, SEG_BIOS, host_bus
               , sizeof(param_far->host_bus));

    const char *iface_type = "        ";
    if ((bus_iface & EDD_IFACE_MASK) == EDD_ATA) {
        iface_type = "ATA     ";
    } else if ((bus_iface & EDD_IFACE_MASK) == EDD_SCSI) {
        iface_type = "SCSI    ";
    }
    if (t13) {
        if ((bus_iface & EDD_IFACE_MASK) == EDD_SATA)
            iface_type = "SATA    ";
    }
    memcpy_far(seg, param_far->iface_type, SEG_BIOS, iface_type
               , sizeof(param_far->iface_type));
    SET_FARVAR(seg, param_far->iface_path, iface_path);
    if (t13) {
        SET_FARVAR(seg, param_far->device_path.t13.generic.reserved0
                   , GET_LOW(dp.t13.generic.reserved0));
        SET_FARVAR(seg, param_far->device_path.t13.generic.reserved1
                   , GET_LOW(dp.t13.generic.reserved1));

        SET_FARVAR(seg, param_far->device_path.t13.generic.checksum
                   , -checksum_far(seg, (void*)param_far+30, 43));
    } else {
        SET_FARVAR(seg, param_far->device_path.phoenix.device_path
                   , GET_LOW(dp.phoenix.device_path));

        SET_FARVAR(seg, param_far->device_path.phoenix.checksum
                   , -checksum_far(seg, (void*)param_far+30, 35));
    }

    return DISK_RET_SUCCESS;
}

// Build an EDD "iface_path" field for a PCI device
static u32
edd_pci_path(u16 bdf, u8 channel)
{
    return (pci_bdf_to_bus(bdf) | (pci_bdf_to_dev(bdf) << 8)
            | (pci_bdf_to_fn(bdf) << 16) | ((u32)channel << 24));
}

struct dpte_s DefaultDPTE VARLOW;
union device_path_u dp VARLOW;

static int
fill_ahci_edd(struct segoff_s edd, struct drive_s *drive_gf)
{
    if (!CONFIG_AHCI)
        return DISK_RET_EPARAM;

    // Fill in dpte
    struct ahci_port_s *port_gf = container_of(
        drive_gf, struct ahci_port_s, drive);
    struct ahci_ctrl_s *ctrl_gf = GET_GLOBALFLAT(port_gf->ctrl);
    u32 bdf = GET_GLOBALFLAT(ctrl_gf->pci_bdf);
    u32 bustype = EDD_PCI | EDD_SATA;
    u32 ifpath = ifpath = edd_pci_path(bdf, 0xff);
    u16 options = 0;

    SET_LOW(dp.t13.generic.reserved0, 0);
    SET_LOW(dp.t13.generic.reserved1, 0);

    SET_LOW(dp.t13.sata.port, GET_GLOBALFLAT(port_gf->pnr));
    SET_LOW(dp.t13.sata.pmp, 0);

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

    SET_LOW(DefaultDPTE.iobase1, GET_GLOBALFLAT(ctrl_gf->iobase));
    SET_LOW(DefaultDPTE.iobase2, 0);
    SET_LOW(DefaultDPTE.prefix, 0);
    SET_LOW(DefaultDPTE.unused, 0);
    SET_LOW(DefaultDPTE.irq, GET_GLOBALFLAT(ctrl_gf->irq));
    SET_LOW(DefaultDPTE.blkcount, 1);
    SET_LOW(DefaultDPTE.dma, 1);
    SET_LOW(DefaultDPTE.pio, 0);
    SET_LOW(DefaultDPTE.options, options);
    SET_LOW(DefaultDPTE.reserved, 0);

    return fill_generic_edd(edd, drive_gf, &DefaultDPTE, bustype, ifpath, dp);
}

// EDD info for ATA and ATAPI drives
static int
fill_ata_edd(struct segoff_s edd, struct drive_s *drive_gf)
{
    if (!CONFIG_ATA)
        return DISK_RET_EPARAM;

    // Fill in dpte
    struct atadrive_s *adrive_gf = container_of(
        drive_gf, struct atadrive_s, drive);
    struct ata_channel_s *chan_gf = GET_GLOBALFLAT(adrive_gf->chan_gf);
    u8 slave = GET_GLOBALFLAT(adrive_gf->slave);
    u16 iobase2 = GET_GLOBALFLAT(chan_gf->iobase2);
    u8 irq = GET_GLOBALFLAT(chan_gf->irq);
    u16 iobase1 = GET_GLOBALFLAT(chan_gf->iobase1);
    int bdf = GET_GLOBALFLAT(chan_gf->pci_bdf);
    u8 channel = GET_GLOBALFLAT(chan_gf->chanid);
    u32 bustype = EDD_ISA | EDD_ATA;
    u32 ifpath = 0;
    u16 options = 0;

    bdf = GET_GLOBALFLAT(drive_gf->cntl_id);

    SET_LOW(dp.t13.generic.reserved0, 0);
    SET_LOW(dp.t13.generic.reserved1, 0);

    SET_LOW(dp.t13.ata.device, channel);

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

    if (bdf != -1) {
        bustype &= ~EDD_ISA;
        bustype |= EDD_PCI;

        ifpath = edd_pci_path(bdf, slave);
    } else {
        ifpath = iobase1;
    }

    options |= 1<<4; // lba translation
    if (CONFIG_ATA_PIO32)
        options |= 1<<7; // 32-bit transfers

    SET_LOW(DefaultDPTE.iobase1, iobase1);
    SET_LOW(DefaultDPTE.iobase2, iobase2 + ATA_CB_DC);
    SET_LOW(DefaultDPTE.prefix, ((slave ? ATA_CB_DH_DEV1 : ATA_CB_DH_DEV0)
                                 | ATA_CB_DH_LBA));
    SET_LOW(DefaultDPTE.unused, 0xcb);
    SET_LOW(DefaultDPTE.irq, irq);
    SET_LOW(DefaultDPTE.blkcount, 1);
    SET_LOW(DefaultDPTE.dma, 0);
    SET_LOW(DefaultDPTE.pio, 0);
    SET_LOW(DefaultDPTE.options, options);
    SET_LOW(DefaultDPTE.reserved, 0);

    return fill_generic_edd(edd, drive_gf, &DefaultDPTE, bustype, ifpath, dp);
}

static int
fill_scsi_edd(struct segoff_s edd, struct drive_s *drive_gf
              ,u32 iobase , u16 target, u32 lun)
{
    SET_LOW(dp.t13.generic.reserved0, 0);
    SET_LOW(dp.t13.generic.reserved1, 0);
    u32 bustype = EDD_PCI | EDD_SCSI;
    u32 ifpath = edd_pci_path(GET_GLOBALFLAT(drive_gf->cntl_id), 0xff);
    u16 options = 0;
    u8 translation = GET_GLOBALFLAT(drive_gf->translation);

    if (translation != TRANSLATION_NONE) {
        options |= 1<<3; // CHS translation
        if (translation == TRANSLATION_LBA)
            options |= 1<<9;
        if (translation == TRANSLATION_RECHS)
            options |= 3<<9;
    }

    options |= 1<<4; // lba translation
    options |= 1<<7; // 32-bit transfer mode

    SET_LOW(dp.t13.scsi.id, target);
    SET_LOW(dp.t13.scsi.lun, lun);

    SET_LOW(DefaultDPTE.iobase1, iobase);
    SET_LOW(DefaultDPTE.iobase2, 0);
    SET_LOW(DefaultDPTE.prefix, 0);
    SET_LOW(DefaultDPTE.unused, 0);
    SET_LOW(DefaultDPTE.irq, 0);
    SET_LOW(DefaultDPTE.blkcount, 1);
    SET_LOW(DefaultDPTE.dma, 0);
    SET_LOW(DefaultDPTE.pio, 0);
    SET_LOW(DefaultDPTE.options, options);
    SET_LOW(DefaultDPTE.reserved, 0);

    return fill_generic_edd(edd, drive_gf, &DefaultDPTE, bustype, ifpath, dp);
}

// Fill Extended Disk Drive (EDD) "Get drive parameters" info for a drive
int
fill_edd(struct segoff_s edd, struct drive_s *drive_gf)
{
    int ret;
    u32 iobase = 0;
    void *iobasep = NULL;
    u16 target = 0;
    u32 lun = 0;

    switch (GET_GLOBALFLAT(drive_gf->type)) {
    case DTYPE_ATA:
    case DTYPE_ATA_ATAPI:
        return fill_ata_edd(edd, drive_gf);
    case DTYPE_AHCI:
    case DTYPE_AHCI_ATAPI:
        return fill_ahci_edd(edd, drive_gf);
    case DTYPE_VIRTIO_BLK:
        ret = virtio_blk_get_device_parameters(drive_gf, &iobase, &target
                                               , &lun);
        if (ret != DISK_RET_SUCCESS)
            return ret;
        return fill_scsi_edd(edd, drive_gf, iobase, target, lun);
    case DTYPE_VIRTIO_SCSI:
        ret = virtio_scsi_get_device_parameters(drive_gf, &iobase, &target
                                                , &lun);
        if (ret != DISK_RET_SUCCESS)
            return ret;
        return fill_scsi_edd(edd, drive_gf, iobase, target, lun);
    case DTYPE_LSI_SCSI:
        ret = lsi_scsi_get_device_parameters(drive_gf, &iobase, &target, &lun);
        if (ret != DISK_RET_SUCCESS)
            return ret;
        return fill_scsi_edd(edd, drive_gf, iobase, target, lun);
    case DTYPE_ESP_SCSI:
        ret = esp_scsi_get_device_parameters(drive_gf, &iobase, &target, &lun);
        if (ret != DISK_RET_SUCCESS)
            return ret;
        return fill_scsi_edd(edd, drive_gf, iobase, target, lun);
    case DTYPE_MEGASAS:
        ret = megasas_get_device_parameters(drive_gf, &iobase, &target, &lun);
        if (ret != DISK_RET_SUCCESS)
            return ret;
        return fill_scsi_edd(edd, drive_gf, iobase, target, lun);
    default:
        SET_LOW(dp.t13.generic.reserved0, 0);
        SET_LOW(dp.t13.generic.reserved1, 0);

        return fill_generic_edd(edd, drive_gf, 0, 0, 0, dp);
    }
}

/****************************************************************
 * 16bit calling interface
 ****************************************************************/

int VISIBLE32FLAT
process_atapi_op(struct disk_op_s *op)
{
    switch (op->command) {
    case CMD_WRITE:
    case CMD_FORMAT:
        return DISK_RET_EWRITEPROTECT;
    default:
        return scsi_process_op(op);
    }
}

// Execute a disk_op request.
int
process_op(struct disk_op_s *op)
{
    ASSERT16();
    int ret, origcount = op->count;
    if (origcount * GET_GLOBALFLAT(op->drive_gf->blksize) > 64*1024) {
        op->count = 0;
        return DISK_RET_EBOUNDARY;
    }
    u8 type = GET_GLOBALFLAT(op->drive_gf->type);
    switch (type) {
    case DTYPE_FLOPPY:
        ret = process_floppy_op(op);
        break;
    case DTYPE_ATA:
        ret = process_ata_op(op);
        break;
    case DTYPE_RAMDISK:
        ret = process_ramdisk_op(op);
        break;
    case DTYPE_CDEMU:
        ret = process_cdemu_op(op);
        break;
    case DTYPE_VIRTIO_BLK:
        ret = process_virtio_blk_op(op);
        break;
    case DTYPE_AHCI: ;
        extern void _cfunc32flat_process_ahci_op(void);
        ret = call32(_cfunc32flat_process_ahci_op
                     , (u32)MAKE_FLATPTR(GET_SEG(SS), op), DISK_RET_EPARAM);
        break;
    case DTYPE_ATA_ATAPI:
        ret = process_atapi_op(op);
        break;
    case DTYPE_AHCI_ATAPI: ;
        extern void _cfunc32flat_process_atapi_op(void);
        ret = call32(_cfunc32flat_process_atapi_op
                     , (u32)MAKE_FLATPTR(GET_SEG(SS), op), DISK_RET_EPARAM);
        break;
    case DTYPE_SDCARD: ;
        extern void _cfunc32flat_process_sdcard_op(void);
        ret = call32(_cfunc32flat_process_sdcard_op
                     , (u32)MAKE_FLATPTR(GET_SEG(SS), op), DISK_RET_EPARAM);
        break;
    case DTYPE_USB:
    case DTYPE_UAS:
    case DTYPE_VIRTIO_SCSI:
    case DTYPE_LSI_SCSI:
    case DTYPE_ESP_SCSI:
    case DTYPE_MEGASAS:
        ret = scsi_process_op(op);
        break;
    case DTYPE_USB_32:
    case DTYPE_UAS_32:
    case DTYPE_PVSCSI: ;
        extern void _cfunc32flat_scsi_process_op(void);
        ret = call32(_cfunc32flat_scsi_process_op
                     , (u32)MAKE_FLATPTR(GET_SEG(SS), op), DISK_RET_EPARAM);
        break;
    default:
        ret = DISK_RET_EPARAM;
        break;
    }
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
