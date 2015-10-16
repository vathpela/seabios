#ifndef __LSI_SCSI_H
#define __LSI_SCSI_H

struct disk_op_s;
int lsi_scsi_process_op(struct disk_op_s *op);
void lsi_scsi_setup(void);
int lsi_scsi_get_device_parameters(struct drive_s *drive_gf
                                   , u32 *iobase, u16 *target, u32 *lun);

#endif /* __LSI_SCSI_H */
