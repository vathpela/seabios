#ifndef __ESP_SCSI_H
#define __ESP_SCSI_H

struct disk_op_s;
int esp_scsi_process_op(struct disk_op_s *op);
void esp_scsi_setup(void);
int esp_scsi_get_device_parameters(struct drive_s *drive_gf
                                   , u32 *iobase, u16 *target, u32 *lun);
#endif /* __ESP_SCSI_H */
