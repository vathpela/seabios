#ifndef __MEGASAS_H
#define __MEGASAS_H

struct disk_op_s;
int megasas_cmd_data(struct disk_op_s *op, void *cdbcmd, u16 blocksize);
void megasas_setup(void);
int megasas_get_device_parameters(struct drive_s *drive_gf
                                  , u32 *iobase, u16 *target, u32 *lun);

#endif /* __MEGASAS_H */
