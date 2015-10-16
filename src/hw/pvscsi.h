#ifndef _PVSCSI_H_
#define _PVSCSI_H_

struct disk_op_s;
int pvscsi_process_op(struct disk_op_s *op);
void pvscsi_setup(void);
int pvscsi_get_device_parameters(struct drive_s *drive_gf
                                 , void **iobase, u16 *target, u32 *lun);

#endif /* _PVSCSI_H_ */
