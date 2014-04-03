/*******************************************************************************
 *                 Eagle Technology - Cape Town, South Africa
 *             Copyright (C) 2004 - 2011 - All Rights Reserved!
 *                 www.eagledaq.com - eagle@eagle.co.za
 *******************************************************************************
 *
 * edredef.h -- EDR Enhanced DAQ Prototype defines
 *
 *******************************************************************************/

extern void __init edre_dev_init(void);
extern void __exit edre_dev_exit(void);
extern void edre_dev_prepare( struct edre_device *edre );
extern void edre_dev_add_device( struct edre_device *edre );
extern void edre_dev_del_device(struct edre_device *edre);

extern void __init edre_sysfs_init(struct class *edre_class);
extern void edre_sysfs_add_device(struct edre_device *edre);
extern void edre_sysfs_del_device(struct edre_device *edre);

extern void edre_proc_add_device(struct edre_device *edre);
void edre_proc_del_device(struct edre_device *edre);

/**
 * End of File
 */

