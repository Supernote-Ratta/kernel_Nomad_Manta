#ifndef __FF_SPIDEV_H__
#define __FF_SPIDEV_H__

/* See spidev_ft.c for implementation. */
extern int  ft_spidev_init(void);
extern void ft_spidev_exit(void);
extern struct spi_device *g_spidev;

#endif