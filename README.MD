# PIC32 ATWILC1000 FreeRTOS+TCP driver

This project was born of a desire to use mainline ASF code, as well as have a more stable platform for wifi.  
This is not a compilable project, rather a collection of basic items neeeded to incorporate into your
code.  The wilc1000 driver code was pulled from xdk-asf-3.43.0.  

There are plenty of docs about incorporating FreeRTOS and FreeRTOS + TCP, so I won't go into that here.

The driver code is exactly as pulled from asf with two exceptions, which mirror fixes done on the pic32mz demo in Amazon FreeRTOS.
In m2m_wifi.c, lines 132 to 139, and the if0 at line 618

### Notes

This code operates the wilc1500 as well, as the boot process and internals must be the same 
between the 1000 and the 1500.

This example is set to use SPI4, and DMA channels 2 and 3. DMA code could be commented out in nm_bus_wrapper_pic32.c
nm_bus_ioctl is set to only dma on non cached to non cached > MIN_DMA_SPI_SIZE

It would appear that debug halts tend to break dma.

### Prerequisites

FreeRTOS 10
FreeRTOS + TCP
You may need Harmony 2.04 or greater, I haven't chased down every dependency.


### Installing

You will need to add these files to your project

src/NetworkInterface_wifi.c
src/wilc1000/bsp/source/nm_bsp_pic32.c
src/wilc1000/bus_wrapper/source/nm_bsp_pic32.c
src/wilc1000/common/source/nm_common.c
src/wilc1000/driver/source/m2m_coex.c
src/wilc1000/driver/source/m2m_hif.c
src/wilc1000/driver/source/m2m_periph.c
src/wilc1000/driver/source/m2m_wifi.c
src/wilc1000/driver/source/nmasic.c
src/wilc1000/driver/source/nmbus.c
src/wilc1000/driver/source/nmdrv.c
src/wilc1000/driver/source/nmdrv_crt.c
src/wilc1000/driver/source/nmi2c.c
src/wilc1000/driver/source/nmsdio.c
src/wilc1000/driver/source/nmspi.c
src/wilc1000/driver/source/nmuart.c
src/wilc1000/driver/source/spi_flash.c

You will need to provide an interrupt for the wilc interrupt pin, and configure nm_bsp_pic32.c and nm_bus_wrapper_pic32.c 
and headers to reflect the spi port and pins correctly.

Note that in 

## License

Mixed licensing, all free types, see individual files.

