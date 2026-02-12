#ifndef BAREMETAL_BOOTLOADER_STARTUP_H
#define BAREMETAL_BOOTLOADER_STARTUP_H

extern int _stext;
extern int _data;
extern int _etext;
extern int _edata;
extern int _sbss;
extern int _ebss;

void Reset_Handler(void);

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);

#endif //BAREMETAL_BOOTLOADER_STARTUP_H
