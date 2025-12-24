/* bootloader.c */
#include <inttypes.h>
#include "memory_map.h"

#define BOOTLOADER_SIZE 0x8000U
#define MAIN_APP_START_ADDRESS 0x08000000U + BOOTLOADER_SIZE

void NMI_Handler(void);
void HardFault_Handler(void);

// Breaks as it causes excedding of allocated size
// const uint8_t data[0x8000] = {0};

int main(void);

static void jump_to_application() {
    typedef void (*void_fn)(void);

    uint32_t *reset_vector_entry = (uint32_t *)(MAIN_APP_START_ADDRESS + 4U);
    uint32_t *reset_vector = (uint32_t *)(*reset_vector_entry);
    void_fn jump_fn = reset_vector;
    
    jump_fn();
}

unsigned int *vectors[] __attribute__((section(".vectors"))) = 
{
    (unsigned int *)0x20030000, //Pointer to the top of our stack memory
    (unsigned int *)main, // Pointer to our reset handler - also our startup code
    (unsigned int *)NMI_Handler, //NMI
    (unsigned int *)HardFault_Handler, //HardFault
};

int main(void) {
    jump_to_application();
    /* Not Reached */
    while (1) {}
}

void HardFault_Handler (void)
{
    #ifdef DEBUG
    __BKPT(0);
    #endif
    while (1)
    {
        /* code */
        
    }

}

void NMI_Handler (void)
{
    while (1)
    {
        /* code */
        
    }

}
