#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "lora_mesh.h"

void main(void)
{
    if (lora_mesh_init() < 0) {
        printk("Erro ao inicializar o rádio LoRa!\n");
        return;
    }

    printk("Node %04X ativo na rede Mesh\n", lora_get_my_address());

    while (1) {
        lora_mesh_send("Hello from mesh node!", 0x1002);
        k_sleep(K_SECONDS(5));
    }
}
