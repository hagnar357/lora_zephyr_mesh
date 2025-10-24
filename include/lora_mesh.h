#ifndef LORA_MESH_H
#define LORA_MESH_H

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/lora.h>

/* --- Configurações --- */
#define RX_BUF_SIZE 255
#define CACHE_SIZE 50

/* --- Estruturas --- */
struct parsed_packet {
    uint16_t dest_addr;
    uint16_t src_addr;
    uint8_t freq_offset;
    uint8_t payload[RX_BUF_SIZE];
    size_t payload_len;
    uint32_t hash;
};

/* --- Funções públicas --- */

/**
 * @brief Inicializa o rádio LoRa e configura o callback assíncrono.
 * @return 0 em caso de sucesso, negativo em erro.
 */
int lora_mesh_init(void);

/**
 * @brief Envia um pacote LoRa com cabeçalho padrão de rede mesh.
 */
int lora_mesh_send(const char *msg, uint16_t dest_addr);

/**
 * @brief Retorna o endereço do nó atual.
 */
uint16_t lora_get_my_address(void);

#endif /* LORA_MESH_H */
