#include "lora_mesh.h"
#include <zephyr/sys/printk.h>
#include <string.h>
#include <stdlib.h>

/* Endereço único do nó (ajuste por dispositivo) */
static uint16_t my_addr = 0x1000;
static uint16_t msg_counter = 0;

struct packet_cache {
    uint32_t hash;
};

static struct packet_cache cache[CACHE_SIZE];
static size_t cache_index = 0;

K_FIFO_DEFINE(forward_fifo);

/* ---------- Funções utilitárias ---------- */
static void build_header(uint8_t *buf, uint16_t src, uint16_t dest, uint8_t freq_offset) {
    buf[0] = (dest >> 8) & 0xFF;
    buf[1] = dest & 0xFF;
    buf[2] = freq_offset;
    buf[3] = (src >> 8) & 0xFF;
    buf[4] = src & 0xFF;
    buf[5] = freq_offset;
}

static int parse_packet(const uint8_t *buf, size_t len, struct parsed_packet *out) {
    if (len < 6) return -1;
    out->dest_addr = (buf[0] << 8) | buf[1];
    out->freq_offset = buf[2];
    out->src_addr = (buf[3] << 8) | buf[4];
    out->payload_len = len - 6;
    if (out->payload_len > RX_BUF_SIZE) out->payload_len = RX_BUF_SIZE;
    memcpy(out->payload, &buf[6], out->payload_len);
    out->hash = 0;
    return 0;
}

static uint32_t packet_hash(struct parsed_packet *pkt) {
    if (pkt->hash != 0) return pkt->hash;
    return pkt->src_addr;
}

static int already_seen(uint32_t hash) {
    for (size_t i = 0; i < CACHE_SIZE; i++)
        if (cache[i].hash == hash) return 1;
    return 0;
}

static void cache_add(uint32_t hash) {
    cache[cache_index].hash = hash;
    cache_index = (cache_index + 1) % CACHE_SIZE;
}

/* ---------- Callback RX ---------- */
static void lora_rx_callback(const struct device *dev,
                             uint8_t *data,
                             uint16_t size,
                             int16_t rssi,
                             int8_t snr,
                             void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    struct parsed_packet *pkt = k_malloc(sizeof(struct parsed_packet));
    if (!pkt) return;

    if (parse_packet(data, size, pkt) != 0) {
        k_free(pkt);
        return;
    }

    uint32_t hash = packet_hash(pkt);
    if (already_seen(hash)) {
        k_free(pkt);
        return;
    }
    cache_add(hash);

    printk("[RX] %04X -> %04X | ", pkt->src_addr, pkt->dest_addr);
    for (size_t i = 0; i < pkt->payload_len; i++) printk("%c", pkt->payload[i]);
    printk(" | RSSI=%d SNR=%d\n", rssi, snr);

    if (pkt->dest_addr != my_addr && pkt->src_addr != my_addr) {
        k_fifo_put(&forward_fifo, pkt);
    } else {
        k_free(pkt);
    }
}

/* ---------- Thread de retransmissão ---------- */
static void forward_thread(void)
{
    const struct device *lora = DEVICE_DT_GET(DT_ALIAS(lora0));
    if (!device_is_ready(lora)) return;

    while (1) {
        struct parsed_packet *pkt = k_fifo_get(&forward_fifo, K_FOREVER);
        if (pkt) {
            uint8_t tx_buf[RX_BUF_SIZE];
            build_header(tx_buf, pkt->src_addr, pkt->dest_addr, pkt->freq_offset);
            memcpy(tx_buf + 6, pkt->payload, pkt->payload_len);

            lora_recv_async(lora, NULL, NULL);
            int err = lora_send(lora, tx_buf, pkt->payload_len + 6);
            lora_recv_async(lora, lora_rx_callback, NULL);

            if (err < 0)
                printk("[FORWARD FAILED] %d\n", err);
            else
                printk("[FORWARD] Packet forwarded %04X -> %04X\n", pkt->src_addr, pkt->dest_addr);

            k_free(pkt);
        }
    }
}

K_THREAD_DEFINE(forward_tid, 1024, forward_thread, NULL, NULL, NULL, 7, 0, 0);

/* ---------- Inicialização do rádio ---------- */
int lora_mesh_init(void)
{
    const struct device *lora = DEVICE_DT_GET(DT_ALIAS(lora0));
    if (!device_is_ready(lora)) {
        printk("LoRa device not found!\n");
        return -1;
    }

    struct lora_modem_config config = {
        .frequency = 915000000,
        .bandwidth = BW_125_KHZ,
        .datarate = SF_7,
        .coding_rate = CR_4_5,
        .preamble_len = 8,
        .tx_power = 22,
        .tx = true,
        .iq_inverted = false,
        .public_network = false,
    };

    if (lora_config(lora, &config) < 0) {
        printk("LoRa config failed!\n");
        return -2;
    }

    int err = lora_recv_async(lora, lora_rx_callback, NULL);
    if (err < 0) {
        printk("Failed to enable async RX: %d\n", err);
        return -3;
    }

    printk("LoRa Mesh initialized at 915 MHz (Node %04X)\n", my_addr);
    return 0;
}

/* ---------- Envio de mensagem ---------- */
int lora_mesh_send(const char *msg, uint16_t dest_addr)
{
    const struct device *lora = DEVICE_DT_GET(DT_ALIAS(lora0));
    if (!device_is_ready(lora)) return -1;

    uint8_t freq_offset = 0x08;
    uint8_t tx_buf[RX_BUF_SIZE];

    build_header(tx_buf, my_addr, dest_addr, freq_offset);
    memcpy(tx_buf + 6, msg, strlen(msg));

    uint32_t hash = ((uint32_t)msg_counter << 16) | my_addr;
    msg_counter++;

    lora_recv_async(lora, NULL, NULL);
    int err = lora_send(lora, tx_buf, strlen(msg) + 6);
    lora_recv_async(lora, lora_rx_callback, NULL);

    if (err < 0)
        printk("[TX FAILED] %d\n", err);
    else
        printk("[TX] Sent: %s | hash=%08X\n", msg, hash);

    return err;
}

/* ---------- Getter do endereço ---------- */
uint16_t lora_get_my_address(void) {
    return my_addr;
}
