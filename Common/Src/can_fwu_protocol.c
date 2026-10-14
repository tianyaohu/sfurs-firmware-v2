#include "can_fwu_protocol.h"

void can_fwu_put_u32(uint8_t out[8], uint32_t v0, uint32_t v1)
{
  out[0] = (uint8_t)v0;
  out[1] = (uint8_t)(v0 >> 8);
  out[2] = (uint8_t)(v0 >> 16);
  out[3] = (uint8_t)(v0 >> 24);
  out[4] = (uint8_t)v1;
  out[5] = (uint8_t)(v1 >> 8);
  out[6] = (uint8_t)(v1 >> 16);
  out[7] = (uint8_t)(v1 >> 24);
}

void can_fwu_enter_pack(uint8_t out[8], uint32_t img_size)
{
  can_fwu_put_u32(out, CAN_FWU_MAGIC, img_size);
}

void can_fwu_erase_pack(uint8_t out[8], uint32_t start_page, uint32_t num_pages)
{
  can_fwu_put_u32(out, start_page, num_pages);
}

void can_fwu_data_pack(uint8_t out[8], uint16_t seq, const uint8_t chunk[6],
                       uint32_t chunk_len)
{
  unsigned i;
  out[0] = (uint8_t)seq;
  out[1] = (uint8_t)(seq >> 8);
  for (i = 0; i < CAN_FWU_CHUNK; i++) {
    out[2 + i] = (i < chunk_len && chunk != 0) ? chunk[i] : 0xFFU;
  }
}

void can_fwu_verify_pack(uint8_t out[8], uint32_t crc, uint32_t size)
{
  can_fwu_put_u32(out, crc, size);
}

void can_fwu_jump_pack(uint8_t out[8])
{
  can_fwu_put_u32(out, CAN_FWU_MAGIC, 0U);
}

void can_fwu_ack_pack(uint8_t out[8], uint32_t status, uint32_t detail)
{
  can_fwu_put_u32(out, status, detail);
}

uint32_t can_fwu_get_u32(const uint8_t in[8], unsigned word)
{
  const unsigned o = word ? 4U : 0U;
  return (uint32_t)in[o] | ((uint32_t)in[o + 1] << 8) |
         ((uint32_t)in[o + 2] << 16) | ((uint32_t)in[o + 3] << 24);
}

uint16_t can_fwu_data_seq(const uint8_t in[8])
{
  return (uint16_t)((uint16_t)in[0] | ((uint16_t)in[1] << 8));
}

uint32_t can_fwu_data_count(uint32_t img_size)
{
  return (img_size + CAN_FWU_CHUNK - 1U) / CAN_FWU_CHUNK;
}

uint32_t can_fwu_pages_for_size(uint32_t img_size)
{
  return (img_size + CAN_FWU_PAGE_SIZE - 1U) / CAN_FWU_PAGE_SIZE;
}

uint32_t can_fwu_crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
  size_t i;
  unsigned k;
  crc ^= 0xFFFFFFFFU;
  for (i = 0; i < len; i++) {
    crc ^= data[i];
    for (k = 0; k < 8; k++) {
      crc = (crc & 1U) ? (crc >> 1) ^ 0xEDB88320U : crc >> 1;
    }
  }
  return crc ^ 0xFFFFFFFFU;
}

uint32_t can_fwu_crc32(const uint8_t *data, size_t len)
{
  return can_fwu_crc32_update(0U, data, len);
}
