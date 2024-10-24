#include <can.h>

#include "hal.h"
#include "hal_obj.h"
#include "mcp_can.h"

extern HAL_Handle halHandle;

void can_init(void) {}

uint8_t can_read(uint32_t *id, void *data, uint8_t *len) {
  int rs = 1;
  HAL_setupSpi_MCP2515(halHandle);
  if (MCP2515_checkReceive(halHandle->mcp2515Handle) != CAN_MSGAVAIL) {
    rs = 0;
    goto error;
  }

  if (MCP2515_readMsgBuf(halHandle->mcp2515Handle, len, (uint8_t *)data) !=
      CAN_OK) {
    rs = 0;
    goto error;
  }

  *id = MCP2515_getCanId(/* halHandle->mcp2515Handle */);

error:
  // HAL_setupSpiA(halHandle);
  return rs;
}

uint8_t can_read_N(uint32_t *id, void *data, uint8_t *len, uint8_t num) {
  int rs = 1;
  if (MCP2515_readMsgBufN(halHandle->mcp2515Handle, len, (uint8_t *)data,
                          num) != CAN_OK) {
    rs = 0;
    goto error;
  }
  *id = MCP2515_getCanId();
error:
  return rs;
}

uint8_t can_write(uint32_t id, void *data, uint8_t len) {
  // HAL_setupSpi_MCP2515(halHandle);
  int rs =
      MCP2515_sendMsgBuf(halHandle->mcp2515Handle, id, 1, len, (uint8_t *)data);
  // HAL_setupSpiA(halHandle);
  return rs == CAN_OK ? 1 : 0;
}

uint8_t can_writeN(uint32_t id, void *data, uint8_t len, uint8_t num) {
  int rs = MCP2515_sendMsgBufN(halHandle->mcp2515Handle, id, 1, len,
                               (uint8_t *)data, num);
  return rs == CAN_OK ? 1 : 0;
}
