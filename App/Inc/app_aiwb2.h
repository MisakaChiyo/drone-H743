#ifndef APP_AIWB2_H
#define APP_AIWB2_H

#include <stdint.h>

typedef enum {
    APP_AIWB2_STATE_START_DELAY = 0,
    APP_AIWB2_STATE_WAIT_PROBE,
    APP_AIWB2_STATE_ESCAPE_BEFORE,
    APP_AIWB2_STATE_ESCAPE_AFTER,
    APP_AIWB2_STATE_SEND_COMMAND,
    APP_AIWB2_STATE_WAIT_COMMAND,
    APP_AIWB2_STATE_WAIT_BOOT_CONNECT,
    APP_AIWB2_STATE_WAIT_TRANSPARENT_OK,
    APP_AIWB2_STATE_TRANSPARENT,
    APP_AIWB2_STATE_SOCKET_READY,
    APP_AIWB2_STATE_RETRY_DELAY
} APP_AiWB2_State;

typedef enum {
    APP_AIWB2_LINK_UDP_SERVER = 1,
    APP_AIWB2_LINK_TCP_CLIENT = 4
} APP_AiWB2_LinkMode;

void APP_AiWB2_Init(void);
void APP_AiWB2_Tick(void);
void APP_AiWB2_ProcessLine(const char *line);
uint8_t APP_AiWB2_IsTransparent(void);
uint8_t APP_AiWB2_IsSocketReady(void);
uint32_t APP_AiWB2_GetSocketConId(void);
void APP_AiWB2_OnSocketSendPrompt(void);
void APP_AiWB2_OnSocketSendOkOrError(uint8_t ok);
uint8_t APP_AiWB2_TakeSocketSendPrompt(void);
int8_t APP_AiWB2_TakeSocketSendResult(void);
uint8_t APP_AiWB2_IsControlPayload(const char *line);
uint8_t APP_AiWB2_ShouldConsumeTransparentLine(const char *line);
void APP_AiWB2_AssumeTransparent(void);
uint8_t APP_AiWB2_StartProvision(const char *ssid,
                                 const char *password,
                                 APP_AiWB2_LinkMode mode,
                                 const char *host,
                                 const char *port);
uint8_t APP_AiWB2_StartSoftAp(const char *ssid,
                              const char *password,
                              const char *channel,
                              const char *port);
uint8_t APP_AiWB2_StartSocketConfig(APP_AiWB2_LinkMode mode,
                                    const char *host,
                                    const char *port);
uint8_t APP_AiWB2_SendRawCommand(const char *command);
void APP_AiWB2_SendDiagCommands(void);
APP_AiWB2_State APP_AiWB2_GetState(void);
uint32_t APP_AiWB2_GetRetryCount(void);
int32_t APP_AiWB2_GetLastSocketError(void);
uint8_t APP_AiWB2_IsPowerRecycleActive(void);
uint32_t APP_AiWB2_GetDeadlineRemainingMs(void);
uint8_t APP_AiWB2_IsProvisionActive(void);
uint32_t APP_AiWB2_GetCommandIndex(void);
uint32_t APP_AiWB2_GetCommandCount(void);
const char *APP_AiWB2_GetCurrentCommand(void);

#endif
