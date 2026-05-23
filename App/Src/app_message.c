#include "app_message.h"

#include "app_baro.h"
#include "app_flash.h"
#include "app_messages.h"
#include "app_proto.h"
#include "app_tasks.h"

#define APP_MESSAGE_STARTUP_REPORT_ENABLED 0U

void APP_Message_Task_Init(void)
{
#if (APP_MESSAGE_STARTUP_REPORT_ENABLED != 0U)
    APP_Flash_ReportStartup();
    APP_Baro_ReportStartup();
#endif
}

void APP_Message_Task_Step(void)
{
    osDelay(100U);
}
