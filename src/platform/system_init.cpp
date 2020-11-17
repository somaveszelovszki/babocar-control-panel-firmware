#include <micro/panel/panelVersion.hpp>
#include <micro/port/timer.hpp>

#include <cfg_board.hpp>
#include <system_init.h>

#include <FreeRTOS.h>
#include <task.h>

using namespace micro;

extern "C" void Error_Handler(void);

extern "C" void system_init(void) {
    if (PANEL_VERSION != getPanelVersion()) {
        Error_Handler();
    }

    time_init(timer_t{ tim_System });
}

void vApplicationStackOverflowHook(TaskHandle_t, char*) {
    Error_Handler();
}
