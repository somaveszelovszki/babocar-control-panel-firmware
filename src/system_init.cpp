#include <micro/panel/panelVersion.h>
#include <micro/utils/timer.hpp>

#include <cfg_board.h>
#include <system_init.h>

using namespace micro;

extern "C" void Error_Handler(void);

extern "C" void system_init(void) {
    if (PANEL_VERSION != panelVersion_get()) {
        Error_Handler();
    }

    time_init(tim_System);
}
