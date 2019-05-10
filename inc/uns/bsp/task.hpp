#pragma once

namespace uns {

typedef void* task_handle_t;    // Task handle - type is OS library-dependent.

enum class TASK {
    IDLE_TASK,
    CONTROL_TASK,
    DEBUG_TASK,
    SETUP_TASK
};

task_handle_t* getTaskHandle(TASK task);

void taskResume(task_handle_t *hTask);

void taskResumeAll();

void taskSuspend(task_handle_t *hTask);

void taskSuspendAll();

void taskDelete(task_handle_t *hTask);

void taskDeleteCurrent();

} // namespace uns
