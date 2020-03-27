#pragma once

#include <micro/utils/Line.hpp>
#include <micro/control/PD_Controller.hpp>

#include <utility>

namespace micro {

class LineController {
public:
    LineController(float front_P, float front_D, float front_outMin, float front_outMax,
        float rear_P, float rear_D, float rear_outMin, float rear_outMax);

    void setParams(float front_P, float front_D, float rear_P, float rear_D) {
        this->frontCtrl.setParams(front_P, front_D);
        this->rearCtrl.setParams(rear_P, rear_D);
    }

    /* @brief Gets output.
     * @returns The output.
     **/
    std::pair<radian_t, radian_t> getOutput() const {
        return { degree_t(this->frontCtrl.getOutput()), degree_t(this->rearCtrl.getOutput()) };
    }

    Status run(millimeter_t currentLinePos, radian_t currentLineAngle) {
        Status status;
        if (isOk((status = this->frontCtrl.run(static_cast<centimeter_t>(currentLinePos).get())))) {
            status = this->rearCtrl.run(static_cast<degree_t>(currentLineAngle).get());
        }
        return status;
    }

    millimeter_t desiredLinePos;
    radian_t desiredLineAngle;

private:
    PD_Controller frontCtrl;
    PD_Controller rearCtrl;
};

} // namespace micro
