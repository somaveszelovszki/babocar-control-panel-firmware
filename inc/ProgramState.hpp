#pragma once

#include <micro/utils/types.hpp>

namespace micro {

class ProgramState {
public:
    enum class ActiveModule {
        Labyrinth = 0,
        RaceTrack = 1,
        INVALID   = 2
    };

    ProgramState(ActiveModule module, uint8_t subCntr)
        : module_(module)
        , subCntr_(subCntr) {}

    ProgramState(const ProgramState& other) {
        this->module_ = other.module_;
        this->subCntr_ = other.subCntr_;
    }

    ActiveModule activeModule(void) const { return this->module_; }
    uint8_t subCntr(void) const { return this->subCntr_; }

    void set(ActiveModule module, uint8_t subCntr) {
        this->module_ = module;
        this->subCntr_ = subCntr;
    }

private:
    ActiveModule module_;
    uint8_t subCntr_;
};

} // namespace micro
