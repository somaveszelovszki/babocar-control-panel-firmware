#pragma once

#include <micro/control/maneuver.hpp>
#include <micro/math/random_generator.hpp>

#include <LabyrinthGraph.hpp>
#include <LabyrinthRoute.hpp>

class LabyrinthNavigator : public micro::Maneuver {
public:
    LabyrinthNavigator(const LabyrinthGraph& graph, const Segment *startSeg, const Connection *prevConn,
        const micro::m_per_sec_t targetSpeed, const micro::m_per_sec_t targetFastSpeed);

    void initialize();

    const Segment* currentSegment() const;
    const Segment* targetSegment() const;
    const micro::Pose& correctedCarPose() const;

    bool isLastTarget() const;

    void setTargetSegment(const Segment *targetSeg, bool isLast);

    void update(const micro::CarProps& car, const micro::LineInfo& lineInfo, micro::MainLine& mainLine, micro::ControlData& controlData) override;

    bool isReducedLineScanEnabled() const;

private:
    const micro::LinePattern& frontLinePattern(const micro::LineInfo& lineInfo) const;
    const micro::LinePattern& rearLinePattern(const micro::LineInfo& lineInfo) const;

    const micro::Lines& frontLines(const micro::LineInfo& lineInfo) const;
    const micro::Lines& rearLines(const micro::LineInfo& lineInfo) const;

    void updateCarOrientation(const micro::CarProps& car, const micro::LineInfo& lineInfo);

    void handleJunction(const micro::CarProps& car, uint8_t numInSegments, uint8_t numOutSegments);

    void tryToggleTargetSpeedSign();

    void setTargetLine(const micro::CarProps& car, const micro::LineInfo& lineInfo, micro::MainLine& mainLine) const;

    void setControl(const micro::CarProps& car, const micro::LineInfo& lineInfo, micro::MainLine& mainLine, micro::ControlData& controlData) const;

    void reset(const Junction& junc, micro::radian_t negOri);

    void updateRoute();

    bool isTargetLineOverrideEnabled(const micro::CarProps& car, const micro::LineInfo& lineInfo) const;

    bool isDeadEnd(const micro::CarProps& car, const micro::LinePattern& pattern) const;

    static bool isJunction(const micro::LinePattern& pattern);

    static uint8_t numJunctionSegments(const micro::LinePattern& pattern);

    const micro::m_per_sec_t targetSpeed_;
    const micro::m_per_sec_t targetFastSpeed_;
    const LabyrinthGraph& graph_;
    const Segment *startSeg_;
    const Connection *prevConn_;
    const Segment *currentSeg_;
    const Segment *targetSeg_;
    LabyrinthRoute route_;
    bool isLastTarget_;
    micro::meter_t lastJuncDist_;
    micro::Direction targetDir_;
    micro::Sign targetSpeedSign_;
    bool isSpeedSignChangeInProgress_;
    micro::LineInfo prevLineInfo_;
    micro::Pose correctedCarPose_;
    micro::meter_t lastOrientationUpdateDist_;
    bool hasSpeedSignChanged_;
    bool isInJunction_;
    micro::random_generator random_;
};
