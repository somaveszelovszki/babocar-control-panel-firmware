#pragma once

#include <optional>

#include <micro/control/maneuver.hpp>
#include <micro/math/random_generator.hpp>

#include <LabyrinthGraph.hpp>
#include <LabyrinthRoute.hpp>

class LabyrinthNavigator : public micro::Maneuver {
public:
    LabyrinthNavigator(const LabyrinthGraph& graph, micro::irandom_generator& random);

    void initialize(
        const micro::set<Segment::Id, cfg::MAX_NUM_LABYRINTH_SEGMENTS>& unvisitedSegments,
        const Segment *startSeg,
        const Connection *prevConn,
        const Segment *laneChangeSeg,
        const micro::m_per_sec_t targetSpeed,
        const micro::m_per_sec_t targetDeadEndSpeed);

    const micro::Pose& correctedCarPose() const;

    void setForbiddenSegment(const Segment* segment);
    void navigateToLaneChange();

    void update(const micro::CarProps& car, const micro::LineInfo& lineInfo, micro::MainLine& mainLine, micro::ControlData& controlData) override;

private:
    const micro::LinePattern& frontLinePattern(const micro::LineInfo& lineInfo) const;
    const micro::LinePattern& rearLinePattern(const micro::LineInfo& lineInfo) const;

    const micro::Lines& frontLines(const micro::LineInfo& lineInfo) const;
    const micro::Lines& rearLines(const micro::LineInfo& lineInfo) const;

    void updateCarOrientation(const micro::CarProps& car, const micro::LineInfo& lineInfo);

    void handleJunction(const micro::CarProps& car, const micro::LinePattern::type_t patternType);

    void tryToggleTargetSpeedSign(const micro::meter_t currentDist, const char* reason);

    void setTargetLine(const micro::CarProps& car, const micro::LineInfo& lineInfo, micro::MainLine& mainLine) const;

    void setControl(const micro::CarProps& car, const micro::LineInfo& lineInfo, micro::MainLine& mainLine, micro::ControlData& controlData) const;

    void reset(const Junction& junc, micro::radian_t negOri);

    void createRoute();

    bool isDeadEnd(const micro::CarProps& car, const micro::LinePattern& pattern) const;

    bool isRestricted(const Junction& prevJunction, const Segment& segment) const;

    const Connection* randomConnection(const Junction& junc, const Segment& seg);

    micro::Direction randomDirection(const micro::LinePattern::type_t patternType);

    void stepToNextSegment(const Junction& junction);

    const Junction* findExpectedJunction() const;

    static bool isJunction(const micro::LinePattern& pattern);

    micro::m_per_sec_t targetSpeed_;
    micro::m_per_sec_t targetDeadEndSpeed_;
    const LabyrinthGraph& graph_;
    const Connection *prevConn_{nullptr};
    const Segment *currentSeg_{nullptr};
    const Segment *targetSeg_{nullptr};
    const Segment *laneChangeSeg_;
    LabyrinthRoute route_;
    micro::meter_t lastJuncDist_;
    micro::Direction targetDir_;
    micro::Sign targetSpeedSign_;
    micro::meter_t lastSpeedSignChangeDistance_;
    micro::LineInfo prevLineInfo_;
    micro::Pose correctedCarPose_;
    micro::meter_t lastOrientationUpdateDist_;
    micro::meter_t lastPosUpdateDist_;
    bool hasSpeedSignChanged_;
    bool isInJunction_;
    micro::irandom_generator& random_;
    micro::set<Segment::Id, cfg::MAX_NUM_LABYRINTH_SEGMENTS> unvisitedSegments_;
    JunctionIds forbiddenJunctions_;
};
