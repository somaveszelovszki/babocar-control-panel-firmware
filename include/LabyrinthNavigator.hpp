#pragma once

#include <optional>

#include <micro/control/maneuver.hpp>
#include <micro/math/random_generator.hpp>

#include <LabyrinthGraph.hpp>
#include <LabyrinthRoute.hpp>

class LabyrinthNavigator : public micro::Maneuver {
public:
    struct RestrictedSegments {
        const Segment* current{};
        const Segment* next{};
        micro::millisecond_t lastUpdateTime;

        RestrictedSegments& operator=(const RestrictedSegments&) = default;

        char prevJunction() const;
        char nextJunction() const;
    };

    LabyrinthNavigator(const LabyrinthGraph& graph, micro::irandom_generator& random);

    void initialize(
        const micro::set<Segment::Id, cfg::MAX_NUM_LABYRINTH_SEGMENTS>& unvisitedSegments,
        const Segment *startSeg,
        const Connection *prevConn,
        const Segment *laneChangeSeg,
        const Junction *lastJunctionBeforeLaneChange,
        const micro::m_per_sec_t targetSpeed,
        const micro::m_per_sec_t targetDeadEndSpeed);

    const micro::Pose& correctedCarPose() const;

    void setRestrictedSegments(const RestrictedSegments& restrictedSegments);
    void navigateToLaneChange();

    void update(const micro::CarProps& car, const micro::LineInfo& lineInfo, micro::MainLine& mainLine, micro::ControlData& controlData) override;

private:
    const micro::LinePattern& frontLinePattern(const micro::LineInfo& lineInfo) const;
    const micro::LinePattern& rearLinePattern(const micro::LineInfo& lineInfo) const;

    const micro::Lines& frontLines(const micro::LineInfo& lineInfo) const;
    const micro::Lines& rearLines(const micro::LineInfo& lineInfo) const;

    std::pair<bool, const char*> isSpeedSignChangeNeeded(const micro::CarProps& car, const micro::LinePattern& frontPattern) const;

    void updateCarOrientation(const micro::CarProps& car, const micro::LineInfo& lineInfo);

    void handleJunction(const micro::CarProps& car, const micro::LinePattern::type_t patternType);

    void setTargetLine(const micro::CarProps& car, const micro::LineInfo& lineInfo, micro::MainLine& mainLine) const;

    void setControl(const micro::CarProps& car, const micro::LineInfo& lineInfo, micro::MainLine& mainLine, micro::ControlData& controlData) const;

    void reset(const Junction& junc, micro::radian_t negOri);

    void createRoute();

    bool isDeadEnd(const micro::CarProps& car, const micro::LinePattern& pattern) const;

    const Connection* randomConnection(const Junction& junc, const Segment& seg);

    micro::Direction randomDirection(const micro::LinePattern::type_t patternType);

    void stepToNextSegment(const Junction& junction);

    const Junction* findExpectedJunction() const;

    static bool isJunction(const micro::LinePattern& pattern);

    micro::m_per_sec_t targetSpeed_;
    micro::m_per_sec_t targetDeadEndSpeed_;
    const LabyrinthGraph& graph_;
    const Connection *prevConn_{};
    const Segment *currentSeg_{};
    micro::millisecond_t currentSegStartTime_;
    const Segment *targetSeg_{};
    const Junction *lastJunctionBeforeTargetSeg_{};
    const Segment *laneChangeSeg_{};
    const Junction *lastJunctionBeforeLaneChange_{};
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
    RestrictedSegments restrictedSegments_;
};
