#pragma once

#include <micro/utils/CarProps.hpp>

#include <LabyrinthGraph.hpp>
#include <LabyrinthRoute.hpp>

class LabyrinthNavigator {
public:
    LabyrinthNavigator(const LabyrinthGraph& graph, const Connection& prevConn, const Segment& currentSeg);

    const Segment* currentSegment() const;
    const Segment* targetSegment() const;
    const Connection* nextConnection() const;

    void setTargetSegment(const Segment& targetSeg);

    void onJunctionDetected(const micro::meter_t distance);

    micro::Direction update(const micro::meter_t distance);

    void reset(const Connection& prevConn, const Segment& currentSeg);

private:
    void updateTargetDirection();

    LabyrinthGraph graph_;
    const Connection *prevConn_;
    LabyrinthRoute plannedRoute_;
    micro::meter_t lastJuncDist_;
    micro::Direction targetDir_;
};
