//#include <config/cfg_car.hpp>
//#include <uns/Trajectory.hpp>
//#include <uns/util/unit_utils.hpp>
//
//using namespace uns;
//
//void Trajectory::updateRadiuses() {
//    float32_t angle = pCar->steeringAngle.get<radians>();
//    if (!(isNoSteering = (abs(angle) <= 0.05f))) {
//        float32_t tanAngle = uns::tan(pCar->steeringAngle);
//
//        int32_t steerMul = static_cast<int32_t>(pCar->getSteeringDir());
//
//        R_rearMid = cfg::CAR_PIVOT_DIST_FRONT_REAR / tanAngle;
//
//        R_outer = uns::pythag(R_rearMid + cfg::CAR_PIVOT_LENGTH * steerMul, cfg::CAR_PIVOT_DIST_FRONT_REAR + cfg::CAR_PIVOT_FRONT_DIST) * steerMul;
//        R_inner = R_rearMid - cfg::CAR_PIVOT_LENGTH * steerMul;
//        R_rearFar = R_rearMid + cfg::CAR_PIVOT_LENGTH * steerMul;
//        R_frontNear = uns::pythag(R_inner, cfg::CAR_PIVOT_DIST_FRONT_REAR) * steerMul;
//    } else
//        R_rearMid = R_outer = R_inner = R_rearFar = R_frontNear = distance_t::from<millimeters>(0);
//}
//
//void Trajectory::update(speed_t _speed, angle_t _steeringAngle) {
//    uns::time_t d_time = uns::time_t::from<milliseconds>(0);
//
//    pCar->speed = _speed;
//
//    if (pCar->steeringAngle != _steeringAngle) {
//        pCar->steeringAngle = _steeringAngle;
//        updateRadiuses();
//    }
//
//    if (!isNoSteering) {
//        pCar->orientation += angle_t::from<radians>(d_time * pCar->speed / R_rearFar);
//
//        // normalizes angle to the [0, 2*PI) interval
//        if (pCar->orientation >= 2 * PI)
//            pCar->orientation -= 2 * PI;
//
//        if (pCar->orientation.get<radians>() < 0.0f)
//            pCar->orientation += 2 * PI;
//    } else {
//
//    }
//
//    pCar->pos.X += pCar->speed * uns::cos(pCar->orientation) * d_time;
//    pCar->pos.Y += pCar->speed * uns::sin(pCar->orientation) * d_time;
//}
//
//Trajectory::TrackDistance Trajectory::trackdistancePoint(const Point2<distance_t>& relativePos, bool forceCalcRemainingTime) const {
//    TrackDistance td;
//
//    if (isNoSteering) {
//        distance_t innerDist = abs(-cfg::CAR_PIVOT_LENGTH - relativePos.X),
//            outerDist = abs(cfg::CAR_PIVOT_LENGTH - relativePos.X);
//
//        // checks if car hits obstacle - if yes, minimum distance is negative
//        td.dist = (uns::isBtw(relativePos.X, -cfg::CAR_PIVOT_LENGTH, cfg::CAR_PIVOT_LENGTH) ? -1 : 1) * min(innerDist, outerDist);
//        td.dir = innerDist <= outerDist ? uns::RotationDir::LEFT : uns::RotationDir::RIGHT;
//
//        // calculates remaining time
//        if(forceCalcRemainingTime) {
//            td.remainingTime = (relativePos.Y - (cfg::CAR_LENGTH / 2) * (relativePos.Y.get<millimeters>() > 0 ? 1 : -1)) / pCar->speed;
//        }
//    } else {
//        // origo    center of the trajectory circle of the rear pivot's center
//        // obs        position of the obstacle relative to the rear pivot's center
//        Point2<distance_t> origo(-R_rearMid, distance_t::from<millimeters>(0)), obs;
//
//        obs.X = relativePos.X + R_rearMid;
//        obs.Y = relativePos.Y + cfg::CAR_PIVOT_DIST_MID;
//
//        angle_t obsAngle = origo.getAngle(obs, pCar->getSteeringDir());
//
//        distance_t abs_R_inner = abs(R_inner), abs_R_outer = abs(R_outer);
//
//        distance_t obsDist = obs.distance(origo),
//            innerDist = abs(obsDist - abs_R_inner),
//            outerDist = abs(obsDist - abs_R_outer);
//
//        // checks if car hits obstacle - if yes, minimum distance is negative
//        td.dist = (uns::isBtw(obsDist, abs_R_inner, abs_R_outer) ? -1 : 1) * min(innerDist, outerDist);
//        td.dir = static_cast<uns::RotationDir>((innerDist < outerDist ? 1 : -1) * static_cast<int32_t>(pCar->getSteeringDir()));
//
//        // calculates remaining time
//        if (forceCalcRemainingTime) {
//
//            // delta angle on the trajectory
//            // -> the given part of the car that hits the obstacle will reach it before the rear pivot does
//            // this angle specifies this difference
//            angle_t dAngle = angle_t::from<radians>(0.0f);
//
//            angle_t hitAngle = obsAngle - dAngle;
//            td.remainingTime = R_rearFar * hitAngle / pCar->speed;
//        }
//    }
//
//    return td;
//}
