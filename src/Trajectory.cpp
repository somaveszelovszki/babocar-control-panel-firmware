//#include <cfg_car.hpp>
//#include <micro/Trajectory.hpp>
//#include <micro/math/unit_utils.hpp>
//
//using namespace micro;
//
//void Trajectory::updateRadiuses() {
//    float angle = pCar->steeringAngle.get<radians>();
//    if (!(isNoSteering = (abs(angle) <= 0.05f))) {
//        float tanAngle = micro::tan(pCar->steeringAngle);
//
//        int32_t steerMul = static_cast<int32_t>(pCar->getSteeringDir());
//
//        R_rearMid = cfg::CAR_PIVOT_DIST_FRONT_REAR / tanAngle;
//
//        R_outer = micro::pythag(R_rearMid + cfg::CAR_PIVOT_LENGTH * steerMul, cfg::CAR_PIVOT_DIST_FRONT_REAR + cfg::CAR_PIVOT_FRONT_DIST) * steerMul;
//        R_inner = R_rearMid - cfg::CAR_PIVOT_LENGTH * steerMul;
//        R_rearFar = R_rearMid + cfg::CAR_PIVOT_LENGTH * steerMul;
//        R_frontNear = micro::pythag(R_inner, cfg::CAR_PIVOT_DIST_FRONT_REAR) * steerMul;
//    } else
//        R_rearMid = R_outer = R_inner = R_rearFar = R_frontNear = millimeter_t(0);
//}
//
//void Trajectory::update(speed_t _speed, angle_t _steeringAngle) {
//    millisecond_t d_time = millisecond_t(0);
//
//    pCar->speed = _speed;
//
//    if (pCar->steeringAngle != _steeringAngle) {
//        pCar->steeringAngle = _steeringAngle;
//        updateRadiuses();
//    }
//
//    if (!isNoSteering) {
//        pCar->orientation += radian_t(d_time * pCar->speed / R_rearFar);
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
//    pCar->pos.X += pCar->speed * micro::cos(pCar->orientation) * d_time;
//    pCar->pos.Y += pCar->speed * micro::sin(pCar->orientation) * d_time;
//}
//
//Trajectory::TrackDistance Trajectory::trackdistancePoint(const point2<distance_t>& relativePos, bool forceCalcRemainingTime) const {
//    TrackDistance td;
//
//    if (isNoSteering) {
//        distance_t innerDist = abs(-cfg::CAR_PIVOT_LENGTH - relativePos.X),
//            outerDist = abs(cfg::CAR_PIVOT_LENGTH - relativePos.X);
//
//        // checks if car hits obstacle - if yes, minimum distance is negative
//        td.dist = (micro::isBtw(relativePos.X, -cfg::CAR_PIVOT_LENGTH, cfg::CAR_PIVOT_LENGTH) ? -1 : 1) * min(innerDist, outerDist);
//        td.dir = innerDist <= outerDist ? micro::RotationDir::LEFT : micro::RotationDir::RIGHT;
//
//        // calculates remaining time
//        if(forceCalcRemainingTime) {
//            td.remainingTime = (relativePos.Y - (cfg::CAR_LENGTH / 2) * (relativePos.Y.get<millimeters>() > 0 ? 1 : -1)) / pCar->speed;
//        }
//    } else {
//        // origo    center of the trajectory circle of the rear pivot's center
//        // obs        position of the obstacle relative to the rear pivot's center
//        point2<distance_t> origo(-R_rearMid, millimeter_t(0)), obs;
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
//        td.dist = (micro::isBtw(obsDist, abs_R_inner, abs_R_outer) ? -1 : 1) * min(innerDist, outerDist);
//        td.dir = static_cast<micro::RotationDir>((innerDist < outerDist ? 1 : -1) * static_cast<int32_t>(pCar->getSteeringDir()));
//
//        // calculates remaining time
//        if (forceCalcRemainingTime) {
//
//            // delta angle on the trajectory
//            // -> the given part of the car that hits the obstacle will reach it before the rear pivot does
//            // this angle specifies this difference
//            angle_t dAngle = radian_t(0.0f);
//
//            angle_t hitAngle = obsAngle - dAngle;
//            td.remainingTime = R_rearFar * hitAngle / pCar->speed;
//        }
//    }
//
//    return td;
//}
