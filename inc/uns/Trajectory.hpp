//#pragma once
//
//#include <config/cfg_track.hpp>
//#include <uns/CarProps.hpp>
//
//namespace uns {
///* @brief Calculates car trajectory from steering angle, and calculates object distance from this trajectory.
// **/
//class Trajectory {
//public:
//    /* @brief Stores information about the distance of an obstacle and the car track.
//     **/
//    class TrackDistance {
//    public:
//        /* @brief Distance between the obstacle and the car track (the side of the car).
//        If distance is negative, then is is in the track - the car will hit it eventually.
//         **/
//        distance_t dist;
//
//        /* @brief Direction of the obstacle relative to the car track middle.
//        Helps determining the steering direction in which it is easier to avoid the crash.
//         **/
//        uns::RotationDir dir;
//
//        /* @brief Remaining time before the car reaches the point of the trajectory where it is nearest to (or where it hits) the obstacle.
//         **/
//        millisecond_t remainingTime;
//    };
//
//private:
//    /* @brief Pointer to the structure storing the car properties (speed, steering angle, etc).
//     **/
//    CarProps *pCar;
//
//    /* @brief The current radius of the rear middle point of the car.
//     **/
//    distance_t R_rearMid;
//
//    /* @brief The current radius of the most inner point of the car.
//     **/
//    distance_t R_inner;
//
//    /* @brief The current radius of the most outer point of the car.
//     **/
//    distance_t R_outer;
//
//    /* @brief The current radius of the near front point of the car.
//     **/
//    distance_t R_frontNear;
//
//    /* @brief The current radius of the far rear point of the car.
//     **/
//    distance_t R_rearFar;
//
//    /* @brief Indicates if steering angle is so small that it is not worth calculating with,
//    but calculations can be done with 0.
//     **/
//    bool isNoSteering;
//
//    /* @brief Updates radiuses according to the steering angle.
//     **/
//    void updateRadiuses();
//
//public:
//    /* @brief Constructor - sets period time, initializes position and angle.
//     * @param _periodTime The period time.
//     * @param _pCar Pointer to the structure storing the car properties.
//     **/
//    Trajectory(int _periodTime, CarProps *_pCar)
//        : pCar(_pCar)
//        , isNoSteering(true) {}
//
//    /* @brief Updates speed, steering angle and then updates radiuses according to the steering angle.
//     * @param _speed The current speed of the car.
//     * @param _steeringAngle The current steering angle.
//     **/
//    void update(speed_t _speed, angle_t _steeringAngle);
//
//    /* @brief Calculates distance between given point and the car trajectory.
//     * @param relativePos Position of the obstacle relative to the car.
//     * @param forceCalcRemainingTime Indicates if remaining time should be calculated even if car will not hit the obstacle.
//     * @returns A TrackDistance object storing information about the distance between the obstacle and the trajectory.
//     **/
//    TrackDistance trackdistancePoint(const Point2<distance_t>& relativePos, bool forceCalcRemainingTime = false) const;
//};
//} // namespace uns
