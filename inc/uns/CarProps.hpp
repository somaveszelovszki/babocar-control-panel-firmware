#pragma once

#include <uns/Point2.hpp>

namespace uns {

struct PosOri {
    /* @brief Position of the car relative to its start position.
    **/
    Point2<distance_t> pos_;

    /* @brief Car's orientation.
    @note Orientation is relative to the X axis!
     **/
    angle_t orientation_;
};

/* @brief Car properties.
 **/
class CarProps  {
public:
    /* @brief Default constructor - initializes fields.
     **/
    CarProps() : orientation_(PI_2) {}

    /* @brief Constructor - sets fields.
     * @param _speed The speed.
     * @param _steeringAngle The steering angle.
     * @param _pos The position.
     * @param _orientation The orientation.
     **/
    CarProps(speed_t _speed, angle_t _steeringAngle, const Point2<distance_t>& _pos, angle_t _orientation)
        : speed_(_speed)
        , steeringAngle_(_steeringAngle)
        , pos_(_pos)
        , orientation_(_orientation) {}

    /* @brief Gets car's steering direction.
     * @returns The car's steering direction.
     **/
    RotationDir getSteeringDir() const {
        static constexpr angle_t TOLERANCE(degrees(), 1.0f);
        return this->steeringAngle_.isZero(TOLERANCE) ? RotationDir::CENTER : this->steeringAngle_ > angle_t::ZERO() ? RotationDir::LEFT : RotationDir::RIGHT;
    }

    speed_t speed() const { return this->speed_; };
    angle_t steeringAngle() const { return this->steeringAngle_; };
    Point2<distance_t> pos() const { return this->pos_; };
    angle_t orientation() const { return this->orientation_; };

    PosOri getPosOri() const { return PosOri{ this->pos_, this->orientation_ }; }

    void setPosOri(const PosOri& posOri) { this->pos_ = posOri.pos_; this->orientation_ = posOri.orientation_; }

    //TODO make these fields protected

    /* @brief The current speed of the car.
    **/
    speed_t speed_;

    /* @brief The current steering angle.
    @note Steering angle is relative to the Y axis!
     **/
    angle_t steeringAngle_;

    /* @brief Position of the car relative to its start position.
    **/
    Point2<distance_t> pos_;

    /* @brief Car's orientation.
    @note Orientation is relative to the X axis!
     **/
    angle_t orientation_;
};
} // namespace uns
