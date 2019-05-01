#pragma once

#include <uns/Point2.hpp>

namespace uns {

struct PosOri {
    /* @brief Position of the car relative to its start position.
    **/
    Point2<meter_t> pos_;

    /* @brief Car's orientation.
    @note Orientation is relative to the X axis!
     **/
    radian_t orientation_;
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
    CarProps(m_per_sec_t _speed, radian_t _steeringAngle, const Point2<meter_t>& _pos, radian_t _orientation)
        : speed_(_speed)
        , steeringAngle_(_steeringAngle)
        , pos_(_pos)
        , orientation_(_orientation) {}

    /* @brief Gets car's steering direction.
     * @returns The car's steering direction.
     **/
    RotationDir getSteeringDir() const {
        static constexpr radian_t EPS = degree_t(1.0f);
        return uns::isZero(this->steeringAngle_, EPS) ? RotationDir::CENTER : this->steeringAngle_ > radian_t::ZERO() ? RotationDir::LEFT : RotationDir::RIGHT;
    }

    m_per_sec_t speed() const { return this->speed_; };
    radian_t steeringAngle() const { return this->steeringAngle_; };
    Point2<meter_t> pos() const { return this->pos_; };
    radian_t orientation() const { return this->orientation_; };

    PosOri getPosOri() const { return PosOri{ this->pos_, this->orientation_ }; }

    void setPosOri(const PosOri& posOri) { this->pos_ = posOri.pos_; this->orientation_ = posOri.orientation_; }

    //TODO make these fields protected

    /* @brief The current speed of the car.
    **/
    m_per_sec_t speed_;

    /* @brief The current steering angle.
    @note Steering angle is relative to the Y axis!
     **/
    radian_t steeringAngle_;

    /* @brief Position of the car relative to its start position.
    **/
    Point2<meter_t> pos_;

    /* @brief Car's orientation.
    @note Orientation is relative to the X axis!
     **/
    radian_t orientation_;
};
} // namespace uns
