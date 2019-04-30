#pragma once

#include <uns/util/unit_utils.hpp>

namespace uns {
/* @brief Template implementation for 2-dimensional points.
 * @tparam T Numeric type of the coordinates.
 **/
template <typename T> struct Point2;

template <typename T> using Vec2 = Point2<T>;

template <typename T> struct bbox2;

template <typename T> struct Point2 {
    /* @brief The X coordinate.
     **/
    T X;

    /* @brief The Y coordinate.
     **/
    T Y;

    /* @brief Constructor - does not initializes coordinates.
     **/
    Point2() {}

    /* @brief Constructor - initializes X and Y coordinates.
     * @param _X The X coordinate.
     * @param _Y The Y coordinate.
     **/
    Point2(const T& _X, const T& _Y)
        : X(_X)
        , Y(_Y) {}

    /* @brief Adds coordinates of this and the other point.
    * @param other The other point.
    * @returns The result of the addition.
    **/
    Point2<T> operator+(const Point2<T>& other) const {
        return Point2<T>(this->X + other.X, this->Y + other.Y);
    }

    /* @brief Subtracts coordinates of the other point from the coordinates of this point.
     * @param other The other point.
     * @returns The result of the subtraction.
     **/
    Point2<T> operator-(const Point2<T>& other) const {
        return Point2<T>(this->X - other.X, this->Y - other.Y);
    }

    /* @brief Adds coordinates of this and the other point and stores the result in this point.
     * @param other The other point.
     * @returns This point.
     **/
    Point2<T>& operator+=(const Point2<T>& other) {
        this->X += other.X;
        this->Y += other.Y;
        return *this;
    }

    /* @brief Subtracts coordinates of the other point from the coordinates of this point and stores the result in this point.
     * @param other The other point.
     * @returns This point.
     **/
    Point2<T>& operator-=(const Point2<T>& other) {
        this->X -= other.X;
        this->Y -= other.Y;
        return *this;
    }

    /* @brief Multiplies coordinates of the point with the given constant.
     * @param c The constant.
     * @returns The result of the multiplication.
     **/
    template <typename T2, class = typename std::enable_if<std::is_arithmetic<T2>::value>::type>
    Point2<T> operator*(const T2& c) const {
        return Point2<T>(this->X * c, this->Y * c);
    }

    /* @brief Divides coordinates of the point by the given constant.
     * @param c The constant.
     * @returns The result of the division.
     **/
    template <typename T2, class = typename std::enable_if<std::is_arithmetic<T2>::value>::type>
    Point2<T> operator/(const T2& c) const {
        return Point2<T>(this->X / c, this->Y / c);
    }

    /* @brief Multiplies coordinates of the point with the given constant and stores the result in the point.
     * @param c The constant.
     * @returns This point.
     **/
    template <typename T2, class = typename std::enable_if<std::is_arithmetic<T2>::value>::type>
    Point2<T>& operator*=(const T2& c) {
        this->X *= c;
        this->Y *= c;
        return *this;
    }

    /* @brief Divides coordinates of the point by the given constant and stores the result in the point.
     * @param c The constant.
     * @returns This point.
     **/
    template <typename T2, class = typename std::enable_if<std::is_arithmetic<T2>::value>::type>
    Point2<T>& operator/=(const T2& c) {
        this->X /= c;
        this->Y /= c;
        return *this;
    }

    /* @brief Multiplies coordinates of the point with the given constant.
     * @param c The constant.
     * @param p The point.
     * @returns The result of the multiplication.
     **/
    template <typename T2, class = typename std::enable_if<std::is_arithmetic<T2>::value>::type>
    friend Point2<T> operator*(const T2& c, const Point2<T>& p) {
        return p * c;
    }

    /* @brief Casts point to another type.
     * @returns Point cast to another type.
     **/
    template <typename T2>
    operator Point2<T2>() const {
        return Point2<T2>(static_cast<T>(this->X), static_cast<T>(this->Y));
    }

    /* @brief Checks if two points are equal
     * @param other The other point.
     * @returns Boolean value indicating if the two points are equal.
     **/
    bool operator==(const Point2<T>& other) const {
        return this->X == other.X && this->Y == other.Y;
    }

    /* @brief Checks if two points are not equal
     * @param other The other point.
     * @returns Boolean value indicating if the two points are not equal.
     **/
    bool operator!=(const Point2<T>& other) const {
        return this->X != other.X || this->Y != other.Y;
    }

    /* @brief Calculates length of the point vector.
     * @returns The length of the point vector.
     **/
    T length() const {
        return uns::pythag(this->X, this->Y);
    }

    /* @brief Calculates distance between the two points.
     * @param other The other point.
     * @returns The distance between the two points.
     **/
    T distance(Point2<T> other) const {
        return uns::pythag(this->X - other.X, this->Y - other.Y);
    }

    /* @brief Converts point to byte array.
     * @param The result byte array.
     **/
    void toBytes(uint8_t result[]) const;

    /* @brief Creates point from byte array.
     * @param bytes The source byte array.
     * @returns The point created from the byte array.
     **/
    static Point2<T> fromBytes(const uint8_t bytes[]);

    /* @brief Calculates steering angle of given vector using this point as the origo.
     * @param other The vector.
     * @param dir Indicates which is the positive steering direction.
     **/
    angle_t getAngle(const Vec2<T>& other, uns::RotationDir dir) const;

    /* @brief Calculates weighted average of the two points.
     * @param other The other point.
     * @param otherWeight The weight of the other point relative to this point - 1 by default.
     * @returns The average of the two points.
     **/
    Point2<T> average(const Point2<T>& other, float32_t otherWeight = 1.0f) const {
        float32_t weightSum = 1.0f + otherWeight;
        return Point2<T>((this->X + other.X * otherWeight) / weightSum, (this->Y + other.Y * otherWeight) / weightSum);
    }

    // @note numPoints must be at least 1
    static void bbox(const Point2<T> points[], uint32_t numPoints, bbox2<T> *pResult);

    uns::Sign getAngleSign(const Vec2<T>& other, uns::RotationDir dir) const;

    bool isInside(const Point2<T>& a, const Point2<T>& b, const Point2<T>& c) const;

    bool isInside(const bbox2<T>& bbox) const;
};

/* @brief 2-dimensional bounding box.
 * @tparam T Numeric type of the coordinates.
 **/
template <typename T> struct bbox2 {
    Point2<T> bl, tr;   // Bottom left and top right points.
};

template<typename T>
void Point2<T>::toBytes(uint8_t result[]) const {
    // maps X and Y coordinates to fit into a byte
    int _X = uns::incarcerate(static_cast<int>(this->X * 128 / /*ULTRA_MAX_DIST      !!!  TODO*/200), -128, 127),
        _Y = uns::incarcerate(static_cast<int>(this->Y * 128 / /*ULTRA_MAX_DIST*/200), -128, 127);

    result[0] = static_cast<uint8_t>(_X);
    result[1] = static_cast<uint8_t>(_Y);
}

template<typename T>
Point2<T> Point2<T>::fromBytes(const uint8_t bytes[]) {
    return Point2<T>(
        static_cast<T>(static_cast<int32_t>(bytes[0]) * /*ULTRA_MAX_DIST*/200 / 128.0f),
        static_cast<T>(static_cast<int32_t>(bytes[1]) * /*ULTRA_MAX_DIST*/200 / 128.0f)
    );
}

template <typename T>
angle_t Point2<T>::getAngle(const Vec2<T>& other, uns::RotationDir dir) const {
    angle_t angle;

    switch(dir) {
    case RotationDir::LEFT:
        if(uns::eq(this->X, other.X))
            angle = other.Y > this->Y ? PI_2 : 3 * PI_2;
        else if(other.X > this->X)
            angle = other.Y >= this->Y ? uns::atan((other.Y - this->Y) / (other.X - this->X)) : 2 * PI + uns::atan((other.Y - this->Y) / (other.X - this->X));
        else
            angle = PI + uns::atan((other.Y - this->Y) /(other.X - this->X));
        break;

    case RotationDir::RIGHT:
        angle = -1 * getAngle(Point2<T>(2 * this->X - other.X, other.Y), uns::RotationDir::LEFT);
        break;
    case RotationDir::CENTER:
        angle = angle_t::from<radians>(0.0f);
        break;
    }

    return angle;
}

template<typename T>
void Point2<T>::bbox(const Point2<T> points[], uint32_t numPoints, bbox2<T> *pResult) {
    pResult->bl = pResult->tr = points[0];
    for(uint32_t i = 1; i < numPoints; ++i) {
        Point2<T> p = points[i];

        if(p.X < pResult->bl->X)
            pResult->bl->X = p.X;
        else if(p.X > pResult->tr->X)
            pResult->tr->X = p.X;

        if(p.Y < pResult->bl->Y)
            pResult->bl->Y = p.Y;
        else if(p.Y > pResult->tr->Y)
            pResult->tr->Y = p.Y;
    }
}

template<typename T>
uns::Sign Point2<T>::getAngleSign(const Vec2<T>& other, uns::RotationDir dir) const {
    float32_t m;
    if(uns::isZero(this->X)) {
        m = static_cast<float32_t>(this->Y > 0 ? -other.X : other.X);
    } else if(uns::isZero(other.X)) {
        m = static_cast<float32_t>(other.Y > 0 ? this->X : -this->X);
    } else {
        m = this->Y / static_cast<float32_t>(this->X) - other.Y / static_cast<float32_t>(other.X);
    }

    return uns::sgn(m * static_cast<int8_t>(dir));
}

template<typename T>
bool Point2<T>::isInside(const bbox2<T>& bbox) const {
    return this->X >= bbox.bl.X && this->X < bbox.tr.X && this->Y >= bbox.bl.Y && this->Y < bbox.tr.Y;
}

template<typename T>
bool Point2<T>::isInside(const Point2<T>& a, const Point2<T>& b, const Point2<T>& c) const {
    const Vec2<T> ap = *this - a, bp = *this - b, cp = *this - c,
        ab = b - a, bc = c - b, ca = a - c;

    const uns::RotationDir dir = uns::RotationDir::LEFT;
    const uns::Sign sA = ab.getAngleSign(ap, dir), sB = bc.getAngleSign(bp, dir), sC = ca.getAngleSign(cp, dir);

    // sum can be +-3 when all values are either POSITIVE or NEGATIVE,
    return abs(static_cast<int8_t>(sA) + static_cast<int8_t>(sB) + static_cast<int8_t>(sC)) == 3;
}

typedef Point2<float32_t>   Point2f, Vec2f;         // 32-bit floating point types.
typedef Point2<uint8_t>     Point2ui8, Vec2ui8;     // 8-bit unsigned integer point types.

} // namespace uns
