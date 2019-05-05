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

    /* @brief Casts point to another type.
     * @returns Point cast to another type.
     **/
    template <typename T2>
    operator Point2<T2>() const {
        return Point2<T2>(T2(this->X), T2(this->Y));
    }

    /* @brief Checks if two points are equal
     * @param other The other point.
     * @returns Boolean value indicating if the two points are equal.
     **/
    template <typename T2>
    bool operator==(const Point2<T2>& other) const {
        return this->X == other.X && this->Y == other.Y;
    }

    /* @brief Checks if two points are not equal
     * @param other The other point.
     * @returns Boolean value indicating if the two points are not equal.
     **/
    template <typename T2>
    bool operator!=(const Point2<T2>& other) const {
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
    template <typename T2>
    T distance(Point2<T2> other) const {
        return uns::pythag(this->X - other.X, this->Y - other.Y);
    }

    /* @brief Calculates angle of given vector using this point as the origo.
     * @param other The vector.
     **/
    template <typename T2>
    radian_t getAngle(const Vec2<T2>& other) const {
        return uns::atan2(other.Y - this->Y, other.X - this->X);
    }

    /* @brief Calculates weighted average of the two points.
     * @param other The other point.
     * @param otherWeight The weight of the other point relative to this point - 1 by default.
     * @returns The average of the two points.
     **/
    template <typename T2>
    Point2<T> average(const Point2<T2>& other, float32_t otherWeight = 1.0f) const {
        float32_t weightSum = 1.0f + otherWeight;
        return Point2<T>((this->X + other.X * otherWeight) / weightSum, (this->Y + other.Y * otherWeight) / weightSum);
    }

    // @note numPoints must be at least 1
    static void bbox(const Point2<T> points[], uint32_t numPoints, bbox2<T> *pResult);

    uns::Sign getAngleSign(const Vec2<T>& other, uns::Direction dir) const;

    bool isInside(const Point2<T>& a, const Point2<T>& b, const Point2<T>& c) const;

    bool isInside(const bbox2<T>& bbox) const;
};

/* @brief Adds coordinates of this and the other point.
* @param other The other point.
* @returns The result of the addition.
**/
template <typename T1, typename T2>
Point2<T1> operator+(const Point2<T1>& p1, const Point2<T2>& p2) {
    return Point2<T1>(p1.X + p2.X, p1.Y + p2.Y);
}

/* @brief Subtracts coordinates of the other point from the coordinates of this point.
 * @param other The other point.
 * @returns The result of the subtraction.
 **/
template <typename T1, typename T2>
Point2<T1> operator-(const Point2<T1>& p1, const Point2<T2>& p2) {
    return Point2<T1>(p1.X - p2.X, p1.Y - p2.Y);
}

/* @brief Adds coordinates of this and the other point and stores the result in this point.
 * @param other The other point.
 * @returns This point.
 **/
template <typename T1, typename T2>
Point2<T1>& operator+=(Point2<T1>& p1, const Point2<T2>& p2) {
    p1.X += p2.X;
    p1.Y += p2.Y;
    return p1;
}

/* @brief Subtracts coordinates of the other point from the coordinates of this point and stores the result in this point.
 * @param other The other point.
 * @returns This point.
 **/
template <typename T1, typename T2>
Point2<T1>& operator-=(Point2<T1>& p1, const Point2<T2>& p2) {
    p1.X -= p2.X;
    p1.Y -= p2.Y;
    return p1;
}

/* @brief Multiplies coordinates of the point with the given constant.
 * @param c The constant.
 * @returns The result of the multiplication.
 **/
template <typename T1, typename T2, typename R = decltype (std::declval<T1>() * std::declval<T2>())>
typename std::enable_if<!is_base_of_template<Point2, T2>::value, Point2<R>>::type operator*(const Point2<T1>& p, const T2& c) {
    return Point2<R>(p.X * c, p.Y * c);
}

/* @brief Divides coordinates of the point by the given constant.
 * @param c The constant.
 * @returns The result of the division.
 **/
template <typename T1, typename T2, typename R = decltype (std::declval<T1>() / std::declval<T2>())>
typename std::enable_if<!is_base_of_template<Point2, T2>::value, Point2<R>>::type operator/(const Point2<T1>& p, const T2& c) {
    return Point2<R>(p.X / c, p.Y / c);
}

/* @brief Multiplies coordinates of the point with the given constant and stores the result in the point.
 * @param c The constant.
 * @returns This point.
 **/
template <typename T1, typename T2>
typename std::enable_if<std::is_arithmetic<T2>::value, Point2<T1>&>::type operator*=(Point2<T1>& p, const T2& c) {
    p.X *= c;
    p.Y *= c;
    return p;
}

/* @brief Divides coordinates of the point by the given constant and stores the result in the point.
 * @param c The constant.
 * @returns This point.
 **/
template <typename T1, typename T2>
typename std::enable_if<std::is_arithmetic<T2>::value, Point2<T1>&>::type operator/=(Point2<T1>& p, const T2& c) {
    p.X /= c;
    p.Y /= c;
    return p;
}

/* @brief Multiplies coordinates of the point with the given constant.
 * @param c The constant.
 * @param p The point.
 * @returns The result of the multiplication.
 **/
template <typename T1, typename T2>
typename std::enable_if<std::is_arithmetic<T2>::value, Point2<T1>&>::type operator*=(const T2& c, const Point2<T1>& p) {
    return p * c;
}

/* @brief 2-dimensional bounding box.
 * @tparam T Numeric type of the coordinates.
 **/
template <typename T> struct bbox2 {
    Point2<T> bl, tr;   // Bottom left and top right points.
};

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
uns::Sign Point2<T>::getAngleSign(const Vec2<T>& other, uns::Direction dir) const {
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

    const uns::Direction dir = uns::Direction::LEFT;
    const uns::Sign sA = ab.getAngleSign(ap, dir), sB = bc.getAngleSign(bp, dir), sC = ca.getAngleSign(cp, dir);

    // sum can be +-3 when all values are either POSITIVE or NEGATIVE,
    return abs(static_cast<int8_t>(sA) + static_cast<int8_t>(sB) + static_cast<int8_t>(sC)) == 3;
}

typedef Point2<int32_t>     Point2i, Vec2i;         // 32-bit integer types.
typedef Point2<float32_t>   Point2f, Vec2f;         // 32-bit floating point types.

typedef Point2<meter_t>   Point2m, Vec2m;           // meter types.
typedef Point2<centimeter_t>   Point2cm, Vec2cm;    // centimeter types.
typedef Point2<m_per_sec_t>   Point2mps, Vec2mps;   // meter/sec types.

} // namespace uns

