#ifndef BASIC_DEFINE_POSE2D_H
#define BASIC_DEFINE_POSE2D_H

#include <cmath>


class Pose2d {
public:
    //! Constructor which takes x- and y-coordinates.
    Pose2d(const float x, const float y, const float t) : x_(x), y_(y), theta_(t) {}

    //! Constructor returning the zero vector.
    Pose2d() : Pose2d(0, 0, 0) {}

    //! Getter for x component
    float x() const { return x_; }

    //! Getter for y component
    float y() const { return y_; }

    float theta() const { return theta_; }

    //! Setter for x component
    void set_x(const float x) { x_ = x; }

    //! Setter for y component
    void set_y(const float y) { y_ = y; }

    void set_theta(const float t) { theta_ = t; }

    //! Gets the length of the vector
    float Length() const { return hypot(x_, y_); }

    //! Gets the squared length of the vector
    float LengthSquare() const { return x_ * x_ + y_ * y_; }

    //! Returns the unit vector that is co-linear with this vector
    void Normalize()  {
        const float l = Length();
        const float kMathEpsilon = 1e-10;
        if (l > kMathEpsilon) {
            x_ /= l;
            y_ /= l;
        }
    }

    //! Returns the distance to the given vector
    float DistanceTo(const Pose2d &other) const  {
        return hypot(x_ - other.x_, y_ - other.y_);
    }

    //! Returns the squared distance to the given vector
    float DistanceSquareTo(const Pose2d &other) const  {
        const float dx = x_ - other.x_;
        const float dy = y_ - other.y_;
        return dx * dx + dy * dy;
    }

protected:
    float x_ = 0.0;
    float y_ = 0.0;
    float theta_ = 0.0;
};


#endif //BASIC_DEFINE_POSE2D_H
