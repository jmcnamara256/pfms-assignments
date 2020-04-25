#ifndef VECTOR2_H
#define VECTOR2_H

/**
 * @brief A 2D vector class.
 * @details Contains basic add and subtract functionality, as well as vector dot product and cross product methods.
 * 
 */
class Vector2 {
public:
    Vector2(double x_, double y_): x(x_), y(y_) {};
    Vector2() {};

    /**
     * @brief Piecewise add operation.
     * 
     * @param v 
     * @return Vector2 
     */
    Vector2 operator +(const Vector2& v) {
        return Vector2(x + v.x, y + v.y);
    }

    /**
     * @brief Piecewise subtract operation.
     * 
     * @param v 
     * @return Vector2 
     */
    Vector2 operator -(const Vector2& v) {
        return Vector2(x - v.x, y - v.y);
    }

    /**
     * @brief Assignment operator.
     * 
     * @param v 
     * @return Vector2& 
     */
    Vector2& operator =(const Vector2& v) {
        x = v.x;
        y = v.y;
        return *this;
    }

    /**
     * @brief Performs the dot-product of two vectors.
     * 
     * @param v1 
     * @param v2 
     * @return double 
     */
    static double dot(Vector2 v1, Vector2 v2) {
        return v1.x * v2.x + v1.y * v2.y;
    }


    /**
     * @brief Returns the 2D vector dot product of V1XV2
     * 
     * @param v1 Vector 1, LHS.
     * @param v2 Vector 2, RHS.
     * @return double 
     */
    static double cross(Vector2 v1, Vector2 v2) {
        return v1.x * v2.y - v1.y * v2.x;
    }

    /**
     * @brief Returns a zero vector.
     * 
     * @return Vector2 
     */
    static Vector2 zero() {
        Vector2 v(0,0);
        return v;
    }

    double x;
    double y;
};
#endif
