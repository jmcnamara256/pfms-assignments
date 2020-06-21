#ifndef POINT_H
#define POINT_H

/**
 * @brief 2D point data struct.
 */
struct Point
{
    int x;
    int y;

    // Equality operator is needed for graph functions
    bool operator==(const Point &p) const {return x == p.x && y == p.y;};
    bool operator!=(const Point &p) const {return x != p.x || y != p.y;};

    // Constructor for convenience
    Point() {};
    Point(int x_, int y_) : x(x_), y(y_) {};
};


namespace std
{
    /**
     * @brief Create a hash specialisation for Points.
     * 
     * Hashes the x nad y coord and combines them with some shifting.
     */
    template <>
    struct hash< Point >
    {
        size_t operator()( const Point& p ) const
        {
            size_t hx = hash<int>{}(p.x);
            size_t hy = hash<int>{}(p.y);
            
            hy ^= hx + 0x9e3779b9 + (hy << 6) + (hy >> 2);
            return hy;
        }
    };
} // std

#endif