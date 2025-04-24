#include "../potrace/hj_mg_datas.h"

#include "../potrace/hj_mg_math.h"
#include <cmath>

namespace hj_mgs_implement_edge
{
    Point::Point()
    {

    }

    Point::Point(float vx, float vy)
        :x(vx), y(vy)
    {

    }

    Point Point::move(const Point& offset)
    {
        *this += offset;
        return *this;
    }

    float Point::angle() const
    {
        double atan = atan2(y, x);
        return float(fmod(N_PI + remainder(atan - N_PI, N_2PI), N_2PI));
    }

    float Point::length() const
    {
        return hypot(x, y);
    }

    Point Point::normalized()
    {
        float l = length();
   /*     if (l < 1.0e-10) return *this;*/
        return { x / l, y / l };
    }

    Point Point::normalized() const
    {
        float l = length();
    /*    if (l < 1.0e-10) return *this;*/
        return { x / l, y / l };
    }

    float Point::distanceTo(const Point& v) const
    {
        return (*this - v).length();
    }

    float Point::squared() const
    {
        return x * x + y * y;
    }

    Point Point::rotate(float angle)
    {
        float cosA = cos(angle);
        float sinA = sin(angle);
        float x0 = x * cosA - y * sinA;
        y = x * sinA + y * cosA;
        x = x0;
        return *this;
    }

    Point Point::rotate(const Point& center, float angle)
    {
        *this = center + (*this - center).rotate(angle);
        return *this;
    }

    Point Point::operator +(const Point& v) const
    {
        return { x + v.x, y + v.y };
    }

    Point Point::operator +(float d) const
    {
        return { x + d, y + d };
    }

    Point Point::operator -(const Point& v) const
    {
        return { x - v.x, y - v.y };
    }

    Point Point::operator -(float d) const
    {
        return { x - d, y - d };
    }

    Point Point::operator *(const Point& v) const
    {
        return { x * v.x, y * v.y };
    }

    Point Point::operator /(const Point& v) const
    {
        if (fabs(v.x) > 1.0e-3 && fabs(v.y) > 1.0e-3)
            return { x / v.x, y / v.y };
        return *this;
    }

    Point Point::operator *(float s) const
    {
        return { x * s, y * s };
    }

    Point Point::operator /(float s) const
    {
        if (fabs(s) > 1.0e-3)
            return { x / s, y / s };
        return *this;
    }

    Point Point::operator -() const
    {
        return { -x, -y };
    }

    Point Point::operator +=(const Point& v)
    {
        x += v.x;
        y += v.y;
        return *this;
    }

    Point Point::operator -=(const Point& v)
    {
        x -= v.x;
        y -= v.y;
        return *this;
    }

    Point Point::operator *=(const Point& v)
    {
        x *= v.x;
        y *= v.y;
        return *this;
    }

    Point Point::operator /=(const Point& v)
    {
        if (fabs(v.x) > 1.0e-3 && fabs(v.y) > 1.0e-3) {
            x /= v.x;
            y /= v.y;
        }
        return *this;
    }

    Point Point::operator *=(float s)
    {
        x *= s;
        y *= s;
        return *this;
    }

    Point Point::operator /=(float s)
    {
        if (fabs(s) > 1.0e-3) {
            x /= s;
            y /= s;
        }
        return *this;
    }

    bool Point::operator ==(const Point& v) const
    {
        return (fabs(x - v.x) < 0.001f && fabs(y - v.y) < 0.001f);
    }

    bool Point::operator < (const Point& v) const
    {
        return (fabs(x - v.x) < 1.0e-6) ? (y < v.y) : (x < v.x);
    }

    Line::Line()
    {

    }

    Line::Line(const Point& s, const Point& e)
        :sp(s), ep(e)
    {
        minV = Point(std::min(sp.x, ep.x),
            std::min(sp.y, ep.y));

        maxV = Point(std::max(sp.x, ep.x),
            std::max(sp.y, ep.y));
    }

    bool Line::operator ==(const Line& v) const
    {
        return sp == v.sp && ep == v.ep;
    }

    bool Line::operator !=(const Line& v) const
    {
        return !operator==(v);
    }

    AABB::AABB(const Polyline& poly)
    {
        float minX = N_FLT_MAX, minY = N_FLT_MAX;
        float maxX = N_MIN_VAL, maxY = N_MIN_VAL;
        for (size_t i = 0; i < poly.size(); i++)
        {
            minX = std::min(minX, poly[i].ep.x);
            minY = std::min(minY, poly[i].ep.y);
            maxX = std::max(maxX, poly[i].ep.x);
            maxY = std::max(maxY, poly[i].ep.y);
        }
        minPos = Point(minX, minY);
        maxPos = Point(maxX, maxY);
    }

    AABB::AABB(const vector<Point>& pos) {
        float minX = N_FLT_MAX, minY = N_FLT_MAX;
        float maxX = N_MIN_VAL, maxY = N_MIN_VAL;
        for (size_t i = 0; i < pos.size(); i++)
        {
            minX = min(minX, pos[i].x);
            minY = min(minY, pos[i].y);
            maxX = max(maxX, pos[i].x);
            maxY = max(maxY, pos[i].y);
        }
        minPos = Point(minX, minY);
        maxPos = Point(maxX, maxY);
    }
    float AABB::area() const
    {
        return (maxPos.x - minPos.x) * (maxPos.y - minPos.y);
    }
    bool AABB::is_contain(const AABB& child, float accuracy) const
    {
        return (child.minPos.x - minPos.x > accuracy) && (child.minPos.y - minPos.y > accuracy)
            && (maxPos.x - child.maxPos.x > accuracy) && (maxPos.y - child.maxPos.y > accuracy);
    }
    bool AABB::is_contain(const Point& pt, float accuracy) const
    {
        return (pt.x - minPos.x > accuracy) && (maxPos.x - pt.x > accuracy)
            && (pt.y - minPos.y > accuracy) && (maxPos.y - pt.y > accuracy);
    }
    bool AABB::is_intersect(const AABB& ref, float accuracy) const
    {
        return maxPos.x + accuracy >= ref.minPos.x &&
            maxPos.y + accuracy >= ref.minPos.y &&
            ref.maxPos.x + accuracy >= minPos.x &&
            ref.maxPos.y + accuracy >= minPos.y;
    }
}