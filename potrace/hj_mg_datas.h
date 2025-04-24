#ifndef HJ_MG_DATAS
#define HJ_MG_DATAS

#include <vector>
#include <iostream>
#include <cmath>
#include <math.h>
#include <float.h>
#include <algorithm>
using namespace std;

namespace hj_mgs_implement_edge
{
	struct Point
	{
        Point();
        Point(float vx, float vy);

        Point move(const Point& offset);
        float angle() const;
        float length() const;
        Point normalized();
		Point normalized() const;
        float distanceTo(const Point& v) const;
        float squared() const;
        Point rotate(float angle);
        Point rotate(const Point& center, float angle);

        Point operator + (const Point& v) const;
        Point operator + (float d) const;
        Point operator - (const Point& v) const;
        Point operator - (float d) const;
        Point operator * (const Point& v) const;
        Point operator / (const Point& v) const;
        Point operator * (float s) const;
        Point operator / (float s) const;
        Point operator - () const;

        Point operator += (const Point& v);
        Point operator -= (const Point& v);
        Point operator *= (const Point& v);
        Point operator /= (const Point& v);
        Point operator *= (float s);
        Point operator /= (float s);

        bool operator < (const Point& v) const;
        bool operator == (const Point& v) const;
        bool operator != (const Point& v) const {
            return !operator==(v);
        }
        
        float x = 0;
        float y = 0;
	};

	struct Line {
		Line();
		Line(const Point& s, const Point& e);

		bool operator == (const Line& v) const;
		bool operator != (const Line& v) const;

		Point sp;
		Point ep;

		Point minV;
		Point maxV;
	};
	using Poly = std::vector<Point>;

	using Polyline = std::vector<Line>;
	static Polyline toPolyline(const vector<Point>& points)
	{
		Polyline lines;
		for (int i = 1; i < points.size(); ++i) {
			if (points[i - 1] == points[i]) continue;
			lines.push_back(Line(points[i - 1], points[i]));
		}
		return lines;
	}

	struct Matrix
	{
		float T[16];
	public:
		Matrix()
		{
			identity();
		}
		~Matrix() {};
		void copy(const float m[16])
		{
			copy(T, m);
		}
		void identity()
		{
			const static float t[16] = {
				1.0f, 0.0f, 0.0f, 0.0f,
				0.0f, 1.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 1.0f, 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f };
			copy(T, t);
		}
		void create_ortho(float left, float right, float bottom, float top, float zNear, float zFar)
		{
			const float invWidth = 1.0f / (right - left);
			const float invHeight = 1.0f / (top - bottom);
			const float invDepth = 1.0f / (zFar - zNear);
			identity();
			T[0 * 4 + 0] = 2.0f * invWidth;
			T[1 * 4 + 1] = 2.0f * invHeight;
			T[2 * 4 + 2] = -2.0f * invDepth;
			T[3 * 4 + 0] = -(right + left) * invWidth;
			T[3 * 4 + 1] = -(top + bottom) * invHeight;
			T[3 * 4 + 2] = -(zFar + zNear) * invDepth;
		}
		void inverse()
		{
			float t[16];
			copy(t, T);
			//det(行列式)
			float det =
				+t[3 * 4 + 0] * t[2 * 4 + 1] * t[1 * 4 + 2] * t[0 * 4 + 3] - t[2 * 4 + 0] * t[3 * 4 + 1] * t[1 * 4 + 2] * t[0 * 4 + 3]
				- t[3 * 4 + 0] * t[1 * 4 + 1] * t[2 * 4 + 2] * t[0 * 4 + 3] + t[1 * 4 + 0] * t[3 * 4 + 1] * t[2 * 4 + 2] * t[0 * 4 + 3]
				+ t[2 * 4 + 0] * t[1 * 4 + 1] * t[3 * 4 + 2] * t[0 * 4 + 3] - t[1 * 4 + 0] * t[2 * 4 + 1] * t[3 * 4 + 2] * t[0 * 4 + 3]
				- t[3 * 4 + 0] * t[2 * 4 + 1] * t[0 * 4 + 2] * t[1 * 4 + 3] + t[2 * 4 + 0] * t[3 * 4 + 1] * t[0 * 4 + 2] * t[1 * 4 + 3]
				+ t[3 * 4 + 0] * t[0 * 4 + 1] * t[2 * 4 + 2] * t[1 * 4 + 3] - t[0 * 4 + 0] * t[3 * 4 + 1] * t[2 * 4 + 2] * t[1 * 4 + 3]
				- t[2 * 4 + 0] * t[0 * 4 + 1] * t[3 * 4 + 2] * t[1 * 4 + 3] + t[0 * 4 + 0] * t[2 * 4 + 1] * t[3 * 4 + 2] * t[1 * 4 + 3]
				+ t[3 * 4 + 0] * t[1 * 4 + 1] * t[0 * 4 + 2] * t[2 * 4 + 3] - t[1 * 4 + 0] * t[3 * 4 + 1] * t[0 * 4 + 2] * t[2 * 4 + 3]
				- t[3 * 4 + 0] * t[0 * 4 + 1] * t[1 * 4 + 2] * t[2 * 4 + 3] + t[0 * 4 + 0] * t[3 * 4 + 1] * t[1 * 4 + 2] * t[2 * 4 + 3]
				+ t[1 * 4 + 0] * t[0 * 4 + 1] * t[3 * 4 + 2] * t[2 * 4 + 3] - t[0 * 4 + 0] * t[1 * 4 + 1] * t[3 * 4 + 2] * t[2 * 4 + 3]
				- t[2 * 4 + 0] * t[1 * 4 + 1] * t[0 * 4 + 2] * t[3 * 4 + 3] + t[1 * 4 + 0] * t[2 * 4 + 1] * t[0 * 4 + 2] * t[3 * 4 + 3]
				+ t[2 * 4 + 0] * t[0 * 4 + 1] * t[1 * 4 + 2] * t[3 * 4 + 3] - t[0 * 4 + 0] * t[2 * 4 + 1] * t[1 * 4 + 2] * t[3 * 4 + 3]
				- t[1 * 4 + 0] * t[0 * 4 + 1] * t[2 * 4 + 2] * t[3 * 4 + 3] + t[0 * 4 + 0] * t[1 * 4 + 1] * t[2 * 4 + 2] * t[3 * 4 + 3];
			//计算伴随矩阵
			T[0 * 4 + 0] = t[2 * 4 + 1] * t[3 * 4 + 2] * t[1 * 4 + 3] - t[3 * 4 + 1] * t[2 * 4 + 2] * t[1 * 4 + 3] + t[3 * 4 + 1] * t[1 * 4 + 2] * t[2 * 4 + 3] - t[1 * 4 + 1] * t[3 * 4 + 2] * t[2 * 4 + 3] - t[2 * 4 + 1] * t[1 * 4 + 2] * t[3 * 4 + 3] + t[1 * 4 + 1] * t[2 * 4 + 2] * t[3 * 4 + 3];
			T[1 * 4 + 0] = t[3 * 4 + 0] * t[2 * 4 + 2] * t[1 * 4 + 3] - t[2 * 4 + 0] * t[3 * 4 + 2] * t[1 * 4 + 3] - t[3 * 4 + 0] * t[1 * 4 + 2] * t[2 * 4 + 3] + t[1 * 4 + 0] * t[3 * 4 + 2] * t[2 * 4 + 3] + t[2 * 4 + 0] * t[1 * 4 + 2] * t[3 * 4 + 3] - t[1 * 4 + 0] * t[2 * 4 + 2] * t[3 * 4 + 3];
			T[2 * 4 + 0] = t[2 * 4 + 0] * t[3 * 4 + 1] * t[1 * 4 + 3] - t[3 * 4 + 0] * t[2 * 4 + 1] * t[1 * 4 + 3] + t[3 * 4 + 0] * t[1 * 4 + 1] * t[2 * 4 + 3] - t[1 * 4 + 0] * t[3 * 4 + 1] * t[2 * 4 + 3] - t[2 * 4 + 0] * t[1 * 4 + 1] * t[3 * 4 + 3] + t[1 * 4 + 0] * t[2 * 4 + 1] * t[3 * 4 + 3];
			T[3 * 4 + 0] = t[3 * 4 + 0] * t[2 * 4 + 1] * t[1 * 4 + 2] - t[2 * 4 + 0] * t[3 * 4 + 1] * t[1 * 4 + 2] - t[3 * 4 + 0] * t[1 * 4 + 1] * t[2 * 4 + 2] + t[1 * 4 + 0] * t[3 * 4 + 1] * t[2 * 4 + 2] + t[2 * 4 + 0] * t[1 * 4 + 1] * t[3 * 4 + 2] - t[1 * 4 + 0] * t[2 * 4 + 1] * t[3 * 4 + 2];
			T[0 * 4 + 1] = t[3 * 4 + 1] * t[2 * 4 + 2] * t[0 * 4 + 3] - t[2 * 4 + 1] * t[3 * 4 + 2] * t[0 * 4 + 3] - t[3 * 4 + 1] * t[0 * 4 + 2] * t[2 * 4 + 3] + t[0 * 4 + 1] * t[3 * 4 + 2] * t[2 * 4 + 3] + t[2 * 4 + 1] * t[0 * 4 + 2] * t[3 * 4 + 3] - t[0 * 4 + 1] * t[2 * 4 + 2] * t[3 * 4 + 3];
			T[1 * 4 + 1] = t[2 * 4 + 0] * t[3 * 4 + 2] * t[0 * 4 + 3] - t[3 * 4 + 0] * t[2 * 4 + 2] * t[0 * 4 + 3] + t[3 * 4 + 0] * t[0 * 4 + 2] * t[2 * 4 + 3] - t[0 * 4 + 0] * t[3 * 4 + 2] * t[2 * 4 + 3] - t[2 * 4 + 0] * t[0 * 4 + 2] * t[3 * 4 + 3] + t[0 * 4 + 0] * t[2 * 4 + 2] * t[3 * 4 + 3];
			T[2 * 4 + 1] = t[3 * 4 + 0] * t[2 * 4 + 1] * t[0 * 4 + 3] - t[2 * 4 + 0] * t[3 * 4 + 1] * t[0 * 4 + 3] - t[3 * 4 + 0] * t[0 * 4 + 1] * t[2 * 4 + 3] + t[0 * 4 + 0] * t[3 * 4 + 1] * t[2 * 4 + 3] + t[2 * 4 + 0] * t[0 * 4 + 1] * t[3 * 4 + 3] - t[0 * 4 + 0] * t[2 * 4 + 1] * t[3 * 4 + 3];
			T[3 * 4 + 1] = t[2 * 4 + 0] * t[3 * 4 + 1] * t[0 * 4 + 2] - t[3 * 4 + 0] * t[2 * 4 + 1] * t[0 * 4 + 2] + t[3 * 4 + 0] * t[0 * 4 + 1] * t[2 * 4 + 2] - t[0 * 4 + 0] * t[3 * 4 + 1] * t[2 * 4 + 2] - t[2 * 4 + 0] * t[0 * 4 + 1] * t[3 * 4 + 2] + t[0 * 4 + 0] * t[2 * 4 + 1] * t[3 * 4 + 2];
			T[0 * 4 + 2] = t[1 * 4 + 1] * t[3 * 4 + 2] * t[0 * 4 + 3] - t[3 * 4 + 1] * t[1 * 4 + 2] * t[0 * 4 + 3] + t[3 * 4 + 1] * t[0 * 4 + 2] * t[1 * 4 + 3] - t[0 * 4 + 1] * t[3 * 4 + 2] * t[1 * 4 + 3] - t[1 * 4 + 1] * t[0 * 4 + 2] * t[3 * 4 + 3] + t[0 * 4 + 1] * t[1 * 4 + 2] * t[3 * 4 + 3];
			T[1 * 4 + 2] = t[3 * 4 + 0] * t[1 * 4 + 2] * t[0 * 4 + 3] - t[1 * 4 + 0] * t[3 * 4 + 2] * t[0 * 4 + 3] - t[3 * 4 + 0] * t[0 * 4 + 2] * t[1 * 4 + 3] + t[0 * 4 + 0] * t[3 * 4 + 2] * t[1 * 4 + 3] + t[1 * 4 + 0] * t[0 * 4 + 2] * t[3 * 4 + 3] - t[0 * 4 + 0] * t[1 * 4 + 2] * t[3 * 4 + 3];
			T[2 * 4 + 2] = t[1 * 4 + 0] * t[3 * 4 + 1] * t[0 * 4 + 3] - t[3 * 4 + 0] * t[1 * 4 + 1] * t[0 * 4 + 3] + t[3 * 4 + 0] * t[0 * 4 + 1] * t[1 * 4 + 3] - t[0 * 4 + 0] * t[3 * 4 + 1] * t[1 * 4 + 3] - t[1 * 4 + 0] * t[0 * 4 + 1] * t[3 * 4 + 3] + t[0 * 4 + 0] * t[1 * 4 + 1] * t[3 * 4 + 3];
			T[3 * 4 + 2] = t[3 * 4 + 0] * t[1 * 4 + 1] * t[0 * 4 + 2] - t[1 * 4 + 0] * t[3 * 4 + 1] * t[0 * 4 + 2] - t[3 * 4 + 0] * t[0 * 4 + 1] * t[1 * 4 + 2] + t[0 * 4 + 0] * t[3 * 4 + 1] * t[1 * 4 + 2] + t[1 * 4 + 0] * t[0 * 4 + 1] * t[3 * 4 + 2] - t[0 * 4 + 0] * t[1 * 4 + 1] * t[3 * 4 + 2];
			T[0 * 4 + 3] = t[2 * 4 + 1] * t[1 * 4 + 2] * t[0 * 4 + 3] - t[1 * 4 + 1] * t[2 * 4 + 2] * t[0 * 4 + 3] - t[2 * 4 + 1] * t[0 * 4 + 2] * t[1 * 4 + 3] + t[0 * 4 + 1] * t[2 * 4 + 2] * t[1 * 4 + 3] + t[1 * 4 + 1] * t[0 * 4 + 2] * t[2 * 4 + 3] - t[0 * 4 + 1] * t[1 * 4 + 2] * t[2 * 4 + 3];
			T[1 * 4 + 3] = t[1 * 4 + 0] * t[2 * 4 + 2] * t[0 * 4 + 3] - t[2 * 4 + 0] * t[1 * 4 + 2] * t[0 * 4 + 3] + t[2 * 4 + 0] * t[0 * 4 + 2] * t[1 * 4 + 3] - t[0 * 4 + 0] * t[2 * 4 + 2] * t[1 * 4 + 3] - t[1 * 4 + 0] * t[0 * 4 + 2] * t[2 * 4 + 3] + t[0 * 4 + 0] * t[1 * 4 + 2] * t[2 * 4 + 3];
			T[2 * 4 + 3] = t[2 * 4 + 0] * t[1 * 4 + 1] * t[0 * 4 + 3] - t[1 * 4 + 0] * t[2 * 4 + 1] * t[0 * 4 + 3] - t[2 * 4 + 0] * t[0 * 4 + 1] * t[1 * 4 + 3] + t[0 * 4 + 0] * t[2 * 4 + 1] * t[1 * 4 + 3] + t[1 * 4 + 0] * t[0 * 4 + 1] * t[2 * 4 + 3] - t[0 * 4 + 0] * t[1 * 4 + 1] * t[2 * 4 + 3];
			T[3 * 4 + 3] = t[1 * 4 + 0] * t[2 * 4 + 1] * t[0 * 4 + 2] - t[2 * 4 + 0] * t[1 * 4 + 1] * t[0 * 4 + 2] + t[2 * 4 + 0] * t[0 * 4 + 1] * t[1 * 4 + 2] - t[0 * 4 + 0] * t[2 * 4 + 1] * t[1 * 4 + 2] - t[1 * 4 + 0] * t[0 * 4 + 1] * t[2 * 4 + 2] + t[0 * 4 + 0] * t[1 * 4 + 1] * t[2 * 4 + 2];
			//得到逆矩阵
			for (int i = 0; i < 16; i++)
				T[i] /= det;
		}
		void prepend(const Matrix& m)
		{
			static float tem[16];
			copy(tem, this->T);
			copy(this->T, m.T);
			mult(this->T, tem);
		}
		void mult(const Matrix& m)
		{
			mult(T, m.T);
		}
		void translate(const float& x, const float& y)
		{
			const float t[16] = {  //行主
				1.0f, 0.0f, 0.0f, 0.0f,
				0.0f, 1.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 1.0f, 0.0f,
				x,    y, 0.0f, 1.0f };
			mult(T, t);
		}
		void rotate(const float& angle/*角度制*/)
		{
			float a = angle * 0.01745329f;
			float t1 = float(cos(a));
			float t2 = float(sin(a));
			float t3 = float(-sin(a));
			float t4 = float(cos(a));

			float t[16] = { /*行主*/
				 t1,  t2, 0.0, 0.0,
				 t3,  t4, 0.0, 0.0,
				0.0, 0.0, 1.0, 0.0,
				0.0, 0.0, 0.0, 1.0 };
			mult(T, t);
		}
		void scale(const float& x, const float& y)
		{
			float t[16] = {
				x , 0.0f, 0.0f, 0.0f,
				0.0f,   y , 0.0f, 0.0f,
				0.0f, 0.0f, 1.0f, 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f };
			mult(T, t);
		}
		void pre_translate(const float& x, const float& y)
		{
			const float t[16] = {  //行主
				1.0f, 0.0f, 0.0f, 0.0f,
				0.0f, 1.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 1.0f, 0.0f,
				x,    y, 0.0f, 1.0f };
			prepend(T, t);
		}
		void pre_rotate(const float& angle/*角度制*/)
		{
			float a = angle * 0.01745329f;
			float t1 = float(cos(a));
			float t2 = float(sin(a));
			float t3 = float(-sin(a));
			float t4 = float(cos(a));

			float t[16] = { /*行主*/
				t1,  t2, 0.0, 0.0,
				t3,  t4, 0.0, 0.0,
				0.0, 0.0, 1.0, 0.0,
				0.0, 0.0, 0.0, 1.0 };
			prepend(T, t);
		}
		void pre_scale(const float& x, const float& y)
		{
			float t[16] = {
				x , 0.0f, 0.0f, 0.0f,
				0.0f,   y , 0.0f, 0.0f,
				0.0f, 0.0f, 1.0f, 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f };
			prepend(T, t);
		}
		void transform(Point& pt) const
		{
			float tx = float(pt.x * T[0] + pt.y * T[4] + T[12]);
			float ty = float(pt.x * T[1] + pt.y * T[5] + T[13]);
			float  w = float(pt.x * T[3] + pt.y * T[7] + T[15]);
			pt.x = tx / w;
			pt.y = ty / w;
		}
		void transform(vector<Point>& data) const
		{
			for (int i = 0; i<int(data.size()); i++)
				transform(data[i]);
		}
		void transform(vector<vector<Point>>& data) const
		{
			for (int i = 0; i<int(data.size()); i++)
				transform(data[i]);
		}
		void transform(vector<vector<vector<Point>>>& data) const
		{
			for (int i = 0; i<int(data.size()); i++)
				transform(data[i]);
		}
	private:
		void copy(float M[16], const float m[16])
		{
			for (int i = 0; i < 16; i++)
				M[i] = m[i];
		}
		void mult(float M[16], const float m[16])
		{
			float c[16];
			c[0] = M[0] * m[0] + M[4] * m[1] + M[8] * m[2] + M[12] * m[3];
			c[1] = M[1] * m[0] + M[5] * m[1] + M[9] * m[2] + M[13] * m[3];
			c[2] = M[2] * m[0] + M[6] * m[1] + M[10] * m[2] + M[14] * m[3];
			c[3] = M[3] * m[0] + M[7] * m[1] + M[11] * m[2] + M[15] * m[3];
			c[4] = M[0] * m[4] + M[4] * m[5] + M[8] * m[6] + M[12] * m[7];
			c[5] = M[1] * m[4] + M[5] * m[5] + M[9] * m[6] + M[13] * m[7];
			c[6] = M[2] * m[4] + M[6] * m[5] + M[10] * m[6] + M[14] * m[7];
			c[7] = M[3] * m[4] + M[7] * m[5] + M[11] * m[6] + M[15] * m[7];
			c[8] = M[0] * m[8] + M[4] * m[9] + M[8] * m[10] + M[12] * m[11];
			c[9] = M[1] * m[8] + M[5] * m[9] + M[9] * m[10] + M[13] * m[11];
			c[10] = M[2] * m[8] + M[6] * m[9] + M[10] * m[10] + M[14] * m[11];
			c[11] = M[3] * m[8] + M[7] * m[9] + M[11] * m[10] + M[15] * m[11];
			c[12] = M[0] * m[12] + M[4] * m[13] + M[8] * m[14] + M[12] * m[15];
			c[13] = M[1] * m[12] + M[5] * m[13] + M[9] * m[14] + M[13] * m[15];
			c[14] = M[2] * m[12] + M[6] * m[13] + M[10] * m[14] + M[14] * m[15];
			c[15] = M[3] * m[12] + M[7] * m[13] + M[11] * m[14] + M[15] * m[15];
			/*float c[16] = {
			0.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 0.0f };
			for (int j = 0; j < 4; j++)
			for (int i = 0; i < 4; i++)
			{
			float &cc = c[j * 4 + i];
			for (int k = 0; k < 4; k++)
			cc += M[k * 4 + i] * m[j * 4 + k];
			}*/
			copy(M, c);
		}
		void prepend(float M[16], const float m[16])
		{
			static float tem[16];
			copy(tem, this->T);
			copy(this->T, m);
			mult(this->T, tem);
		}
	};

	struct AABB {
		AABB() {}
		AABB(const Polyline& poly);
		AABB(const vector<Point>& pos);

		float area() const;
		bool is_contain(const AABB& child, float accuracy = 0.001f) const;
		bool is_contain(const Point& pt, float accuracy = 0.001f) const;
		bool is_intersect(const AABB& ref, float accuracy = 0.001f) const;
		bool operator == (const AABB& box) const {
			return minPos == box.minPos && maxPos == box.maxPos;
		}
		Point minPos;
		Point maxPos;
	};

	inline ostream& operator<<(ostream& output, const Point& p)
	{
		output << "Point(" << p.x << "," << p.y << ")";
		return output;
	}
	inline ostream& operator<<(ostream& output, const Line& p)
	{
		output << "Start:" << p.sp << " End:" << p.ep << endl;
		return output;
	}
	inline ostream& operator<<(ostream& output, const Polyline& p)
	{
		for (size_t i = 0; i < p.size(); i++)
		{
			output << p[i];
		}
		return output;
	}
	inline ostream& operator<<(ostream& output, const AABB& aabb)
	{
		output << "Max=" << aabb.maxPos << " Min=" << aabb.minPos << endl;
		return output;
	}
}

#endif