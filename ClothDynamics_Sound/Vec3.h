#ifndef vector3d_h
#define vector3d_h

#include <cassert>
#include <math.h>
#include <iostream>

#define NEARLY_ZERO 1.e-16
#define PI			3.141592

class Vec3;

Vec3 operator+(const Vec3&, const Vec3&);
Vec3 operator-(const Vec3&, const Vec3&);
Vec3 operator*(double, const Vec3&);
Vec3 operator*(const Vec3&, double);
double operator*(const Vec3&, const Vec3&);
Vec3 operator^(const Vec3&, const Vec3&);

class Vec3
{
public:
	Vec3(double vx, double vy, double vz) : x(vx), y(vy), z(vz) {}
	Vec3() : x(0.0), y(0.0), z(0.0) {}
	Vec3(const Vec3& rhs) {
		x = rhs.x; y = rhs.y; z = rhs.z;
	}
	virtual ~Vec3() {}

	void SetVector(double vx, double vy, double vz) { x = vx; y = vy; z = vz; }

	inline const Vec3 operator-() const { return -1.0*(*this); }
	inline const Vec3 operator+() const { return *this; }
	inline Vec3& operator=(const Vec3& rhs) {
		if (this != &rhs) { x = rhs.x; y = rhs.y; z = rhs.z; }
		return *this;
	}
	inline Vec3& operator+=(const Vec3& rhs) {
		x += rhs.x; y += rhs.y; z += rhs.z;
		return *this;
	}
	inline Vec3& operator+=(const double rhs) {
		x += rhs; y += rhs; z += rhs;
		return *this;
	}
	inline Vec3& operator-=(const Vec3& rhs) {
		x -= rhs.x; y -= rhs.y; z -= rhs.z;
		return *this;
	}
	inline Vec3& operator-=(const double rhs) {
		x -= rhs; y -= rhs; z -= rhs;
		return *this;
	}
	inline Vec3& operator*=(double d) {
		x *= d; y *= d; z *= d;
		return *this;
	}
	inline Vec3& operator/=(double d) {
		if (fabs(d) < NEARLY_ZERO) { return *this; }
		x /= d; y /= d; z /= d;
		return *this;
	}
	inline double operator[](int i) const {
		if (i == 0) return x;
		if (i == 1) return y;
		if (i == 2) return z;
		return 0;
	}
	inline double& operator[](int i) {
		if (i == 0) return x;
		if (i == 1) return y;
		if (i == 2) return z;
		assert(0);
		return x;
	}
	inline Vec3 operator+() { return *this; }
	inline Vec3 operator-() { return Vec3(-x, -y, -z); }

	friend bool operator==(const Vec3&, const Vec3&);
	friend bool operator!=(const Vec3&, const Vec3&);

	friend Vec3 Cross(const Vec3&, const Vec3&);
	friend double Dot(const Vec3&, const Vec3&);

	inline double Length()  const { return sqrt(x*x + y * y + z * z); }
	inline double DLength() const { return x * x + y * y + z * z; }

	void SetNormalizedVector()
	{
		double invmag = 1.0 / Length();
		x *= invmag;
		y *= invmag;
		z *= invmag;
	}
	inline void Set(double *val)
	{
		x = val[0];
		y = val[1];
		z = val[2];
	}
	inline void Set(double val)
	{
		x = y = z = val;
	}
	inline void Set(double x, double y, double z)
	{
		this->x = x;
		this->y = y;
		this->z = z;
	}

	void SetZero()
	{
		x = 0.0;
		y = 0.0;
		z = 0.0;
	}
public:
	double x;	//!< x axis coordinate
	double y;	//!< y axis coordinate
	double z;	//!< z axis coordinate
public:
	void Rotate(float degreeX, float degreeY) {
		float cosTheta = cosf(degreeX * PI / 180.0f);
		float sinTheta = sinf(degreeX * PI / 180.0f);
		Set(x, y * cosTheta - z * sinTheta, y * sinTheta + z * cosTheta);
		cosTheta = cosf(degreeY * PI / 180.0f);
		sinTheta = sinf(degreeY * PI / 180.0f);
		Set(x * cosTheta + z * sinTheta, y, -x * sinTheta + z * cosTheta);
	}
};

//! add
inline Vec3 operator + (const Vec3& lhs, const Vec3& rhs) {
	Vec3 temp = lhs;
	temp += rhs;
	return temp;
}

//! subtract
inline Vec3 operator - (const Vec3& lhs, const Vec3& rhs) {
	Vec3 temp = lhs;
	temp -= rhs;
	return temp;
}
inline Vec3 operator + (const Vec3& lhs, const double& rhs) {
	Vec3 temp = lhs;
	temp.x += rhs;
	temp.y += rhs;
	temp.z += rhs;
	return temp;
}

//! subtract
inline Vec3 operator - (const Vec3& lhs, const double& rhs) {
	Vec3 temp = lhs;
	temp.x -= rhs;
	temp.y -= rhs;
	temp.z -= rhs;
	return temp;
}

//! scale
inline Vec3 operator * (double d, const Vec3& rhs) {
	Vec3 temp = rhs;
	temp *= d;
	return temp;
}

//! scale
inline Vec3 operator * (const Vec3& vec, double d) {
	Vec3 temp = vec;
	temp *= d;
	return temp;
}

//! divide by real number
inline Vec3 operator / (const Vec3& vec, double d) {
	Vec3 temp = vec;
	temp /= d;
	return temp;
}


//! mult
inline double operator * (const Vec3& lhs, const Vec3& rhs) {
	return Dot(lhs, rhs);
}


//! mult
inline Vec3 operator ^ (const Vec3& lhs, const Vec3& rhs) {
	return Cross(lhs, rhs);
}

inline std::ostream &operator<<(std::ostream &output, const Vec3& v)
{
	output.setf(std::ios::scientific);
	output << v.x << " " << v.y << " " << v.z;
	return output;
}

inline bool operator == (const Vec3& lhs, const Vec3& rhs) {
	if (fabs(lhs.x - rhs.x) < NEARLY_ZERO
		&& fabs(lhs.y - rhs.y) < NEARLY_ZERO
		&& fabs(lhs.z - rhs.z) < NEARLY_ZERO)
		return true;
	else return false;
}

inline bool operator != (const Vec3& lhs, const Vec3& rhs) {
	if (lhs == rhs)	return false;
	else return true;
}

//! length of vector
inline double Length(const Vec3& point)
{
	return	sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
}
inline double LengthSquared(const Vec3& point)
{
	return	point.x * point.x + point.y * point.y + point.z * point.z;
}
inline Vec3 Normalize(Vec3 point)
{
	double invmag = 1.0 / Length(point);
	point.x *= invmag;
	point.y *= invmag;
	point.z *= invmag;
	return point;
}

inline double SquareDistance(const Vec3& ipo0, const Vec3& ipo1)
{
	return
		(ipo1.x - ipo0.x)*(ipo1.x - ipo0.x)
		+ (ipo1.y - ipo0.y)*(ipo1.y - ipo0.y)
		+ (ipo1.z - ipo0.z)*(ipo1.z - ipo0.z);
}

//! distance between two points
inline double Distance(const Vec3& ipo0, const Vec3& ipo1)
{
	return	sqrt(SquareDistance(ipo0, ipo1));
}

inline double Dot(const Vec3 &arg1, const Vec3 &arg2)
{
	return arg1.x*arg2.x + arg1.y*arg2.y + arg1.z*arg2.z;
}

inline Vec3 Cross(const Vec3& arg1, const Vec3& arg2)
{
	Vec3 temp;
	temp.x = arg1.y*arg2.z - arg1.z*arg2.y;
	temp.y = arg1.z*arg2.x - arg1.x*arg2.z;
	temp.z = arg1.x*arg2.y - arg1.y*arg2.x;
	return temp;
}
inline double AngleBetweenVectors(const Vec3& arg1, const Vec3& arg2)
{
	double c = Dot(arg1, arg2);
	double s = Length(Cross(arg1, arg2));
	return atan2(s, c);
}

inline double ScalarTripleProduct(const Vec3& a, const Vec3& b, const Vec3& c) {
	return a.x*(b.y*c.z - b.z*c.y) + a.y*(b.z*c.x - b.x*c.z) + a.z*(b.x*c.y - b.y*c.x);
}



//! Volume of a tetrahedra
inline double TetVolume(const Vec3& v1,
	const Vec3& v2,
	const Vec3& v3,
	const Vec3& v4)
{
	return
		((v2.x - v1.x)*((v3.y - v1.y)*(v4.z - v1.z) - (v4.y - v1.y)*(v3.z - v1.z))
			- (v2.y - v1.y)*((v3.x - v1.x)*(v4.z - v1.z) - (v4.x - v1.x)*(v3.z - v1.z))
			+ (v2.z - v1.z)*((v3.x - v1.x)*(v4.y - v1.y) - (v4.x - v1.x)*(v3.y - v1.y))
			) * 0.16666666666666666666666666666667;
}



inline void UnitNormal
(Vec3& vnorm,
	const Vec3& v1,
	const Vec3& v2,
	const Vec3& v3)
{
	vnorm.x = (v2.y - v1.y)*(v3.z - v1.z) - (v2.z - v1.z)*(v3.y - v1.y);
	vnorm.y = (v2.z - v1.z)*(v3.x - v1.x) - (v2.x - v1.x)*(v3.z - v1.z);
	vnorm.z = (v2.x - v1.x)*(v3.y - v1.y) - (v2.y - v1.y)*(v3.x - v1.x);
	const double dtmp1 = 1.0 / Length(vnorm);
	vnorm.x *= dtmp1;
	vnorm.y *= dtmp1;
	vnorm.z *= dtmp1;
}


//! Hight of a tetrahedra
inline double Height(const Vec3& v1, const Vec3& v2, const Vec3& v3, const Vec3& v4) {
	// get normal vector
	double dtmp_x = (v2.y - v1.y)*(v3.z - v1.z) - (v2.z - v1.z)*(v3.y - v1.y);
	double dtmp_y = (v2.z - v1.z)*(v3.x - v1.x) - (v2.x - v1.x)*(v3.z - v1.z);
	double dtmp_z = (v2.x - v1.x)*(v3.y - v1.y) - (v2.y - v1.y)*(v3.x - v1.x);

	// normalize normal vector
	const double dtmp1 = 1.0 / sqrt(dtmp_x*dtmp_x + dtmp_y * dtmp_y + dtmp_z * dtmp_z);
	dtmp_x *= dtmp1;
	dtmp_y *= dtmp1;
	dtmp_z *= dtmp1;

	return (v4.x - v1.x)*dtmp_x + (v4.y - v1.y)*dtmp_y + (v4.z - v1.z)*dtmp_z;
}


inline Vec3 GetMinDist_LinePoint(const Vec3& p, // point
	const Vec3& s, // source
	const Vec3& d) // direction
{
	assert(Dot(d, d) > 1.0e-20);
	const Vec3 ps = s - p;
	double a = Dot(d, d);
	double b = Dot(d, s - p);
	double t = -b / a;
	return s + t * d;
}

inline Vec3 GetMinDist_LineSegPoint(const Vec3& p, // point
	const Vec3& s, // source
	const Vec3& e) // direction
{
	Vec3 d = e - s;
	assert(Dot(d, d) > 1.0e-20);
	const Vec3 ps = s - p;
	double a = Dot(d, d);
	double b = Dot(d, s - p);
	double t = -b / a;
	if (t < 0) t = 0;
	if (t > 1) t = 1;
	return s + t * d;
}

#endif