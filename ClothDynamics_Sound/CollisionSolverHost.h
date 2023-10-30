#ifndef __COLLISION_SOLEVR_HOST_H__
#define __COLLISION_SOLEVR_HOST_H__

#pragma once
#include "SoundManager.h"

#define USED_R_TRI

#define RESPONSE0
//#define RESPONSE1
//#define RESPONSE2
//#define RESPONSE3
//#define RESPONSE4
//#define RESPONSE5

//#define SDF_NORMAL

//#define USED_EV_VV

#define PLUS_TIME_1

#define CCD_PRINT_TRI_TV
#define CCD_PRINT_TRI_EV
#define CCD_PRINT_TRI_VV

#define CCD_PRINT_EDGE_EE
#define CCD_PRINT_EDGE_EV
#define CCD_PRINT_EDGE_VV


#define COL_CULLING			1.0e-10
#define COL_EPS				1.0e-5

#define COL_VV				1.0e-10
#define COL_EV				1.0e-10
#define COL_EE				1.0e-5
#define COL_TV				1.0e-5

#define HALF_TIME			0.0

#define CUBIC_ITERATOR		30

//#define COL_ROBUST
#ifndef COL_ROBUST
#define COL_STABILITY_PROXIMITY		0.25
#define COL_STABILITY_CCD			0.25
#define COL_THICKNESS				0.1
#endif

class ClothCollisionElement
{
public:
	bool is_fv; // True : edge-edge contact, False : vertex-face contact
	int ino0, ino1, ino2, ino3; // Four points in the contact 
	double time;
public:
	ClothCollisionElement(void) {}
	virtual ~ClothCollisionElement(void) {}
	ClothCollisionElement(bool is_fv, int j0, int j1, int j2, int j3, double time = 0.0)
	{
		this->is_fv = is_fv;
		this->time = time;
		if (is_fv) {
			if (j0 < j1 && j0 < j2 && j1 < j2) {
				ino0 = j0;  ino1 = j1;  ino2 = j2;  ino3 = j3;
			}
			else if (j0 < j1 && j0 < j2 && j2 < j1) {
				ino0 = j0;  ino1 = j2;  ino2 = j1;  ino3 = j3;
			}
			else if (j1 < j0 && j1 < j2 && j0 < j2) {
				ino0 = j1;  ino1 = j0;  ino2 = j2;  ino3 = j3;
			}
			else if (j1 < j0 && j1 < j2 && j2 < j0) {
				ino0 = j1;  ino1 = j2;  ino2 = j0;  ino3 = j3;
			}
			else if (j2 < j0 && j2 < j1 && j0 < j1) {
				ino0 = j2;  ino1 = j0;  ino2 = j1;  ino3 = j3;
			}
			else if (j2 < j0 && j2 < j1 && j1 < j0) {
				ino0 = j2;  ino1 = j1;  ino2 = j0;  ino3 = j3;
			}
			else {
				assert(0);
			}
		}
		else {
			if (j0 < j1 && j0 < j2 && j0 < j3 && j2 < j3) {
				ino0 = j0;  ino1 = j1;  ino2 = j2;  ino3 = j3;
			}
			else if (j0 < j1 && j0 < j2 && j0 < j3 && j3 < j2) {
				ino0 = j0;  ino1 = j1;  ino2 = j3;  ino3 = j2;
			}
			else if (j1 < j0 && j1 < j2 && j1 < j3 && j2 < j3) {
				ino0 = j1;  ino1 = j0;  ino2 = j2;  ino3 = j3;
			}
			else if (j1 < j0 && j1 < j2 && j1 < j3 && j3 < j2) {
				ino0 = j1;  ino1 = j0;  ino2 = j3;  ino3 = j2;
			}
			else if (j2 < j0 && j2 < j1 && j2 < j3 && j0 < j1) {
				ino0 = j2;  ino1 = j3;  ino2 = j0;  ino3 = j1;
			}
			else if (j2 < j0 && j2 < j1 && j2 < j3 && j1 < j0) {
				ino0 = j2;  ino1 = j3;  ino2 = j1;  ino3 = j0;
			}
			else if (j3 < j0 && j3 < j1 && j3 < j2 && j0 < j1) {
				ino0 = j3;  ino1 = j2;  ino2 = j0;  ino3 = j1;
			}
			else if (j3 < j0 && j3 < j1 && j3 < j2 && j1 < j0) {
				ino0 = j3;  ino1 = j2;  ino2 = j1;  ino3 = j0;
			}
			else {
				assert(0);
			}
		}
	}
	bool operator < (const ClothCollisionElement& p2) const
	{
		if (time != p2.time) return time < p2.time;
		if (ino0 != p2.ino0) return ino0 < p2.ino0;
		if (ino1 != p2.ino1) return ino1 < p2.ino1;
		if (ino2 != p2.ino2) return ino2 < p2.ino2;
		return ino3 < p2.ino3;
	}
};
class CollisionElement
{
public:
	int type;
	int ino0, ino1, ino2, ino3;
	double t;
public:
	CollisionElement(void) {}
	virtual ~CollisionElement(void) {}
	CollisionElement(int type, int j0, int j1, int j2, int j3, double t=0)
	{
		this->type = type;
		this->t = t;
		ino0 = j0;  ino1 = j1;  ino2 = j2;  ino3 = j3;
		if (type == 1) {
			if (j0 < j1 && j2 < j3) {
				ino0 = j0;  ino1 = j1;  ino2 = j2;  ino3 = j3;
			}
			else if (j0 < j1 && j3 < j2) {
				ino0 = j0;  ino1 = j1;  ino2 = j3;  ino3 = j2;
			}
			else if (j1 < j0 && j2 < j3) {
				ino0 = j1;  ino1 = j0;  ino2 = j2;  ino3 = j3;
			}
			else if (j1 < j0 && j3 < j2) {
				ino0 = j1;  ino1 = j0;  ino2 = j3;  ino3 = j2;
			}
			else {
				assert(0);
			}
		}
		/*else {
			if (j0 < j1 && j0 < j2 && j1 < j2) {
				ino0 = j0;  ino1 = j1;  ino2 = j2;  ino3 = j3;
			}
			else if (j0 < j1 && j0 < j2 && j2 < j1) {
				ino0 = j0;  ino1 = j2;  ino2 = j1;  ino3 = j3;
			}
			else if (j1 < j0 && j1 < j2 && j0 < j2) {
				ino0 = j1;  ino1 = j0;  ino2 = j2;  ino3 = j3;
			}
			else if (j1 < j0 && j1 < j2 && j2 < j0) {
				ino0 = j1;  ino1 = j2;  ino2 = j0;  ino3 = j3;
			}
			else if (j2 < j0 && j2 < j1 && j0 < j1) {
				ino0 = j2;  ino1 = j0;  ino2 = j1;  ino3 = j3;
			}
			else if (j2 < j0 && j2 < j1 && j1 < j0) {
				ino0 = j2;  ino1 = j1;  ino2 = j0;  ino3 = j3;
			}
			else {
				assert(0);
			}
		}*/
	}
	bool operator < (const CollisionElement& p2) const
	{
		if (type != p2.type) return type < p2.type;
		if (t != p2.t) return t < p2.t;
		if (ino0 != p2.ino0) return ino0 < p2.ino0;
		if (ino1 != p2.ino1) return ino1 < p2.ino1;
		if (ino2 != p2.ino2) return ino2 < p2.ino2;
		return ino3 < p2.ino3;
	}
};
class ResponseParam {
public:
	double impulse[3];
	double pene;
public:
	ResponseParam() {
		impulse[0] = impulse[1] = impulse[2] = 0.0;
		pene = 0.0;
	}
	~ResponseParam() {}
public:
	inline void setImpulse(Vec3 I) {
		impulse[0] = I.x;
		impulse[1] = I.y;
		impulse[2] = I.z;
	}
	void Print() {
		printf("%f, %f, %f, %f\n", impulse[0], impulse[1], impulse[2], pene);
	}
};
class CollisionSolverHost {
public:
	static bool Debugging;
	static vector<Vec3> lineTest;
public:
	CollisionSolverHost() {}
	~CollisionSolverHost() {}
public:
	static void Debug(void) {
		Debugging = !Debugging;
		if (Debugging)
			printf("Debugging On\n");
		else
			printf("Debugging Off\n");
	}
public:
	static inline double CubicEval(double a, double b, double c, double d, double x) {
		return ((a * x + b) * x + c) * x + d;
	}
	static double CubicSolver(double a, double b, double c, double d, double r0, double r1, double x0, double x1) {
		double r2, x2;
		if (x0 == 0.0)	return r0;
		else if (x1 == 0.0) return r1;
		for (int itr = 0; itr < CUBIC_ITERATOR; itr++) {
			r2 = 0.5 * (r0 + r1);
			x2 = CubicEval(a, b, c, d, r2);
			if (x2 == 0.0)
				return r2;
			
			if (x0 * x2 < 0)	r1 = r2;
			else				r0 = r2;
		}
		return (r0 + r1) * 0.5;
	}
	static int FinePlaneCoTime(
		const Vec3& a0, const Vec3& b0, const Vec3& c0, const Vec3& d0,
		const Vec3& a1, const Vec3& b1, const Vec3& c1, const Vec3& d1,
		double* times) 
	{
		int num = 0;
		Vec3 v01_0 = b0 - a0; Vec3 v01_1 = b1 - a1 - v01_0;
		Vec3 v02_0 = c0 - a0; Vec3 v02_1 = c1 - a1 - v02_0;
		Vec3 v0p_0 = d0 - a0; Vec3 v0p_1 = d1 - a1 - v0p_0;
		auto cross0 = Cross(v01_1, v02_1);
		auto cross1 = Cross(v01_0, v02_1) + Cross(v01_1, v02_0);
		auto cross2 = Cross(v01_0, v02_0);

		double a = Dot(cross0, v0p_1);
		double b = Dot(cross0, v0p_0) + Dot(cross1, v0p_1);
		double c = Dot(cross1, v0p_0) + Dot(cross2, v0p_1);
		double d = Dot(cross2, v0p_0);

		if (a == 0.0) {
			return QuadraticSolver(b, c, d, times);
		}
		else if (a < 0.0) 
			{ a = -a; b = -b; c = -c; d = -d; }

		double r0 = 0.0;
		double r1 = +1.0;
		double f0 = CubicEval(a, b, c, d, r0);
		double f1 = CubicEval(a, b, c, d, r1);
		double det = b * b - 3.0 * a * c;
		if (det >= 0.0)
		{
			double r3 = (-b - sqrt(det)) / (3.0 * a);
			double r4 = (-b + sqrt(det)) / (3.0 * a);
			const double f3 = CubicEval(a, b, c, d, r3);
			const double f4 = CubicEval(a, b, c, d, r4);

			if (r3 > -0.0 && r4 < 1.0) {
				if (f0 * f3 <= 0.0)	times[num++] = CubicSolver(a, b, c, d, r0, r3, f0, f3);
				if (f3 * f4 <= 0.0)	times[num++] = CubicSolver(a, b, c, d, r3, r4, f3, f4);
				if (f4 * f1 <= 0.0)	times[num++] = CubicSolver(a, b, c, d, r4, r1, f4, f1);
				return num;
			}
			else if (r3 > -0.0 && r3 < 1.0) {
				if (f0 * f3 <= 0.0)	times[num++] = CubicSolver(a, b, c, d, r0, r3, f0, f3);
				if (f3 * f1 <= 0.0)	times[num++] = CubicSolver(a, b, c, d, r3, r1, f3, f1);
				return num;
			}
			else if (r4 > -0.0 && r4 < 1.0) {
				if (f0 * f4 <= 0.0)	times[num++] = CubicSolver(a, b, c, d, r0, r4, f0, f4);
				if (f4 * f1 <= 0.0)	times[num++] = CubicSolver(a, b, c, d, r4, r1, f4, f1);
				return num;
			}
		}
		if (f0 * f1 > 0)
			return 0;
		times[num++] = CubicSolver(a, b, c, d, r0, r1, f0, f1);
		return num;
	}
	static inline double QuadraticEval(double a, double b, double c, double x) {
		return (a * x + b) * x + c;
	}
	static int QuadraticSolver(double fa, double fb, double fc, double* times)
	{
		int num = 0;
		if (fa == 0.0) {
			if (fc == 0.0)
				times[num++] = 0.0;
			else if (fb != 0.0) {
				double t = -fc / fb;
				if (t >= 0.0 && t <= 1.0)
					times[num++] = t;
			}
			return num;
		}
		else if (fa < 0.0) { fa = -fa; fb = -fb; fc = -fc; }

		if (fc > 0.0) {
			if (fb > 0.0)
				return 0;

			double cx = -fb / (fa + fa);
			double cy = QuadraticEval(fa, fb, fc, cx);
			if (cy > 0.0)
				return 0;
			else if (cy == 0.0) {
				if (cx >= 0.0 && cx <= 1.0)
					times[num++] = cx;
				return num;
			}
			double det = fb * fb - 4.0 * fa * fc;
			//fa = 1.0 / (fa + fa);
			double invfa = 1.0 / (fa + fa);
			det = sqrt(det);
			double t = (-fb - det) * invfa;
			if (t >= 0.0 && t <= 1.0)
				times[num++] = t;
			t = (-fb + det) * invfa;
			if (t >= 0.0 && t <= 1.0)
				times[num++] = t;

			return num;
		}
		else if(fc < 0.0) {
			double r1 = QuadraticEval(fa, fb, fc, 1.0);
			if (r1 < 0.0)
				return 0;
			else if (r1 == 0.0)
				times[num++] = 1.0;
			else {
				double t = (-fb + sqrt(fb * fb - 4.0 * fa * fc)) / (fa + fa);
				if (t >= 0.0 && t <= 1.0)
					times[num++] = t;
			}
			return num;
		}
		times[num++] = 0.0;
		double t = -fb / fa;
		if (t >= 0.0 && t <= 1.0)
			times[num++] = t;
		return num;
	}
	static void QuadraticSolver(double fa, double fb, double fc, set<double>& times)
	{
		if (fa == 0.0) {
			if (fc == 0.0)
				times.insert(0.0);
			else if (fb != 0.0) {
				double t = -fc / fb;
				if (t >= 0.0 && t <= 1.0)
					times.insert(t);
			}
			return;
		}
		if (fa < 0.0)	{ fa = -fa; fb = -fb; fc = -fc; }

		if (fc > 0.0) {
			if (fb >= 0.0)
				return;

			double cx = -fb / (fa + fa);
			double rc = QuadraticEval(fa, fb, fc, cx);
			if (rc > 0.0)
				return;
			else if (rc == 0.0) {
				if (cx >= 0.0 && cx <= 1.0)
					times.insert(cx);
			}
			else {
				double det = fb * fb - 4.0 * fa * fc;
				fa = 1.0 / (fa + fa);
				det = sqrt(det);
				double t = (-fb - det) * fa;
				if (t >= 0.0 && t <= 1.0)
					times.insert(t);
				t = (-fb + det) * fa;
				if (t >= 0.0 && t <= 1.0)
					times.insert(t);
			}
		}
		else if (fc < 0.0) {
			double r1 = QuadraticEval(fa, fb, fc, 1.0);
			if (r1 < 0.0)
				return;
			else if (r1 == 0.0)
				times.insert(1.0);
			else {
				double t = (-fb + sqrt(fb * fb - 4.0 * fa * fc)) / (fa + fa);
				if (t >= 0.0 && t <= 1.0)
					times.insert(t);
			}
		}
		else {
			times.insert(0.0);
			double t = -fb / fa;
			if (t >= 0.0 && t <= 1.0)
				times.insert(t);
		}
	}
public:
	static double getDistanceTV(
		const Vec3& v0, const Vec3& v1, const Vec3& v2, const Vec3& p,
		double* w0, double* w1, double error = 0.0);
	static double getDistanceEE(
		const Vec3 & pa, const Vec3 & pb, const Vec3 & pc, const Vec3 & pd,
		double* w0, double* w1, bool* isParallel = nullptr, double error = 0.0);
	static double getDistanceEV(
		const Vec3& v0, const Vec3& v1, const Vec3& p,
		double* w, double error = 0.0);
public:
	static bool DetectionVV_CCD(
		const Vec3& p0, const Vec3& p1, const Vec3& q0, const Vec3& q1,
		double delta, double* t);
	static bool DetectionEV_CCD(
		const Vec3& p0, const Vec3& p1, const Vec3& p2,
		const Vec3& q0, const Vec3& q1, const Vec3& q2,
		double delta, double* t, double* w0 = nullptr);
	static bool DetectionTV_CCD(
		const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
		const Vec3& q0, const Vec3& q1, const Vec3& q2, const Vec3& q3,
		double delta, double* t, double* w0 = nullptr, double* w1 = nullptr);
	static bool DetectionEE_CCD(
		const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
		const Vec3& q0, const Vec3& q1, const Vec3& q2, const Vec3& q3,
		double delta, double* t, double* w0 = nullptr, double* w1 = nullptr);
public:
	static bool isContactTV_Proximity(
		const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
		double delta, double* w0 = nullptr, double* w1 = nullptr);
	static bool isContactEE_Proximity(
		const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
		double delta, double* w0 = nullptr, double* w1 = nullptr);
	static bool isSelfContactTV_Proximity(
		int i0, int i1, int i2, int i3,
		const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
		double delta, double* w0 = nullptr, double* w1 = nullptr);
	static bool isSelfContactEE_Proximity(
		int i0, int i1, int i2, int i3,
		const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
		double delta, double* w0 = nullptr, double* w1 = nullptr);
public:
	static bool isContactTV_CCD(
		const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
		const Vec3& q0, const Vec3& q1, const Vec3& q2, const Vec3& q3,
		double* t, double* w0 = nullptr, double* w1 = nullptr);
	static bool isContactEE_CCD(
		const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
		const Vec3& q0, const Vec3& q1, const Vec3& q2, const Vec3& q3,
		double* t, double* w0 = nullptr, double* w1 = nullptr);
	static bool isSelfContactTV_CCD(
		int i0, int i1, int i2, int i3,
		const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
		const Vec3& q0, const Vec3& q1, const Vec3& q2, const Vec3& q3,
		double* t, double* w0 = nullptr, double* w1 = nullptr);
	static bool isSelfContactEE_CCD(
		int i0, int i1, int i2, int i3,
		const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
		const Vec3& q0, const Vec3& q1, const Vec3& q2, const Vec3& q3,
		double* t, double* w0 = nullptr, double* w1 = nullptr);
public:
	static void getSelfContactElements_Proximity(
		set<ClothCollisionElement>& contacts,
		const vector<int>& faces,
		const vector<double>& vertices,
		vector<double>& velocities,
		BVHNodeHost* nodeL, BVHNodeHost* nodeR, double thickness, double dt);
	static void getSelfContactElements_Proximity(
		set<ClothCollisionElement>& contacts,
		const vector<int>& faces,
		const vector<double>& vertices,
		vector<double>& velocities,
		BVHNodeHost* node, double thickness, double dt);
	static void getSelfContactElements_CCD(
		set<ClothCollisionElement>& contacts,
		const vector<int>& faces,
		const vector<double>& vertices,
		vector<double>& velocities,
		BVHNodeHost* nodeL, BVHNodeHost* nodeR, double dt);
	static void getSelfContactElements_CCD(
		set<ClothCollisionElement>& contacts,
		const vector<int>& faces,
		const vector<double>& vertices,
		vector<double>& velocities,
		BVHNodeHost* node, double dt);
public:
	static void MakeRigidImpactZone(
		vector< set<int> >& aRIZ,
		const set<ClothCollisionElement>& aContactElem,
		const EdgeBufferHost& aEdge);
	static inline void CalcInvMat3(double ainv[], const double a[]);
	static void ApplyRigidImpactZone(
		vector<double>& aUVWm,
		const vector< std::set<int> >& aRIZ,
		const vector<double>& aXYZ,
		const vector<double>& aUVWm0,
		const vector<double>& masses);
public:
	static void resolveSelfCollision_Proximity(
		set<ClothCollisionElement>& contacts,
		const vector<double>& vertices,
		vector<double>& velocities, 
		const vector<double>& masses,
		double thickness, double friction, double dt,
		int cloth_id);
	static void resolveSelfCollision_CCD(
		set<ClothCollisionElement>& contacts,
		const vector<double>& vertices,
		vector<double>& velocities, 
		const vector<double>& masses,
		double thickness, double dt);
public:
	static bool isObsContactTV(
		const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
		const Vec3& q0, const Vec3& q1, const Vec3& q2, const Vec3& q3,
		double delta, double* t, double* w0 = nullptr, double* w1 = nullptr);
	static bool isObsContactEE(
		const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
		const Vec3& q0, const Vec3& q1, const Vec3& q2, const Vec3& q3,
		double delta, double* t, double* w0 = nullptr, double* w1 = nullptr);
	static bool resolveObsContactTV(
		uint i0, uint i1, uint i2, uint i3,
		const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
		const Vec3& v0, const Vec3& v1, const Vec3& v2, const Vec3& v3,
		const vector<double>& masses,
		const vector<double>& obs_masses,
		vector<ResponseParam>& responseParams,
		double delta, double friction, double dt,
		bool sampling, int cloth_id);
	static bool resolveObsContactVT(
		uint i0, uint i1, uint i2, uint i3,
		const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
		const Vec3& v0, const Vec3& v1, const Vec3& v2, const Vec3& v3,
		const vector<double>& masses,
		const vector<double>& obs_masses,
		vector<ResponseParam>& responseParams,
		double delta, double friction, double dt,
		bool sampling, int cloth_id);
	static bool resolveObsContactEE(
		uint i0, uint i1, uint i2, uint i3,
		const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
		const Vec3& v0, const Vec3& v1, const Vec3& v2, const Vec3& v3,
		const vector<double>& masses,
		const vector<double>& obs_masses,
		vector<ResponseParam>& responseParams,
		double delta, double friction, double dt,
		bool sampling, int cloth_id);
public:
	static void resolveObstacleCollision(
		uint& num,
		const vector<int>& faces,
		const vector<double>& vertices,
		const vector<double>& velocities,
		const vector<double>& masses,
		const vector<int>& obs_faces,
		const vector<double>& obs_vertices,
		const vector<double>& obs_velocities,
		const vector<double>& obs_masses,
		BVHNodeHost* nodeL, BVHNodeHost* nodeR, 
		vector<ResponseParam>& responseParams,
		double delta, double friction, double dt,
		bool sampling, int cloth_id);
public:
	static bool ResolveSelfCollision(
		const vector<int>& faces,
		const vector<double>& vertices,
		vector<double>& velocities,
		const EdgeBufferHost& edges,
		const vector<double>& masses,
		BVHHost* bvh,
		double thickness, double friction, double dt,
		int cloth_id);
	static bool ResolveBoundaryCollision(
		const vector<double>& vertices,
		vector<double>& velocities,
		const vector<double>& masses,
		const Vec3& minBoundary, const Vec3& maxBoundary,
		double thickness, double friction, double dt,
		int cloth_id);
	static bool ResolveObstacleCollision(
		const vector<int>& faces,
		const vector<double>& vertices,
		vector<double>& velocities,
		const vector<double>& masses,
		BVHHost* bvh,
		const vector<int>& obs_faces,
		const vector<double>& obs_vertices,
		vector<double>& obs_velocities,
		const vector<double>& obs_masses,
		BVHHost* obs_bvh,
		double thickness, double friction, double dt,
		int cloth_id);
};

#endif