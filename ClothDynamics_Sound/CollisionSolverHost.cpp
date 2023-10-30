#include "CollisionSolverHost.h"

bool CollisionSolverHost::Debugging = false;

double CollisionSolverHost::getDistanceTV(
	const Vec3& v0, const Vec3& v1, const Vec3& v2, const Vec3& p,
	double* w0, double* w1, double error)
{
	/*const Vec3 v01 = v1 - v0;
	const Vec3 v02 = v2 - v0;
	double v0pDotv01 = Dot(p - v0, v01);
	double v0pDotv02 = Dot(p - v0, v02);
	double v01Dotv01 = LengthSquared(v01);
	double v02Dotv02 = LengthSquared(v02);
	double v01Dotv02 = Dot(v01, v02);
	double v1pDotv12 = v0pDotv02 - v0pDotv01 - v01Dotv02 + v01Dotv01;

	Vec3 normal = Cross(v01, v02);
	double det = LengthSquared(normal);
	if (fabs(det) <= 1.0e-20)
		return DBL_MAX;
	normal /= sqrt(det);

	Vec3 rv = p;
	double result = 0.0;
	double dist = Dot(p - v0, normal);
	if (v0pDotv01 < 0.0 && v0pDotv02 < 0.0) {
		rv -= v0;
		*w0 = 1.0;
		*w1 = 0.0;
	}
	else if (v0pDotv02 + v01Dotv01 < v0pDotv01 + v01Dotv02 && v01Dotv01 < v0pDotv01) {
		rv -= v1;
		*w0 = 0.0;
		*w1 = 1.0;
	}
	else if (v02Dotv02 < v0pDotv02 && v0pDotv01 + v02Dotv02 < v0pDotv02 + v01Dotv02) {
		rv -= v2;
		*w0 = 0.0;
		*w1 = 0.0;
	}
	else if (v0pDotv01 * v01Dotv02 > v0pDotv02 * v01Dotv01 && v0pDotv01 >= 0.0 && v01Dotv01 >= v0pDotv01) {
		rv -= v0;
		auto tmp = v0pDotv01 * v0pDotv01 / v01Dotv01;
		result -= tmp;
		*w1 = sqrt(tmp / v01Dotv01);
		*w0 = 1.0 - *w1;
	}
	else if ((v0pDotv01 - v01Dotv01) * (v02Dotv02 - v01Dotv02) > (v0pDotv02 - v01Dotv02) * (v01Dotv02 - v01Dotv01) && v0pDotv01 + v02Dotv02 >= v0pDotv02 + v01Dotv02) {
		rv -= v1;
		auto sqr = v01Dotv01 + v02Dotv02 - v01Dotv02 - v01Dotv02;
		auto tmp = v1pDotv12 * v1pDotv12 / sqr;
		result -= tmp;
		*w0 = 0.0;
		*w1 = 1.0 - sqrt(tmp / sqr);
	}
	else if (v0pDotv02 * v01Dotv02 > v0pDotv01 * v02Dotv02) {
		rv -= v0;
		auto tmp = v0pDotv02 * v0pDotv02 / v02Dotv02;
		result -= tmp;
		*w0 = 1.0 - sqrt(tmp / v02Dotv02);
		*w1 = 0.0;
	}
	else {
		double invdet = 1.0 / det;
		*w1 = (v02Dotv02 * v0pDotv01 - v01Dotv02 * v0pDotv02) * invdet;
		*w0 = 1.0 - *w1 - (v01Dotv01 * v0pDotv02 - v01Dotv02 * v0pDotv01) * invdet;
		return fabs(dist);
	}
	result += LengthSquared(rv) - dist * dist;
	if (result <= 0.0)
		return fabs(dist);
	else if (sqrt(result) > error)
		return DBL_MAX;
	return fabs(dist);*/
	Vec3 v20 = v0 - v2;
	Vec3 v21 = v1 - v2;
	double t0 = Dot(v20, v20);
	double t1 = Dot(v21, v21);
	double t2 = Dot(v20, v21);
	double t3 = Dot(v20, p - v2);
	double t4 = Dot(v21, p - v2);
	double det = t0 * t1 - t2 * t2;
	if (fabs(det) <= 1.0e-20)
		return DBL_MAX;
	double invdet = 1.0 / det;
	*w0 = (+t1 * t3 - t2 * t4) * invdet;
	if (*w0 < 0.0 || *w0 > 1.0) return DBL_MAX;
	*w1 = (-t2 * t3 + t0 * t4) * invdet;
	if (*w1 < 0.0 || *w1 > 1.0) return DBL_MAX;
	double w2 = 1 - *w0 - *w1;
	if (w2 < 0.0 || w2 > 1.0) return DBL_MAX;
	if (isnan(*w0) || isnan(*w1)) {
		printf("%f, %f, %f, %f, %f, %f\n", v20.x, v20.y, v20.z, v21.x, v21.y, v21.z);
		printf("%f, %f, %f\n", t0, t1, t2);
	}
	Vec3 pw = *w0 * v0 + *w1 * v1 + w2 * v2;
	return Length(pw - p);
}
double CollisionSolverHost::getDistanceEE(
	const Vec3& pa, const Vec3& pb, const Vec3& pc, const Vec3& pd,
	double* w0, double* w1, bool* isParallel, double error)
{
	/*Vec3 vp = pb - pa;
	Vec3 vq = pd - pc;
	double w01, w23;
	double t0 = Dot(vp, vp);
	double t1 = Dot(vq, vq);
	double t2 = Dot(vp, vq);
	double det = t0 * t1 - t2 * t2;
	if (fabs(det) < 1.0e-20) {
		if (isParallel)
			*isParallel = true;
		double lp0 = Dot(pa, vp);
		double lp1 = Dot(pb, vp);
		double lq0 = Dot(pc, vp);
		double lq1 = Dot(pd, vp);
		double p_min = (lp0 < lp1) ? lp0 : lp1;
		double p_max = (lp0 > lp1) ? lp0 : lp1;
		double q_min = (lq0 < lq1) ? lq0 : lq1;
		double q_max = (lq0 > lq1) ? lq0 : lq1;
		double lm;
		if (p_max < q_min)		lm = (p_max + q_min) * 0.5;
		else if (p_min > q_max) lm = (q_max + p_min) * 0.5;
		else if (p_max < q_max)
			if (p_min < q_min)	lm = (p_max + q_min) * 0.5;
			else				lm = (p_max + p_min) * 0.5;
		else
			if (p_min < q_min)	lm = (q_max + q_min) * 0.5;
			else				lm = (q_max + p_min) * 0.5;
		w01 = (lm - lp0) / (lp1 - lp0);
		w23 = (lm - lq0) / (lq1 - lq0);
		if (w01 < 0.0 || w01 > 1.0 || w23 < 0.0 || w23 > 1.0) {
			double dist = 0.0;
			double t0 = Dot(vp, vp);
			if (w01 < 0) {
				dist = sqrt(t0) * -w01;
				w01 = 0.0;
			}
			else if (w01 > 1.0) {
				dist = sqrt(t0) * (w01 - 1.0);
				w01 = 1.0;
			}
			if (w23 < 0)		w23 = 0.0;
			else if (w23 > 1.0)	w23 = 1.0;
			*w0 = w01;
			*w1 = w23;
			if (dist + dist > error)
				return DBL_MAX;
		}
		*w0 = w01;
		*w1 = w23;
		vp.SetNormalizedVector();
		auto ppc = pa - pc;
		auto vert = ppc - Dot(ppc, vp) * vp;
		return Length(vert);
	}
	if (isParallel)
		*isParallel = false;
	double t3 = Dot(vp, pc - pa);
	double t4 = Dot(vq, pc - pa);
	double invdet = 1.0 / det;
	w01 = (+t1 * t3 - t2 * t4) * invdet;
	w23 = (+t2 * t3 - t0 * t4) * invdet;
	if (w01 < 0.0 || w01 > 1.0 || w23 < 0.0 || w23 > 1.0) {
		double w0_0 = w01;
		double w1_0 = w23;
		double inv_t0 = 1.0 / t0;
		double inv_t1 = 1.0 / t1;
		if (w1_0 < 0.0)			w01 = t3 * inv_t0;
		else if (w1_0 > 1.0)	w01 = (t3 + t2) * inv_t0;
		if (w0_0 < 0.0)			w23 = -t4 * inv_t1;
		else if (w0_0 > 1.0)	w23 = (-t4 + t2) * inv_t1;
		if (w01 < 0.0)			w01 = 0.0;
		else if (w01 > 1.0)		w01 = 1.0;
		if (w23 < 0.0)			w23 = 0.0;
		else if (w23 > 1.0)		w23 = 1.0;
		*w0 = w01;
		*w1 = w23;
		Vec3 normal = Normalize(Cross(vp, vq));
		Vec3 p = pa + w01 * vp - pc - w23 * vq;
		double h = Dot(p, normal);
		double distSqr = LengthSquared(p) - h * h;
		if (distSqr <= 0.0)
			return fabs(h);
		else if (sqrt(distSqr) > error)
			return DBL_MAX;
		return fabs(h);
	}
	*w0 = w01;
	*w1 = w23;
	return Length(pa + w01 * vp - pc - w23 * vq);*/
	const Vec3& vp = pb - pa;
	const Vec3& vq = pd - pc;
	double t0 = Dot(vp, vp);
	double t1 = Dot(vq, vq);
	double t2 = Dot(vp, vq);
	double det = t0 * t1 - t2 * t2;
	if (fabs(det) < 1.0e-20) {
		double lp0 = Dot(pa, vp);
		double lp1 = Dot(pb, vp);
		double lq0 = Dot(pc, vp);
		double lq1 = Dot(pd, vp);
		double p_min = (lp0 < lp1) ? lp0 : lp1;
		double p_max = (lp0 > lp1) ? lp0 : lp1;
		double q_min = (lq0 < lq1) ? lq0 : lq1;
		double q_max = (lq0 > lq1) ? lq0 : lq1;
		double lm;
		if (p_max < q_min)		lm = (p_max + q_min) * 0.5;
		else if (p_min > q_max) lm = (q_max + p_min) * 0.5;
		else if (p_max < q_max)
			if (p_min < q_min)	lm = (p_max + q_min) * 0.5;
			else				lm = (p_max + p_min) * 0.5;
		else
			if (p_min < q_min)	lm = (q_max + q_min) * 0.5;
			else				lm = (q_max + p_min) * 0.5;
		*w0 = (lm - lp0) / (lp1 - lp0);
		if (*w0 < 0.0 || *w0 > 1.0) return DBL_MAX;
		*w1 = (lm - lq0) / (lq1 - lq0);
		if (*w1 < 0.0 || *w1 > 1.0) return DBL_MAX;
		if (isnan(*w0) || isnan(*w1)) {
			printf("%f, %f, %f, %f, %f, %f\n", vp.x, vp.y, vp.z, vq.x, vq.y, vq.z);
			printf("%f, %f, %f\n", t0, t1, t2);
		}
		Vec3 nvp = Normalize(vp);
		Vec3 ppc = pa - pc;
		Vec3 vert = ppc - Dot(ppc, nvp) * nvp;
		return Length(vert);
	}
	double t3 = Dot(vp, pc - pa);
	double t4 = Dot(vq, pc - pa);
	double invdet = 1.0 / det;
	*w0 = (+t1 * t3 - t2 * t4) * invdet;
	if (*w0 < 0.0 || *w0 > 1.0) return DBL_MAX;
	*w1 = (+t2 * t3 - t0 * t4) * invdet;
	if (*w1 < 0.0 || *w1 > 1.0) return DBL_MAX;
	if (isnan(*w0) || isnan(*w1)) {
		printf("%f, %f, %f, %f, %f, %f\n", vp.x, vp.y, vp.z, vq.x, vq.y, vq.z);
		printf("%f, %f, %f\n", t0, t1, t2);
	}

	return Length(pa + *w0 * vp - pc - *w1 * vq);
}
double CollisionSolverHost::getDistanceEV(
	const Vec3& v0, const Vec3& v1, const Vec3& p,
	double* w, double error)
{
	Vec3 normal = Normalize(v1 - v0);
	double t1 = Dot(p - v0, normal);
	double t2 = Dot(p - v1, normal);
	if (isnan(normal.x) || isnan(normal.y) || isnan(normal.z))
		printf("¿©±â´Ù CCdsfsefD!!!\n");
	if (t1 < 0.0) {
		if (-t1 > error)
			return DBL_MAX;
		*w = 0.0;
	}
	else if (t2 > 0.0) {
		if (t2 > error)
			return DBL_MAX;
		*w = 1.0;
	}
	else *w = t1 / (t1 - t2);
	double distsqr = LengthSquared(p - v0) - t1 * t1;
	if (distsqr < 0.0)	return 0.0;
	return sqrt(distsqr);
}

bool CollisionSolverHost::DetectionVV_CCD(
	const Vec3& p0, const Vec3& p1, const Vec3& q0, const Vec3& q1,
	double delta, double* t)
{
	/*if (min(p0.x, q1.x) - COL_EPS > max(p1.x, q1.x) ||
		min(p0.y, q1.y) - COL_EPS > max(p1.y, q1.y) ||
		min(p0.z, q1.z) - COL_EPS > max(p1.z, q1.z) ||
		max(p0.x, q1.x) + COL_EPS < min(p1.x, q1.x) ||
		max(p0.y, q1.y) + COL_EPS < min(p1.y, q1.y) ||
		max(p0.z, q1.z) + COL_EPS < min(p1.z, q1.z))
		return false;

	double deltaSqr = delta * delta;
	Vec3 x1 = p1 - p0;
	Vec3 v1 = q1 - q0 - x1;
	double fc = LengthSquared(x1) - deltaSqr + 1.0e-20;
	if (fc <= 0.0) {
		t = 0.0;
		return true;
	}
	double fa = LengthSquared(v1);
	double fb = Dot(x1, v1);

	double ts[2];
	int tn = QuadraticSolver(fa, fb + fb, fc, ts);
	for (int i = 0; i < tn; i++) {
		t = ts[i];
		if (t < 0.0 || t > 1.0)
			continue;
		if (LengthSquared(x1 + v1 * t) <= deltaSqr)
			return true;
	}
	return false;*/
	if (min(p0.x, q1.x) - delta > max(p1.x, q1.x) ||
		min(p0.y, q1.y) - delta > max(p1.y, q1.y) ||
		min(p0.z, q1.z) - delta > max(p1.z, q1.z) ||
		max(p0.x, q1.x) + delta < min(p1.x, q1.x) ||
		max(p0.y, q1.y) + delta < min(p1.y, q1.y) ||
		max(p0.z, q1.z) + delta < min(p1.z, q1.z))
		return false;

	double deltaSqr = delta * delta;
	Vec3 x1 = p1 - p0;
	Vec3 v1 = q1 - q0 - x1;
	double fa = LengthSquared(v1);
	double fb = Dot(x1, v1);

	*t = -fb / fa;
	if (*t >= 0.0 && *t <= 1.0) {
		if (LengthSquared(x1 + v1 * *t) < deltaSqr)
			return true;
	}
	*t = 1.0;
	if (LengthSquared(q1 - q0) < deltaSqr)
		return true;
	return false;
}
bool CollisionSolverHost::DetectionEV_CCD(
	const Vec3& p0, const Vec3& p1, const Vec3& p2,
	const Vec3& q0, const Vec3& q1, const Vec3& q2,
	double delta, double* t, double* w)
{
	if (min(p2.x, q2.x) - delta > max(max(max(p0.x, p1.x), q0.x), q1.x) ||
		min(p2.y, q2.y) - delta > max(max(max(p0.y, p1.y), q0.y), q1.y) ||
		min(p2.z, q2.z) - delta > max(max(max(p0.z, p1.z), q0.z), q1.z) ||
		max(p2.x, q2.x) + delta < min(min(min(p0.x, p1.x), q0.x), q1.x) ||
		max(p2.y, q2.y) + delta < min(min(min(p0.y, p1.y), q0.y), q1.y) ||
		max(p2.z, q2.z) + delta < min(min(min(p0.z, p1.z), q0.z), q1.z))
		return false;

	Vec3 x1 = p1 - p0; Vec3 x2 = p2 - p0;
	Vec3 v1 = q1 - q0 - x1; Vec3 v2 = q2 - q0 - x2;
	Vec3 va = Cross(v1, v2);
	Vec3 vb = Cross(x1, v2) + Cross(v1, x2);
	Vec3 vc = Cross(x1, x2);
	set<double> sts;
	{
		QuadraticSolver(va.x, vb.x, vc.x, sts);
		QuadraticSolver(va.y, vb.y, vc.y, sts);
		QuadraticSolver(va.z, vb.z, vc.z, sts);

		QuadraticSolver(Dot(v1, v2), Dot(x1, v2) + Dot(v1, x2), Dot(x1, x2), sts);
	}
	x1 = p0 - p1; x2 = p2 - p1;
	v1 = q0 - q1 - x1; v2 = q2 - q1 - x2;
	va = Cross(v1, v2);
	vb = Cross(x1, v2) + Cross(v1, x2);
	vc = Cross(x1, x2);
	{
		QuadraticSolver(va.x, vb.x, vc.x, sts);
		QuadraticSolver(va.y, vb.y, vc.y, sts);
		QuadraticSolver(va.z, vb.z, vc.z, sts);

		QuadraticSolver(Dot(v1, v2), Dot(x1, v2) + Dot(v1, x2), Dot(x1, x2), sts);
	}
#ifdef PLUS_TIME_1
	sts.insert(1.0);
#endif
	for (set<double>::iterator itr = sts.begin(); itr != sts.end(); itr++) {
		*t = *itr;
		if (*t < 0.0 || *t > 1.0)
			continue;
		Vec3 p0m = p0 + (q0 - p0) * *t;
		Vec3 p1m = p1 + (q1 - p1) * *t;
		Vec3 p2m = p2 + (q2 - p2) * *t;
		if (w) {
			double dist = getDistanceEV(p0m, p1m, p2m, w, COL_EPS);
			if (dist < delta)
				return true;
		}
		else {
			double _w;
			double dist = getDistanceEV(p0m, p1m, p2m, &_w, COL_EPS);
			if (dist < delta)
				return true;
		}
	}
	return false;
}
bool CollisionSolverHost::DetectionTV_CCD(
	const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
	const Vec3& q0, const Vec3& q1, const Vec3& q2, const Vec3& q3,
	double delta, double* t, double* w0, double* w1)
{
	if (min(p3.x, q3.x) - delta > max(max(max(max(max(p1.x, p2.x), p0.x), q1.x), q2.x), q0.x) ||
		min(p3.y, q3.y) - delta > max(max(max(max(max(p1.y, p2.y), p0.y), q1.y), q2.y), q0.y) ||
		min(p3.z, q3.z) - delta > max(max(max(max(max(p1.z, p2.z), p0.z), q1.z), q2.z), q0.z) ||
		max(p3.x, q3.x) + delta < min(min(min(min(min(p1.x, p2.x), p0.x), q1.x), q2.x), q0.x) ||
		max(p3.y, q3.y) + delta < min(min(min(min(min(p1.y, p2.y), p0.y), q1.y), q2.y), q0.y) ||
		max(p3.z, q3.z) + delta < min(min(min(min(min(p1.z, p2.z), p0.z), q1.z), q2.z), q0.z))
		return false;

	double ts[3];
	int num = FinePlaneCoTime(p0, p1, p2, p3, q0, q1, q2, q3, ts);

	for (int i = 0; i < num; i++) {
		if (ts[i] < 0.0 || ts[i] > 1.0)
			continue;
		*t = ts[i];
		Vec3 p0m = p0 + (q0 - p0) * ts[i];
		Vec3 p1m = p1 + (q1 - p1) * ts[i];
		Vec3 p2m = p2 + (q2 - p2) * ts[i];
		Vec3 p3m = p3 + (q3 - p3) * ts[i];
		if (w0 && w1) {
			double dist = getDistanceTV(p0m, p1m, p2m, p3m, w0, w1, COL_EPS);
			if (dist == DBL_MAX)
				continue;
		}
		else {
			double _w0, _w1;
			double dist = getDistanceTV(p0m, p1m, p2m, p3m, &_w0, &_w1, COL_EPS);
			if (dist == DBL_MAX)
				continue;
		}
		return true;
	}
#ifdef PLUS_TIME_1
	if (*t < 1.0) {
		*t = 1.0;
		if (w0 && w1) {
			double dist = getDistanceTV(q0, q1, q2, q3, w0, w1, COL_EPS);
			if (dist < delta)
				return true;
		}
		else {
			double _w0, _w1;
			double dist = getDistanceTV(q0, q1, q2, q3, &_w0, &_w1, COL_EPS);
			if (dist < delta)
				return true;
		}
	}
#endif
	return false;
}
bool CollisionSolverHost::DetectionEE_CCD(
	const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
	const Vec3& q0, const Vec3& q1, const Vec3& q2, const Vec3& q3,
	double delta, double* t, double* w0, double* w1)
{
	if (min(min(min(p0.x, p1.x), q0.x), q1.x) - delta > max(max(max(p2.x, p3.x), q2.x), q3.x) ||
		min(min(min(p0.y, p1.y), q0.y), q1.y) - delta > max(max(max(p2.y, p3.y), q2.y), q3.y) ||
		min(min(min(p0.z, p1.z), q0.z), q1.z) - delta > max(max(max(p2.z, p3.z), q2.z), q3.z) ||
		max(max(max(p0.x, p1.x), q0.x), q1.x) + delta < min(min(min(p2.x, p3.x), q2.x), q3.x) ||
		max(max(max(p0.y, p1.y), q0.y), q1.y) + delta < min(min(min(p2.y, p3.y), q2.y), q3.y) ||
		max(max(max(p0.z, p1.z), q0.z), q1.z) + delta < min(min(min(p2.z, p3.z), q2.z), q3.z))
		return false;

	double ts[3];
	bool isParallel;

	int num = FinePlaneCoTime(p0, p1, p2, p3, q0, q1, q2, q3, ts);
	for (int i = 0; i < num; i++) {
		if (ts[i] < 0.0 || ts[i] > 1.0)
			continue;
		*t = ts[i];
		Vec3 p0m = p0 + (q0 - p0) * ts[i];
		Vec3 p1m = p1 + (q1 - p1) * ts[i];
		Vec3 p2m = p2 + (q2 - p2) * ts[i];
		Vec3 p3m = p3 + (q3 - p3) * ts[i];
		if (w0 && w1) {
			double dist = getDistanceEE(p0m, p1m, p2m, p3m, w0, w1, &isParallel, COL_EPS);
			if (dist == DBL_MAX)
				continue;
			if (isParallel && dist >= delta)
				continue;
		}
		else {
			double _w0, _w1;
			double dist = getDistanceEE(p0m, p1m, p2m, p3m, &_w0, &_w1, &isParallel, COL_EPS);
			if (dist == DBL_MAX)
				continue;
			if (isParallel && dist >= delta)
				continue;
		}
		return true;
	}
#ifdef PLUS_TIME_1
	if (*t < 1.0) {
		*t = 1.0;
		if (w0 && w1) {
			double dist = getDistanceEE(q0, q1, q2, q3, w0, w1, &isParallel, COL_EPS);
			if (dist < delta)
				return true;
		}
		else {
			double _w0, _w1;
			double dist = getDistanceEE(q0, q1, q2, q3, &_w0, &_w1, &isParallel, COL_EPS);
			if (dist < delta)
				return true;
		}
	}
#endif
	return false;
}

bool CollisionSolverHost::isContactTV_Proximity(
	const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
	double delta, double* w0, double* w1)
{
	if (p3.x - delta > max(max(p1.x, p2.x), p0.x) ||
		p3.y - delta > max(max(p1.y, p2.y), p0.y) ||
		p3.z - delta > max(max(p1.z, p2.z), p0.z) ||
		p3.x + delta < min(min(p1.x, p2.x), p0.x) ||
		p3.y + delta < min(min(p1.y, p2.y), p0.y) ||
		p3.z + delta < min(min(p1.z, p2.z), p0.z))
		return false;

	if (w0 && w1) {
		double dist = getDistanceTV(p0, p1, p2, p3, w0, w1, COL_EPS);
		if (dist >= delta) return false;
	}
	else {
		double _w0, _w1;
		double dist = getDistanceTV(p0, p1, p2, p3, &_w0, &_w1, COL_EPS);
		if (dist >= delta) return false;
	}
	return true;
}
bool CollisionSolverHost::isContactEE_Proximity(
	const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
	double delta, double* w0, double* w1)
{
	if (min(p0.x, p1.x) - delta > max(p2.x, p3.x) ||
		min(p0.y, p1.y) - delta > max(p2.y, p3.y) ||
		min(p0.z, p1.z) - delta > max(p2.z, p3.z) ||
		max(p0.x, p1.x) + delta < min(p2.x, p3.x) ||
		max(p0.y, p1.y) + delta < min(p2.y, p3.y) ||
		max(p0.z, p1.z) + delta < min(p2.z, p3.z))
		return false;

	if (w0 && w1) {
		double dist = getDistanceEE(p0, p1, p2, p3, w0, w1, nullptr, COL_EPS);
		if (dist >= delta) return false;
	}
	else {
		double _w0, _w1;
		double dist = getDistanceEE(p0, p1, p2, p3, &_w0, &_w1, nullptr, COL_EPS);
		if (dist >= delta) return false;
	}
	return true;
}
bool CollisionSolverHost::isSelfContactTV_Proximity(
	int i0, int i1, int i2, int i3,
	const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
	double delta, double* w0, double* w1)
{
	if (i3 == i0 || i3 == i1 || i3 == i2)
		return false;

	return isContactTV_Proximity(p0, p1, p2, p3, delta, w0, w1);
}
bool CollisionSolverHost::isSelfContactEE_Proximity(
	int i0, int i1, int i2, int i3,
	const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
	double delta, double* w0, double* w1)
{
	if (i0 == i2 || i0 == i3 || i1 == i2 || i1 == i3)
		return false;

	return isContactEE_Proximity(p0, p1, p2, p3, delta, w0, w1);
}

bool CollisionSolverHost::isContactTV_CCD(
	const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
	const Vec3& q0, const Vec3& q1, const Vec3& q2, const Vec3& q3,
	double* t, double* w0, double* w1)
{
	if (w0 && w1) {
		if (DetectionTV_CCD(p0, p1, p2, p3, q0, q1, q2, q3, COL_TV, t, w0, w1)) {
#ifdef CCD_PRINT_TRI_TV
			if (Debugging)
				printf("r tri0 %f %f %f\n", *w0, *w1, *t);
#endif
			return true;
		}

#ifdef USED_EV_VV
		if (DetectionEV_CCD(p1, p0, p3, q1, q0, q3, COL_EV, t, w0)) {
			*w1 = 1.0 - *w0;
#ifdef CCD_PRINT_TRI_EV
			if (Debugging)
				printf("r tri1 %f %f %f\n", *w0, *w1, *t);
#endif
			return true;
		}
		if (DetectionEV_CCD(p2, p1, p3, q2, q1, q3, COL_EV, t, w1)) {
			*w0 = 0.0;
#ifdef CCD_PRINT_TRI_EV
			if (Debugging)
				printf("r tri2 %f %f %f\n", *w0, *w1, *t);
#endif
			return true;
		}
		if (DetectionEV_CCD(p2, p0, p3, q2, q0, q3, COL_EV, t, w0)) {
			*w1 = 0.0;
#ifdef CCD_PRINT_TRI_EV
			if (Debugging)
				printf("r tri3 %f %f %f\n", *w0, *w1, *t);
#endif
			return true;
		}
		if (DetectionVV_CCD(p0, p3, q0, q3, COL_VV, t)) {
			*w0 = 1.0; *w1 = 0.0;
#ifdef CCD_PRINT_TRI_VV
			if (Debugging)
				printf("r tri4 %f %f %f\n", *w0, *w1, *t);
#endif
			return true;
		}
		if (DetectionVV_CCD(p1, p3, q1, q3, COL_VV, t)) {
			*w0 = 0.0; *w1 = 1.0;
#ifdef CCD_PRINT_TRI_VV
			if (Debugging)
				printf("r tri5 %f %f %f\n", *w0, *w1, *t);
#endif
			return true;
		}
		if (DetectionVV_CCD(p2, p3, q2, q3, COL_VV, t)) {
			*w0 = 0.0; *w1 = 0.0;
#ifdef CCD_PRINT_TRI_VV
			if (Debugging)
				printf("r tri6 %f %f %f\n", *w0, *w1, *t);
#endif
			return true;
		}
#endif
	}
	else {
		if (DetectionTV_CCD(p0, p1, p2, p3, q0, q1, q2, q3, COL_TV, t)) {
#ifdef CCD_PRINT_TRI_TV
			if (Debugging)
				printf("tri0 %f\n", *t);
#endif
			return true;
		}

#ifdef USED_EV_VV
		if (DetectionEV_CCD(p1, p0, p3, q1, q0, q3, COL_EV, t)) {
#ifdef CCD_PRINT_TRI_EV
			if (Debugging)
				printf("tri1 %f\n", *t);
#endif
			return true;
		}
		if (DetectionEV_CCD(p2, p1, p3, q2, q1, q3, COL_EV, t)) {
#ifdef CCD_PRINT_TRI_EV
			if (Debugging)
				printf("tri2 %f\n", *t);
#endif
			return true;
		}
		if (DetectionEV_CCD(p2, p0, p3, q2, q0, q3, COL_EV, t)) {
#ifdef CCD_PRINT_TRI_EV
			if (Debugging)
				printf("tri3 %f\n", *t);
#endif
			return true;
		}
		if (DetectionVV_CCD(p0, p3, q0, q3, COL_VV, t)) {
#ifdef CCD_PRINT_TRI_VV
			if (Debugging)
				printf("tri4 %f\n", *t);
#endif
			return true;
		}
		if (DetectionVV_CCD(p1, p3, q1, q3, COL_VV, t)) {
#ifdef CCD_PRINT_TRI_VV
			if (Debugging)
				printf("tri5 %f\n", *t);
#endif
			return true;
		}
		if (DetectionVV_CCD(p2, p3, q2, q3, COL_VV, t)) {
#ifdef CCD_PRINT_TRI_VV
			if (Debugging)
				printf("tri6 %f\n", *t);
#endif
			return true;
		}
#endif
	}
	return false;
}
bool CollisionSolverHost::isContactEE_CCD(
	const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
	const Vec3& q0, const Vec3& q1, const Vec3& q2, const Vec3& q3,
	double* t, double* w0, double* w1)
{
	if (w0 && w1) {
		if (DetectionEE_CCD(p0, p1, p2, p3, q0, q1, q2, q3, COL_EE, t, w0, w1)) {
#ifdef CCD_PRINT_EDGE_EE
			if (Debugging)
				printf("r edge0 %f %f %f\n", *w0, *w1, *t);
#endif
			return true;
		}

#ifdef USED_EV_VV
		if (DetectionEV_CCD(p0, p1, p2, q0, q1, q2, COL_EV, t, w0)) {
			*w1 = 0.0;
#ifdef CCD_PRINT_EDGE_EV
			if (Debugging)
				printf("r edge1 %f %f %f\n", *w0, *w1, *t);
#endif
			return true;
		}
		if (DetectionEV_CCD(p0, p1, p3, q0, q1, q3, COL_EV, t, w0)) {
			*w1 = 1.0;
#ifdef CCD_PRINT_EDGE_EV
			if (Debugging)
				printf("r edge2 %f %f %f\n", *w0, *w1, *t);
#endif
			return true;
		}
		if (DetectionEV_CCD(p2, p3, p0, q2, q3, q0, COL_EV, t, w1)) {
			*w0 = 0.0;
#ifdef CCD_PRINT_EDGE_EV
			if (Debugging)
				printf("r edge3 %f %f %f\n", *w0, *w1, *t);
#endif
			return true;
		}
		if (DetectionEV_CCD(p2, p3, p1, q2, q3, q1, COL_EV, t, w1)) {
			*w0 = 1.0;
#ifdef CCD_PRINT_EDGE_EV
			if (Debugging)
				printf("r edge4 %f %f %f\n", *w0, *w1, *t);
#endif
			return true;
		}
		if (DetectionVV_CCD(p0, p2, q0, q2, COL_VV, t)) {
			*w0 = 0.0; *w1 = 0.0;
#ifdef CCD_PRINT_EDGE_VV
			if (Debugging)
				printf("r edge5 %f %f %f\n", *w0, *w1, *t);
#endif
			return true;
		}
		if (DetectionVV_CCD(p0, p3, q0, q3, COL_VV, t)) {
			*w0 = 0.0; *w1 = 1.0;
#ifdef CCD_PRINT_EDGE_VV
			if (Debugging)
				printf("r edge6 %f %f %f\n", *w0, *w1, *t);
#endif
			return true;
		}
		if (DetectionVV_CCD(p1, p2, q1, q2, COL_VV, t)) {
			*w0 = 1.0; *w1 = 0.0;
#ifdef CCD_PRINT_EDGE_VV
			if (Debugging)
				printf("r edge7 %f %f %f\n", *w0, *w1, *t);
#endif
			return true;
		}
		if (DetectionVV_CCD(p1, p3, q1, q3, COL_VV, t)) {
			*w0 = 1.0; *w1 = 1.0;
#ifdef CCD_PRINT_EDGE_VV
			if (Debugging)
				printf("r edge8 %f %f %f\n", *w0, *w1, *t);
#endif
			return true;
		}
#endif
	}
	else {
		if (DetectionEE_CCD(p0, p1, p2, p3, q0, q1, q2, q3, COL_EE, t)) {
#ifdef CCD_PRINT_EDGE_EE
			if (Debugging)
				printf("edge0 %f\n", *t);
#endif
			return true;
		}

#ifdef USED_EV_VV
		if (DetectionEV_CCD(p0, p1, p2, q0, q1, q2, COL_EV, t)) {
#ifdef CCD_PRINT_EDGE_EV
			if (Debugging)
				printf("edge1 %f\n", *t);
#endif
			return true;
		}
		if (DetectionEV_CCD(p0, p1, p3, q0, q1, q3, COL_EV, t)) {
#ifdef CCD_PRINT_EDGE_EV
			if (Debugging)
				printf("edge2 %f\n", *t);
#endif
			return true;
		}
		if (DetectionEV_CCD(p2, p3, p0, q2, q3, q0, COL_EV, t)) {
#ifdef CCD_PRINT_EDGE_EV
			if (Debugging)
				printf("edge3 %f\n", *t);
#endif
			return true;
		}
		if (DetectionEV_CCD(p2, p3, p1, q2, q3, q1, COL_EV, t)) {
#ifdef CCD_PRINT_EDGE_EV
			if (Debugging)
				printf("edge4 %f\n", *t);
#endif
			return true;
		}
		if (DetectionVV_CCD(p0, p2, q0, q2, COL_VV, t)) {
#ifdef CCD_PRINT_EDGE_VV
			if (Debugging)
				printf("edge5 %f\n", *t);
#endif
			return true;
		}
		if (DetectionVV_CCD(p0, p3, q0, q3, COL_VV, t)) {
#ifdef CCD_PRINT_EDGE_VV
			if (Debugging)
				printf("edge6 %f\n", *t);
#endif
			return true;
		}
		if (DetectionVV_CCD(p1, p2, q1, q2, COL_VV, t)) {
#ifdef CCD_PRINT_EDGE_VV
			if (Debugging)
				printf("edge7 %f\n", *t);
#endif
			return true;
		}
		if (DetectionVV_CCD(p1, p3, q1, q3, COL_VV, t)) {
#ifdef CCD_PRINT_EDGE_VV
			if (Debugging)
				printf("edge8 %f\n", *t);
#endif
			return true;
		}
#endif
	}
	return false;
}
bool CollisionSolverHost::isSelfContactTV_CCD(
	int i0, int i1, int i2, int i3,
	const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
	const Vec3& q0, const Vec3& q1, const Vec3& q2, const Vec3& q3,
	double* t, double* w0, double* w1)
{
	if (i3 == i0 || i3 == i1 || i3 == i2)
		return false;
	return isContactTV_CCD(p0, p1, p2, p3, q0, q1, q2, q3, t, w0, w1);
}
bool CollisionSolverHost::isSelfContactEE_CCD(
	int i0, int i1, int i2, int i3,
	const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
	const Vec3& q0, const Vec3& q1, const Vec3& q2, const Vec3& q3,
	double* t, double* w0, double* w1)
{
	if (i0 == i2 || i0 == i3 || i1 == i2 || i1 == i3)
		return false;

	return isContactEE_CCD(p0, p1, p2, p3, q0, q1, q2, q3, t, w0, w1);
}

void CollisionSolverHost::getSelfContactElements_Proximity(
	set<ClothCollisionElement>& contacts,
	const vector<int>& faces,
	const vector<double>& vertices,
	vector<double>& velocities,
	BVHNodeHost* nodeL, BVHNodeHost* nodeR, double thickness, double dt)
{
	if (!nodeL->intersect(nodeR)) { return; }
	auto isLeafL = (nodeL->_childs[0] == nullptr);
	auto isLeafR = (nodeR->_childs[0] == nullptr);
	if (!isLeafL && !isLeafR) {
		getSelfContactElements_Proximity(contacts, faces, vertices, velocities, nodeL->_childs[0], nodeR->_childs[0], thickness, dt);
		getSelfContactElements_Proximity(contacts, faces, vertices, velocities, nodeL->_childs[1], nodeR->_childs[0], thickness, dt);
		getSelfContactElements_Proximity(contacts, faces, vertices, velocities, nodeL->_childs[0], nodeR->_childs[1], thickness, dt);
		getSelfContactElements_Proximity(contacts, faces, vertices, velocities, nodeL->_childs[1], nodeR->_childs[1], thickness, dt);
	}
	else if (isLeafL && !isLeafR) {
		getSelfContactElements_Proximity(contacts, faces, vertices, velocities, nodeL, nodeR->_childs[0], thickness, dt);
		getSelfContactElements_Proximity(contacts, faces, vertices, velocities, nodeL, nodeR->_childs[1], thickness, dt);
	}
	else if (!isLeafL && isLeafR) {
		getSelfContactElements_Proximity(contacts, faces, vertices, velocities, nodeL->_childs[0], nodeR, thickness, dt);
		getSelfContactElements_Proximity(contacts, faces, vertices, velocities, nodeL->_childs[1], nodeR, thickness, dt);
	}
	else {
		for (auto i : nodeL->_faces) {
			int i0 = faces[i * 3 + 0] * 3;
			int i1 = faces[i * 3 + 1] * 3;
			int i2 = faces[i * 3 + 2] * 3;
			const Vec3 pa0(vertices[i0 + 0], vertices[i0 + 1], vertices[i0 + 2]);
			const Vec3 pb0(vertices[i1 + 0], vertices[i1 + 1], vertices[i1 + 2]);
			const Vec3 pc0(vertices[i2 + 0], vertices[i2 + 1], vertices[i2 + 2]);
			i0 = faces[i * 3 + 0]; 
			i1 = faces[i * 3 + 1];
			i2 = faces[i * 3 + 2];
			for (auto j : nodeR->_faces) {
				int j0 = faces[j * 3 + 0] * 3;
				int j1 = faces[j * 3 + 1] * 3;
				int j2 = faces[j * 3 + 2] * 3;
				const Vec3 qa0(vertices[j0 + 0], vertices[j0 + 1], vertices[j0 + 2]);
				const Vec3 qb0(vertices[j1 + 0], vertices[j1 + 1], vertices[j1 + 2]);
				const Vec3 qc0(vertices[j2 + 0], vertices[j2 + 1], vertices[j2 + 2]);
				j0 = faces[j * 3 + 0];
				j1 = faces[j * 3 + 1]; 
				j2 = faces[j * 3 + 2];

#ifndef USED_R_TRI
				if (isSelfContactTV_Proximity(j0, j1, j2, i0, qa0, qb0, qc0, pa0, thickness))
					contacts.insert(ClothCollisionElement(true, j0, j1, j2, i0));
				if (isSelfContactTV_Proximity(j0, j1, j2, i1, qa0, qb0, qc0, pb0, thickness))
					contacts.insert(ClothCollisionElement(true, j0, j1, j2, i1));
				if (isSelfContactTV_Proximity(j0, j1, j2, i2, qa0, qb0, qc0, pc0, thickness))
					contacts.insert(ClothCollisionElement(true, j0, j1, j2, i2));
				if (isSelfContactTV_Proximity(i0, i1, i2, j0, pa0, pb0, pc0, qa0, thickness))
					contacts.insert(ClothCollisionElement(true, i0, i1, i2, j0));
				if (isSelfContactTV_Proximity(i0, i1, i2, j1, pa0, pb0, pc0, qb0, thickness))
					contacts.insert(ClothCollisionElement(true, i0, i1, i2, j1));
				if (isSelfContactTV_Proximity(i0, i1, i2, j2, pa0, pb0, pc0, qc0, thickness))
					contacts.insert(ClothCollisionElement(true, i0, i1, i2, j2));

				if (isSelfContactEE_Proximity(i0, i1, j0, j1, pa0, pb0, qa0, qb0, thickness))
					contacts.insert(ClothCollisionElement(false, i0, i1, j0, j1));
				if (isSelfContactEE_Proximity(i0, i1, j1, j2, pa0, pb0, qb0, qc0, thickness))
					contacts.insert(ClothCollisionElement(false, i0, i1, j1, j2));
				if (isSelfContactEE_Proximity(i0, i1, j2, j0, pa0, pb0, qc0, qa0, thickness))
					contacts.insert(ClothCollisionElement(false, i0, i1, j2, j0));
				if (isSelfContactEE_Proximity(i1, i2, j0, j1, pb0, pc0, qa0, qb0, thickness))
					contacts.insert(ClothCollisionElement(false, i1, i2, j0, j1));
				if (isSelfContactEE_Proximity(i1, i2, j1, j2, pb0, pc0, qb0, qc0, thickness))
					contacts.insert(ClothCollisionElement(false, i1, i2, j1, j2));
				if (isSelfContactEE_Proximity(i1, i2, j2, j0, pb0, pc0, qc0, qa0, thickness))
					contacts.insert(ClothCollisionElement(false, i1, i2, j2, j0));
				if (isSelfContactEE_Proximity(i2, i0, j0, j1, pc0, pa0, qa0, qb0, thickness))
					contacts.insert(ClothCollisionElement(false, i2, i0, j0, j1));
				if (isSelfContactEE_Proximity(i2, i0, j1, j2, pc0, pa0, qb0, qc0, thickness))
					contacts.insert(ClothCollisionElement(false, i2, i0, j1, j2));
				if (isSelfContactEE_Proximity(i2, i0, j2, j0, pc0, pa0, qc0, qa0, thickness))
					contacts.insert(ClothCollisionElement(false, i2, i0, j2, j0));
#else
				if (nodeL->RTriVertex(0))
					if (isSelfContactTV_Proximity(j0, j1, j2, i0, qa0, qb0, qc0, pa0, thickness))
						contacts.insert(ClothCollisionElement(true, j0, j1, j2, i0));
				if (nodeL->RTriVertex(1))
					if (isSelfContactTV_Proximity(j0, j1, j2, i1, qa0, qb0, qc0, pb0, thickness))
						contacts.insert(ClothCollisionElement(true, j0, j1, j2, i1));
				if (nodeL->RTriVertex(2))
					if (isSelfContactTV_Proximity(j0, j1, j2, i2, qa0, qb0, qc0, pc0, thickness))
						contacts.insert(ClothCollisionElement(true, j0, j1, j2, i2));
				if (nodeR->RTriVertex(0))
					if (isSelfContactTV_Proximity(i0, i1, i2, j0, pa0, pb0, pc0, qa0, thickness))
						contacts.insert(ClothCollisionElement(true, i0, i1, i2, j0));
				if (nodeR->RTriVertex(1))
					if (isSelfContactTV_Proximity(i0, i1, i2, j1, pa0, pb0, pc0, qb0, thickness))
						contacts.insert(ClothCollisionElement(true, i0, i1, i2, j1));
				if (nodeR->RTriVertex(2))
					if (isSelfContactTV_Proximity(i0, i1, i2, j2, pa0, pb0, pc0, qc0, thickness))
						contacts.insert(ClothCollisionElement(true, i0, i1, i2, j2));

				if (nodeL->RTriEdge(0)) {
					if (nodeR->RTriEdge(0))
						if (isSelfContactEE_Proximity(i0, i1, j0, j1, pa0, pb0, qa0, qb0, thickness))
							contacts.insert(ClothCollisionElement(false, i0, i1, j0, j1));
					if (nodeR->RTriEdge(1))
						if (isSelfContactEE_Proximity(i0, i1, j1, j2, pa0, pb0, qb0, qc0, thickness))
							contacts.insert(ClothCollisionElement(false, i0, i1, j1, j2));
					if (nodeR->RTriEdge(2))
						if (isSelfContactEE_Proximity(i0, i1, j2, j0, pa0, pb0, qc0, qa0, thickness))
							contacts.insert(ClothCollisionElement(false, i0, i1, j2, j0));
				}
				if (nodeL->RTriEdge(1)) {
					if (nodeR->RTriEdge(0))
						if (isSelfContactEE_Proximity(i1, i2, j0, j1, pb0, pc0, qa0, qb0, thickness))
							contacts.insert(ClothCollisionElement(false, i1, i2, j0, j1));
					if (nodeR->RTriEdge(1))
						if (isSelfContactEE_Proximity(i1, i2, j1, j2, pb0, pc0, qb0, qc0, thickness))
							contacts.insert(ClothCollisionElement(false, i1, i2, j1, j2));
					if (nodeR->RTriEdge(2))
						if (isSelfContactEE_Proximity(i1, i2, j2, j0, pb0, pc0, qc0, qa0, thickness))
							contacts.insert(ClothCollisionElement(false, i1, i2, j2, j0));
				}
				if (nodeL->RTriEdge(2)) {
					if (nodeR->RTriEdge(0))
						if (isSelfContactEE_Proximity(i2, i0, j0, j1, pc0, pa0, qa0, qb0, thickness))
							contacts.insert(ClothCollisionElement(false, i2, i0, j0, j1));
					if (nodeR->RTriEdge(1))
						if (isSelfContactEE_Proximity(i2, i0, j1, j2, pc0, pa0, qb0, qc0, thickness))
							contacts.insert(ClothCollisionElement(false, i2, i0, j1, j2));
					if (nodeR->RTriEdge(2))
						if (isSelfContactEE_Proximity(i2, i0, j2, j0, pc0, pa0, qc0, qa0, thickness))
							contacts.insert(ClothCollisionElement(false, i2, i0, j2, j0));
				}
#endif
			}
		}
	}
}
void CollisionSolverHost::getSelfContactElements_Proximity
(set<ClothCollisionElement>& contacts,
	const vector<int>& faces,
	const vector<double>& vertices,
	vector<double>& velocities,
	BVHNodeHost* node, double thickness, double dt)
{
	if (!node->_childs[0]) return;
	getSelfContactElements_Proximity(contacts, faces, vertices, velocities, node->_childs[0], node->_childs[1], thickness, dt);
	getSelfContactElements_Proximity(contacts, faces, vertices, velocities, node->_childs[0], thickness, dt);
	getSelfContactElements_Proximity(contacts, faces, vertices, velocities, node->_childs[1], thickness, dt);
}
void CollisionSolverHost::getSelfContactElements_CCD(
	set<ClothCollisionElement>& contacts,
	const vector<int>& faces,
	const vector<double>& vertices,
	vector<double>& velocities,
	BVHNodeHost* nodeL, BVHNodeHost* nodeR, double dt)
{
	if (!nodeL->intersect(nodeR)) { return; }
	auto isLeafL = (nodeL->_childs[0] == nullptr);
	auto isLeafR = (nodeR->_childs[0] == nullptr);
	double t;
	if (!isLeafL && !isLeafR) {
		getSelfContactElements_CCD(contacts, faces, vertices, velocities, nodeL->_childs[0], nodeR->_childs[0], dt);
		getSelfContactElements_CCD(contacts, faces, vertices, velocities, nodeL->_childs[1], nodeR->_childs[0], dt);
		getSelfContactElements_CCD(contacts, faces, vertices, velocities, nodeL->_childs[0], nodeR->_childs[1], dt);
		getSelfContactElements_CCD(contacts, faces, vertices, velocities, nodeL->_childs[1], nodeR->_childs[1], dt);
	}
	else if (isLeafL && !isLeafR) {
		getSelfContactElements_CCD(contacts, faces, vertices, velocities, nodeL, nodeR->_childs[0], dt);
		getSelfContactElements_CCD(contacts, faces, vertices, velocities, nodeL, nodeR->_childs[1], dt);
	}
	else if (!isLeafL && isLeafR) {
		getSelfContactElements_CCD(contacts, faces, vertices, velocities, nodeL->_childs[0], nodeR, dt);
		getSelfContactElements_CCD(contacts, faces, vertices, velocities, nodeL->_childs[1], nodeR, dt);
	}
	else {
		for (auto i : nodeL->_faces) {
			int i0 = faces[i * 3 + 0] * 3;
			int i1 = faces[i * 3 + 1] * 3;
			int i2 = faces[i * 3 + 2] * 3;
			const Vec3 pa0(vertices[i0 + 0], vertices[i0 + 1], vertices[i0 + 2]);
			const Vec3 pb0(vertices[i1 + 0], vertices[i1 + 1], vertices[i1 + 2]);
			const Vec3 pc0(vertices[i2 + 0], vertices[i2 + 1], vertices[i2 + 2]);
			const Vec3 pa1(pa0.x + velocities[i0 + 0] * dt, pa0.y + velocities[i0 + 1] * dt, pa0.z + velocities[i0 + 2] * dt);
			const Vec3 pb1(pb0.x + velocities[i1 + 0] * dt, pb0.y + velocities[i1 + 1] * dt, pb0.z + velocities[i1 + 2] * dt);
			const Vec3 pc1(pc0.x + velocities[i2 + 0] * dt, pc0.y + velocities[i2 + 1] * dt, pc0.z + velocities[i2 + 2] * dt);
			i0 = faces[i * 3 + 0];
			i1 = faces[i * 3 + 1]; 
			i2 = faces[i * 3 + 2];
			for (auto j : nodeR->_faces) {
				int j0 = faces[j * 3 + 0] * 3;
				int j1 = faces[j * 3 + 1] * 3;
				int j2 = faces[j * 3 + 2] * 3;
				const Vec3 qa0(vertices[j0 + 0], vertices[j0 + 1], vertices[j0 + 2]);
				const Vec3 qb0(vertices[j1 + 0], vertices[j1 + 1], vertices[j1 + 2]);
				const Vec3 qc0(vertices[j2 + 0], vertices[j2 + 1], vertices[j2 + 2]);
				const Vec3 qa1(qa0.x + velocities[j0 + 0] * dt, qa0.y + velocities[j0 + 1] * dt, qa0.z + velocities[j0 + 2] * dt);
				const Vec3 qb1(qb0.x + velocities[j1 + 0] * dt, qb0.y + velocities[j1 + 1] * dt, qb0.z + velocities[j1 + 2] * dt);
				const Vec3 qc1(qc0.x + velocities[j2 + 0] * dt, qc0.y + velocities[j2 + 1] * dt, qc0.z + velocities[j2 + 2] * dt);
				j0 = faces[j * 3 + 0];
				j1 = faces[j * 3 + 1]; 
				j2 = faces[j * 3 + 2];
#ifndef USED_R_TRI
				if (isSelfContactTV_CCD(j0, j1, j2, i0, qa0, qb0, qc0, pa0, qa1, qb1, qc1, pa1, &t))
					contacts.insert(ClothCollisionElement(true, j0, j1, j2, i0, t));
				if (isSelfContactTV_CCD(j0, j1, j2, i1, qa0, qb0, qc0, pb0, qa1, qb1, qc1, pb1, &t))
					contacts.insert(ClothCollisionElement(true, j0, j1, j2, i1, t));
				if (isSelfContactTV_CCD(j0, j1, j2, i2, qa0, qb0, qc0, pc0, qa1, qb1, qc1, pc1, &t))
					contacts.insert(ClothCollisionElement(true, j0, j1, j2, i2, t));
				if (isSelfContactTV_CCD(i0, i1, i2, j0, pa0, pb0, pc0, qa0, pa1, pb1, pc1, qa1, &t))
					contacts.insert(ClothCollisionElement(true, i0, i1, i2, j0, t));
				if (isSelfContactTV_CCD(i0, i1, i2, j1, pa0, pb0, pc0, qb0, pa1, pb1, pc1, qb1, &t))
					contacts.insert(ClothCollisionElement(true, i0, i1, i2, j1, t));
				if (isSelfContactTV_CCD(i0, i1, i2, j2, pa0, pb0, pc0, qc0, pa1, pb1, pc1, qc1, &t))
					contacts.insert(ClothCollisionElement(true, i0, i1, i2, j2, t));

				if (isSelfContactEE_CCD(i0, i1, j0, j1, pa0, pb0, qa0, qb0, pa1, pb1, qa1, qb1, &t))
					contacts.insert(ClothCollisionElement(false, i0, i1, j0, j1, t));
				if (isSelfContactEE_CCD(i0, i1, j1, j2, pa0, pb0, qb0, qc0, pa1, pb1, qb1, qc1, &t))
					contacts.insert(ClothCollisionElement(false, i0, i1, j1, j2, t));
				if (isSelfContactEE_CCD(i0, i1, j2, j0, pa0, pb0, qc0, qa0, pa1, pb1, qc1, qa1, &t))
					contacts.insert(ClothCollisionElement(false, i0, i1, j2, j0, t));
				if (isSelfContactEE_CCD(i1, i2, j0, j1, pb0, pc0, qa0, qb0, pb1, pc1, qa1, qb1, &t))
					contacts.insert(ClothCollisionElement(false, i1, i2, j0, j1, t));
				if (isSelfContactEE_CCD(i1, i2, j1, j2, pb0, pc0, qb0, qc0, pb1, pc1, qb1, qc1, &t))
					contacts.insert(ClothCollisionElement(false, i1, i2, j1, j2, t));
				if (isSelfContactEE_CCD(i1, i2, j2, j0, pb0, pc0, qc0, qa0, pb1, pc1, qc1, qa1, &t))
					contacts.insert(ClothCollisionElement(false, i1, i2, j2, j0, t));
				if (isSelfContactEE_CCD(i2, i0, j0, j1, pc0, pa0, qa0, qb0, pc1, pa1, qa1, qb1, &t))
					contacts.insert(ClothCollisionElement(false, i2, i0, j0, j1, t));
				if (isSelfContactEE_CCD(i2, i0, j1, j2, pc0, pa0, qb0, qc0, pc1, pa1, qb1, qc1, &t))
					contacts.insert(ClothCollisionElement(false, i2, i0, j1, j2, t));
				if (isSelfContactEE_CCD(i2, i0, j2, j0, pc0, pa0, qc0, qa0, pc1, pa1, qc1, qa1, &t))
					contacts.insert(ClothCollisionElement(false, i2, i0, j2, j0, t));
#else
				if (nodeL->RTriVertex(0))
					if (isSelfContactTV_CCD(j0, j1, j2, i0, qa0, qb0, qc0, pa0, qa1, qb1, qc1, pa1, &t))
						contacts.insert(ClothCollisionElement(true, j0, j1, j2, i0, t));
				if (nodeL->RTriVertex(1))
					if (isSelfContactTV_CCD(j0, j1, j2, i1, qa0, qb0, qc0, pb0, qa1, qb1, qc1, pb1, &t))
						contacts.insert(ClothCollisionElement(true, j0, j1, j2, i1, t));
				if (nodeL->RTriVertex(2))
					if (isSelfContactTV_CCD(j0, j1, j2, i2, qa0, qb0, qc0, pc0, qa1, qb1, qc1, pc1, &t))
						contacts.insert(ClothCollisionElement(true, j0, j1, j2, i2, t));
				if (nodeR->RTriVertex(0))
					if (isSelfContactTV_CCD(i0, i1, i2, j0, pa0, pb0, pc0, qa0, pa1, pb1, pc1, qa1, &t))
						contacts.insert(ClothCollisionElement(true, i0, i1, i2, j0, t));
				if (nodeR->RTriVertex(1))
					if (isSelfContactTV_CCD(i0, i1, i2, j1, pa0, pb0, pc0, qb0, pa1, pb1, pc1, qb1, &t))
						contacts.insert(ClothCollisionElement(true, i0, i1, i2, j1, t));
				if (nodeR->RTriVertex(2))
					if (isSelfContactTV_CCD(i0, i1, i2, j2, pa0, pb0, pc0, qc0, pa1, pb1, pc1, qc1, &t))
						contacts.insert(ClothCollisionElement(true, i0, i1, i2, j2, t));

				if (nodeL->RTriEdge(0)) {
					if (nodeR->RTriEdge(0))
						if (isSelfContactEE_CCD(i0, i1, j0, j1, pa0, pb0, qa0, qb0, pa1, pb1, qa1, qb1, &t))
							contacts.insert(ClothCollisionElement(false, i0, i1, j0, j1, t));
					if (nodeR->RTriEdge(1))
						if (isSelfContactEE_CCD(i0, i1, j1, j2, pa0, pb0, qb0, qc0, pa1, pb1, qb1, qc1, &t))
							contacts.insert(ClothCollisionElement(false, i0, i1, j1, j2, t));
					if (nodeR->RTriEdge(2))
						if (isSelfContactEE_CCD(i0, i1, j2, j0, pa0, pb0, qc0, qa0, pa1, pb1, qc1, qa1, &t))
							contacts.insert(ClothCollisionElement(false, i0, i1, j2, j0, t));
				}
				if (nodeL->RTriEdge(1)) {
					if (nodeR->RTriEdge(0))
						if (isSelfContactEE_CCD(i1, i2, j0, j1, pb0, pc0, qa0, qb0, pb1, pc1, qa1, qb1, &t))
							contacts.insert(ClothCollisionElement(false, i1, i2, j0, j1, t));
					if (nodeR->RTriEdge(1))
						if (isSelfContactEE_CCD(i1, i2, j1, j2, pb0, pc0, qb0, qc0, pb1, pc1, qb1, qc1, &t))
							contacts.insert(ClothCollisionElement(false, i1, i2, j1, j2, t));
					if (nodeR->RTriEdge(2))
						if (isSelfContactEE_CCD(i1, i2, j2, j0, pb0, pc0, qc0, qa0, pb1, pc1, qc1, qa1, &t))
							contacts.insert(ClothCollisionElement(false, i1, i2, j2, j0, t));
				}
				if (nodeL->RTriEdge(2)) {
					if (nodeR->RTriEdge(0))
						if (isSelfContactEE_CCD(i2, i0, j0, j1, pc0, pa0, qa0, qb0, pc1, pa1, qa1, qb1, &t))
							contacts.insert(ClothCollisionElement(false, i2, i0, j0, j1, t));
					if (nodeR->RTriEdge(1))
						if (isSelfContactEE_CCD(i2, i0, j1, j2, pc0, pa0, qb0, qc0, pc1, pa1, qb1, qc1, &t))
							contacts.insert(ClothCollisionElement(false, i2, i0, j1, j2, t));
					if (nodeR->RTriEdge(2))
						if (isSelfContactEE_CCD(i2, i0, j2, j0, pc0, pa0, qc0, qa0, pc1, pa1, qc1, qa1, &t))
							contacts.insert(ClothCollisionElement(false, i2, i0, j2, j0, t));
				}
#endif
			}
		}
	}
}
void CollisionSolverHost::getSelfContactElements_CCD(
	set<ClothCollisionElement>& contacts,
	const vector<int>& faces,
	const vector<double>& vertices,
	vector<double>& velocities,
	BVHNodeHost* node, double dt)
{
	if (!node->_childs[0]) return;
	getSelfContactElements_CCD(contacts, faces, vertices, velocities, node->_childs[0], node->_childs[1], dt);
	getSelfContactElements_CCD(contacts, faces, vertices, velocities, node->_childs[0], dt);
	getSelfContactElements_CCD(contacts, faces, vertices, velocities, node->_childs[1], dt);
}

void CollisionSolverHost::MakeRigidImpactZone(
	std::vector< std::set<int> >& aRIZ,
	const set<ClothCollisionElement>& aContactElem,
	const EdgeBufferHost& aEdge)
{
	for (set<ClothCollisionElement>::iterator ce = aContactElem.begin(); ce != aContactElem.end(); ce++) {
		int n[4] = { ce->ino0, ce->ino1, ce->ino2, ce->ino3 };
		std::set<int> ind_inc;
		for (int i = 0; i < 4; i++) {
			const int ino = n[i];
			for (int iriz = 0; iriz < aRIZ.size(); iriz++) {
				if (aRIZ[iriz].find(ino) != aRIZ[iriz].end()) {
					ind_inc.insert(iriz);
				}
				else {
					/*for (int iedge = aEdge.index[ino]; iedge < aEdge.index[ino + 1]; iedge++) {
						int jno = aEdge.array[iedge];
						if (aRIZ[iriz].find(jno) != aRIZ[iriz].end()) {
							ind_inc.insert(iriz);  break;
						}
					}*/
					for (int iedge: aEdge[ino]) {
						if (aRIZ[iriz].find(iedge) != aRIZ[iriz].end()) {
							ind_inc.insert(iriz);  break;
						}
					}
				}
			}
		}
		if (ind_inc.size() == 0) {
			int ind0 = (int)aRIZ.size();
			aRIZ.resize(ind0 + 1);
			aRIZ[ind0].insert(n[0]); aRIZ[ind0].insert(n[1]); aRIZ[ind0].insert(n[2]); aRIZ[ind0].insert(n[3]);
		}
		else if (ind_inc.size() == 1) {
			int ind0 = *(ind_inc.begin());
			aRIZ[ind0].insert(n[0]); aRIZ[ind0].insert(n[1]); aRIZ[ind0].insert(n[2]); aRIZ[ind0].insert(n[3]);
		}
		else { // overlapping two reagion£¬
			std::vector< std::set<int> > aRIZ1;
			for (int iriz = 0; iriz < aRIZ.size(); iriz++) {
				if (ind_inc.find(iriz) != ind_inc.end()) continue;
				aRIZ1.push_back(aRIZ[iriz]);
			}
			int ind0 = (int)aRIZ1.size();
			aRIZ1.resize(ind0 + 1);
			for (std::set<int>::iterator itr = ind_inc.begin(); itr != ind_inc.end(); itr++) {
				int ind1 = *itr;
				assert(ind1 < aRIZ.size());
				for (std::set<int>::iterator jtr = aRIZ[ind1].begin(); jtr != aRIZ[ind1].end(); jtr++) {
					aRIZ1[ind0].insert(*jtr);
				}
			}
			aRIZ1[ind0].insert(n[0]); aRIZ1[ind0].insert(n[1]); aRIZ1[ind0].insert(n[2]); aRIZ1[ind0].insert(n[3]);
			aRIZ = aRIZ1;
		}
	}
}
void CollisionSolverHost::CalcInvMat3(double ainv[], const double a[])
{
	const double det =
		+a[0] * a[4] * a[8] + a[3] * a[7] * a[2] + a[6] * a[1] * a[5]
		- a[0] * a[7] * a[5] - a[6] * a[4] * a[2] - a[3] * a[1] * a[8];
	const double inv_det = 1.0 / det;

	ainv[0] = inv_det * (a[4] * a[8] - a[5] * a[7]);
	ainv[1] = inv_det * (a[2] * a[7] - a[1] * a[8]);
	ainv[2] = inv_det * (a[1] * a[5] - a[2] * a[4]);

	ainv[3] = inv_det * (a[5] * a[6] - a[3] * a[8]);
	ainv[4] = inv_det * (a[0] * a[8] - a[2] * a[6]);
	ainv[5] = inv_det * (a[2] * a[3] - a[0] * a[5]);

	ainv[6] = inv_det * (a[3] * a[7] - a[4] * a[6]);
	ainv[7] = inv_det * (a[1] * a[6] - a[0] * a[7]);
	ainv[8] = inv_det * (a[0] * a[4] - a[1] * a[3]);
}
void CollisionSolverHost::ApplyRigidImpactZone(
	vector<double>& aUVWm,
	const vector< set<int> >& aRIZ,
	const vector<double>& aXYZ,
	const vector<double>& aUVWm0,
	const vector<double>& masses)
{
	for (int iriz = 0; iriz < aRIZ.size(); iriz++) {
		std::vector<int> aInd; // index of points belong to this RIZ
		for (std::set<int>::iterator jtr = aRIZ[iriz].begin(); jtr != aRIZ[iriz].end(); jtr++) {
			aInd.push_back(*jtr);
		}
		Vec3 gc(0.0, 0.0, 0.0);
		Vec3 av(0.0, 0.0, 0.0);
		for (int iv = 0; iv < aInd.size(); iv++) {
			int ino = aInd[iv] * 3;
			gc += Vec3(aXYZ[ino + 0], aXYZ[ino + 1], aXYZ[ino + 2]);
			av += Vec3(aUVWm0[ino + 0], aUVWm0[ino + 1], aUVWm0[ino + 2]);// *masses[aInd[iv]];
		}
		gc /= (double)aInd.size();
		av /= (double)aInd.size();
		Vec3 L(0.0, 0.0, 0.0);
		double I[9] = { 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0 };
		for (int iv = 0; iv < aInd.size(); iv++) {
			int ino = aInd[iv] * 3;
			const Vec3 p(aXYZ[ino + 0], aXYZ[ino + 1], aXYZ[ino + 2]);
			const Vec3 v(aUVWm0[ino + 0], aUVWm0[ino + 1], aUVWm0[ino + 2]);
			L += Cross(p - gc, v - av);
			const Vec3 q = p - gc;
			double vdotv = Dot(v, v);
			I[0] += vdotv - q[0] * q[0];    I[1] += -q[0] * q[1];			I[2] += -q[0] * q[2];
			I[3] += -q[1] * q[0];			I[4] += vdotv - q[1] * q[1];    I[5] += -q[1] * q[2];
			I[6] += -q[2] * q[0];			I[7] += -q[2] * q[1];			I[8] += vdotv - q[2] * q[2];
		}

		double Iinv[9];
		CalcInvMat3(Iinv, I);
		Vec3 omg;
		omg.x = Iinv[0] * L.x + Iinv[1] * L.y + Iinv[2] * L.z;
		omg.y = Iinv[3] * L.x + Iinv[4] * L.y + Iinv[5] * L.z;
		omg.z = Iinv[6] * L.x + Iinv[7] * L.y + Iinv[8] * L.z;

		for (int iv = 0; iv < aInd.size(); iv++) {
			int ino = aInd[iv] * 3;
			const Vec3 p(aXYZ[ino + 0], aXYZ[ino + 1], aXYZ[ino + 2]);
			const Vec3 rot = -Cross(p - gc, omg);
			aUVWm[ino + 0] = av.x + rot.x;// / masses[aInd[iv]];
			aUVWm[ino + 1] = av.y + rot.y;// / masses[aInd[iv]];
			aUVWm[ino + 2] = av.z + rot.z;// / masses[aInd[iv]];
		}
	}
}
#ifdef COL_ROBUST
void CollisionSolverHost::resolveSelfCollision_Proximity(
	set<ClothCollisionElement>& contacts,
	const vector<double>& vertices,
	vector<double>& velocities,
	const vector<double>& masses,
	double thickness, double friction, double dt,
	int cloth_id)
{
	double stiffness = 10.0;
	for (set<ClothCollisionElement>::iterator ce = contacts.begin(); ce != contacts.end(); ce++) {
		const int i0 = ce->ino0;
		const int i1 = ce->ino1;
		const int i2 = ce->ino2;
		const int i3 = ce->ino3;
		const Vec3 p0(vertices[i0 * 3 + 0], vertices[i0 * 3 + 1], vertices[i0 * 3 + 2]);
		const Vec3 p1(vertices[i1 * 3 + 0], vertices[i1 * 3 + 1], vertices[i1 * 3 + 2]);
		const Vec3 p2(vertices[i2 * 3 + 0], vertices[i2 * 3 + 1], vertices[i2 * 3 + 2]);
		const Vec3 p3(vertices[i3 * 3 + 0], vertices[i3 * 3 + 1], vertices[i3 * 3 + 2]);
		const Vec3 v0(velocities[i0 * 3 + 0], velocities[i0 * 3 + 1], velocities[i0 * 3 + 2]);
		const Vec3 v1(velocities[i1 * 3 + 0], velocities[i1 * 3 + 1], velocities[i1 * 3 + 2]);
		const Vec3 v2(velocities[i2 * 3 + 0], velocities[i2 * 3 + 1], velocities[i2 * 3 + 2]);
		const Vec3 v3(velocities[i3 * 3 + 0], velocities[i3 * 3 + 1], velocities[i3 * 3 + 2]);
		double w0, w1;
		if (ce->is_fv) {
			if (!isContactTV_Proximity(p0, p1, p2, p3, thickness, &w0, &w1))
				continue;

			double w2 = 1.0 - w0 - w1;
			double iP0 = masses[i0] <= 0.0 ? 0.0 : w0 * w0 / masses[i0];
			double iP1 = masses[i1] <= 0.0 ? 0.0 : w1 * w1 / masses[i1];
			double iP2 = masses[i2] <= 0.0 ? 0.0 : w2 * w2 / masses[i2];
			double iP3 = masses[i3] <= 0.0 ? 0.0 : 1.0 / masses[i3];
			double iPt = iP0 + iP1 + iP2 + iP3;
			if (iPt == 0.0)
				continue;

			Vec3 pc = w0 * p0 + w1 * p1 + w2 * p2;
			Vec3 norm = Normalize(p3 - pc);
			if (isnan(norm.x) || isnan(norm.y) || isnan(norm.z)) {
				printf("Self Proximity TV\n");
				printf("%d, %d, %d, %d\n", i0, i1, i2, i3);
				printf("%f, %f, %f\n", (p3 - pc).x, (p3 - pc).y, (p3 - pc).z);
				printf("%f, %f, %f\n", w0, w1, w2);
				printf("%f, %f, %f\n", norm.x, norm.y, norm.z);
				printf("%f, %f, %f\n", p0.x, p0.y, p0.z);
				printf("%f, %f, %f\n", p1.x, p1.y, p1.z);
				printf("%f, %f, %f\n", p2.x, p2.y, p2.z);
				printf("%f, %f, %f\n", p3.x, p3.y, p3.z);
				printf("%f, %f, %f\n", v0.x, v0.y, v0.z);
				printf("%f, %f, %f\n", v1.x, v1.y, v1.z);
				printf("%f, %f, %f\n", v2.x, v2.y, v2.z);
				printf("%f, %f, %f\n", v3.x, v3.y, v3.z);
				double ht = -dt;
				Vec3 q0 = p0 + v0 * ht;
				Vec3 q1 = p1 + v1 * ht;
				Vec3 q2 = p2 + v2 * ht;
				Vec3 q3 = p3 + v3 * ht;
				pc = w0 * q0 + w1 * q1 + w2 * q2;
				norm = Normalize(p3 - pc);
				printf("%f, %f, %f\n", norm.x, norm.y, norm.z);
				printf("%f, %f, %f\n", pc.x, pc.y, pc.z);
				printf("%f, %f, %f\n", p3.x, p3.y, p3.z);
				printf("%f, %f, %f\n", q3.x, q3.y, q3.z);
				printf("%f, %f, %f\n", w0, w1, w2);
			}
			double p_depth = thickness - Dot(p3 - pc, norm);
			Vec3 vc = w0 * v0 + w1 * v1 + w2 * v2;
			double rel_v = Dot(v3 - vc, norm);
			if (rel_v >= 0.1 * p_depth / dt) 
				continue;

			double imp_el = dt * stiffness * p_depth;
			double imp_ie = (0.1 * p_depth / dt - rel_v);
			double imp_min = (imp_el < imp_ie) ? imp_el : imp_ie;
			double imp_mod = 2 * imp_min / iPt;
			imp_mod *= 0.25;

			Vec3 d_vct(0.0, 0.0, 0.0);
			Vec3 d_v3t(0.0, 0.0, 0.0);
			if (p_depth > 0.9 * thickness) {
				double vcn = Dot(vc, norm);
				double v3n = Dot(v3, norm);
				Vec3 vct = vc - vcn * norm;
				Vec3 v3t = v3 - v3n * norm;
				Vec3 relVT = v3t - vct;
				double l_relVT = Length(relVT);
				if (l_relVT != 0.0) {
					double ratioFric = max(1.0 + friction * min(rel_v, 0.0) / l_relVT, 0.0) - 1.0;
					d_vct = -ratioFric * vct * 0.25;
					d_v3t = ratioFric * v3t * 0.25;
				}
			}
			/*if (p_depth > 0.9 * thickness) {
				Vec3 relVT = v3 - vc - rel_v * norm;
				double l_relVT = Length(relVT);
				if (l_relVT != 0.0) {
					double ratioFric = max(1.0 + friction * min(rel_v, 0.0) / l_relVT, 0.0) - 1.0;
					d_vct = ratioFric * relVT;
					d_v3t = ratioFric * relVT;
				}
			}*/

			if (masses[i0] > 0.0) {
				Vec3 imp = (imp_mod * norm + d_vct) * w0 / masses[i0];
				velocities[i0 * 3 + 0] -= imp.x;
				velocities[i0 * 3 + 1] -= imp.y;
				velocities[i0 * 3 + 2] -= imp.z;
			}
			if (masses[i1] > 0.0) {
				Vec3 imp = (imp_mod * norm + d_vct) * w1 / masses[i1];
				velocities[i1 * 3 + 0] -= imp.x;
				velocities[i1 * 3 + 1] -= imp.y;
				velocities[i1 * 3 + 2] -= imp.z;
			}
			if (masses[i2] > 0.0) {
				Vec3 imp = (imp_mod * norm + d_vct) * w2 / masses[i2];
				velocities[i2 * 3 + 0] -= imp.x;
				velocities[i2 * 3 + 1] -= imp.y;
				velocities[i2 * 3 + 2] -= imp.z;
			}
			if (masses[i3] > 0.0) {
				Vec3 imp = (imp_mod * norm + d_v3t) / masses[i3];
				velocities[i3 * 3 + 0] += imp.x;
				velocities[i3 * 3 + 1] += imp.y;
				velocities[i3 * 3 + 2] += imp.z;
			}
		}
		else {
			if (!isContactEE_Proximity(p0, p1, p2, p3, thickness, &w0, &w1))
				continue;

			double iP0 = masses[i0] <= 0.0 ? 0.0 : (1.0 - w0) * (1.0 - w0) / masses[i0];
			double iP1 = masses[i1] <= 0.0 ? 0.0 : w0 * w0 / masses[i1];
			double iP2 = masses[i2] <= 0.0 ? 0.0 : (1.0 - w1) * (1.0 - w1) / masses[i2];
			double iP3 = masses[i3] <= 0.0 ? 0.0 : w1 * w1 / masses[i3];
			double iPt = iP0 + iP1 + iP2 + iP3;
			if (iPt == 0.0)
				continue;

			Vec3 c01 = p0 + (p1 - p0) * w0;
			Vec3 c23 = p2 + (p3 - p2) * w1;
			Vec3 norm = Normalize(c23 - c01);
			if (isnan(norm.x) || isnan(norm.y) || isnan(norm.z)) {
				printf("Self Proximity EE\n");
				printf("%d, %d, %d, %d\n", i0, i1, i2, i3);
				printf("%f, %f, %f\n", (c23 - c01).x, (c23 - c01).y, (c23 - c01).z);
				printf("%f, %f\n", w0, w1);
				printf("%f, %f, %f\n", norm.x, norm.y, norm.z);
				printf("%f, %f, %f\n", p0.x, p0.y, p0.z);
				printf("%f, %f, %f\n", p1.x, p1.y, p1.z);
				printf("%f, %f, %f\n", p2.x, p2.y, p2.z);
				printf("%f, %f, %f\n", p3.x, p3.y, p3.z);
				printf("%f, %f, %f\n", v0.x, v0.y, v0.z);
				printf("%f, %f, %f\n", v1.x, v1.y, v1.z);
				printf("%f, %f, %f\n", v2.x, v2.y, v2.z);
				printf("%f, %f, %f\n", v3.x, v3.y, v3.z);
				double ht = -dt;// *(1.0 - COL_HALFTIME);
				Vec3 q0 = p0 + v0 * ht;
				Vec3 q1 = p1 + v1 * ht;
				Vec3 q2 = p2 + v2 * ht;
				Vec3 q3 = p3 + v3 * ht;
				c01 = q0 + (q1 - q0) * w0;
				c23 = q2 + (q3 - q2) * w1;
				norm = Normalize(c23 - c01);
				printf("%f, %f, %f\n", norm.x, norm.y, norm.z);
				printf("%f, %f, %f\n", c01.x, c01.y, c01.z);
				printf("%f, %f, %f\n", c23.x, c23.y, c23.z);
				printf("%f, %f\n", w0, w1);
			}
			double p_depth = thickness - Dot(c23 - c01, norm);
			Vec3 v01 = v0 + w0 * (v1 - v0);
			Vec3 v23 = v2 + w1 * (v3 - v2);
			double rel_v = Dot(v23 - v01, norm);
			if (rel_v >= 0.1 * p_depth / dt)
				continue;

			double imp_el = dt * stiffness * p_depth;
			double imp_ie = 0.1 * p_depth / dt - rel_v;
			double imp_min = (imp_el < imp_ie) ? imp_el : imp_ie;
			double imp_mod = 2 * imp_min / iPt;
			imp_mod *= 0.25;

			Vec3 d_v01t(0.0, 0.0, 0.0);
			Vec3 d_v23t(0.0, 0.0, 0.0);
			
			if (p_depth > 0.9 * thickness) {
				double v01n = Dot(v01, norm);
				double v23n = Dot(v23, norm);
				Vec3 v01t = v01 - v01n * norm;
				Vec3 v23t = v23 - v23n * norm;
				Vec3 relVT = v23t - v01t;
				double l_relVT = Length(relVT);
				if (l_relVT != 0.0) {
					double ratioFric = max(1.0 + friction * min(rel_v, 0.0) / l_relVT, 0.0) - 1.0;
					d_v01t = -ratioFric * v01t * 0.25;
					d_v23t = ratioFric * v23t * 0.25;
				}
			}
			/*if (p_depth > 0.9 * thickness) {
				Vec3 relVT = v23 - v01 - rel_v * norm;
				double l_relVT = Length(relVT);
				if (l_relVT != 0.0) {
					double ratioFric = max(1.0 + friction * min(rel_v, 0.0) / l_relVT, 0.0) - 1.0;
					d_v01t = ratioFric * relVT;
					d_v23t = ratioFric * relVT;
				}
			}*/

			if (masses[i0] > 0.0) {
				Vec3 imp = (imp_mod * norm + d_v01t) * (1.0 - w0) / masses[i0];
				velocities[i0 * 3 + 0] -= imp.x;
				velocities[i0 * 3 + 1] -= imp.y;
				velocities[i0 * 3 + 2] -= imp.z;
			}
			if (masses[i1] > 0.0) {
				Vec3 imp = (imp_mod * norm + d_v01t) * w0 / masses[i1];
				velocities[i1 * 3 + 0] -= imp.x;
				velocities[i1 * 3 + 1] -= imp.y;
				velocities[i1 * 3 + 2] -= imp.z;
			}
			if (masses[i2] > 0.0) {
				Vec3 imp = (imp_mod * norm + d_v23t) * (1.0 - w1) / masses[i2];
				velocities[i2 * 3 + 0] += imp.x;
				velocities[i2 * 3 + 1] += imp.y;
				velocities[i2 * 3 + 2] += imp.z;
			}
			if (masses[i3] > 0.0) {
				Vec3 imp = (imp_mod * norm + d_v23t) * w1 / masses[i3];
				velocities[i3 * 3 + 0] += imp.x;
				velocities[i3 * 3 + 1] += imp.y;
				velocities[i3 * 3 + 2] += imp.z;
			}
		}
	}
}
void CollisionSolverHost::resolveSelfCollision_CCD(
	set<ClothCollisionElement>& contacts,
	const vector<double>& vertices,
	vector<double>& velocities, 
	const vector<double>& masses,
	double thickness, double dt)
{
	for (set<ClothCollisionElement>::iterator ce = contacts.begin(); ce != contacts.end(); ce++) {
		const int i0 = ce->ino0;
		const int i1 = ce->ino1;
		const int i2 = ce->ino2;
		const int i3 = ce->ino3;
		const Vec3 p0(vertices[i0 * 3 + 0], vertices[i0 * 3 + 1], vertices[i0 * 3 + 2]);
		const Vec3 p1(vertices[i1 * 3 + 0], vertices[i1 * 3 + 1], vertices[i1 * 3 + 2]);
		const Vec3 p2(vertices[i2 * 3 + 0], vertices[i2 * 3 + 1], vertices[i2 * 3 + 2]);
		const Vec3 p3(vertices[i3 * 3 + 0], vertices[i3 * 3 + 1], vertices[i3 * 3 + 2]);
		const Vec3 v0(velocities[i0 * 3 + 0], velocities[i0 * 3 + 1], velocities[i0 * 3 + 2]);
		const Vec3 v1(velocities[i1 * 3 + 0], velocities[i1 * 3 + 1], velocities[i1 * 3 + 2]);
		const Vec3 v2(velocities[i2 * 3 + 0], velocities[i2 * 3 + 1], velocities[i2 * 3 + 2]);
		const Vec3 v3(velocities[i3 * 3 + 0], velocities[i3 * 3 + 1], velocities[i3 * 3 + 2]);
		Vec3 q0 = p0 + v0 * dt;
		Vec3 q1 = p1 + v1 * dt;
		Vec3 q2 = p2 + v2 * dt;
		Vec3 q3 = p3 + v3 * dt;
		double t, w0, w1;
		if (ce->is_fv) {
			if (!isContactTV_CCD(p0, p1, p2, p3, q0, q1, q2, q3, &t, &w0, &w1))
				continue;

			double w2 = 1.0 - w0 - w1;
			double iP0 = masses[i0] <= 0.0 ? 0.0 : w0 * w0 / masses[i0];
			double iP1 = masses[i1] <= 0.0 ? 0.0 : w1 * w1 / masses[i1];
			double iP2 = masses[i2] <= 0.0 ? 0.0 : w2 * w2 / masses[i2];
			double iP3 = masses[i3] <= 0.0 ? 0.0 : 1.0 / masses[i3];
			double iPt = iP0 + iP1 + iP2 + iP3;
			if (iPt == 0.0)
				continue;

			/*double ht = t * 0.5 * dt;
			const Vec3 pc = w0 * (p0 + ht * v0) + w1 * (p1 + ht * v1) + w2 * (p2 + ht * v2);
			const Vec3 norm = Normalize(p3 + ht * v3 - pc);*/
			Vec3 pc = w0 * p0 + w1 * p1 + w2 * p2;
			Vec3 norm = Normalize(p3 - pc);
			if (isnan(norm.x) || isnan(norm.y) || isnan(norm.z)) {
				printf("Self CCD TV\n");
				double ht = -dt;
				q0 = p0 + v0 * ht;
				q1 = p1 + v1 * ht;
				q2 = p2 + v2 * ht;
				q3 = p3 + v3 * ht;
				pc = w0 * p0 + w1 * p1 + w2 * p2;
				norm = Normalize(p3 - pc);
			}
			double imp = 0.1 * thickness / dt - Dot(v3 - w0 * v0 - w1 * v1 - w2 * v2, norm);
			if (imp <= 0.0)
				continue;

			double imp_mod = 2 * imp / iPt;
			imp_mod *= 0.1;
			if (masses[i0] > 0.0) {
				double imp = imp_mod * w0 / masses[i0];
				velocities[i0 * 3 + 0] -= norm.x * imp;
				velocities[i0 * 3 + 1] -= norm.y * imp;
				velocities[i0 * 3 + 2] -= norm.z * imp;
			}
			if (masses[i1] > 0.0) {
				double imp = imp_mod * w1 / masses[i1];
				velocities[i1 * 3 + 0] -= norm.x * imp;
				velocities[i1 * 3 + 1] -= norm.y * imp;
				velocities[i1 * 3 + 2] -= norm.z * imp;
			}
			if (masses[i2] > 0.0) {
				double imp = imp_mod * w2 / masses[i2];
				velocities[i2 * 3 + 0] -= norm.x * imp;
				velocities[i2 * 3 + 1] -= norm.y * imp;
				velocities[i2 * 3 + 2] -= norm.z * imp;
			}
			if (masses[i3] > 0.0) {
				double imp = imp_mod / masses[i3];
				velocities[i3 * 3 + 0] += norm.x * imp;
				velocities[i3 * 3 + 1] += norm.y * imp;
				velocities[i3 * 3 + 2] += norm.z * imp;
			}
		}
		else {
			if (!isContactEE_CCD(p0, p1, p2, p3, q0, q1, q2, q3, &t, &w0, &w1))
				continue;
			
			double iP0 = masses[i0] <= 0.0 ? 0.0 : (1.0 - w0) * (1.0 - w0) / masses[i0];
			double iP1 = masses[i1] <= 0.0 ? 0.0 : w0 * w0 / masses[i1];
			double iP2 = masses[i2] <= 0.0 ? 0.0 : (1.0 - w1) * (1.0 - w1) / masses[i2];
			double iP3 = masses[i3] <= 0.0 ? 0.0 : w1 * w1 / masses[i3];
			double iPt = iP0 + iP1 + iP2 + iP3;
			if (iPt == 0.0)
				continue;

			/*double ht = t * 0.5 * dt;
			const Vec3 c01 = p0 + ht * v0 + (p1 - p0 + ht * (v1 - v0)) * w0;
			const Vec3 c23 = p2 + ht * v2 + (p3 - p2 + ht * (v3 - v2)) * w1;*/
			Vec3 c01 = p0 + (p1 - p0) * w0;
			Vec3 c23 = p2 + (p3 - p2) * w1;
			Vec3 norm = Normalize(c23 - c01);
			if (isnan(norm.x) || isnan(norm.y) || isnan(norm.z)) {
				printf("Self CCD EE\n");
				double ht = -dt;
				q0 = p0 + v0 * ht;
				q1 = p1 + v1 * ht;
				q2 = p2 + v2 * ht;
				q3 = p3 + v3 * ht;
				c01 = p0 + (p1 - p0) * w0;
				c23 = p2 + (p3 - p2) * w1;
				norm = Normalize(c23 - c01);
			}
			double imp = 0.1 * thickness / dt - Dot(v2 + w1 * (v3 - v2) - v0 - w0 * (v1 - v0), norm);
			if (imp <= 0.0) 
				continue;

			double imp_mod = 2 * imp / iPt;
			imp_mod *= 0.1;
			if (masses[i0] > 0.0) {
				double imp = imp_mod * (1.0 - w0) / masses[i0];
				velocities[i0 * 3 + 0] -= norm.x * imp;
				velocities[i0 * 3 + 1] -= norm.y * imp;
				velocities[i0 * 3 + 2] -= norm.z * imp;
			}
			if (masses[i1] > 0.0) {
				double imp = imp_mod * w0 / masses[i1];
				velocities[i1 * 3 + 0] -= norm.x * imp;
				velocities[i1 * 3 + 1] -= norm.y * imp;
				velocities[i1 * 3 + 2] -= norm.z * imp;
			}
			if (masses[i2] > 0.0) {
				double imp = imp_mod * (1.0 - w1) / masses[i2];
				velocities[i2 * 3 + 0] += norm.x * imp;
				velocities[i2 * 3 + 1] += norm.y * imp;
				velocities[i2 * 3 + 2] += norm.z * imp;
			}
			if (masses[i3] > 0.0) {
				double imp = imp_mod * w1 / masses[i3];
				velocities[i3 * 3 + 0] += norm.x * imp;
				velocities[i3 * 3 + 1] += norm.y * imp;
				velocities[i3 * 3 + 2] += norm.z * imp;
			}
		}
	}
}
#else
void CollisionSolverHost::resolveSelfCollision_Proximity(
	set<ClothCollisionElement>& contacts,
	const vector<double>& vertices,
	vector<double>& velocities,
	const vector<double>& masses,
	double thickness, double friction, double dt,
	int cloth_id)
{
	double stiffness = 10.0;
	for (set<ClothCollisionElement>::iterator ce = contacts.begin(); ce != contacts.end(); ce++) {
		const int i0 = ce->ino0;
		const int i1 = ce->ino1;
		const int i2 = ce->ino2;
		const int i3 = ce->ino3;
		const Vec3 p0(vertices[i0 * 3 + 0], vertices[i0 * 3 + 1], vertices[i0 * 3 + 2]);
		const Vec3 p1(vertices[i1 * 3 + 0], vertices[i1 * 3 + 1], vertices[i1 * 3 + 2]);
		const Vec3 p2(vertices[i2 * 3 + 0], vertices[i2 * 3 + 1], vertices[i2 * 3 + 2]);
		const Vec3 p3(vertices[i3 * 3 + 0], vertices[i3 * 3 + 1], vertices[i3 * 3 + 2]);
		const Vec3 v0(velocities[i0 * 3 + 0], velocities[i0 * 3 + 1], velocities[i0 * 3 + 2]);
		const Vec3 v1(velocities[i1 * 3 + 0], velocities[i1 * 3 + 1], velocities[i1 * 3 + 2]);
		const Vec3 v2(velocities[i2 * 3 + 0], velocities[i2 * 3 + 1], velocities[i2 * 3 + 2]);
		const Vec3 v3(velocities[i3 * 3 + 0], velocities[i3 * 3 + 1], velocities[i3 * 3 + 2]);
		double w0, w1;
		if (ce->is_fv) {
			if (!isContactTV_Proximity(p0, p1, p2, p3, thickness, &w0, &w1))
				continue;

			double w2 = 1.0 - w0 - w1;
			double iP0 = masses[i0] <= 0.0 ? 0.0 : w0 * w0 / masses[i0];
			double iP1 = masses[i1] <= 0.0 ? 0.0 : w1 * w1 / masses[i1];
			double iP2 = masses[i2] <= 0.0 ? 0.0 : w2 * w2 / masses[i2];
			double iP3 = masses[i3] <= 0.0 ? 0.0 : 1.0 / masses[i3];
			double iPt = iP0 + iP1 + iP2 + iP3;
			if (iPt == 0.0)
				continue;

			Vec3 pc = w0 * p0 + w1 * p1 + w2 * p2;
			Vec3 norm = Normalize(p3 - pc);
			if (isnan(norm.x) || isnan(norm.y) || isnan(norm.z)) {
				printf("Self Proximity TV\n");
				printf("%d, %d, %d, %d\n", i0, i1, i2, i3);
				printf("%f, %f, %f\n", (p3 - pc).x, (p3 - pc).y, (p3 - pc).z);
				printf("%f, %f, %f\n", w0, w1, w2);
				printf("%f, %f, %f\n", norm.x, norm.y, norm.z);
				printf("%f, %f, %f\n", p0.x, p0.y, p0.z);
				printf("%f, %f, %f\n", p1.x, p1.y, p1.z);
				printf("%f, %f, %f\n", p2.x, p2.y, p2.z);
				printf("%f, %f, %f\n", p3.x, p3.y, p3.z);
				printf("%f, %f, %f\n", v0.x, v0.y, v0.z);
				printf("%f, %f, %f\n", v1.x, v1.y, v1.z);
				printf("%f, %f, %f\n", v2.x, v2.y, v2.z);
				printf("%f, %f, %f\n", v3.x, v3.y, v3.z);
				double ht = -dt;
				Vec3 q0 = p0 + v0 * ht;
				Vec3 q1 = p1 + v1 * ht;
				Vec3 q2 = p2 + v2 * ht;
				Vec3 q3 = p3 + v3 * ht;
				pc = w0 * q0 + w1 * q1 + w2 * q2;
				norm = Normalize(p3 - pc);
				printf("%f, %f, %f\n", norm.x, norm.y, norm.z);
				printf("%f, %f, %f\n", pc.x, pc.y, pc.z);
				printf("%f, %f, %f\n", p3.x, p3.y, p3.z);
				printf("%f, %f, %f\n", q3.x, q3.y, q3.z);
				printf("%f, %f, %f\n", w0, w1, w2);
			}
			double p_depth = thickness - Dot(p3 - pc, norm);
			double depth = thickness * (1.0 - COL_THICKNESS) - p_depth;
			Vec3 vc = w0 * v0 + w1 * v1 + w2 * v2;
			double rel_v = Dot(v3 - vc, norm);
			if (rel_v >= p_depth / dt)
				continue;

			Vec3 d_vct(0.0, 0.0, 0.0);
			Vec3 d_v3t(0.0, 0.0, 0.0);
			double imp_mod = 0;
			if (rel_v < -depth / dt) {
				imp_mod += -depth / dt - rel_v;

				double vcn = Dot(vc, norm);
				double v3n = Dot(v3, norm);
				Vec3 vct = vc - vcn * norm;
				Vec3 v3t = v3 - v3n * norm;
				Vec3 relVT = v3t - vct;
				double l_relVT = Length(relVT);
				if (l_relVT != 0.0) {
					double ratioFric = max(1.0 + friction * min(rel_v, 0.0) / l_relVT, 0.0) - 1.0;
					d_vct = -ratioFric * vct * COL_STABILITY_PROXIMITY;
					d_v3t = ratioFric * v3t * COL_STABILITY_PROXIMITY;
					SoundManager::_fric_vs[cloth_id] += Length(v3 - vc);
					//SoundManager::_fric_vs[cloth_id] += Length(relVT);
					SoundManager::_fric_num[cloth_id]++;
				}
				/*Vec3 relVT = v3 - vc - rel_v * norm;
				double l_relVT = Length(relVT);
				if (l_relVT != 0.0) {
					double ratioFric = max(1.0 + friction * min(rel_v, 0.0) / l_relVT, 0.0) - 1.0;
					d_vct = ratioFric * relVT;
					d_v3t = ratioFric * relVT;
				}*/
			}

			double imp_el = dt * stiffness * p_depth;
			double imp_ie = (p_depth / dt - rel_v);
			imp_mod += (imp_el < imp_ie) ? imp_el : imp_ie;

			imp_mod *= 2 / iPt;
			imp_mod *= COL_STABILITY_PROXIMITY;

			if (masses[i0] > 0.0) {
				Vec3 imp = (imp_mod * norm + d_vct) * w0 / masses[i0];
				velocities[i0 * 3 + 0] -= imp.x;
				velocities[i0 * 3 + 1] -= imp.y;
				velocities[i0 * 3 + 2] -= imp.z;
			}
			if (masses[i1] > 0.0) {
				Vec3 imp = (imp_mod * norm + d_vct) * w1 / masses[i1];
				velocities[i1 * 3 + 0] -= imp.x;
				velocities[i1 * 3 + 1] -= imp.y;
				velocities[i1 * 3 + 2] -= imp.z;
			}
			if (masses[i2] > 0.0) {
				Vec3 imp = (imp_mod * norm + d_vct) * w2 / masses[i2];
				velocities[i2 * 3 + 0] -= imp.x;
				velocities[i2 * 3 + 1] -= imp.y;
				velocities[i2 * 3 + 2] -= imp.z;
			}
			if (masses[i3] > 0.0) {
				Vec3 imp = (imp_mod * norm + d_v3t) / masses[i3];
				velocities[i3 * 3 + 0] += imp.x;
				velocities[i3 * 3 + 1] += imp.y;
				velocities[i3 * 3 + 2] += imp.z;
			}
		}
		else {
			if (!isContactEE_Proximity(p0, p1, p2, p3, thickness, &w0, &w1))
				continue;

			double iP0 = masses[i0] <= 0.0 ? 0.0 : (1.0 - w0) * (1.0 - w0) / masses[i0];
			double iP1 = masses[i1] <= 0.0 ? 0.0 : w0 * w0 / masses[i1];
			double iP2 = masses[i2] <= 0.0 ? 0.0 : (1.0 - w1) * (1.0 - w1) / masses[i2];
			double iP3 = masses[i3] <= 0.0 ? 0.0 : w1 * w1 / masses[i3];
			double iPt = iP0 + iP1 + iP2 + iP3;
			if (iPt == 0.0)
				continue;

			Vec3 c01 = p0 + (p1 - p0) * w0;
			Vec3 c23 = p2 + (p3 - p2) * w1;
			Vec3 norm = Normalize(c23 - c01);
			if (isnan(norm.x) || isnan(norm.y) || isnan(norm.z)) {
				printf("Self Proximity EE\n");
				printf("%d, %d, %d, %d\n", i0, i1, i2, i3);
				printf("%f, %f, %f\n", (c23 - c01).x, (c23 - c01).y, (c23 - c01).z);
				printf("%f, %f\n", w0, w1);
				printf("%f, %f, %f\n", norm.x, norm.y, norm.z);
				printf("%f, %f, %f\n", p0.x, p0.y, p0.z);
				printf("%f, %f, %f\n", p1.x, p1.y, p1.z);
				printf("%f, %f, %f\n", p2.x, p2.y, p2.z);
				printf("%f, %f, %f\n", p3.x, p3.y, p3.z);
				printf("%f, %f, %f\n", v0.x, v0.y, v0.z);
				printf("%f, %f, %f\n", v1.x, v1.y, v1.z);
				printf("%f, %f, %f\n", v2.x, v2.y, v2.z);
				printf("%f, %f, %f\n", v3.x, v3.y, v3.z);
				double ht = -dt;// *(1.0 - COL_HALFTIME);
				Vec3 q0 = p0 + v0 * ht;
				Vec3 q1 = p1 + v1 * ht;
				Vec3 q2 = p2 + v2 * ht;
				Vec3 q3 = p3 + v3 * ht;
				c01 = q0 + (q1 - q0) * w0;
				c23 = q2 + (q3 - q2) * w1;
				norm = Normalize(c23 - c01);
				printf("%f, %f, %f\n", norm.x, norm.y, norm.z);
				printf("%f, %f, %f\n", c01.x, c01.y, c01.z);
				printf("%f, %f, %f\n", c23.x, c23.y, c23.z);
				printf("%f, %f\n", w0, w1);
			}
			double p_depth = thickness - Dot(c23 - c01, norm);
			double depth = thickness * (1.0 - COL_THICKNESS) - p_depth;
			Vec3 v01 = v0 + w0 * (v1 - v0);
			Vec3 v23 = v2 + w1 * (v3 - v2);
			double rel_v = Dot(v23 - v01, norm);
			if (rel_v >= p_depth / dt)
				continue;

			Vec3 d_v01t(0.0, 0.0, 0.0);
			Vec3 d_v23t(0.0, 0.0, 0.0);
			double imp_mod = 0.0;
			if (rel_v < -depth / dt) {
				imp_mod += -depth / dt - rel_v;

				double v01n = Dot(v01, norm);
				double v23n = Dot(v23, norm);
				Vec3 v01t = v01 - v01n * norm;
				Vec3 v23t = v23 - v23n * norm;
				Vec3 relVT = v23t - v01t;
				double l_relVT = Length(relVT);
				if (l_relVT != 0.0) {
					double ratioFric = max(1.0 + friction * min(rel_v, 0.0) / l_relVT, 0.0) - 1.0;
					d_v01t = -ratioFric * v01t * COL_STABILITY_PROXIMITY;
					d_v23t = ratioFric * v23t * COL_STABILITY_PROXIMITY;
					SoundManager::_fric_vs[cloth_id] += Length(v23 - v01);
					//SoundManager::_fric_vs[cloth_id] += Length(relVT);
					SoundManager::_fric_num[cloth_id]++;
				}

				/*Vec3 relVT = v23 - v01 - rel_v * norm;
				double l_relVT = Length(relVT);
				if (l_relVT != 0.0) {
					double ratioFric = max(1.0 + friction * min(rel_v, 0.0) / l_relVT, 0.0) - 1.0;
					d_v01t = ratioFric * relVT;
					d_v23t = ratioFric * relVT;
				}*/
			}
			double imp_el = dt * stiffness * p_depth;
			double imp_ie = (p_depth / dt - rel_v);
			imp_mod += (imp_el < imp_ie) ? imp_el : imp_ie;

			imp_mod *= 2 / iPt;
			imp_mod *= COL_STABILITY_PROXIMITY;

			if (masses[i0] > 0.0) {
				Vec3 imp = (imp_mod * norm + d_v01t) * (1.0 - w0) / masses[i0];
				velocities[i0 * 3 + 0] -= imp.x;
				velocities[i0 * 3 + 1] -= imp.y;
				velocities[i0 * 3 + 2] -= imp.z;
			}
			if (masses[i1] > 0.0) {
				Vec3 imp = (imp_mod * norm + d_v01t) * w0 / masses[i1];
				velocities[i1 * 3 + 0] -= imp.x;
				velocities[i1 * 3 + 1] -= imp.y;
				velocities[i1 * 3 + 2] -= imp.z;
			}
			if (masses[i2] > 0.0) {
				Vec3 imp = (imp_mod * norm + d_v23t) * (1.0 - w1) / masses[i2];
				velocities[i2 * 3 + 0] += imp.x;
				velocities[i2 * 3 + 1] += imp.y;
				velocities[i2 * 3 + 2] += imp.z;
			}
			if (masses[i3] > 0.0) {
				Vec3 imp = (imp_mod * norm + d_v23t) * w1 / masses[i3];
				velocities[i3 * 3 + 0] += imp.x;
				velocities[i3 * 3 + 1] += imp.y;
				velocities[i3 * 3 + 2] += imp.z;
			}
		}
	}
}
void CollisionSolverHost::resolveSelfCollision_CCD(
	set<ClothCollisionElement>& contacts,
	const vector<double>& vertices,
	vector<double>& velocities, 
	const vector<double>& masses,
	double thickness, double dt)
{
	for (set<ClothCollisionElement>::iterator ce = contacts.begin(); ce != contacts.end(); ce++) {
		const int i0 = ce->ino0;
		const int i1 = ce->ino1;
		const int i2 = ce->ino2;
		const int i3 = ce->ino3;
		const Vec3 p0(vertices[i0 * 3 + 0], vertices[i0 * 3 + 1], vertices[i0 * 3 + 2]);
		const Vec3 p1(vertices[i1 * 3 + 0], vertices[i1 * 3 + 1], vertices[i1 * 3 + 2]);
		const Vec3 p2(vertices[i2 * 3 + 0], vertices[i2 * 3 + 1], vertices[i2 * 3 + 2]);
		const Vec3 p3(vertices[i3 * 3 + 0], vertices[i3 * 3 + 1], vertices[i3 * 3 + 2]);
		const Vec3 v0(velocities[i0 * 3 + 0], velocities[i0 * 3 + 1], velocities[i0 * 3 + 2]);
		const Vec3 v1(velocities[i1 * 3 + 0], velocities[i1 * 3 + 1], velocities[i1 * 3 + 2]);
		const Vec3 v2(velocities[i2 * 3 + 0], velocities[i2 * 3 + 1], velocities[i2 * 3 + 2]);
		const Vec3 v3(velocities[i3 * 3 + 0], velocities[i3 * 3 + 1], velocities[i3 * 3 + 2]);
		Vec3 q0 = p0 + v0 * dt;
		Vec3 q1 = p1 + v1 * dt;
		Vec3 q2 = p2 + v2 * dt;
		Vec3 q3 = p3 + v3 * dt;
		double t, w0, w1;
		if (ce->is_fv) {
			if (!isContactTV_CCD(p0, p1, p2, p3, q0, q1, q2, q3, &t, &w0, &w1))
				continue;

			double w2 = 1.0 - w0 - w1;
			double iP0 = masses[i0] <= 0.0 ? 0.0 : w0 * w0 / masses[i0];
			double iP1 = masses[i1] <= 0.0 ? 0.0 : w1 * w1 / masses[i1];
			double iP2 = masses[i2] <= 0.0 ? 0.0 : w2 * w2 / masses[i2];
			double iP3 = masses[i3] <= 0.0 ? 0.0 : 1.0 / masses[i3];
			double iPt = iP0 + iP1 + iP2 + iP3;
			if (iPt == 0.0)
				continue;

			//double ht = t * 0.5 * dt;
			//Vec3 pc = w0 * (p0 + ht * v0) + w1 * (p1 + ht * v1) + w2 * (p2 + ht * v2);
			//Vec3 norm = Normalize(p3 + ht * v3 - pc);

			Vec3 pc = w0 * p0 + w1 * p1 + w2 * p2;
			Vec3 norm = Normalize(p3 - pc);
			if (isnan(norm.x) || isnan(norm.y) || isnan(norm.z)) {
				printf("Self CCD TV\n");
				double ht = -dt;
				q0 = p0 + v0 * ht;
				q1 = p1 + v1 * ht;
				q2 = p2 + v2 * ht;
				q3 = p3 + v3 * ht;
				pc = w0 * q0 + w1 * q1 + w2 * q2;
				norm = Normalize(q3 - pc);
			}
			double imp = COL_THICKNESS * thickness / dt - Dot(v3 - w0 * v0 - w1 * v1 - w2 * v2, norm) * dt * (1.0 - t);
			if (imp <= 0.0)
				continue;

			double imp_mod = 2 * imp / iPt;
			imp_mod *= COL_STABILITY_CCD;
			if (masses[i0] > 0.0) {
				double imp = imp_mod * w0 / masses[i0];
				velocities[i0 * 3 + 0] -= norm.x * imp;
				velocities[i0 * 3 + 1] -= norm.y * imp;
				velocities[i0 * 3 + 2] -= norm.z * imp;
			}
			if (masses[i1] > 0.0) {
				double imp = imp_mod * w1 / masses[i1];
				velocities[i1 * 3 + 0] -= norm.x * imp;
				velocities[i1 * 3 + 1] -= norm.y * imp;
				velocities[i1 * 3 + 2] -= norm.z * imp;
			}
			if (masses[i2] > 0.0) {
				double imp = imp_mod * w2 / masses[i2];
				velocities[i2 * 3 + 0] -= norm.x * imp;
				velocities[i2 * 3 + 1] -= norm.y * imp;
				velocities[i2 * 3 + 2] -= norm.z * imp;
			}
			if (masses[i3] > 0.0) {
				double imp = imp_mod / masses[i3];
				velocities[i3 * 3 + 0] += norm.x * imp;
				velocities[i3 * 3 + 1] += norm.y * imp;
				velocities[i3 * 3 + 2] += norm.z * imp;
			}
		}
		else {
			if (!isContactEE_CCD(p0, p1, p2, p3, q0, q1, q2, q3, &t, &w0, &w1))
				continue;
			
			double iP0 = masses[i0] <= 0.0 ? 0.0 : (1.0 - w0) * (1.0 - w0) / masses[i0];
			double iP1 = masses[i1] <= 0.0 ? 0.0 : w0 * w0 / masses[i1];
			double iP2 = masses[i2] <= 0.0 ? 0.0 : (1.0 - w1) * (1.0 - w1) / masses[i2];
			double iP3 = masses[i3] <= 0.0 ? 0.0 : w1 * w1 / masses[i3];
			double iPt = iP0 + iP1 + iP2 + iP3;
			if (iPt == 0.0)
				continue;

			//double ht = t * 0.5 * dt;
			//Vec3 c01 = p0 + ht * v0 + (p1 - p0 + ht * (v1 - v0)) * w0;
			//Vec3 c23 = p2 + ht * v2 + (p3 - p2 + ht * (v3 - v2)) * w1;
			Vec3 c01 = p0 + (p1 - p0) * w0;
			Vec3 c23 = p2 + (p3 - p2) * w1;
			Vec3 norm = Normalize(c23 - c01);
			if (isnan(norm.x) || isnan(norm.y) || isnan(norm.z)) {
				printf("Self CCD EE\n");
				double ht = -dt;
				q0 = p0 + v0 * ht;
				q1 = p1 + v1 * ht;
				q2 = p2 + v2 * ht;
				q3 = p3 + v3 * ht;
				c01 = q0 + (q1 - q0) * w0;
				c23 = q2 + (q3 - q2) * w1;
				norm = Normalize(c23 - c01);
			}
			double imp = COL_THICKNESS * thickness / dt - Dot(v2 + w1 * (v3 - v2) - v0 - w0 * (v1 - v0), norm) * dt * (1.0 - t);
			if (imp <= 0.0) 
				continue;

			double imp_mod = 2 * imp / iPt;
			imp_mod *= COL_STABILITY_CCD;
			if (masses[i0] > 0.0) {
				double imp = imp_mod * (1.0 - w0) / masses[i0];
				velocities[i0 * 3 + 0] -= norm.x * imp;
				velocities[i0 * 3 + 1] -= norm.y * imp;
				velocities[i0 * 3 + 2] -= norm.z * imp;
			}
			if (masses[i1] > 0.0) {
				double imp = imp_mod * w0 / masses[i1];
				velocities[i1 * 3 + 0] -= norm.x * imp;
				velocities[i1 * 3 + 1] -= norm.y * imp;
				velocities[i1 * 3 + 2] -= norm.z * imp;
			}
			if (masses[i2] > 0.0) {
				double imp = imp_mod * (1.0 - w1) / masses[i2];
				velocities[i2 * 3 + 0] += norm.x * imp;
				velocities[i2 * 3 + 1] += norm.y * imp;
				velocities[i2 * 3 + 2] += norm.z * imp;
			}
			if (masses[i3] > 0.0) {
				double imp = imp_mod * w1 / masses[i3];
				velocities[i3 * 3 + 0] += norm.x * imp;
				velocities[i3 * 3 + 1] += norm.y * imp;
				velocities[i3 * 3 + 2] += norm.z * imp;
			}
		}
	}
}
#endif
bool CollisionSolverHost::ResolveSelfCollision(
	const vector<int>& faces,
	const vector<double>& vertices,
	vector<double>& velocities,
	const EdgeBufferHost& edges,
	const vector<double>& masses, 
	BVHHost* bvh,
	double thickness, double friction, double dt,
	int cloth_id)
{
	bool result = false;
	set<ClothCollisionElement> contacts;
	printf("---------------------------------------\n");

	//START();
	bvh->refit(faces, vertices, velocities, thickness, dt, false);
	getSelfContactElements_Proximity(contacts, faces, vertices, velocities, bvh->_root, thickness, dt);

	result = contacts.size() > 0;
	printf("Self Proximity %d\n", contacts.size());
	resolveSelfCollision_Proximity(contacts, vertices, velocities, masses, thickness, friction, dt, cloth_id);
	//printf("%f msec\n", END() * 1000.0);
	for (int itr = 0; itr < 15; itr++) {
		//START();
		contacts.clear();
		bvh->refit(faces, vertices, velocities, COL_CULLING, dt, true);
		getSelfContactElements_CCD(contacts, faces, vertices, velocities, bvh->_root, dt);
		if (!contacts.size()) {
			//printf("%f msec\n", END() * 1000.0);
			return result;
		}
		printf("Self CCD %d, %d\n", itr, contacts.size());
		result = true;
		resolveSelfCollision_CCD(contacts, vertices, velocities, masses, thickness, dt);
		//printf("%f msec\n", END() * 1000.0);
	}

	vector< set<int> > aRIZ;
	vector<double> aUVWm0;
	for (int itr = 0; itr < 200; itr++) {
		//START();
		aUVWm0 = velocities;
		aRIZ.clear();
		contacts.clear();
		bvh->refit(faces, vertices, velocities, COL_CULLING, dt, true);
		getSelfContactElements_CCD(contacts, faces, vertices, velocities, bvh->_root, dt);
		//printf("get elem %f msec\n", END() * 1000.0);
		if (!contacts.size()) {
			//printf("impactZone %f msec\n", END() * 1000.0);
			return result;
		}
		printf("Self ImpactZone %d, %d\n", itr, contacts.size());

		//START();
		MakeRigidImpactZone(aRIZ, contacts, edges);
		ApplyRigidImpactZone(velocities, aRIZ, vertices, aUVWm0, masses);
		//printf("impactZone %f msec\n", END() * 1000.0);
		//if (itr == 199)
		//	exit(0);
	}

	return result;
}