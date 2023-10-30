#ifndef __LIB_H__
#define __LIB_H__

#pragma once

#define M_PI 3.14159265358979323846

#include <chrono>
#include "Sparse"
#include "Dense"
#include "svd"
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL 
#include <glm/gtx/transform.hpp>
#include "Vec3.h"

typedef Eigen::VectorXd VectorXd;
typedef Eigen::VectorXi VectorXi;
typedef Eigen::Vector3f Vector3f;
typedef Eigen::Vector3d Vector3d;
typedef Eigen::MatrixXd MatrixXd;
typedef Eigen::Matrix3d Matrix3d;
typedef Eigen::MatrixXi MatrixXi;
typedef Eigen::Matrix2f Matrix2f;
typedef Eigen::Matrix2d Matrix2d;
typedef Eigen::MatrixX3d MatrixX3d;
typedef Eigen::MatrixXf MatrixXf;

typedef Eigen::SparseMatrix<float> SparseMatrixXf;
typedef Eigen::SparseMatrix<double> SparseMatrixXd;
typedef Eigen::SimplicialLLT<SparseMatrixXd> CholeskyFactorizXd;
typedef Eigen::SimplicialLLT<SparseMatrixXf> CholeskyFactorizXf;
typedef Eigen::SimplicialCholesky<SparseMatrixXd> CholeksySolver;

static std::chrono::system_clock::time_point _start;

static void START(void)
{
	_start = std::chrono::system_clock::now();
}

static std::chrono::duration<double> END(void)
{
	return std::chrono::system_clock::now() - _start;
}

inline Vec3 RotateX(Vec3 v, double degree)
{
	double radian = degree * 3.141592 / 180.0;
	double c = cos(radian);
	double s = sin(radian);
	Vec3 nv = v;
	nv[1] = v[1] * c - v[2] * s;
	nv[2] = v[1] * s + v[2] * c;
	return nv;
}

// triangle-triangle intersect
static glm::vec3 P[3], Q[3];
static float eps_interp = float(10e-5);

inline double VdotV(double V1[3], double V2[3])
{
	return (V1[0] * V2[0] + V1[1] * V2[1] + V1[2] * V2[2]);
}

inline int project6(double *ax, double *p1, double *p2, double *p3, double *q1, double *q2, double *q3)
{
	double P1 = VdotV(ax, p1);
	double P2 = VdotV(ax, p2);
	double P3 = VdotV(ax, p3);
	double Q1 = VdotV(ax, q1);
	double Q2 = VdotV(ax, q2);
	double Q3 = VdotV(ax, q3);

	double mx1 = std::max(P1, std::max(P2, P3));
	double mn1 = std::min(P1, std::min(P2, P3));
	double mx2 = std::max(Q1, std::max(Q2, Q3));
	double mn2 = std::min(Q1, std::min(Q2, Q3));

	if (mn1 > mx2) return 0;
	if (mn2 > mx1) return 0;
	return 1;
}

static double CLAMP(double value, double min, double max)
{
	if (value > max) {
		return max;
	}
	else if (value < min) {
		return min;
	}
	return value;
}

static void VcrossV(double Vr[3], const double V1[3], const double V2[3])
{
	Vr[0] = V1[1] * V2[2] - V1[2] * V2[1];
	Vr[1] = V1[2] * V2[0] - V1[0] * V2[2];
	Vr[2] = V1[0] * V2[1] - V1[1] * V2[0];
}

static int Triangle_Contact(glm::vec3 P1, glm::vec3 P2, glm::vec3 P3, glm::vec3 Q1, glm::vec3 Q2, glm::vec3 Q3)
{
	//One triangle is (p1,p2,p3).  Other is (q1,q2,q3).
	//Edges are (e1,e2,e3) and (f1,f2,f3).
	//Normals are n1 and m1
	//Outwards are (g1,g2,g3) and (h1,h2,h3).
	//We assume that the triangle vertices are in the same coordinate system.
	//First thing we do is establish a new c.s. so that p1 is at (0,0,0).

	double p1[3], p2[3], p3[3];
	double q1[3], q2[3], q3[3];
	double e1[3], e2[3], e3[3];
	double f1[3], f2[3], f3[3];
	double g1[3], g2[3], g3[3];
	double h1[3], h2[3], h3[3];
	double n1[3], m1[3];
	double z[3];

	double ef11[3], ef12[3], ef13[3];
	double ef21[3], ef22[3], ef23[3];
	double ef31[3], ef32[3], ef33[3];

	z[0] = 0.0;  z[1] = 0.0;  z[2] = 0.0;

	p1[0] = P1[0] - P1[0];  p1[1] = P1[1] - P1[1];  p1[2] = P1[2] - P1[2];
	p2[0] = P2[0] - P1[0];  p2[1] = P2[1] - P1[1];  p2[2] = P2[2] - P1[2];
	p3[0] = P3[0] - P1[0];  p3[1] = P3[1] - P1[1];  p3[2] = P3[2] - P1[2];

	q1[0] = Q1[0] - P1[0];  q1[1] = Q1[1] - P1[1];  q1[2] = Q1[2] - P1[2];
	q2[0] = Q2[0] - P1[0];  q2[1] = Q2[1] - P1[1];  q2[2] = Q2[2] - P1[2];
	q3[0] = Q3[0] - P1[0];  q3[1] = Q3[1] - P1[1];  q3[2] = Q3[2] - P1[2];

	e1[0] = p2[0] - p1[0];  e1[1] = p2[1] - p1[1];  e1[2] = p2[2] - p1[2];
	e2[0] = p3[0] - p2[0];  e2[1] = p3[1] - p2[1];  e2[2] = p3[2] - p2[2];
	e3[0] = p1[0] - p3[0];  e3[1] = p1[1] - p3[1];  e3[2] = p1[2] - p3[2];

	f1[0] = q2[0] - q1[0];  f1[1] = q2[1] - q1[1];  f1[2] = q2[2] - q1[2];
	f2[0] = q3[0] - q2[0];  f2[1] = q3[1] - q2[1];  f2[2] = q3[2] - q2[2];
	f3[0] = q1[0] - q3[0];  f3[1] = q1[1] - q3[1];  f3[2] = q1[2] - q3[2];

	VcrossV(n1, e1, e2);
	VcrossV(m1, f1, f2);

	VcrossV(g1, e1, n1);
	VcrossV(g2, e2, n1);
	VcrossV(g3, e3, n1);
	VcrossV(h1, f1, m1);
	VcrossV(h2, f2, m1);
	VcrossV(h3, f3, m1);

	VcrossV(ef11, e1, f1);
	VcrossV(ef12, e1, f2);
	VcrossV(ef13, e1, f3);
	VcrossV(ef21, e2, f1);
	VcrossV(ef22, e2, f2);
	VcrossV(ef23, e2, f3);
	VcrossV(ef31, e3, f1);
	VcrossV(ef32, e3, f2);
	VcrossV(ef33, e3, f3);

	// now begin the series of tests

	if (!project6(n1, p1, p2, p3, q1, q2, q3)) return 0;
	if (!project6(m1, p1, p2, p3, q1, q2, q3)) return 0;

	if (!project6(ef11, p1, p2, p3, q1, q2, q3)) return 0;
	if (!project6(ef12, p1, p2, p3, q1, q2, q3)) return 0;
	if (!project6(ef13, p1, p2, p3, q1, q2, q3)) return 0;
	if (!project6(ef21, p1, p2, p3, q1, q2, q3)) return 0;
	if (!project6(ef22, p1, p2, p3, q1, q2, q3)) return 0;
	if (!project6(ef23, p1, p2, p3, q1, q2, q3)) return 0;
	if (!project6(ef31, p1, p2, p3, q1, q2, q3)) return 0;
	if (!project6(ef32, p1, p2, p3, q1, q2, q3)) return 0;
	if (!project6(ef33, p1, p2, p3, q1, q2, q3)) return 0;

	if (!project6(g1, p1, p2, p3, q1, q2, q3)) return 0;
	if (!project6(g2, p1, p2, p3, q1, q2, q3)) return 0;
	if (!project6(g3, p1, p2, p3, q1, q2, q3)) return 0;
	if (!project6(h1, p1, p2, p3, q1, q2, q3)) return 0;
	if (!project6(h2, p1, p2, p3, q1, q2, q3)) return 0;
	if (!project6(h3, p1, p2, p3, q1, q2, q3)) return 0;

	return 1;
}

static glm::vec3 interp(glm::vec3 &p1, glm::vec3 &p2, float t)
{
	return p1 * (1 - t) + p2 * t;
}

static int Triangle_Contact(glm::vec3 *p, glm::vec3 *q, int cov)
{
	if (cov == 3)
		return true;

	if (cov == 0) {
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++) {
				P[i][j] = (p[i])[j];
				Q[i][j] = (q[i])[j];
			}

		return Triangle_Contact(P[0], P[1], P[2], Q[0], Q[1], Q[2]);
	}

	if (cov == 1) {
		for (int i = 1; i < 3; i++)
			for (int j = 0; j < 3; j++) {
				P[i][j] = (p[i])[j];
				Q[i][j] = (q[i])[j];
			}
		glm::vec3 mid = interp(p[1], p[2], 0.5);
		glm::vec3 p0 = interp(p[0], mid, eps_interp);
		mid = interp(q[1], q[2], 0.5);
		glm::vec3 q0 = interp(q[0], mid, eps_interp);

		for (int j = 0; j < 3; j++) {
			P[0][j] = p0[j];
			Q[0][j] = q0[j];
		}
		return Triangle_Contact(P[0], P[1], P[2], Q[0], Q[1], Q[2]);
	}

	if (cov == 2) {
		for (int j = 0; j < 3; j++) {
			P[0][j] = (p[0])[j];
			Q[0][j] = (q[0])[j];
		}

		glm::vec3 p1 = interp(p[1], p[0], eps_interp);
		glm::vec3 p2 = interp(p[2], p[0], eps_interp);
		for (int j = 0; j < 3; j++) {
			P[1][j] = p1[j];
			P[2][j] = p2[j];
		}

		glm::vec3 q1 = interp(q[1], q[0], eps_interp);
		glm::vec3 q2 = interp(q[2], q[0], eps_interp);
		for (int j = 0; j < 3; j++) {
			Q[1][j] = q1[j];
			Q[2][j] = q2[j];
		}

		return Triangle_Contact(P[0], P[1], P[2], Q[0], Q[1], Q[2]);
	}
	return false;
}

#endif