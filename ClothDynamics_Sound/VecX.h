//#ifndef __VECTOR_X_H__
//#define __VECTOR_X_H__
//
//#pragma once
//#include <iostream>
//#include <algorithm>
//#include <vector>
//
//using namespace std;
//
//class VecXi
//{
//public:
//	vector<int>	_values;
//	int	_size;
//public:
//	VecXi(void) {}
//	VecXi(int size)
//	{
//		_size = size;
//		_values.resize(_size);
//		fill(_values.begin(), _values.end(), 0);
//	}
//	~VecXi(void) {}
//public:
//	void resize(int size)
//	{
//		_size = size;
//		_values.resize(_size);
//		fill(_values.begin(), _values.end(), 0);
//	}
//	void set(int v)
//	{
//		fill(_values.begin(), _values.end(), v);
//	}
//};
//
//class VecXd
//{
//public:
//	vector<double>	_values;
//	int				_size;
//public:
//	VecXd(void) {}
//	VecXd(int size)
//	{
//		_size = size;
//		_values.resize(_size);
//		fill(_values.begin(), _values.end(), 0.0);
//	}
//	~VecXd(void) {}
//public:
//	void resize(int size)
//	{
//		_size = size;
//		_values.resize(_size);
//		fill(_values.begin(), _values.end(), 0.0);
//	}
//	void set(double v)
//	{
//		fill(_values.begin(), _values.end(), v);
//	}
//};
//
//#endif