#ifndef __SOUND_MANAGER_H__
#define __SOUND_MANAGER_H__

#pragma once
#include <unordered_set>
#include <Windows.h>
#include "BVHHost.h"
#include "sndfile.hh"
#include "audiere.h"

using namespace audiere;

#define MAX_FREQUENCY		22050
#define SOUND_DELTATIME		1.0e-6
#define SOUND_FRAME			1470

#define SOUND_SUBDIV
#ifdef SOUND_SUBDIV
#define SUBDIV_FRAME		8
#endif

class float2 {
public:
	float x, y;
public:
	float2() {}
	float2(float _x, float _y) {
		x = _x;
		y = _y;
	}
	~float2() {};
};
class SoundManager
{
public:
	//unordered_set<int>	C_t;
	static vector<float>	_fric_vs;
	static vector<int>		_fric_num;
	static vector<float>	_crump_Es;
	static vector<int>		_crump_num;
public:
	static vector<vector<float2>>	_fric_samples;
	static vector<vector<float2>>	_crump_samples;
	static vector<vector<float>>	_samples;
	static int						_sampleRate;
	static int						_unitId;
public:
	SoundManager() {}
	virtual ~SoundManager() {}
public:
	static void		init(void);
	static void		reset(void);
	static int		add(void);
	static void		del(int id);
	static void		sampling(void);
	static void		saveSampling(void);
	static void		loadSampling(void);
public:
	static void		sound(float amplitude, float fre);
	static void		sampling(int id, float amplitude, float fre);
	static void		sampling(int id, float amplitude0, float fre0, float amplitude1, float fre1);
	static void		save(void);
	static void		save(vector<float> buffer);
};

#endif