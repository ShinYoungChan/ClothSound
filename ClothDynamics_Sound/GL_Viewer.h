#ifndef __GL_VIEWER_H__
#define __GL_VIEWER_H__
#pragma once

#include <Windows.h>
#include <stdio.h>
#include "Mesh.h"

class GL_Viewer
{
public:
	int				_frame;
	int				_width, _height;
	int				_lastx, _lasty;
	float			_zoom;
	float			_rotx, _roty;
	float			_tx, _ty;
	float			_fovy;
	char			_FPS_str[100];
	unsigned char	_buttons[3];
	bool			_info;
	bool			_simulation;
public:
	GL_Viewer(int width = 800, int height = 600);
	~GL_Viewer(void) {}
public:
	void	FPS(void);
	void	capture(void);
	void	printCamera(void);
	void	drawText(float x, float y, const char *text, void *font = NULL);
	void	setView(double zoom, double tx, double ty, double rotx, double roty);
	void	setView(double rotx, double roty);
	Vec3	getPointViewToMap(int x, int y);
};

#endif