#include <Windows.h>
#include <stdio.h>
#include <tuple>

#include "GL_Viewer.h"
#include "ClothDynamics_PBD.h"

GL_Viewer *_viewer = new GL_Viewer();
Mesh *_cloth_mesh = nullptr, *_obstacle_mesh = nullptr;
Obstacle *_obstacle = nullptr;

ClothDynamics_PBD	*_cloth_pbd;

#define SCREEN_CAPTURE
#define RENDER_VIEW
//#define CLOTH_DRAG

//#define PBD
#define XPBD

void Init(void)
{
	glEnable(GL_DEPTH_TEST);
	bool apply_rotate_x = true;
	bool apply_resize = false;

	_cloth_mesh = new Mesh((char*)"../include/obj/LR_cloth.obj");
	_obstacle_mesh = new Mesh((char*)"../include/obj/sphere.obj");
	_obstacle = new Obstacle(_obstacle_mesh, 0.0, 0.0, -2.0, 0.0, 0.8, true);
	//_cloth_mesh->rotateX(-90.0);

#ifdef PBD
	_cloth_pbd = new ClothDynamics_PBD(_cloth_mesh, 0);
#endif
#ifdef XPBD
	_cloth_pbd = new ClothDynamics_PBD(_cloth_mesh, 1);
	_cloth_pbd->addObstacle(_obstacle);
	_cloth_pbd->_cloth_id = SoundManager::add();
#endif
	SoundManager::init();

	/*SoundManager::loadSampling();
	SoundManager::save();
	exit(0);*/
}

void Darw(void)
{
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	char text[100];

#ifdef PBD
	_cloth_pbd->draw();
#endif
#ifdef XPBD
	_cloth_pbd->draw();
#endif

	double ratio = (double)_viewer->_width / (double)_viewer->_height;

	if (_viewer->_info) {
		glDisable(GL_LIGHTING);
#ifdef PBD
		_viewer->drawText(ratio, _viewer->_height - 10 * ratio, "Position-Based Dynamics, VRIPHYS 2006");
#endif
#ifdef XPBD
		_viewer->drawText(ratio, _viewer->_height - 10 * ratio, "XPBD, International Conference on Motion in Games 2016");
#endif
		_viewer->drawText(ratio, _viewer->_height - 36 * ratio, text);
		sprintf(text, "Frame : %d", _viewer->_frame);
		_viewer->drawText(ratio, _viewer->_height - 49 * ratio, text);
		_viewer->drawText(ratio, _viewer->_height - 62 * ratio, _viewer->_FPS_str);
	}
}

void Update(void)
{
	if (_viewer->_simulation) {
#ifdef SCREEN_CAPTURE
		_viewer->capture();
#endif
		SoundManager::reset();
#ifdef PBD
		_cloth_pbd->simulation();
#endif
#ifdef XPBD
		_cloth_pbd->simulation();
#endif
		SoundManager::sampling();
		printf("Frame %d\n", _viewer->_frame);

		//if (_viewer->_frame % 50 == 0)
		//	SoundManager::save(_cloth_pbd->_buffer);

		if (_viewer->_frame == 500) {
			/*for (int i = 0; i < _cloth_pbd->_buffer.size(); i++) {
				_cloth_pbd->_buffer[i] /= (float)_cloth_pbd->_nums[i];
			}*/
			//_cloth_pbd->calcSoundEffects2apply();
			//SoundManager::save(_cloth_pbd->_buffer);

			SoundManager::saveSampling();
			SoundManager::save();
			exit(0);
		}
		_viewer->_frame++;
		//SoundManager::save();
	}
	::glutPostRedisplay();
}

void DrawBox(void)
{
	glPushMatrix();
	glDisable(GL_LIGHTING);
	glColor3f(0.0, 0.0, 0.0);
	glTranslatef(0.5, 0.5, 0.5);
	glutWireCube(3.0);
	glEnable(GL_LIGHTING);
	glPopMatrix();
}

void Display(void)
{
	glClearColor(0.8980392156862745f, 0.9490196078431373f, 1.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.1f, 4.0f);
	glLoadIdentity();

#ifdef RENDER_VIEW
	//_viewer->setView(-1.1, 0.2, 0.1, 5.0, 12.500999);
	//_viewer->setView(-3.2, 0.2, 0.1, 5.0, 12.500999);
	//_viewer->setView(-4.800001, 0.55, 0.9, 0.0, 0.0001);
	//_viewer->setView(-2.050000, -0.240000, 0.900000, 10.000000, 75.500999);
	//_viewer->setView(-2.550000, -0.150000, 0.630000, 12.000000, 68.000999);
	//_viewer->setView(-4.050001, -1.110000, 0.570000, 13.000000, 7.500999); // fixed right corner
	//_viewer->setView(-3.500000, 0.060000, 0.060000, 7.000000, 21.000999); // hanging cloth1
	//_viewer->setView(-2.200000, 0.540000, 0.780000, 13.000000, 203.001007); // hanging cloth2
	//_viewer->setView(-5.000000, 0.180000, 1.200000, 14.000000, 33.500000);
	//_viewer->setView(-1.650002, 0.030000, 1.320000, 45.000000, 54.000000); // rotating sphere
	//_viewer->setView(-1.100000, -0.030000, -0.360000, 8.000000, 31.000000); // avatar
	//_viewer->setView(-2.800003, -0.060000, 1.020000, 17.000000, 29.000000); // cloth-bunny
	//_viewer->setView(-1.600000, -0.060000, 1.260000, 8.000000, 134.000000);
	//_viewer->setView(-2.550000, 0.030000, 0.270000, 69.500000, 47.000000);

	//_viewer->setView(-3.500000, 0.0, 0.00000, 0.000000, 0.000000);
	//_viewer->setView(-13.500000, 0.0, 0.00000, 0.000000, 0.000000);
	//_viewer->setView(-3.500000, 1.00000, 0.78000, 0.000000, -90.000000);
	_viewer->setView(-3.950000, -0.630000, 0.150000, 21.500000, 57.500000);
#ifdef CLOTH_DRAG
	//_viewer->setView(-5.500000, 0.0, 0.00000, 0.000000, 0.000000);
	//_viewer->setView(0.000000, 0.000000); //tearing
#endif
#endif
	
	glTranslatef(0, 0, _viewer->_zoom);
	glTranslatef(_viewer->_tx, _viewer->_ty, 0);
	glRotatef(_viewer->_rotx, 1, 0, 0);
	glRotatef(_viewer->_roty, 0, 1, 0);
	//glTranslatef(-0.5f, -0.5f, -0.5f);
	
	Darw();
	//DrawBox();
	_viewer->FPS();
	glutSwapBuffers();
}

void Reshape(int w, int h)
{
	if (w == 0) {
		h = 1;
	}
	_viewer->_width = w;
	_viewer->_height = h;
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(_viewer->_fovy, (float)w / h, 0.1, 100);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void Motion(int x, int y)
{
	int diffx = x - _viewer->_lastx;
	int diffy = y - _viewer->_lasty;
	_viewer->_lastx = x;
	_viewer->_lasty = y;

	if (_viewer->_buttons[2]) {
		_viewer->_zoom += (float) 0.05f * diffx;
	}
	else if (_viewer->_buttons[0]) {
		_viewer->_rotx += (float) 0.5f * diffy;
		_viewer->_roty += (float) 0.5f * diffx;

		if (_viewer->_simulation)
			_cloth_pbd->moveVertex(_viewer->getPointViewToMap(x, y));
	}
	else if (_viewer->_buttons[1]) {
		_viewer->_tx += (float) 0.03f * diffx;
		_viewer->_ty -= (float) 0.03f * diffy;
	}
	glutPostRedisplay();
}

void Mouse(int button, int state, int x, int y)
{
	_viewer->_lastx = x;
	_viewer->_lasty = y;
	switch (button)
	{
	case GLUT_LEFT_BUTTON:
		_viewer->_buttons[0] = ((GLUT_DOWN == state) ? 1 : 0);

#ifdef CLOTH_DRAG
		if (_viewer->_simulation)
			if (state == 0)
				_cloth_pbd->clickVertex(_viewer->getPointViewToMap(x, y));
		if (state == 1)
			if (_cloth_pbd->_clickID > 0)
				_cloth_pbd->resetCickVertex();
#endif
		break;
	case GLUT_MIDDLE_BUTTON:
		_viewer->_buttons[1] = ((GLUT_DOWN == state) ? 1 : 0);
		break;
	case GLUT_RIGHT_BUTTON:
		_viewer->_buttons[2] = ((GLUT_DOWN == state) ? 1 : 0);
		break;
	default:
		break;
	}
	glutPostRedisplay();
}

void Keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'q':
	case 'Q':
		exit(0);
	case ' ':
		_viewer->_simulation = !_viewer->_simulation;
		break;
	case 'p':
	case 'P':
		_viewer->_info = !_viewer->_info;
		break;
	case 'r':
	case 'R':
#ifdef PBD
		_cloth_pbd->reset();
#endif
#ifdef XPBD
		_cloth_pbd->reset();
#endif
		break;
	case 'c':
	case 'C':
		_viewer->printCamera();
		break;
	case 'a':
	case 'A':
		for (auto n : _cloth_pbd->_fixedPoints) {
			if (_cloth_pbd->_nodes[n * 3 + 0] > 0.0) {
				_cloth_pbd->_nodes[n * 3 + 0] -= 0.02;
			}
		}
		break;
	case 'd':
	case 'D':
		for (auto n : _cloth_pbd->_fixedPoints) {
			if (_cloth_pbd->_nodes[n * 3 + 0] > 0.0) {
				_cloth_pbd->_nodes[n * 3 + 0] += 0.02;
			}
		}
		break;
	case 'f':
	case 'F':
		_cloth_pbd->_fixedPoints.clear();
		_cloth_pbd->_fixedPoints0 = 0;
		_cloth_pbd->updateMass();
		break;
	}
	glutPostRedisplay();
}

int main(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(_viewer->_width, _viewer->_height);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Cloth Simulator");
	glutDisplayFunc(Display);
	glutReshapeFunc(Reshape);
	glutIdleFunc(Update);
	glutMouseFunc(Mouse);
	glutMotionFunc(Motion);
	glutKeyboardFunc(Keyboard);
	Init();
	glutMainLoop();
}