#include "GL_Viewer.h"

GL_Viewer::GL_Viewer(int width, int height)
{
	_info = false;
	_frame = 0;
	_width = width;
	_height = height;
	_zoom = -2.5f;
	_rotx = 0.0f;
	_roty = 0.0f;
	_fovy = 45.0;
	_tx = 0;
	_ty = 0;
	_lastx = 0;
	_lasty = 0;
	_buttons[0] = 0;
	_buttons[1] = 0;
	_buttons[2] = 0;
	_simulation = false;
}

void GL_Viewer::printCamera(void)
{
	printf("%f, %f, %f, %f, %f\n", _zoom, _tx, _ty, _rotx, _roty);
}

void GL_Viewer::setView(double zoom, double tx, double ty, double rotx, double roty)
{
	_zoom = zoom;
	_tx = tx;
	_ty = ty;
	_rotx = rotx;
	_roty = roty;
}
void GL_Viewer::setView(double rotx, double roty)
{
	_rotx = rotx;
	_roty = roty;
}

void GL_Viewer::capture(void)
{
	//if (_frame == 0 || _frame % 2 == 0) {
		static int index = 0;
		char filename[100];
		sprintf_s(filename, "capture\\capture-%d.bmp", index);
		BITMAPFILEHEADER bf;
		BITMAPINFOHEADER bi;
		unsigned char *image = (unsigned char*)malloc(sizeof(unsigned char)*_width*_height * 3);
		FILE *file;
		fopen_s(&file, filename, "wb");
		if (image != NULL) {
			if (file != NULL) {
				glReadPixels(0, 0, _width, _height, 0x80E0, GL_UNSIGNED_BYTE, image);
				memset(&bf, 0, sizeof(bf));
				memset(&bi, 0, sizeof(bi));
				bf.bfType = 'MB';
				bf.bfSize = sizeof(bf) + sizeof(bi) + _width * _height * 3;
				bf.bfOffBits = sizeof(bf) + sizeof(bi);
				bi.biSize = sizeof(bi);
				bi.biWidth = _width;
				bi.biHeight = _height;
				bi.biPlanes = 1;
				bi.biBitCount = 24;
				bi.biSizeImage = _width * _height * 3;
				fwrite(&bf, sizeof(bf), 1, file);
				fwrite(&bi, sizeof(bi), 1, file);
				fwrite(image, sizeof(unsigned char), _height*_width * 3, file);
				fclose(file);
			}
			free(image);
		}
		//if (index == 60) { // cloth-bunny
		if (index == 75) { // avatar
		//if (index == 122) { // rotating sphere
		//if (index == 213) { // rotating bunny
			//exit(0);
		}
		index++;
	//}
}

void GL_Viewer::drawText(float x, float y, const char *text, void *font)
{
	glColor3f(0, 0, 0);
	glDisable(GL_DEPTH_TEST);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0.0, (double)_width, 0.0, (double)_height, -1.0, 1.0);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	if (font == NULL) {
		font = GLUT_BITMAP_9_BY_15;
	}

	size_t len = strlen(text);

	glRasterPos2f(x, y);
	for (const char *letter = text; letter < text + len; letter++) {
		if (*letter == '\n') {
			y -= 12.0f;
			glRasterPos2f(x, y);
		}
		glutBitmapCharacter(font, *letter);
	}

	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glEnable(GL_DEPTH_TEST);
}

void GL_Viewer::FPS(void)
{
	static float framesPerSecond = 0.0f;
	static float lastTime = 0.0f;
	float currentTime = GetTickCount() * 0.001f;
	++framesPerSecond;
	if (currentTime - lastTime > 1.0f) {
		lastTime = currentTime;
		sprintf(_FPS_str, "FPS : %d", (int)framesPerSecond);
		framesPerSecond = 0;
	}
}

Vec3 GL_Viewer::getPointViewToMap(int x, int y) {
	float width = (float)glutGet(GLUT_WINDOW_WIDTH);
	float height = (float)glutGet(GLUT_WINDOW_HEIGHT);
	float px = (float)x - width * 0.5f;
	float py = (float)y - height * 0.5f;
	float midDepth = height * 0.5f / tanf(_fovy * PI / 360.0f);

	Vec3 rayN(px, -py, -midDepth);
	Vec3 camPos(0.0f, 0.0f, -1.0f);
	camPos *= _zoom;
	camPos.x -= _tx;
	camPos.y -= _ty;
	camPos.Rotate(-_rotx, -_roty);

	rayN.Rotate(-_rotx, -_roty);
	rayN.SetNormalizedVector();
	//rayN *= 0.5;

	rayN *= fabsf(camPos.z / rayN.z);

	return camPos + rayN;
}