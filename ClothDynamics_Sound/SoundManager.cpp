#include "SoundManager.h"

vector<float>	SoundManager::_fric_vs;
vector<int>		SoundManager::_fric_num;
vector<float>	SoundManager::_crump_Es;
vector<int>		SoundManager::_crump_num;

vector<vector<float2>>	SoundManager::_fric_samples;
vector<vector<float2>>	SoundManager::_crump_samples;
vector<vector<float>>	SoundManager::_samples;
int						SoundManager::_sampleRate;
int						SoundManager::_unitId;

void SoundManager::init(void) {
	//_sampleRate = 44100;
	_sampleRate = 22050;
	_unitId = 0;

	const int format = SF_FORMAT_WAV | SF_FORMAT_PCM_16;
	//  const int format=SF_FORMAT_WAV | SF_FORMAT_FLOAT;    
	/*const int channels = 1;
	const char* infilename = "sound\\corduroy.wav";
	SndfileHandle infile(infilename, SFM_READ, format, channels, _sampleRate);
	if (!infile) 
		return;
	float sample[44100];
	vector<vector<float>> a;
	infile.read(&sample[0], 44100);
	printf("%d\n", infile.frames());
	for (int i = 0; i < infile.size(); i+=221848) {
		vector<float> v;
		for (int j = 0; j < 221848; j++)
			v.push_back(sample[j]);
		a.push_back(v);
	}*/
}

void SoundManager::reset(void) {
	for (int i = 0; i < _fric_vs.size(); i++) {
		_fric_vs[i] = 0;
		_fric_num[i] = 0;
		_crump_Es[i] = 0.f;
		_crump_num[i] = 0;
	}
}
int SoundManager::add(void) {
	_fric_vs.push_back(0);
	_fric_num.push_back(0);
	_crump_Es.push_back(0.f);
	_crump_num.push_back(0);
	_fric_samples.push_back(vector<float2>());
	_crump_samples.push_back(vector<float2>());
	_samples.push_back(vector<float>());
	return _fric_vs.size() - 1;
}
void SoundManager::del(int id) {
	_fric_vs.erase(_fric_vs.begin() + id);
	_fric_num.erase(_fric_num.begin() + id);
	_crump_Es.erase(_crump_Es.begin() + id);
	_crump_num.erase(_crump_num.begin() + id);
	_fric_samples.erase(_fric_samples.begin() + id);
	_crump_samples.erase(_crump_samples.begin() + id);
	_samples.erase(_samples.begin() + id);
}
void SoundManager::sampling(void) {
	for (int i = 0; i < _fric_vs.size(); i++) {
		float l = _fric_num[i];
		float v_avg = 0.f;
		if (l)
			v_avg = _fric_vs[i] / l;
		_fric_samples[i].push_back(float2(v_avg, l));
	}
	for (int i = 0; i < _crump_Es.size(); i++) {
		float p = _crump_num[i];
		float E = _crump_Es[i];
		_crump_samples[i].push_back(float2(E, p));
	}

	/*for (int i = 0; i < _crump_Es.size(); i++) {
		===< friction >===========================
		float l = _fric_num[i];
		printf("%f\n", l);
		if (l == 0.f)
			sampling(i, 0.f, 0.f);
		else {
			float v_avg = _fric_vs[i].Length() / l;

			l = min(l * 0.02f, 1.f);
			v_avg = min(10.f * v_avg, 30.f);
			printf("%f, %f\n", l, v_avg);

			sampling(i, l, v_avg);
		}
		===< Crumpling >===========================
		float p = _crump_num[i];
		float E = _crump_Es[i];
		p = min(p * 0.02f, 1.f);
		E = min(0.1f * E, 30.f);

		printf("%f, %f\n", p, E);
		sampling(i, p, E);
		===< LR >===========================
		float l = _fric_num[i];
		float v_avg = 0.f;
		if (l > 0.f) {
			v_avg = _fric_vs[i].Length() / l;

			l = min(l * 0.02f, 1.f);
			v_avg = min(10.f * v_avg, 30.f);
		}
		float p = _crump_num[i];
		float E = _crump_Es[i];
		p = min(p * 0.02f, 1.f);
		E = min(0.05f * E, 30.f);
		printf("%f, %f, %f, %f\n", l, v_avg, p, E);
		sampling(i, l, v_avg, p, E);
		===< MR >===========================
		float l = _fric_num[i];
		float v_avg = 0.f;
		if (l > 0.f) {
			v_avg = _fric_vs[i].Length() / l;
			printf("%d, %f, %d, %f\n", _fric_num[i], v_avg, _crump_num[i], _crump_Es[i]);

			l = min(l * 0.00001f, 1.f);
			v_avg = min(100.f * v_avg, 30.f);
		}
		float p = _crump_num[i];
		float E = _crump_Es[i];
		p = min(p * 0.02f, 1.f);
		E = min(0.05f * E, 30.f);
		printf("%f, %f, %f, %f\n", l, v_avg, p, E);
		sampling(i, l, v_avg, p, E);
	}*/
}
#include <fstream>
void SoundManager::saveSampling(void) {
	ofstream fout;
	char tmp[100];
	for (int i = 0; i < _fric_vs.size(); i++) {
		sprintf(tmp, "datas/frictionData%d.txt", i);
		fout.open(tmp);
		for (int j = 0; j < _fric_samples[i].size(); j++) {
			sprintf(tmp, "%.6f %.6f\n", _fric_samples[i][j].x, _fric_samples[i][j].y);
			fout << tmp;
		}
		fout.close();
	}
	for (int i = 0; i < _crump_Es.size(); i++) {
		sprintf(tmp, "datas/crumplingData%d.txt", i);
		fout.open(tmp);
		for (int j = 0; j < _crump_samples[i].size(); j++) {
			sprintf(tmp, "%.6f %.6f\n", _crump_samples[i][j].x, _crump_samples[i][j].y);
			fout << tmp;
		}
		fout.close();
	}
}
void SoundManager::loadSampling(void) {
	ifstream fin;
	char tmp[100];
	int i = 0;
	while (1) {
		sprintf(tmp, "datas/frictionData%d.txt", i);
		fin.open(tmp);
		if (!fin.is_open())
			break;
		if (_fric_samples.size() <= i)
			_fric_samples.push_back(vector<float2>());
		while (!fin.eof()) {
			float2 x;
			fin >> x.x >> x.y;
			_fric_samples[i].push_back(x);
		}
		fin.close();
		i++;
	}
	i = 0;
	while (1) {
		sprintf(tmp, "datas/crumplingData%d.txt", i);
		fin.open(tmp);
		if (!fin.is_open())
			break;
		if (_crump_samples.size() <= i)
			_crump_samples.push_back(vector<float2>());
		while (!fin.eof()) {
			float2 x;
			fin >> x.x >> x.y;
			_crump_samples[i].push_back(x);
		}
		fin.close();
		i++;
	}
}

void SoundManager::sound(float amplitude, float fre) {
	int invsec = 10;
	int rate = 44100;
	const int size = rate / invsec;

	short *sample = (short*)malloc(size * sizeof(short));

	float current = 0.;
	for (int i = 0; i < size; i++) {
		sample[i] = (short)(32767.f * amplitude * sinf(float(i) / size * M_PI * fre));
	}

	AudioDevicePtr device(OpenDevice());
	OutputStreamPtr sound;
	sound = device->openBuffer(sample, size, 1, rate, SF_S16);

	sound->play();
	//Sleep(1000.0);
	free(sample);
}

void SoundManager::sampling(int id, float amplitude, float fre) {
	const int size = _sampleRate / 30;

	int istart = max((int)_samples[id].size() - (size >> 1), 0);
	for (int i = 0; i < size; i++) {
		int ino = istart + i;
		float d = amplitude * sinf(float(i) / size * M_PI * fre);
		if (ino < _samples[id].size())
			_samples[id][ino] = _samples[id][ino] + (d - _samples[id][ino]) * (float)i / (float)(_samples[id].size() - istart);
		else
			_samples[id].push_back(d);
	}
	printf("size : %d\n", _samples[id].size());
}
void SoundManager::sampling(int id, float amplitude0, float fre0, float amplitude1, float fre1) {
	const int size = _sampleRate / 30;

	int istart = max((int)_samples[id].size() - (size >> 1), 0);
	for (int i = 0; i < size; i++) {
		int ino = istart + i;
		float d0 = amplitude0 * sinf(float(i) / size * M_PI * fre0);
		float d1 = amplitude1 * sinf(float(i) / size * M_PI * fre1);
		float d = (d0 + d1) * 0.5;
		if (d > 1.f)		d = 1.f;
		else if (d < -1.f)	d = -1.f;
		if (ino < _samples[id].size())
			_samples[id][ino] = _samples[id][ino] + (d - _samples[id][ino]) * (float)i / (float)(_samples[id].size() - istart);
		else
			_samples[id].push_back(d);
	}
	printf("size : %d\n", _samples[id].size());
}

#define FRICTION_NUM		3
void SoundManager::save(void) {
	vector<vector<vector<vector<float>>>> fric_clips;
	vector<vector<vector<float>>> crump_clips;
	const int format = SF_FORMAT_WAV | SF_FORMAT_PCM_16;
	//  const int format=SF_FORMAT_WAV | SF_FORMAT_FLOAT;    
	const int channels = 1;

#if FRICTION_NUM==0		//friction0
	const int clipRate = _sampleRate * 4;
	int FRIC_COL = 12;
	char fileDir[100];
	sprintf_s(fileDir, "sound/friction/friction0/");
#elif FRICTION_NUM==1		//friction1
	const int clipRate = _sampleRate * 4;
	int FRIC_COL = 12;
	char fileDir[100];
	sprintf_s(fileDir, "sound/friction/friction1/");
#elif FRICTION_NUM==2		//friction2
	const int clipRate = _sampleRate * 4;
	int FRIC_COL = 7;
	char fileDir[100];
	sprintf_s(fileDir, "sound/friction/friction2/");
#elif FRICTION_NUM==3		//friction3
	const int clipRate = _sampleRate * 4;
	int FRIC_COL = 15;
	char fileDir[100];
	sprintf_s(fileDir, "sound/friction/friction3/");
#elif FRICTION_NUM==4
	const int clipRate = _sampleRate * 6;
	int FRIC_COL = 9;
	char fileDir[100];
	sprintf_s(fileDir, "sound/friction/");
#endif
	int FRIC_ROW = 3;

	const int unitRate = 1470;
	const int unithalf = unitRate >> 1;
	const int unitSize = clipRate / unithalf - 1;

	int CRUMP_COL = 3;
	int CRUMP_ROW = 3;
	const int crumpRate = _sampleRate * 2;

	for (int row = 0; row < FRIC_ROW; row++) {
		fric_clips.push_back(vector<vector<vector<float>>>());
		for (int col = 0; col < FRIC_COL; col++) {
			char infilename[100];
			sprintf_s(infilename, "%srow%d/clip%d.wav", fileDir, row, col);
			SndfileHandle infile(infilename, SFM_READ, format, channels, _sampleRate);
			if (!infile)
				return;

			vector<float> s(clipRate);
			infile.read(&s[0], clipRate);
			fric_clips[row].push_back(vector<vector<float>>());
			/*for (int i = 0; i < s.size(); i++) {
				s[i] *= 10.f;
				if (s[i] > 1.f)
					s[i] = 1.f;
				else if (s[i] < -1.f)
					s[i] = -1.f;
			}*/
			for (int i = 0; i < unitSize; i++) {
				fric_clips[row][col].push_back(vector<float>());
				int istart = i * unithalf;
				int iend = min(istart + unitRate, clipRate);
				fric_clips[row][col][i].insert(fric_clips[row][col][i].end(), 
					s.begin() + istart, s.begin() + iend);
			}
		}
	}
	for (int row = 0; row < CRUMP_ROW; row++) {
		crump_clips.push_back(vector<vector<float>>());
		for (int col = 0; col < CRUMP_COL; col++) {
			char infilename[100];
			sprintf_s(infilename, "sound/crumpling/row%d/clip%d.wav", row, col);
			SndfileHandle infile(infilename, SFM_READ, format, channels, _sampleRate);
			if (!infile)
				return;

			crump_clips[row].push_back(vector<float>(crumpRate));
			infile.read(&crump_clips[row].back()[0], crumpRate);
		}
	}

	for (int i = 0; i < _fric_samples.size(); i++) {
		vector<float> fricSamples;
		vector<float> crumpSamples;

		float fric_v_max = -FLT_MAX;
		float fric_v_min = FLT_MAX;
		float fric_n_max = -FLT_MAX;
		float fric_n_min = FLT_MAX;
		float MinFricV = 0.01f;
		float MinFricN = 10.f;
		int prev_fric_row, prev_fric_col = -100;
		float prev_v = 0, thresholdV = 1.4f;

		float crump_e_max = -FLT_MAX;
		float crump_e_min = FLT_MAX;
		float crump_n_max = -FLT_MAX;
		float crump_n_min = FLT_MAX;
		float MinCrumpE = 1000.f;
		float MinCrumpN = 10.f;
		int prev_crump_row, prev_crump_col = -100;
		float prev_e = 0, thresholdE = 10.f;
		int crumpPos = 0;

		int row, col;
		const int MaxlastWave = 10;
		int lastWave = 0;
		int last_row, last_col;

		{
			//int w = 50;
			//vector<float2> temp(_fric_samples[i].size(), float2(0.f,0.f));
			//for (int j = 0; j < _fric_samples[i].size(); j++) {
			//	//for (int k = -w; k <= w; k++) {
			//	for (int k = -w; k <= 0; k++) {
			//		int ino = j + k;
			//		if (ino >= 0 && ino < _fric_samples[i].size()) {
			//			temp[j].x += (1.f - fabsf((float)k) / (float)w) * _fric_samples[i][ino].x;
			//			temp[j].y += (1.f - fabsf((float)k) / (float)w) * _fric_samples[i][ino].y;
			//		}
			//	}
			//}
			//_fric_samples[i] = temp;

			//vector<float2> temp(_fric_samples[i].size(), float2(0.f,0.f));
			//int w = 10;
			//int gw = w + w;

			//float sigma = (float)gw / sqrtf(8 * logf(2)) * 0.5;

			//float c = 2.f * sigma * sigma;
			//float sc = 1.f / (sqrtf(2 * M_PI) * sigma);
			//for (int j = 0; j < _fric_samples[i].size(); j++) {
			//	float smooth = 0.f;
			//	//for (int k = -w; k <= w; k++)
			//	for (int k = -w; k <= 0; k++) {
			//		int ino = j + k;
			//		if (ino >= 0 && ino < _fric_samples[i].size()) {
			//			temp[j].x += _fric_samples[i][ino].x * exp(-k * k / c) * sc;
			//			temp[j].y += _fric_samples[i][ino].y * exp(-k * k / c) * sc;
			//		}
			//	}
			//}
			//_fric_samples[i] = temp;
		}

		for (int j = 0; j < _fric_samples[i].size(); j++) {
			if (_fric_samples[i][j].x > MinFricV && _fric_samples[i][j].y > MinFricN) {
				if (fric_v_max < _fric_samples[i][j].x)
					fric_v_max = _fric_samples[i][j].x;
				if (fric_v_min > _fric_samples[i][j].x)
					fric_v_min = _fric_samples[i][j].x;
				if (fric_n_max < _fric_samples[i][j].y)
					fric_n_max = _fric_samples[i][j].y;
				if (fric_n_min > _fric_samples[i][j].y)
					fric_n_min = _fric_samples[i][j].y;
			}
		}
		for (int j = 0; j < _crump_samples[i].size(); j++) {
			if (_crump_samples[i][j].x > MinCrumpE && _crump_samples[i][j].y > MinCrumpN) {
				if (crump_e_max < _crump_samples[i][j].x)
					crump_e_max = _crump_samples[i][j].x;
				if (crump_e_min > _crump_samples[i][j].x)
					crump_e_min = _crump_samples[i][j].x;
				if (crump_n_max < _crump_samples[i][j].y)
					crump_n_max = _crump_samples[i][j].y;
				if (crump_n_min > _crump_samples[i][j].y)
					crump_n_min = _crump_samples[i][j].y;
			}
		}

		for (int j = 0; j < _fric_samples[i].size(); j++) {
			auto v = _fric_samples[i][j].x;
			auto l = _fric_samples[i][j].y;
			//v = 1.0;
			/*vector<float> asdf(22050, 0);
			if (v > 0.0001 && l > 10) {
				v = (v - fric_v_min) / (fric_v_max - fric_v_min);
				l = (l - fric_n_min) / (fric_n_max - fric_n_min);
				col = min((int)(v * (float)FRIC_COL), FRIC_COL - 1);
				row = min((int)(l * (float)FRIC_ROW), FRIC_ROW - 1);
				asdf = fric_clips[row][col][_unitId];
				for (auto c:fric_clips[row][col])
					fricSamples.insert(fricSamples.end(), c.begin(), c.end());
			}else
				fricSamples.insert(fricSamples.end(), asdf.begin(), asdf.end());*/
			vector<float> clip(unitRate, 0);
			//printf("----------------\n0. %f %f, %f %f\n", v, l, fric_n_max, fric_n_min);
			if (v > MinFricV && l > MinFricN) {
				col = min((int)((v - fric_v_min) / (fric_v_max - fric_v_min) * (float)(FRIC_COL)), FRIC_COL - 1);
				row = min((int)((l - fric_n_min) / (fric_n_max - fric_n_min) * (float)(FRIC_ROW)), FRIC_ROW - 1);

				float aaa = fabs(v / (prev_v + FLT_EPSILON));
				if (aaa < thresholdV && aaa > 1.0 / thresholdV) {
					col = prev_fric_col;
					row = prev_fric_row;
				}
				else prev_v = v;
				printf("fric %d, %f %f %f\n", j, v, prev_v, aaa);

				/*if (abs(col - prev_col) < 2 && abs(row - prev_row) < 1) {
					col = prev_col;
					row = prev_row;
				}*/
				prev_fric_col = col;
				prev_fric_row = row;
				clip = fric_clips[row][col][_unitId];
				printf("fric %d, %d %d\n", j, col, row);
			}
			else {
				/*if (prev_col >= 0) {
					last_col = prev_col;
					last_row = prev_row;
					lastWave = MaxlastWave;
					prev_col = -100;
				}
				if (last_col >= 0 && lastWave > 0) {
					lastWave--;
					clip = fric_clips[last_row][last_col][_unitId];
					float highRatio = (float)lastWave / (float)(MaxlastWave - 1);
					float lowRatio = (float)(lastWave - 1) / (float)(MaxlastWave - 1);
					for (int k = 0; k < unitRate; k++) {
						float ratio = (float)k / (float)(unitRate - 1);
						clip[k] *= highRatio - (highRatio - lowRatio) * ratio;
					}
				}*/
			}
			if (j > 0) {
				int istart = fricSamples.size() - unithalf;
				for (int k = 0; k < unithalf; k++) {
					int ino = istart + k;
					fricSamples[ino] += (clip[k] - fricSamples[ino]) * (float)k / (float)(unithalf - 1);
				}
				fricSamples.insert(fricSamples.end(), clip.begin() + unithalf, clip.end());
			}
			else fricSamples.insert(fricSamples.end(), clip.begin(), clip.end());

			_unitId++;
			if (_unitId >= unitSize)
				_unitId = 0;
		}
		for (int j = 0; j < _crump_samples[i].size(); j++) {
			auto e = _crump_samples[i][j].x;
			auto l = _crump_samples[i][j].y;
			vector<float> clip;
			//printf("----------------\n0. %f %f, %f %f\n", v, l, fric_n_max, fric_n_min);
			if (e > MinCrumpE && l > MinCrumpN) {
				col = min((int)((e - crump_e_min) / (crump_e_max - crump_e_min) * (float)(CRUMP_COL)), CRUMP_COL - 1);
				row = min((int)((l - crump_n_min) / (crump_n_max - crump_n_min) * (float)(CRUMP_ROW)), CRUMP_ROW - 1);

				if (fabs(prev_e - e) >= thresholdE || e / prev_e >= 2.3f) {
					printf("crump %d, %f %f %f, %f\n", j, e, prev_e, fabs(prev_e - e), e / prev_e);
					printf("crump %d, %d %d\n", j, col, row);
					clip = crump_clips[row][col];
				}

				prev_e = e;
				prev_crump_col = col;
				prev_crump_row = row;
			}
			if (clip.size()) {
				if (crumpSamples.size() < crumpPos + clip.size())
					crumpSamples.resize(crumpPos + clip.size(), 0.f);
				for (int k = 0; k < clip.size(); k++)
					crumpSamples[k + crumpPos] += clip[k];
			}
			else {
				if (crumpSamples.size() < crumpPos + unitRate)
					crumpSamples.resize(crumpPos + unitRate, 0.f);
			}
			crumpPos += unitRate;
		}
		_samples[i].clear();
		if (crumpSamples.size() < fricSamples.size())
			crumpSamples.resize(fricSamples.size(), 0.f);
		for (int j = 0; j < fricSamples.size(); j++)
			_samples[i].push_back(fricSamples[j] + crumpSamples[j]);

		char outfilename[100];
		sprintf_s(outfilename, "sound\\sound-%d.wav", i);
		SndfileHandle outfile(outfilename, SFM_WRITE, format, channels, _sampleRate);
		if (!outfile)
			return;
		const int size = _samples[i].size();
		printf("save size : %d\n", size);
		outfile.write(&_samples[i][0], size);
	}
}
void SoundManager::save(vector<float> buffer) {
	float f_max = 0.f;
	int w = ceilf(3.f / ((float)MAX_FREQUENCY * SOUND_DELTATIME));
	//int w = 1;
	/*for (int i = 0; i < buffer.size(); i++) {
		if (f_max < fabs(buffer[i])	) f_max = buffer[i];
	}

	double sigma = 1.0;
	double c = 2.0 * sigma * sigma;
	double sc = 1.0 / (sqrt(2 * M_PI) * sigma);
	for (int i = 0; i < buffer.size(); i++) {
		buffer[i] = sc * exp(-buffer[i] * buffer[i] / c);
	}*/
	/*double sigma = 1.0;
	double c = 2.0 * sigma * sigma;
	double sc = 1.0 / (sqrt(2 * M_PI) * sigma);
	vector<double> kernel(w + w + 1);
	double sum = 0.0;
	for (int i = -w; i <= w; i++) {
		kernel[i + w] = sc * exp(-i * i / c);
		printf("%d : exp: %lf , kernel %.15f\n",i, exp((-i * i) / c), kernel[i + w]);
	}
	for (int i = 0; i < buffer.size(); i++) {
		for (int j = -w; j <= w; j++) {
			buffer[min(max(i + j, 0), (int)buffer.size() - 1)] *= kernel[j + w];
		}
	}
	for (int i = 0; i < buffer.size(); i++) {
		if (f_max < fabs(buffer[i])) f_max = buffer[i];
	}
	for (int i = 0; i < buffer.size(); i++) {
		buffer[i] = buffer[i] / f_max;
	}*/
	//double sigma = 0.1;
	//double c = 2.0 * sigma * sigma;
	//double sc = 1.0 / (sqrt(2 * M_PI) * sigma);
	//for (int i = 0; i < buffer.size() - 1; i++) {
	//	//printf("%d, %f, %f\n", i, buffer[i], sc * exp(-buffer[i] * buffer[i] / c));
	//	double x = buffer[i + 1] - buffer[i];
	//	x = x * sc * exp(-x * x / c);
	//	buffer[i + 1] = buffer[i] + x;
	//}
	/*double sigma = 0.1;
	double c = 2.0 * sigma * sigma;
	double sc = 1.0 / (sqrt(2 * M_PI) * sigma);
	for (int i = 0; i < buffer.size(); i++) {
		double w = 0.9;
		buffer[i] = w * buffer[i] + (1.0 - w) * buffer[i] * sc * exp(-buffer[i] * buffer[i] / c);
	}
	for (int i = 0; i < buffer.size(); i++) {
		if (f_max < fabs(buffer[i])) f_max = buffer[i];
	}
	for (int i = 0; i < buffer.size(); i++) {
		buffer[i] = buffer[i] / f_max;
	}*/

	int gw = w + w;
	//int gw = w;
	//int gw = SOUND_FRAME;
	//int gw = 2 * SOUND_FRAME;

	float sigma = (float)gw / sqrtf(8 * logf(2)) * 0.5;
	//float sigma = (float)500 / sqrtf(8 * logf(2)) * 0.5;

	float c = 2.f * sigma * sigma;
	float sc = 1.f / (sqrtf(2 * M_PI) * sigma);
	{
		/*float sum = 0.f;
		for (int j = -gw; j <= gw; j++) {
			printf("%f\n", exp(-j * j / c) * sc);
			sum += exp(-j * j / c) * sc;
		}
		printf("sum %f\n", sum);*/
	}
	for (int itr = 0; itr < 1; itr++) {
		vector<float> smooths(buffer.size());
		for (int i = 0; i < buffer.size(); i++) {
			float smooth = 0.f;
			for (int j = -gw; j <= gw; j++)
				if (i + j >= 0 && i + j < buffer.size())
					smooth += buffer[i + j] * exp(-j * j / c) * sc;
			smooths[i] = smooth;
		}
		buffer = smooths;
	}
	for (int i = 0; i < buffer.size(); i++)
		if (f_max < fabsf(buffer[i]))
			f_max = fabsf(buffer[i]);
	/*for (int i = 0; i < buffer.size(); i++) {
		buffer[i] = buffer[i] / 15.f;
		if (fabsf(buffer[i]) > 1.0)
			buffer[i] /= fabsf(buffer[i]);
	}*/
	for (int i = 0; i < buffer.size(); i++)
		buffer[i] = buffer[i] / f_max * 0.5;
	printf("%f\n", f_max);

	const int format = SF_FORMAT_WAV | SF_FORMAT_PCM_16;
	//const int format=SF_FORMAT_WAV | SF_FORMAT_FLOAT; 
	const int channels = 1;
	char outfilename[100];
	sprintf_s(outfilename, "sound\\sound.wav");
	SndfileHandle outfile(outfilename, SFM_WRITE, format, channels, _sampleRate);
	if (!outfile)
		return;
	int size = buffer.size();
	printf("save size : %d\n", size);
	outfile.write(&buffer[0], size);
}