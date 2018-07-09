#include "DxLib.h"

int RGBA[640][480][4];
double XYZ[640][480][3];
double LMS[640][480][3];
double PLMS[640][480][3];
double DLMS[640][480][3];
int temp[640][480][3];

int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {
	
	ChangeWindowMode(TRUE);
	
	DxLib_Init();
	
	SetDrawScreen(DX_SCREEN_BACK);

	int c[3];

	int handle = LoadSoftImage("test.bmp");
	GetPixel(0, 0);
	for (int x = 0; x < 640; x++) {
		for (int y = 0; y < 480; y++) {
			GetPixelSoftImage(handle, x, y, &RGBA[x][y][0], &RGBA[x][y][1], &RGBA[x][y][2], &RGBA[x][y][3]);
		}
	}

	DeleteSoftImage(handle);

	for (int x = 0; x < 640; x++) {
		for (int y = 0; y < 480; y++) {
			XYZ[x][y][0] = (0.412391*RGBA[x][y][0] + 0.357584*RGBA[x][y][1] + 0.180481*RGBA[x][y][2]);
			XYZ[x][y][1] = (0.212639*RGBA[x][y][0] + 0.715169*RGBA[x][y][1] + 0.072192*RGBA[x][y][2]);
			XYZ[x][y][2] = (0.019331*RGBA[x][y][0] + 0.119195*RGBA[x][y][1] + 0.950532*RGBA[x][y][2]);
		}
	}

	for (int x = 0; x < 640; x++) {
		for (int y = 0; y < 480; y++) {
			LMS[x][y][0] = (0.8951000*XYZ[x][y][0] + 0.2664000*XYZ[x][y][1] + (-0.1614000)*XYZ[x][y][2]);
			LMS[x][y][1] = (-0.7502000*XYZ[x][y][0] + 1.7135000*XYZ[x][y][1] + 0.0367000*XYZ[x][y][2]);
			LMS[x][y][2] = (0.0389000*XYZ[x][y][0] + (-0.0685000)*XYZ[x][y][1] + 1.0296000*XYZ[x][y][2]);
		}
	}

	for (int x = 0; x < 640; x++) {
		for (int y = 0; y < 480; y++) {
			PLMS[x][y][0] = (0.0*LMS[x][y][0] + 2.02*LMS[x][y][1] + (-2.52)*LMS[x][y][2]);
			PLMS[x][y][1] = (0.0*LMS[x][y][0] + 1.0*LMS[x][y][1] + 0.0*LMS[x][y][2]);
			PLMS[x][y][2] = (0.0*LMS[x][y][0] + 0.0*LMS[x][y][1] + 1.0*LMS[x][y][2]);
		}
	}

	for (int x = 0; x < 640; x++) {
		for (int y = 0; y < 480; y++) {
			DLMS[x][y][0] = (1.0*LMS[x][y][0] + 0.0*LMS[x][y][1] + 0.0*LMS[x][y][2]);
			DLMS[x][y][1] = (0.49*LMS[x][y][0] + 0.0*LMS[x][y][1] + 1.25*LMS[x][y][2]);
			DLMS[x][y][2] = (0.0*LMS[x][y][0] + 0.0*LMS[x][y][1] + 1.0*LMS[x][y][2]);
		}
	}

	for (int x = 0; x < 640; x++) {
		for (int y = 0; y < 480; y++) {
			XYZ[x][y][0] = (0.9869929*PLMS[x][y][0] + (-0.1470543)*PLMS[x][y][1] + 0.1599627*PLMS[x][y][2]);
			XYZ[x][y][1] = (0.4323053*PLMS[x][y][0] + 0.5183603*PLMS[x][y][1] + 0.0492912*PLMS[x][y][2]);
			XYZ[x][y][2] = (-0.0085287*PLMS[x][y][0] + 0.0400428*PLMS[x][y][1] + 0.9684867*PLMS[x][y][2]);
		}
	}

	for (int x = 0; x < 640; x++) {
		for (int y = 0; y < 480; y++) {
			temp[x][y][0] = (int)(3.2409663765843*XYZ[x][y][0] + (-1.5373788523473)*XYZ[x][y][1] + (-0.49861172322833)*XYZ[x][y][2]);
			temp[x][y][1] = (int)(-0.96924203797964*XYZ[x][y][0] + 1.8759652684909*XYZ[x][y][1] + 0.041555768342051*XYZ[x][y][2]);
			temp[x][y][2] = (int)(0.055629567117394*XYZ[x][y][0] + (-0.20397694089526)*XYZ[x][y][1] + 1.0569716994422*XYZ[x][y][2]);
		}
	}

	while (ScreenFlip() == 0 && ProcessMessage() == 0 && ClearDrawScreen() == 0) {
		if (CheckHitKey(KEY_INPUT_C) == 1) {
			for (int x = 0; x < 640; x++) {
				for (int y = 0; y < 480; y++) {
					for (int i = 0; i < 3; i++) {
						c[i] = RGBA[x][y][i];
					}
					DrawPixel(x, y, GetColor(c[0], c[1], c[2]));
				}
			}
		}

		if (CheckHitKey(KEY_INPUT_B) == 1) {
			for (int x = 0; x < 640; x++) {
				for (int y = 0; y < 480; y++) {
					for (int i = 0; i < 3; i++) {
						c[i] = temp[x][y][i];
					}
					DrawPixel(x, y, GetColor(c[0], c[1], c[2]));
				}
			}
		}
			
		if (CheckHitKey(KEY_INPUT_A) == 1) {
			for (int x = 0; x < 640; x++) {
				for (int y = 0; y < 480; y++) {
					RGBA[x][y][0] = (int)(RGBA[x][y][0] * 0.299 + RGBA[x][y][1] * 0.587 + RGBA[x][y][2] * 0.114);
					RGBA[x][y][1] = RGBA[x][y][2] = RGBA[x][y][0];
				}
			}
		}

	}

	DxLib_End();
	return 0;
}