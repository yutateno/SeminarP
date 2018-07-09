#include "DxLib.h"
const int XSize = 1920;
const int YSize = 1080;

int RGBA[XSize][YSize][4];			// 元なるRGBをピクセル単位で取得
double XYZ[XSize][YSize][3];		// RGBからXYZに変換したもの
double PXYZ[XSize][YSize][3];		// LMSから直接P型異常のXYZに変換
double DXYZ[XSize][YSize][3];		// LMSから直接D型異常のXYZに変換
double LMS[XSize][YSize][3];		// XYZからLMSに変換したもの
double PLMS[XSize][YSize][3];		// LMSからP型異常に変換したもの
double DLMS[XSize][YSize][3];		// LMSからD型異常に変換したもの
int temp1[XSize][YSize][3];			// DXYZからRGBに変換したもの
int temp2[XSize][YSize][3];			// PLMSからRGBに変換したもの
int c[3];						// 白黒に生成するためのもので、RGBを一瞬保持するための仮置き

int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {
	
	ChangeWindowMode(TRUE);
	SetGraphMode(XSize, YSize, 32);
	
	DxLib_Init();
	
	SetDrawScreen(DX_SCREEN_BACK);

	SetFontSize(64);

	int handle = LoadSoftImage("100020811_c.png");

	//GetPixel(0, 0);
	for (int x = 0; x < XSize; x++) {
		for (int y = 0; y < YSize; y++) {
			GetPixelSoftImage(handle, x, y, &RGBA[x][y][0], &RGBA[x][y][1], &RGBA[x][y][2], &RGBA[x][y][3]);
		}
	}

	DeleteSoftImage(handle);

	for (int x = 0; x < XSize; x++) {
		for (int y = 0; y < YSize; y++) {
			XYZ[x][y][0] = (0.4124*RGBA[x][y][0] + 0.3576*RGBA[x][y][1] + 0.1805*RGBA[x][y][2]);
			XYZ[x][y][1] = (0.2126*RGBA[x][y][0] + 0.7152*RGBA[x][y][1] + 0.0722*RGBA[x][y][2]);
			XYZ[x][y][2] = (0.0193*RGBA[x][y][0] + 0.1192*RGBA[x][y][1] + 0.9505*RGBA[x][y][2]);
		}
	}

	for (int x = 0; x < XSize; x++) {
		for (int y = 0; y < YSize; y++) {
			LMS[x][y][0] = (0.15514*XYZ[x][y][0] + 0.54312*XYZ[x][y][1] + (-0.03286)*XYZ[x][y][2]);
			LMS[x][y][1] = (-0.15514*XYZ[x][y][0] + 0.45684*XYZ[x][y][1] + 0.03286*XYZ[x][y][2]);
			LMS[x][y][2] = (0.0*XYZ[x][y][0] + 0.0*XYZ[x][y][1] + 0.01608*XYZ[x][y][2]);
		}
	}

	for (int x = 0; x < XSize; x++) {
		for (int y = 0; y < YSize; y++) {
			PLMS[x][y][0] = (0.0*LMS[x][y][0] + 2.02344*LMS[x][y][1] + (-2.52581)*LMS[x][y][2]);
			PLMS[x][y][1] = (0.0*LMS[x][y][0] + 1.0*LMS[x][y][1] + 0.0*LMS[x][y][2]);
			PLMS[x][y][2] = (0.0*LMS[x][y][0] + 0.0*LMS[x][y][1] + 1.0*LMS[x][y][2]);
		}
	}

	for (int x = 0; x < XSize; x++) {
		for (int y = 0; y < YSize; y++) {
			DLMS[x][y][0] = (1.0*LMS[x][y][0] + 0.0*LMS[x][y][1] + 0.0*LMS[x][y][2]);
			DLMS[x][y][1] = (0.494207*LMS[x][y][0] + 0.0*LMS[x][y][1] + 1.24827*LMS[x][y][2]);
			DLMS[x][y][2] = (0.0*LMS[x][y][0] + 0.0*LMS[x][y][1] + 1.0*LMS[x][y][2]);
		}
	}

	/*for (int x = 0; x < XSize; x++) {
		for (int y = 0; y < YSize; y++) {
			PXYZ[x][y][0] = (-0.3813*LMS[x][y][0] + 1.1228*LMS[x][y][1] + 0.1730*LMS[x][y][2]);
			PXYZ[x][y][1] = (-0.4691*LMS[x][y][0] + 1.3813*LMS[x][y][1] + 0.0587*LMS[x][y][2]);
			PXYZ[x][y][2] = (0.0*LMS[x][y][0] + 0.0*LMS[x][y][1] + 1.0*LMS[x][y][2]);
		}
	}

	for (int x = 0; x < XSize; x++) {
		for (int y = 0; y < YSize; y++) {
			DXYZ[x][y][0] = (0.1884*LMS[x][y][0] + 0.6597*LMS[x][y][1] + 0.1016*LMS[x][y][2]);
			DXYZ[x][y][1] = (0.2318*LMS[x][y][0] + 0.8116*LMS[x][y][1] + (-0.0290)*LMS[x][y][2]);
			DXYZ[x][y][2] = (0.0*LMS[x][y][0] + 0.0*LMS[x][y][1] + 1.0*LMS[x][y][2]);
		}
	}*/

	for (int x = 0; x < XSize; x++) {
		for (int y = 0; y < YSize; y++) {
			PXYZ[x][y][0] = (2.9448129066068*PLMS[x][y][0] + (-3.5009779919365)*PLMS[x][y][1] + 13.172182147148*PLMS[x][y][2]);
			PXYZ[x][y][1] = (1.0000400016001*PLMS[x][y][0] + 1.0000400016001*PLMS[x][y][1] + 0*PLMS[x][y][2]);
			PXYZ[x][y][2] = (0.0*PLMS[x][y][0] + 0.0*PLMS[x][y][1] + 62.189054726368*PLMS[x][y][2]);
		}
	}

	for (int x = 0; x < XSize; x++) {
		for (int y = 0; y < YSize; y++) {
			DXYZ[x][y][0] = (2.9448129066068*DLMS[x][y][0] + (-3.5009779919365)*DLMS[x][y][1] + 13.172182147148*DLMS[x][y][2]);
			DXYZ[x][y][1] = (1.0000400016001*DLMS[x][y][0] + 1.0000400016001*DLMS[x][y][1] + 0 * DLMS[x][y][2]);
			DXYZ[x][y][2] = (0.0*DLMS[x][y][0] + 0.0*DLMS[x][y][1] + 62.189054726368*DLMS[x][y][2]);
		}
	}

	for (int x = 0; x < XSize; x++) {
		for (int y = 0; y < YSize; y++) {
			temp1[x][y][0] = (int)(3.2410*DXYZ[x][y][0] + (-1.5374)*DXYZ[x][y][1] + (-0.4986)*DXYZ[x][y][2]);
			temp1[x][y][1] = (int)(-0.9692*DXYZ[x][y][0] + 1.8760*DXYZ[x][y][1] + 0.0416*DXYZ[x][y][2]);
			temp1[x][y][2] = (int)(0.0556*DXYZ[x][y][0] + (-0.2040)*DXYZ[x][y][1] + 1.0570*DXYZ[x][y][2]);
		}
	}

	for (int x = 0; x < XSize; x++) {
		for (int y = 0; y < YSize; y++) {
			temp2[x][y][0] = (int)(3.2410*PXYZ[x][y][0] + (-1.5374)*PXYZ[x][y][1] + (-0.4986)*PXYZ[x][y][2]);
			temp2[x][y][1] = (int)(-0.9692*PXYZ[x][y][0] + 1.8760*PXYZ[x][y][1] + 0.0416*PXYZ[x][y][2]);
			temp2[x][y][2] = (int)(0.0556*PXYZ[x][y][0] + (-0.2040)*PXYZ[x][y][1] + 1.0570*PXYZ[x][y][2]);
		}
	}

	while (ScreenFlip() == 0 && ProcessMessage() == 0 && ClearDrawScreen() == 0) {
		if (CheckHitKey(KEY_INPUT_C) == 1) {
			for (int x = 0; x < XSize; x++) {
				for (int y = 0; y < YSize; y++) {
					for (int i = 0; i < 3; i++) {
						c[i] = RGBA[x][y][i];
					}
					DrawPixel(x, y, GetColor(c[0], c[1], c[2]));
				}
			}
			DrawFormatString(0, 0, GetColor(255, 255, 255), "通常");
		}

		if (CheckHitKey(KEY_INPUT_B) == 1) {
			for (int x = 0; x < XSize; x++) {
				for (int y = 0; y < YSize; y++) {
					for (int i = 0; i < 3; i++) {
						c[i] = temp1[x][y][i];
					}
					DrawPixel(x, y, GetColor(c[0], c[1], c[2]));
				}
			}
			DrawFormatString(0, 0, GetColor(255, 255, 255), "D型異常");
		}

		if (CheckHitKey(KEY_INPUT_D) == 1) {
			for (int x = 0; x < XSize; x++) {
				for (int y = 0; y < YSize; y++) {
					for (int i = 0; i < 3; i++) {
						c[i] = temp2[x][y][i];
					}
					DrawPixel(x, y, GetColor(c[0], c[1], c[2]));
				}
			}
			DrawFormatString(0, 0, GetColor(255, 255, 255), "P型異常");
		}
			
		if (CheckHitKey(KEY_INPUT_A) == 1) {
			for (int x = 0; x < XSize; x++) {
				for (int y = 0; y < YSize; y++) {
					c[0] = (int)(RGBA[x][y][0] * 0.299 + RGBA[x][y][1] * 0.587 + RGBA[x][y][2] * 0.114);
					c[1] = c[2] = c[0]; 
					DrawPixel(x, y, GetColor(c[0], c[1], c[2]));
				}
			}
			DrawFormatString(0, 0, GetColor(255, 255, 255), "白黒");
		}

	}

	DxLib_End();
	return 0;
}