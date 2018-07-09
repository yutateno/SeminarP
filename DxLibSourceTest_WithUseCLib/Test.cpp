#include "DxLib.h"

int Cr[640][480][4];
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {
	
	ChangeWindowMode(TRUE);
	
	DxLib_Init();
	
	SetDrawScreen(DX_SCREEN_BACK);

	int c[3];

	int handle = LoadSoftImage("test.bmp");

	for (int x = 0; x<640; x++) {
		for (int y = 0; y<480; y++) {
			GetPixelSoftImage(handle, x, y, &Cr[x][y][0], &Cr[x][y][1], &Cr[x][y][2], &Cr[x][y][3]);
		}
	}
	DeleteSoftImage(handle);

	while (ScreenFlip() == 0 && ProcessMessage() == 0 && ClearDrawScreen() == 0) {
		for (int x = 0; x<640; x++) {
			for (int y = 0; y<480; y++) {
				for (int i = 0; i<3; i++) {
					c[i] = Cr[x][y][i];
				}
				DrawPixel(x, y, GetColor(c[0], c[1], c[2]));
			}
		}

		if (CheckHitKey(KEY_INPUT_A) == 1) {
			for (int x = 0; x<640; x++) {
				for (int y = 0; y<480; y++) {
					Cr[x][y][0] = (Cr[x][y][0] + Cr[x][y][1] + Cr[x][y][2]) / 3;
					Cr[x][y][1] = Cr[x][y][2] = Cr[x][y][0];
				}
			}
		}
	}

	DxLib_End();
	return 0;
}