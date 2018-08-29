#pragma once
#include "DxLib.h"
#include "InputPad.hpp"
#include "InputKey.hpp"
#include <math.h>

class Camera
{
private:
	// それぞれの位置に関して
	VECTOR cameraArea;		// カメラの位置
	VECTOR viewArea;		// 注視する方向
	VECTOR charaArea;		// キャラ位置
	float angle;		// キャラクターのアングル

	int stageHandle;		// ステージ

	// カメラのズームや回転の動きに関して
	float speed;					// 回転スピード
	bool moveflag;					// カメラ動き中かどうか
	void RLrotate(float speed, VECTOR* p_cameraArea);	// 回転を行う関数

public:
	Camera(VECTOR charaarea, int collStageHandle);				// キャラの位置を引数に取ったコンストラクタ
	~Camera();																// デストラクタ

	void Process(VECTOR charaarea, unsigned __int8 controllNumber);		// キャラの位置を引数に取ったプロセス

	float GetAngle();				// キャラクターのアングル
};