#pragma once
#include "DxLib.h"
#include "InputPad.hpp"
#include "InputKey.hpp"
#include <math.h>

class Camera
{
private:
	// それぞれの位置に関して
	VECTOR area;		// カメラの位置
	VECTOR view;		// 注視する方向
	VECTOR chara;		// キャラ位置
	float angle;		// キャラクターのアングル

	// カメラのズームや回転の動きに関して
	float speed;					// 回転スピード
	bool moveflag;					// カメラ動き中かどうか
	VECTOR RLrotate(float speed);	// 回転を行う関数
	
public:
	Camera(VECTOR charaarea);				// キャラの位置を引数に取ったコンストラクタ
	~Camera();																// デストラクタ

	void Process(VECTOR charaarea);		// キャラの位置を引数に取ったプロセス
};