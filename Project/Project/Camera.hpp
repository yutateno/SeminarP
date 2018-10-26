#pragma once
#include "Basic.hpp"


class Camera
{
private:
	VECTOR cameraArea;		// カメラの位置
	VECTOR viewArea;		// 注視する方向
	VECTOR charaArea;		// キャラ位置
	float angle;		// キャラクターのアングル


	int stageHandle;		// 当たり判定用ステージ


	float speed;					// 回転スピード
	void RLrotate(const float speed, VECTOR& p_cameraArea);	// 回転を行う関数


public:
	Camera(const VECTOR charaarea, const int collStageHandle);				// キャラの位置を引数に取ったコンストラクタ
	~Camera();													// デストラクタ


	void Process(const VECTOR charaarea, const unsigned __int8 controllNumber);		// キャラの位置を引数に取ったプロセス


	void SetUp();


	const float GetAngle() const;				// キャラクターのアングル
};