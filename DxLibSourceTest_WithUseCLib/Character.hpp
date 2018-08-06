#pragma once
#include "DxLib.h"
#include "InputPad.hpp"
#include "InputKey.hpp"
#include "Basic.hpp"

class Character
{
private:
	// それぞれの位置に関して
	VECTOR area;	// キャラ位置
	float angle;	// アングル
	float direYAngle;		// 前後のキャラ向きを扱う変数
	float direXAngle;		// 左右のキャラ向きを扱う変数

	// 動きに関して
	float walkSpeed;	// 移動スピード
	float animSpeed;	// モーションスピード

	// モデルに関して
	int charamodelhandle;	// モデルのハンドル
	float modelHeight;
	float modelWigth;

	// モーションに関して
	int attachMotion;			// モーションのアタッチ
	int attachNum;				// 現在のモーション番号
	float totalTime;			// モーションの総合時間
	float nowPlayTime;			// モーションの現在の時間
	float motionBlendTime;		// ブレンド時間
	int preAttach;				// 直前のモーションアタッチ
	float preMotionPlayTime;	// 直前のモーション時間
	
	void Player_PlayAnim(int attach);		// モーション変更
	void Player_AnimProcess();				// モーション動作
			
public:
	Character();	// コンストラクタ
	~Character();																// デストラクタ

	void Draw();		// 描画
	void Process(unsigned __int8 controllNumber);		// プロセス

	void Process(unsigned __int8 controllNumber, float getAngle);

	VECTOR GetArea();
};


/*
キャラの向きについて
スティックをはじくと最後の入力を占有してしまうため少しいびつになる
スティックの動きが過激すぎる場合を処理させる
*/