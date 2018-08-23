#pragma once
#include "DxLib.h"
#include "InputPad.hpp"
#include "InputKey.hpp"
#include "Basic.hpp"
#include "LoadFile.hpp"

class Character
{
private:
	// それぞれの位置に関して
	VECTOR area;	// キャラ位置
	VECTOR preArea;	// 移動前の直前のキャラ位置
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
	enum MOTION{run, idle, walk};
	int attachMotion;			// モーションのアタッチ
	int attachNum;				// 現在のモーション番号
	float totalTime;			// モーションの総合時間
	float nowPlayTime;			// モーションの現在の時間
	float motionBlendTime;		// ブレンド時間
	int preAttach;				// 直前のモーションアタッチ
	float preMotionPlayTime;	// 直前のモーション時間
	void Player_PlayAnim(int attach);		// モーション変更
	void Player_AnimProcess();				// モーション動作

	// 当たり判定
	int stageHandle;					// ステージハンドル
	int wallNum;						// 判定する壁の数
	int floorNum;						// 判定する床の数
	bool hitFlag;						// 当たっているかどうか
	bool moveFlag;						// 動いているかどうか
	float maxYHit;						// 当たっているY座標の最大
	MV1_COLL_RESULT_POLY_DIM hitDim;			// 周囲のポリゴンを代入する構造体
	MV1_COLL_RESULT_POLY* wallPoly[2048];		// 壁用のポリゴンの構造体
	MV1_COLL_RESULT_POLY* floorPoly[2048];		// 床用のポリゴンの構造体
	MV1_COLL_RESULT_POLY* mainPoly;				// ポリゴンの構造体にアクセスする構造体
	HITRESULT_LINE lineResult;					// 線分との判定を代入する構造体
	void StageHit();							// 当たり判定を行う
			
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