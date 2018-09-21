#pragma once
#include "BasicObject.hpp"


class BasicCreature : public BasicObject
{
protected:
	// 位置について
	VECTOR preArea;	// 移動前の直前のキャラ位置
	float angle;	// アングル


	// 動きに関して
	float walkSpeed;	// 移動スピード
	float animSpeed;	// モーションスピード
	int fallCount;		// 当たり判定モデル全てから落ちているかどうか


	// モデルに関して
	float modelWigth;	// モデルの横幅


	// モーションに関して
	int attachMotion;			// モーションのアタッチ
	int attachNum;				// 現在のモーション番号
	float totalTime;			// モーションの総合時間
	void Player_PlayAnim(int attach);		// モーション変更
	void Player_AnimProcess();				// モーション動作


	// 当たり判定
	bool moveFlag;						// 動いているかどうか
	void StageHit();					// 当たり判定を行う
	void ActorHit(int stageHandle);		// 当たり判定を行う


private:
	// モーションに関して
	float nowPlayTime;			// モーションの現在の時間
	float motionBlendTime;		// ブレンド時間
	int preAttach;				// 直前のモーションアタッチ
	float preMotionPlayTime;	// 直前のモーション時間


	// 当たり判定
	int stageHandle;					// ステージハンドル
	int wallNum;						// 判定する壁の数
	int floorNum;						// 判定する床の数
	bool hitFlag;						// 当たっているかどうか
	float maxYHit;						// 当たっているY座標の最大
	MV1_COLL_RESULT_POLY_DIM hitDim;			// 周囲のポリゴンを代入する構造体
	MV1_COLL_RESULT_POLY* wallPoly[2048];		// 壁用のポリゴンの構造体
	MV1_COLL_RESULT_POLY* floorPoly[2048];		// 床用のポリゴンの構造体
	MV1_COLL_RESULT_POLY* mainPoly;				// ポリゴンの構造体にアクセスする構造体
	HITRESULT_LINE lineResult;					// 線分との判定を代入する構造体


public:
	BasicCreature(const int collStageHandle);
	virtual ~BasicCreature();
};