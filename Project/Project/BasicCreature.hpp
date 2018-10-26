#pragma once
#include "BasicObject.hpp"


/// 動くアクターの親
class BasicCreature : public BasicObject
{
protected:
	// 位置について----------------------
	/// 移動前の直前のキャラ位置
	VECTOR preArea;	
	/// アングル
	float angle;	


	// 動きに関して----------------------------------------
	/// 移動スピード
	float walkSpeed;	
	/// モーションスピード
	float animSpeed;	
	/// 当たり判定モデル全てから落ちているかどうか
	int fallCount;		


	// モデルに関して-----
	/// モデルの横幅
	float modelWigth;	


	// モーションに関して---------------------------
	/// モーションのアタッチ
	int attachMotion;			
	/// 現在のモーション番号
	int attachNum;			
	/// モーションの総合時間
	float totalTime;			
	/// モーション変更
	void Player_PlayAnim(int attach);		
	/// モーション動作
	void Player_AnimProcess();				


	// 当たり判定-----------------------------
	/// 動いているかどうか
	bool moveFlag;						
	/// 当たり判定を行う
	void StageHit();					
	/// 当たり判定を行う
	void ActorHit(int stageHandle);		


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