#pragma once
#include "BasicCreature.hpp"


class EnemyMove1 : public BasicCreature
{
private:
	// 動きに関して
	bool upNow;					// 今上がり中かどうか
	float flyMove;				// 上下のスピード
	void MoveProcess();			// 上下動きのプロセス
	MATERIALPARAM material;		// マテリアルを調整保持


public:
	EnemyMove1(const int collStageHandle, const float areaX, const float areaZ, const float color);
	~EnemyMove1();


	void Draw();
	void Process();


	void StolenChara(const VECTOR characterArea);		// キャラクターが近づいたら
	void Collected();						// 個数分集まったら
};
