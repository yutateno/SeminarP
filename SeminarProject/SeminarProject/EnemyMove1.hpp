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
	EnemyMove1(int collStageHandle, float areaX, float areaZ, float color);
	~EnemyMove1();

	void Draw();
	void Process();

	void StolenChara(VECTOR characterArea);		// キャラクターに捕まった
	void Collected();						// 個数分集まったら
};
