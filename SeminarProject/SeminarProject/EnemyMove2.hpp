#pragma once
#include "BasicCreature.hpp"


#include <random>


class EnemyMove2 : public BasicCreature
{
private:
	// それぞれの位置に関して
	float direZAngle;		// 前後のキャラ向きを扱う変数
	float direXAngle;		// 左右のキャラ向きを扱う変数
	float nextDireZAngle;
	float nextDireXAngle;


	// モーションに関して
	enum MOTION { run, idle, walk };


	// 動きに関して
	void MoveProcess();
	int moveCount;


public:
	EnemyMove2(int modelHandle, int collStageHandle);
	~EnemyMove2();


	void Draw();
	void Process();
};

