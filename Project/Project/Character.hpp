#pragma once
#include "BasicCreature.hpp"


class Character : public BasicCreature
{
private:
	// それぞれの位置に関して
	float direZAngle;		// 前後のキャラ向きを扱う変数
	float direXAngle;		// 左右のキャラ向きを扱う変数


	// モーションに関して
	enum MOTION { run, idle, walk };


	// 動きに関して
	void MoveProcess(unsigned __int8 controllNumber);


public:
	Character(const int modelHandle, const int collStageHandle);	// コンストラクタ
	~Character();					// デストラクタ


	void Draw();
	void Process(const unsigned __int8 controllNumber, const float getAngle);


	void PositionReset();
};