#pragma once
#include "InputPad.hpp"
#include "InputKey.hpp"
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
	Character(int modelHandle, int collStageHandle);	// コンストラクタ
	~Character();					// デストラクタ

	void Draw();
	void Process(unsigned __int8 controllNumber, float getAngle);

	void PositionReset();
};