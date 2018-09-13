#pragma once
#include "InputPad.hpp"
#include "BasicCreature.hpp"


class CharacterSword : public BasicCreature
{
private:
	// それぞれの位置に関して
	float direZAngle;		// 前後のキャラ向きを扱う変数
	float direXAngle;		// 左右のキャラ向きを扱う変数


	// モーションに関して
	enum MOTION { attack, no };


	// 動きに関して
	void MoveProcess(unsigned __int8 controllNumber);


public:
	CharacterSword(int modelHandle, int collStageHandle);
	~CharacterSword();


	void Draw();
	void Process(unsigned __int8 controllNumber, float getAngle);


	void PositionReset();
};

