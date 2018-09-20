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
	enum MOTION { idle, action1, action2, action3, skyAction1, skyAction2, skyAction3, dash, walk, jump, fall };


	// 動きに関して
	void MoveProcess(unsigned __int8 controllNumber);


public:
	CharacterSword(const int modelHandle, const int collStageHandle);
	~CharacterSword();


	void Draw();
	void Process(const unsigned __int8 controllNumber, const float getAngle);


	void PositionReset();
};

