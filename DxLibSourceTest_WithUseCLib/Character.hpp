#pragma once
#include "InputPad.hpp"
#include "InputKey.hpp"
#include "Basic.hpp"
#include "BasicActor.hpp"

class Character : public BasicActor
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
	Character(int collStageHandle);	// コンストラクタ
	~Character();					// デストラクタ
	
	void Draw();
	void Process(unsigned __int8 controllNumber, float getAngle);
};