#pragma once
#include "DxLib.h"
#include "InputPad.hpp"
#include "InputKey.hpp"

class Character
{
private:
	// それぞれの位置に関して
	VECTOR area;	// キャラ位置

	// モデルに関して
	int charamodelhandle;	// モデルのハンドル
		
public:
	Character();	// コンストラクタ
	~Character();																// デストラクタ

	void Draw();		// 描画
	void Process(unsigned __int8 controllNumber);		// プロセス

	VECTOR GetArea();
};