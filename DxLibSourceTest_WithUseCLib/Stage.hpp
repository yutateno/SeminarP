#pragma once
#include "DxLib.h"
#include "LoadFile.hpp"

class Stage
{
private:
	int drawStageHandle;		// ステージ
	int collStageHandle;

public:
	Stage();
	~Stage();

	void Draw();

	int GetCollStageHandle();
};