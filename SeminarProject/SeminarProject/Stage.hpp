#pragma once
#include "DxLib.h"
#include "LoadFile.hpp"
#include "Basic.hpp"

class Stage
{
private:
	int drawStageHandle;		// ステージ

public:
	Stage(int drawStageHandle);
	~Stage();

	void Draw();
};