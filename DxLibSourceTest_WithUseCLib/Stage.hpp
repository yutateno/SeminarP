#pragma once
#include "DxLib.h"

class Stage
{
private:
	int stageHandle;		// ステージ

public:
	Stage();
	~Stage();

	void Draw();
};