#pragma once
#include "DxLib.h"
#include "LoadFile.hpp"

class Stage
{
private:
	int stageHandle;		// ステージ
	int test;

public:
	Stage();
	~Stage();

	void Draw();
};