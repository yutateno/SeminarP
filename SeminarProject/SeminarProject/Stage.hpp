#pragma once
#include "DxLib.h"
#include "LoadFile.hpp"
#include "Basic.hpp"

class Stage
{
private:
	int drawStageHandle;		// �X�e�[�W

public:
	Stage(int drawStageHandle);
	~Stage();

	void Draw();
};