#pragma once
#include "DxLib.h"
#include "LoadFile.hpp"
#include "Basic.hpp"

class Stage
{
private:
	int drawStageHandle;		// ステージ
	int collStageHandle;

public:
	Stage();
	~Stage();

	void LoadInit();		// 保持する必要のないロードモデルを削除

	void Draw();

	int GetCollStageHandle();
};