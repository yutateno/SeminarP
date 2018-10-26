#pragma once
#include "Basic.hpp"


class Stage
{
private:
	int drawStageHandle;		// ステージ


public:
	Stage(const int drawStageHandle);		// コンストラクタ
	~Stage();						// デストラクタ


	void Draw();				// 描画
};