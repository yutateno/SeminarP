#pragma once
#include "LoadFile.hpp"


class LoadScreen
{
private:
	int draw;			// ロード画面的な
	int endDraw;		// ロード終了的な


public:
	LoadScreen();		// コンストラクタ
	~LoadScreen();		// デストラクタ


	void Process(int num, int max);		// 非同期で行うメソッド
};

