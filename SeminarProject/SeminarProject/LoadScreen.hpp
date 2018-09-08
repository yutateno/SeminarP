#pragma once
#include "LoadFile.hpp"

// ロード画面的なもののための非同期
class LoadScreen
{
private:
	int draw;		// ロード画面的な
	int endDraw;		// ロード終了的な

public:
	LoadScreen();
	~LoadScreen();

	void Process(int num, int max);		// 非同期で行うメソッド
};

