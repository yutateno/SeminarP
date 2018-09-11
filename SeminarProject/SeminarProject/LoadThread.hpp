#pragma once
#include "LoadScreen.hpp"


class LoadThread
{
private:
	std::thread ths;		// 非同期を行う


	int num;		// ロードした個数
	std::vector<int> fileName;		// ロードしたもの


	LoadScreen* p_loadScreen;		// ロード画面のポインタ


	void MyNextLoad(std::string path, int& file, ELOADFILE type);		// 非同期を行うメソッド

	   
public:
	LoadThread();		// コンストラクタ
	~LoadThread();		// デストラクタ


	void Process(int max, std::string* path, ELOADFILE* type);		// 行う


	std::vector<int> GetFile();		// ロードしたものを渡す


	int GetNum();			// ロードを終えた数
};