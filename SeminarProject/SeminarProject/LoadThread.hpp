#pragma once
#include "LoadScreen.hpp"


class LoadThread
{
private:
	std::thread ths;		// 非同期を行う


	int num;		// ロードした個数
	std::vector<int> fileName;		// ロードしたもの


	LoadScreen* p_loadScreen;		// ロード画面のポインタ


	void MyNextLoad(const std::string path, int& file, const ELOADFILE type);		// 非同期を行うメソッド

	   
public:
	LoadThread();		// コンストラクタ
	~LoadThread();		// デストラクタ


	void Process(const int max, const std::string* path, const ELOADFILE* type);		// 行う


	const std::vector<int> GetFile() const;		// ロードしたものを渡す


	const int GetNum() const;			// ロードを終えた数
};