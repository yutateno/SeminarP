#pragma once
#include "LoadFile.hpp"
#include "LoadScreen.hpp"

// 非同期を行う
class LoadThread
{
private:
	std::thread ths;		// 非同期を行うだけ
	std::vector<int> fileName;

	LoadScreen* loadScreen;

	void MyNextLoad(std::string path, int& file, ELOADFILE type);		// 非同期を行うメソッド

public:
	LoadThread();
	~LoadThread();

	void Run(int max, std::string* path, ELOADFILE* type);		// 行う

	std::vector<int> GetFile();		// ロードしたものを渡す

	int num;
};