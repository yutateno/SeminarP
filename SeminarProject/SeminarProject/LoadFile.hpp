#pragma once
#ifndef _ME_LoadFile_HPP
#define _ME_LoadFile_HPP

#include "DxLib.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <thread>
#include <mutex>

#include "Basic.hpp"

enum class ELOADFILE { graph, soundmem, fbxmodel, mv1model };

class LoadFile
{
private:
	static const int rad = 0x2546;						// 復号キー
	static unsigned int file_size(std::ifstream &fin);	// ファイル読み込み

public:
	LoadFile() {};
	~LoadFile() {};


	static void MyLoad(std::string path, int& file, ELOADFILE type);	// メディアのロードを行う
};

#endif // !_ME_LoadFile_HPP