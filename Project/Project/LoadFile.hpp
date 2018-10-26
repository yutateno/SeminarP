#pragma once
#include "Basic.hpp"


#ifndef _ME_LoadFile_HPP
#define _ME_LoadFile_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <thread>
#include <mutex>


// 読み込むファイルの種類
enum class ELOADFILE { graph, soundmem, fbxmodel, mv1model };


class LoadFile
{
private:
	static const int rad = 0x2546;						// 復号キー
	static unsigned int file_size(std::ifstream &fin);	// ファイル読み込み

public:
	LoadFile() {};		// コンストラクタ
	~LoadFile() {};		// デストラクタ


	static void MyLoad(const std::string path, int& file, const ELOADFILE type);	// メディアのロードを行う
};

#endif // !_ME_LoadFile_HPP