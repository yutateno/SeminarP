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


// �ǂݍ��ރt�@�C���̎��
enum class ELOADFILE { graph, soundmem, fbxmodel, mv1model };


class LoadFile
{
private:
	static const int rad = 0x2546;						// �����L�[
	static unsigned int file_size(std::ifstream &fin);	// �t�@�C���ǂݍ���

public:
	LoadFile() {};		// �R���X�g���N�^
	~LoadFile() {};		// �f�X�g���N�^


	static void MyLoad(const std::string path, int& file, const ELOADFILE type);	// ���f�B�A�̃��[�h���s��
};

#endif // !_ME_LoadFile_HPP