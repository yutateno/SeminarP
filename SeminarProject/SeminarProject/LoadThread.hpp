#pragma once
#include "LoadFile.hpp"
#include "LoadScreen.hpp"

// �񓯊����s��
class LoadThread
{
private:
	std::thread ths;		// �񓯊����s������
	std::vector<int> fileName;

	LoadScreen* loadScreen;

	void MyNextLoad(std::string path, int& file, ELOADFILE type);		// �񓯊����s�����\�b�h

public:
	LoadThread();
	~LoadThread();

	void Run(int max, std::string* path, ELOADFILE* type);		// �s��

	std::vector<int> GetFile();		// ���[�h�������̂�n��

	int num;
};