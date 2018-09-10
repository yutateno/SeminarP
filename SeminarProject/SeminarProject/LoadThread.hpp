#pragma once
#include "LoadFile.hpp"
#include "LoadScreen.hpp"

// �񓯊����s��
class LoadThread
{
private:
	std::thread ths;		// �񓯊����s������
	std::vector<int> fileName;

	LoadScreen* p_loadScreen;

	void MyNextLoad(std::string path, int& file, ELOADFILE type);		// �񓯊����s�����\�b�h

	int num;

public:
	LoadThread();
	~LoadThread();

	void Process(int max, std::string* path, ELOADFILE* type);		// �s��

	std::vector<int> GetFile();		// ���[�h�������̂�n��

	int GetNum();			// ���[�h���I������
};