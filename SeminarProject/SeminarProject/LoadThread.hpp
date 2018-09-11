#pragma once
#include "LoadScreen.hpp"


class LoadThread
{
private:
	std::thread ths;		// �񓯊����s��


	int num;		// ���[�h������
	std::vector<int> fileName;		// ���[�h��������


	LoadScreen* p_loadScreen;		// ���[�h��ʂ̃|�C���^


	void MyNextLoad(std::string path, int& file, ELOADFILE type);		// �񓯊����s�����\�b�h

	   
public:
	LoadThread();		// �R���X�g���N�^
	~LoadThread();		// �f�X�g���N�^


	void Process(int max, std::string* path, ELOADFILE* type);		// �s��


	std::vector<int> GetFile();		// ���[�h�������̂�n��


	int GetNum();			// ���[�h���I������
};