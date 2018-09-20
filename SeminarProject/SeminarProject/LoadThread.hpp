#pragma once
#include "LoadScreen.hpp"


class LoadThread
{
private:
	std::thread ths;		// �񓯊����s��


	int num;		// ���[�h������
	std::vector<int> fileName;		// ���[�h��������


	LoadScreen* p_loadScreen;		// ���[�h��ʂ̃|�C���^


	void MyNextLoad(const std::string path, int& file, const ELOADFILE type);		// �񓯊����s�����\�b�h

	   
public:
	LoadThread();		// �R���X�g���N�^
	~LoadThread();		// �f�X�g���N�^


	void Process(const int max, const std::string* path, const ELOADFILE* type);		// �s��


	const std::vector<int> GetFile() const;		// ���[�h�������̂�n��


	const int GetNum() const;			// ���[�h���I������
};