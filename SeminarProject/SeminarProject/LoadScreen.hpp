#pragma once
#include "LoadFile.hpp"


class LoadScreen
{
private:
	int draw;			// ���[�h��ʓI��
	int endDraw;		// ���[�h�I���I��


public:
	LoadScreen();		// �R���X�g���N�^
	~LoadScreen();		// �f�X�g���N�^


	void Process(const int num, const int max);		// �񓯊��ōs�����\�b�h
};

