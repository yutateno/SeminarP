#pragma once
#include "LoadFile.hpp"

// ���[�h��ʓI�Ȃ��̂̂��߂̔񓯊�
class LoadScreen
{
private:
	int draw;		// ���[�h��ʓI��
	int endDraw;		// ���[�h�I���I��

public:
	LoadScreen();
	~LoadScreen();

	void Process(int num, int max);		// �񓯊��ōs�����\�b�h
};

