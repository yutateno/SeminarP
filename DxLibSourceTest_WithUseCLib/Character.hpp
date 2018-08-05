#pragma once
#include "DxLib.h"
#include "InputPad.hpp"
#include "InputKey.hpp"

class Character
{
private:
	// ���ꂼ��̈ʒu�Ɋւ���
	VECTOR area;	// �L�����ʒu

	// ���f���Ɋւ���
	int charamodelhandle;	// ���f���̃n���h��
		
public:
	Character();	// �R���X�g���N�^
	~Character();																// �f�X�g���N�^

	void Draw();		// �`��
	void Process();		// �v���Z�X

	VECTOR GetArea();
};