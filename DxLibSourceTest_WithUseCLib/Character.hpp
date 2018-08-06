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
	void Process(unsigned __int8 controllNumber);		// �v���Z�X

	VECTOR GetArea();
};