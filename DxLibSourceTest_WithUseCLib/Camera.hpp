#pragma once
#include "DxLib.h"
#include "InputPad.hpp"
#include "InputKey.hpp"
#include <math.h>

class Camera
{
private:
	// ���ꂼ��̈ʒu�Ɋւ���
	VECTOR area;		// �J�����̈ʒu
	VECTOR view;		// �����������
	VECTOR chara;		// �L�����ʒu
	float angle;		// �L�����N�^�[�̃A���O��

	// �J�����̃Y�[�����]�̓����Ɋւ���
	float speed;					// ��]�X�s�[�h
	bool moveflag;					// �J�������������ǂ���
	VECTOR RLrotate(float speed);	// ��]���s���֐�
	
public:
	Camera(VECTOR charaarea);				// �L�����̈ʒu�������Ɏ�����R���X�g���N�^
	~Camera();																// �f�X�g���N�^

	void Process(VECTOR charaarea);		// �L�����̈ʒu�������Ɏ�����v���Z�X
};