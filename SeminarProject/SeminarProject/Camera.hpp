#pragma once
#include "DxLib.h"
#include "InputPad.hpp"
#include "InputKey.hpp"
#include <math.h>

class Camera
{
private:
	// ���ꂼ��̈ʒu�Ɋւ���
	VECTOR cameraArea;		// �J�����̈ʒu
	VECTOR viewArea;		// �����������
	VECTOR charaArea;		// �L�����ʒu
	float angle;		// �L�����N�^�[�̃A���O��

	int stageHandle;		// �X�e�[�W

	// �J�����̃Y�[�����]�̓����Ɋւ���
	float speed;					// ��]�X�s�[�h
	bool moveflag;					// �J�������������ǂ���
	void RLrotate(float speed, VECTOR* p_cameraArea);	// ��]���s���֐�

public:
	Camera(VECTOR charaarea, int collStageHandle);				// �L�����̈ʒu�������Ɏ�����R���X�g���N�^
	~Camera();																// �f�X�g���N�^

	void Process(VECTOR charaarea, unsigned __int8 controllNumber);		// �L�����̈ʒu�������Ɏ�����v���Z�X

	float GetAngle();				// �L�����N�^�[�̃A���O��
};