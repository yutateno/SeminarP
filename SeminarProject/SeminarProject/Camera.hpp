#pragma once
#include "Basic.hpp"
#include "InputPad.hpp"


class Camera
{
private:
	VECTOR cameraArea;		// �J�����̈ʒu
	VECTOR viewArea;		// �����������
	VECTOR charaArea;		// �L�����ʒu
	float angle;		// �L�����N�^�[�̃A���O��


	int stageHandle;		// �����蔻��p�X�e�[�W


	float speed;					// ��]�X�s�[�h
	void RLrotate(float speed, VECTOR* p_cameraArea);	// ��]���s���֐�


public:
	Camera(VECTOR charaarea, int collStageHandle);				// �L�����̈ʒu�������Ɏ�����R���X�g���N�^
	~Camera();													// �f�X�g���N�^


	void Process(VECTOR charaarea, unsigned __int8 controllNumber);		// �L�����̈ʒu�������Ɏ�����v���Z�X


	float GetAngle();				// �L�����N�^�[�̃A���O��
};