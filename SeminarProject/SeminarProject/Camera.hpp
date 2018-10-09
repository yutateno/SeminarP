#pragma once
#include "Basic.hpp"


class Camera
{
private:
	VECTOR cameraArea;		// �J�����̈ʒu
	VECTOR viewArea;		// �����������
	VECTOR charaArea;		// �L�����ʒu
	float angle;		// �L�����N�^�[�̃A���O��


	int stageHandle;		// �����蔻��p�X�e�[�W


	float speed;					// ��]�X�s�[�h
	void RLrotate(const float speed, VECTOR& p_cameraArea);	// ��]���s���֐�


public:
	Camera(const VECTOR charaarea, const int collStageHandle);				// �L�����̈ʒu�������Ɏ�����R���X�g���N�^
	~Camera();													// �f�X�g���N�^


	void Process(const VECTOR charaarea, const unsigned __int8 controllNumber);		// �L�����̈ʒu�������Ɏ�����v���Z�X


	void SetUp();


	const float GetAngle() const;				// �L�����N�^�[�̃A���O��
};