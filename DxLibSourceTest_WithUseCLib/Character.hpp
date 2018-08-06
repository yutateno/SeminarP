#pragma once
#include "DxLib.h"
#include "InputPad.hpp"
#include "InputKey.hpp"
#include "Basic.hpp"

class Character
{
private:
	// ���ꂼ��̈ʒu�Ɋւ���
	VECTOR area;	// �L�����ʒu
	float angle;	// �A���O��
	float direYAngle;		// �O��̃L���������������ϐ�
	float direXAngle;		// ���E�̃L���������������ϐ�

	// �����Ɋւ���
	float walkSpeed;	// �ړ��X�s�[�h
	float animSpeed;	// ���[�V�����X�s�[�h

	// ���f���Ɋւ���
	int charamodelhandle;	// ���f���̃n���h��
	float modelHeight;
	float modelWigth;

	// ���[�V�����Ɋւ���
	int attachMotion;			// ���[�V�����̃A�^�b�`
	int attachNum;				// ���݂̃��[�V�����ԍ�
	float totalTime;			// ���[�V�����̑�������
	float nowPlayTime;			// ���[�V�����̌��݂̎���
	float motionBlendTime;		// �u�����h����
	int preAttach;				// ���O�̃��[�V�����A�^�b�`
	float preMotionPlayTime;	// ���O�̃��[�V��������
	
	void Player_PlayAnim(int attach);		// ���[�V�����ύX
	void Player_AnimProcess();				// ���[�V��������
			
public:
	Character();	// �R���X�g���N�^
	~Character();																// �f�X�g���N�^

	void Draw();		// �`��
	void Process(unsigned __int8 controllNumber);		// �v���Z�X

	void Process(unsigned __int8 controllNumber, float getAngle);

	VECTOR GetArea();
};


/*
�L�����̌����ɂ���
�X�e�B�b�N���͂����ƍŌ�̓��͂��L���Ă��܂����ߏ������тɂȂ�
�X�e�B�b�N�̓������ߌ�������ꍇ������������
*/