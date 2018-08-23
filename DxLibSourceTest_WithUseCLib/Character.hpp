#pragma once
#include "DxLib.h"
#include "InputPad.hpp"
#include "InputKey.hpp"
#include "Basic.hpp"
#include "LoadFile.hpp"

class Character
{
private:
	// ���ꂼ��̈ʒu�Ɋւ���
	VECTOR area;	// �L�����ʒu
	VECTOR preArea;	// �ړ��O�̒��O�̃L�����ʒu
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
	enum MOTION{run, idle, walk};
	int attachMotion;			// ���[�V�����̃A�^�b�`
	int attachNum;				// ���݂̃��[�V�����ԍ�
	float totalTime;			// ���[�V�����̑�������
	float nowPlayTime;			// ���[�V�����̌��݂̎���
	float motionBlendTime;		// �u�����h����
	int preAttach;				// ���O�̃��[�V�����A�^�b�`
	float preMotionPlayTime;	// ���O�̃��[�V��������
	void Player_PlayAnim(int attach);		// ���[�V�����ύX
	void Player_AnimProcess();				// ���[�V��������

	// �����蔻��
	int stageHandle;					// �X�e�[�W�n���h��
	int wallNum;						// ���肷��ǂ̐�
	int floorNum;						// ���肷�鏰�̐�
	bool hitFlag;						// �������Ă��邩�ǂ���
	bool moveFlag;						// �����Ă��邩�ǂ���
	float maxYHit;						// �������Ă���Y���W�̍ő�
	MV1_COLL_RESULT_POLY_DIM hitDim;			// ���͂̃|���S����������\����
	MV1_COLL_RESULT_POLY* wallPoly[2048];		// �Ǘp�̃|���S���̍\����
	MV1_COLL_RESULT_POLY* floorPoly[2048];		// ���p�̃|���S���̍\����
	MV1_COLL_RESULT_POLY* mainPoly;				// �|���S���̍\���̂ɃA�N�Z�X����\����
	HITRESULT_LINE lineResult;					// �����Ƃ̔����������\����
	void StageHit();							// �����蔻����s��
			
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