#pragma once
#include "BasicObject.hpp"


class BasicCreature : public BasicObject
{
protected:
	// �ʒu�ɂ���
	VECTOR preArea;	// �ړ��O�̒��O�̃L�����ʒu
	float angle;	// �A���O��


	// �����Ɋւ���
	float walkSpeed;	// �ړ��X�s�[�h
	float animSpeed;	// ���[�V�����X�s�[�h
	int fallCount;		// �����蔻�胂�f���S�Ă��痎���Ă��邩�ǂ���


	// ���f���Ɋւ���
	float modelWigth;	// ���f���̉���


	// ���[�V�����Ɋւ���
	int attachMotion;			// ���[�V�����̃A�^�b�`
	int attachNum;				// ���݂̃��[�V�����ԍ�
	float totalTime;			// ���[�V�����̑�������
	void Player_PlayAnim(int attach);		// ���[�V�����ύX
	void Player_AnimProcess();				// ���[�V��������


	// �����蔻��
	bool moveFlag;						// �����Ă��邩�ǂ���
	void StageHit();					// �����蔻����s��
	void ActorHit(int stageHandle);		// �����蔻����s��


private:
	// ���[�V�����Ɋւ���
	float nowPlayTime;			// ���[�V�����̌��݂̎���
	float motionBlendTime;		// �u�����h����
	int preAttach;				// ���O�̃��[�V�����A�^�b�`
	float preMotionPlayTime;	// ���O�̃��[�V��������


	// �����蔻��
	int stageHandle;					// �X�e�[�W�n���h��
	int wallNum;						// ���肷��ǂ̐�
	int floorNum;						// ���肷�鏰�̐�
	bool hitFlag;						// �������Ă��邩�ǂ���
	float maxYHit;						// �������Ă���Y���W�̍ő�
	MV1_COLL_RESULT_POLY_DIM hitDim;			// ���͂̃|���S����������\����
	MV1_COLL_RESULT_POLY* wallPoly[2048];		// �Ǘp�̃|���S���̍\����
	MV1_COLL_RESULT_POLY* floorPoly[2048];		// ���p�̃|���S���̍\����
	MV1_COLL_RESULT_POLY* mainPoly;				// �|���S���̍\���̂ɃA�N�Z�X����\����
	HITRESULT_LINE lineResult;					// �����Ƃ̔����������\����


public:
	BasicCreature(const int collStageHandle);
	virtual ~BasicCreature();
};