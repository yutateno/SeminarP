#pragma once
#include "BasicObject.hpp"


/// �����A�N�^�[�̐e
class BasicCreature : public BasicObject
{
protected:
	// �ʒu�ɂ���----------------------
	/// �ړ��O�̒��O�̃L�����ʒu
	VECTOR preArea;	
	/// �A���O��
	float angle;	


	// �����Ɋւ���----------------------------------------
	/// �ړ��X�s�[�h
	float walkSpeed;	
	/// ���[�V�����X�s�[�h
	float animSpeed;	
	/// �����蔻�胂�f���S�Ă��痎���Ă��邩�ǂ���
	int fallCount;		


	// ���f���Ɋւ���-----
	/// ���f���̉���
	float modelWigth;	


	// ���[�V�����Ɋւ���---------------------------
	/// ���[�V�����̃A�^�b�`
	int attachMotion;			
	/// ���݂̃��[�V�����ԍ�
	int attachNum;			
	/// ���[�V�����̑�������
	float totalTime;			
	/// ���[�V�����ύX
	void Player_PlayAnim(int attach);		
	/// ���[�V��������
	void Player_AnimProcess();				


	// �����蔻��-----------------------------
	/// �����Ă��邩�ǂ���
	bool moveFlag;						
	/// �����蔻����s��
	void StageHit();					
	/// �����蔻����s��
	void ActorHit(int stageHandle);		


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