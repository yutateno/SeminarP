#pragma once
#include "BasicCreature.hpp"


class EnemyMove1 : public BasicCreature
{
private:
	// �����Ɋւ���
	bool upNow;					// ���オ�蒆���ǂ���
	float flyMove;				// �㉺�̃X�s�[�h
	void MoveProcess();			// �㉺�����̃v���Z�X
	MATERIALPARAM material;		// �}�e���A���𒲐��ێ�


public:
	EnemyMove1(const int collStageHandle, const float areaX, const float areaZ, const float color);
	~EnemyMove1();


	void Draw();
	void Process();


	void StolenChara(const VECTOR characterArea);		// �L�����N�^�[���߂Â�����
	void Collected();						// �����W�܂�����
};
