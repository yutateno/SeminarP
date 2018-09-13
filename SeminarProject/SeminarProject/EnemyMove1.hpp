#pragma once
#include "BasicCreature.hpp"


class EnemyMove1 : public BasicCreature
{
private:
	int draw;


	// �����Ɋւ���
	bool upNow;					// ���オ�蒆���ǂ���
	float flyMove;				// �㉺�̃X�s�[�h
	void MoveProcess();			// �㉺�����̃v���Z�X
	MATERIALPARAM material;		// �}�e���A���𒲐��ێ�


public:
	EnemyMove1(int collStageHandle, float areaX, float areaZ, float color);
	~EnemyMove1();


	void Draw();
	void Process();


	void StolenChara(VECTOR characterArea);		// �L�����N�^�[���߂Â�����
	void Collected();						// �����W�܂�����
};