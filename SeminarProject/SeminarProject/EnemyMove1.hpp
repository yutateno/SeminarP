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
	bool viewNow;				// �������Ă��邩�ǂ���(�`�悷�邩)

public:
	EnemyMove1(int collStageHandle, float areaX, float areaZ, float color);
	~EnemyMove1();

	void Draw();
	void Process();

	void ViewLost();		// �`��������Ȃ�
	void StolenChara(VECTOR characterArea);		// �L�����N�^�[�ɕ߂܂���
	void Collected();						// �����W�܂�����
};
