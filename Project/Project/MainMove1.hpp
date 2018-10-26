#pragma once
#include "Stage.hpp"
#include "Character.hpp"
#include "EnemyMove1.hpp"
#include "Camera.hpp"
#include "BaseMove.hpp"
#include "DropItemMove1.hpp"

#include <random>


/*
�����W�߂�V�[��
�����Ă���ʂ��W�߂Č�����ɓ����
*/
class MainMove1 : public BaseMove
{
private:
	enum EFILE { drawStage, collStage, character, sword, sound };				// ���[�h����n�����t�@�C���̏���

	// �X�e�[�W
	Stage* p_stage;					// �X�e�[�W�̃|�C���^


	// �L�����N�^�[
	Character* p_character;			// �L�����N�^�[�̃|�C���^
	int catchEnemyNum;				// �G����ɓ��ꂽ��


	// �G
	const int enemyNum = 30;		// �G�̐�
	struct EnemyAggre
	{
		EnemyMove1* p_enemyMove;		// �G�̃|�C���^
		bool aliveNow;					// �����Ă��邩
	};
	EnemyAggre s_enemyAggre[30];		// �G�̍\���̂�����


	// �J����
	Camera* p_camera;					// �J�����̃|�C���^


	// �ȒP�Ȃ����蔻��
	void ActorHit();				// �A�N�^�[���m�̂����蔻��(�Ȉ�)


	// ���C�g�̊�{
	const int lightNum = 4;				// ���C�g�n���h���̐�
	int lightHandle[4];					// ���C�g�n���h�����ێ�
	float lightRange[4];				// ���C�g�͈̔�
	VECTOR lightArea[4];				// ���C�g�̍��W
	// ���C�g�ɕω���������
	void LightProcess();				// ���C�g�Ɋւ���֐�
	bool lightEventStart;				// �C�x���g���s��
	bool lightEventEnd;					// �C�x���g�̏I�����m�F
	int lightEventCount;				// �C�x���g�̃J�E���g
	bool lightEnd;						// ���C�g������
	float lightRangePreMax;				// �����̍L���̒��O�}�b�N�X
	float lightRangeSpeed;				// �����̍L�����L����X�s�[�h


	// �w�i�F
	int backgroundColor;		// �w�i�F


	// �����Ă錕
	DropItemMove1* p_dropItem;		// ������A�C�e���i���j
	bool touchSword;				// �����Ă�A�C�e���ɐG���


	// �T�E���h
	int soundBG;


#ifdef _DEBUG
	void DebugKeyControll();
#endif // _DEBUG


public:
	MainMove1(const std::vector<int> v_file);			// �R���X�g���N�^
	~MainMove1();							// �f�X�g���N�^


	void Draw();										// �`��
	void Process(const unsigned __int8 controllNumber);		// �v���Z�X
	void CameraProcess();
};