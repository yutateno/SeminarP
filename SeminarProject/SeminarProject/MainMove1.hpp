#pragma once
#include "Stage.hpp"
#include "Character.hpp"
#include "EnemyMove1.hpp"
#include "Camera.hpp"
#include "BaseMove.hpp"

#include <random>

class MainMove1 : public BaseMove
{
private:
	enum EFILE1 { drawStage, collStage, character, feedWhite };				// ���[�h����n�����t�@�C���̏���

	Stage* p_stage;					// �X�e�[�W�̃|�C���^
	Character* p_character;			// �L�����N�^�[�̃|�C���^
	const int enemyNum = 30;		// �G�̐�

	struct EnemyAggre
	{
		EnemyMove1* p_enemyMove;		// �G�̃|�C���^
		bool aliveNow;					// �����Ă��邩
	};
	EnemyAggre s_enemyAggre[30];		// �G�̍\���̂�����
	

	Camera* p_camera;					// �J�����̃|�C���^

	void ActorHit();				// �A�N�^�[���m�̂����蔻��(�Ȉ�)
	int catchEnemyNum;				// �G����ɓ��ꂽ��

	//void ShadowDraw();			// �e

	const int lightNum = 4;			// ���C�g�n���h���̐�
	int lightHandle[4];			// ���C�g�n���h�����ێ�
	float lightRange[4];				// ���C�g�͈̔�
	VECTOR lightArea[4];				// ���C�g�̍��W
	void LightProcess();				// ���C�g�Ɋւ���֐�

	bool lightEventStart;				// �C�x���g���s��
	bool lightEventEnd;					// �C�x���g�̏I�����m�F
	int lightEventCount;				// �C�x���g�̃J�E���g
	bool lightEnd;						// ���C�g������
	float lightRangePreMax;
	float lightRangeSpeed;

	int backgroundColor;		// �w�i�F

	int drawWhite;			// �t�F�[�h�C���p�z���C�g�摜


#ifdef _DEBUG
	void DebugKeyControll();
#endif // _DEBUG


public:
	MainMove1(std::vector<int> v_file);			// �R���X�g���N�^
	~MainMove1();							// �f�X�g���N�^


	void Draw();										// �`��
	void Process(unsigned __int8 controllNumber);		// �v���Z�X
};