#pragma once
#include "Basic.hpp"


// �V�[���̓���
enum class ESceneNumber
{
	STARTLOAD, FIRSTMOVE, SECONDLOAD, SECONDMOVE
};


class BaseMove
{
private:
	// �e�Ɋւ���-----------------------------------------------
	int shadowMapCharaHandle;				// �L�����N�^�[�̉e�̃n���h��
	int shadowMapAnotherCharaHandle;		// �L�����N�^�[�ȊO�̓����A�N�^�[�̃n���h��
	int shadowMapNoMoveHandle;				// �S�������Ȃ���̃n���h��

	VECTOR shadowCharaLowArea;				// �L�����N�^�[�̉e�̃}�C�i�X�͈�
	VECTOR shadowCharaHighArea;				// �L�����N�^�[�̉e�̃v���X�͈�

	VECTOR shadowAnotherCharaLowArea;		// �L�����N�^�[�ȊO������̉e�̃}�C�i�X�͈�
	VECTOR shadowAnotherCharaHighArea;		// �L�����N�^�[�ȊO������̉e�̃v���X�͈�

	VECTOR shadowNoMoveLowArea;				// �S�������Ȃ���̉e�̃}�C�i�X�͈�
	VECTOR shadowNoMoveHighArea;			// �S�������Ȃ���̉e�̃v���X�͈�

	VECTOR lightDire;						// ���C�g�̃f�B���N�V��������
	// ---------------------------------------------------------


protected:
	static bool endFlag;		// �V�[���̏I���t���b�O

	static ESceneNumber scene;	// ���݂̃V�[��


	// �e�Ɋւ���-----------------------------------------------
	// �V���h�E�}�b�v�O�ԁF��l��
	void ShadowCharaSetUpBefore();
	void ShadowCharaSetUpAfter();

	// �V���h�E�}�b�v�P�ԁF��l���ȊO
	void ShadowAnotherCharaSetUpBefore();
	void ShadowAnotherCharaSetUpAfter();

	// �V���h�E�}�b�v�Q�ԁF�����Ȃ�����
	void ShadowNoMoveSetUpBefore();
	void ShadowNoMoveSetUpAfter();

	// �`��֎g�p����
	// �V���h�E�}�b�v�O�ԁF��l��
	void ShadowCharaDrawBefore();
	void ShadowCharaDrawAfter();

	// �V���h�E�}�b�v�P�ԁF��l���ȊO
	void ShadowAnotherCharaDrawBefore();
	void ShadowAnotherCharaDrawAfter();

	// �V���h�E�}�b�v�Q�ԁF�����Ȃ�����
	void ShadowNoMoveDrawBefore();
	void ShadowNoMoveDrawAfter();

	// ���W���X�V��������
	void ShadowArea(VECTOR charaArea);
	// ---------------------------------------------------------


	// ��̃��f���̋���
	int GetDistance(VECTOR alpha, VECTOR beta);


public:
	BaseMove();					// �R���X�g���N�^
	virtual ~BaseMove();		// �f�X�g���N�^

	virtual void Draw() = 0;										// �`��
	virtual void Process(unsigned __int8 controllNumber) = 0;		// �v���Z�X

	static bool GetEndFlag();		// �I���Q�b�^�[
	static ESceneNumber GetScene();	// ���̃V�[���Q�b�^�[

	void SetScene(ESceneNumber scene);	// ���̃V�[���Z�b�^�[
};
