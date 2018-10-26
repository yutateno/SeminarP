#pragma once
#include "Basic.hpp"


// �V�[���̓���
enum class ESceneNumber
{
	STARTLOAD, FIRSTMOVE, SECONDLOAD, SECONDMOVE, THORDLOAD, THORDMOVE, FORTHLOAD, FORTHMOVE, FIFTHLOAD, FIFTHMOVE
	, SIXTHLOAD, SIXTHMOVE, TITLELOAD, TITLEMOVE
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

	// �X�J�C�{�b�N�X�Ɋւ���
	int skyBoxUp, skyBoxUnder;

protected:
	// �����׊o��Ŏ��������̂Ŏ���
	int backGround;


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
	void ShadowArea(const VECTOR charaArea);
	// ---------------------------------------------------------


	// ��̃��f���̋���
	int GetDistance(const VECTOR alpha, const VECTOR beta);


	// �X�J�C�{�b�N�X�Ɋւ���
	void SkyBoxDraw();
	void SkyBoxProcess(const VECTOR characterArea);
	void SetInitSkyBox(const int skyBoxUp);


public:
	BaseMove();					// �R���X�g���N�^
	virtual ~BaseMove();		// �f�X�g���N�^

	virtual void Draw() = 0;										// �`��
	virtual void Process(const unsigned __int8 controllNumber) = 0;		// �v���Z�X
	virtual void CameraProcess() = 0;

	static const bool GetEndFlag();		// �I���Q�b�^�[
	static const ESceneNumber GetScene();	// ���̃V�[���Q�b�^�[

	void SetScene(const ESceneNumber scene);	// ���̃V�[���Z�b�^�[
};

