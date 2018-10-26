#pragma once
#include "Basic.hpp"


/// �V�[���̓���
enum class ESceneNumber
{
	STARTLOAD, FIRSTMOVE, SECONDLOAD, SECONDMOVE, THORDLOAD, THORDMOVE, FORTHLOAD, FORTHMOVE, FIFTHLOAD, FIFTHMOVE
	, SIXTHLOAD, SIXTHMOVE, TITLELOAD, TITLEMOVE
};

/// �V�[���̐e
/// �V�[�����ƂɕK�{�ȃV���h�E�}�b�v�ƃt�H�O���s��
class BaseMove
{
private:
	// �e�Ɋւ���-----------------------------------------------
	/// �L�����N�^�[�̉e�̃n���h��
	int shadowMapCharaHandle;
	/// �L�����N�^�[�ȊO�̓����A�N�^�[�̃n���h��
	int shadowMapAnotherCharaHandle;
	/// �S�������Ȃ���̃n���h��
	int shadowMapNoMoveHandle;

	/// �L�����N�^�[�̉e�̃}�C�i�X�͈�
	VECTOR shadowCharaLowArea;				
	/// �L�����N�^�[�̉e�̃v���X�͈�
	VECTOR shadowCharaHighArea;				

	/// �L�����N�^�[�ȊO������̉e�̃}�C�i�X�͈�
	VECTOR shadowAnotherCharaLowArea;		
	/// �L�����N�^�[�ȊO������̉e�̃v���X�͈�
	VECTOR shadowAnotherCharaHighArea;		

	/// �S�������Ȃ���̉e�̃}�C�i�X�͈�
	VECTOR shadowNoMoveLowArea;			
	/// �S�������Ȃ���̉e�̃v���X�͈�
	VECTOR shadowNoMoveHighArea;			

	/// ���C�g�̃f�B���N�V��������
	VECTOR lightDire;						
	// ---------------------------------------------------------

	/// �X�J�C�{�b�N�X�Ɋւ���
	int skyBoxUp, skyBoxUnder;



protected:
	/// �V�[���̏I���t���b�O
	static bool endFlag;		

	/// ���݂̃V�[��
	static ESceneNumber scene;	


	// �e�Ɋւ���-----------------------------------------------
	// �ݒ肷��
	/// �V���h�E�}�b�v�O�ԁF��l��
	void ShadowCharaSetUpBefore();
	void ShadowCharaSetUpAfter();

	/// �V���h�E�}�b�v�P�ԁF��l���ȊO
	void ShadowAnotherCharaSetUpBefore();
	void ShadowAnotherCharaSetUpAfter();

	/// �V���h�E�}�b�v�Q�ԁF�����Ȃ�����
	void ShadowNoMoveSetUpBefore();
	void ShadowNoMoveSetUpAfter();

	// �`��֎g�p����
	/// �V���h�E�}�b�v�O�ԁF��l��
	void ShadowCharaDrawBefore();
	void ShadowCharaDrawAfter();

	/// �V���h�E�}�b�v�P�ԁF��l���ȊO
	void ShadowAnotherCharaDrawBefore();
	void ShadowAnotherCharaDrawAfter();

	/// �V���h�E�}�b�v�Q�ԁF�����Ȃ�����
	void ShadowNoMoveDrawBefore();
	void ShadowNoMoveDrawAfter();

	/// ���W���X�V��������
	void ShadowArea(const VECTOR charaArea);
	// ---------------------------------------------------------


	/// �����̃��f���̋����𒲂ׂ�
	int GetDistance(const VECTOR alpha, const VECTOR beta);


	// �X�J�C�{�b�N�X�Ɋւ���-----------------------------------
	/// �`�悷��
	void SkyBoxDraw();
	/// �L�����N�^�[�̍��W�Ƃ̑��΂ɍX�V����
	void SkyBoxProcess(const VECTOR characterArea);
	/// ������
	void SetInitSkyBox(const int skyBoxUp);



public:
	/// �R���X�g���N�^
	BaseMove();					
	/// �f�X�g���N�^
	virtual ~BaseMove();		

	/// �`��
	virtual void Draw() = 0;										
	/// �v���Z�X
	virtual void Process(const unsigned __int8 controllNumber) = 0;		
	/// �J�����̃v���Z�X
	virtual void CameraProcess() = 0;

	/// �I���Q�b�^�[
	static const bool GetEndFlag();		
	/// ���̃V�[���Q�b�^�[
	static const ESceneNumber GetScene();	

	/// ���̃V�[���Z�b�^�[
	void SetScene(const ESceneNumber scene);	
};

