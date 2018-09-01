#pragma once
#include "DxLib.h"

#include <math.h>

enum class ESceneNumber
{
	STARTLOAD, FIRSTMOVE
};

class BaseMove
{
private:
	// �e�Ɋւ���-----------------------------------------------
	int shadowMapCharaHandle;
	int shadowMapAnotherCharaHandle;
	int shadowMapNoMoveHandle;

	VECTOR shadowCharaLowArea;
	VECTOR shadowCharaHighArea;

	VECTOR shadowAnotherCharaLowArea;
	VECTOR shadowAnotherCharaHighArea;

	VECTOR shadowNoMoveLowArea;
	VECTOR shadowNoMoveHighArea;

	VECTOR lightDire;
	// ---------------------------------------------------------

protected:
	static bool endFlag;		// �I���t���b�O

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
	BaseMove();
	virtual ~BaseMove();

	virtual void Draw() = 0;
	virtual void Process(unsigned __int8 controllNumber) = 0;

	static bool GetEndFlag();		// �I���Q�b�^�[
	static ESceneNumber GetScene();	// ���̃V�[���Q�b�^�[

	void SetScene(ESceneNumber scene);	// ���̃V�[���Z�b�^�[
};

