#pragma once
#include "DxLib.h"

#include <math.h>

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
};

