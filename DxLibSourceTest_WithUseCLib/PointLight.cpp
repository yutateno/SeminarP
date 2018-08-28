#include "PointLight.hpp"

PointLight::PointLight()
{
	// �e�p�����[�^��������
	OutAngle = 90.0f;
	InAngle = 45.0f;
	Range = 20000.0f;
	Atten0 = 0.0f;
	Atten1 = 0.0006f;
	Atten2 = 0.0f;
}

PointLight::~PointLight()
{
}

void PointLight::Process(VECTOR area)
{
	// �`�y�L�[�� OutAngle �̒l��ύX
	if (CheckHitKey(KEY_INPUT_A) == 1)
	{
		OutAngle += 1.0f;
	}
	if (CheckHitKey(KEY_INPUT_Z) == 1)
	{
		OutAngle -= 1.0f;
	}

	// �r�w�L�[�� InAngle �̒l��ύX
	if (CheckHitKey(KEY_INPUT_S) == 1)
	{
		InAngle += 1.0f;
	}
	if (CheckHitKey(KEY_INPUT_X) == 1)
	{
		InAngle -= 1.0f;
	}

	// �c�b�L�[�Ń��C�g�̉e���͈͂�ύX
	if (CheckHitKey(KEY_INPUT_D) == 1)
	{
		Range += 20.0f;
	}
	if (CheckHitKey(KEY_INPUT_C) == 1)
	{
		Range -= 20.0f;
	}

	// �e�u�L�[�Ń��C�g�̋��������p�����[�^�O�̒l��ύX
	if (CheckHitKey(KEY_INPUT_F) == 1)
	{
		Atten0 += 0.001f;
	}
	if (CheckHitKey(KEY_INPUT_V) == 1)
	{
		Atten0 -= 0.001f;
	}

	// �f�a�L�[�Ń��C�g�̋��������p�����[�^�P�̒l��ύX
	if (CheckHitKey(KEY_INPUT_G) == 1)
	{
		Atten1 += 0.00001f;
	}
	if (CheckHitKey(KEY_INPUT_B) == 1)
	{
		Atten1 -= 0.00001f;
	}

	// �g�m�L�[�Ń��C�g�̋��������p�����[�^�Q�̒l��ύX
	if (CheckHitKey(KEY_INPUT_H) == 1)
	{
		Atten2 += 0.0000001f;
	}
	if (CheckHitKey(KEY_INPUT_N) == 1)
	{
		Atten2 -= 0.0000001f;
	}

	// �p�x�̒l��␳
	if (OutAngle < 0.0f) OutAngle = 0.0f;
	if (OutAngle > 180.0f) OutAngle = 180.0f;
	if (InAngle < 0.0f) InAngle = 0.0f;
	if (InAngle > OutAngle) InAngle = OutAngle;

	// �e�������̒l��␳
	if (Range < 0.0f) Range = 0.0f;

	// ���������p�����[�^�̒l��␳
	if (Atten0 < 0.0f) Atten0 = 0.0f;
	if (Atten1 < 0.0f) Atten1 = 0.0f;
	if (Atten2 < 0.0f) Atten2 = 0.0f;

	// �x���̃}�C�i�X�����̃X�|�b�g���C�g��ݒ�
	ChangeLightTypeSpot(
		VAdd(area, VGet(0.0f, 1000.0f,0.0f)),
		area,
		OutAngle * DX_PI_F / 180.0f,
		InAngle * DX_PI_F / 180.0f,
		Range,
		Atten0,
		Atten1,
		Atten2);
}

void PointLight::Draw(VECTOR area)
{
	// �p�����[�^�̓��e����ʂɕ\��
	DrawFormatString(0, 0, GetColor(255, 255, 255), "Key:A.Z  OutAngle( �x ) %f", OutAngle);
	DrawFormatString(0, 16, GetColor(255, 255, 255), "Key:S.X  InAngle( �x )  %f", InAngle);
	DrawFormatString(0, 32, GetColor(255, 255, 255), "Key:D.C  Range          %f", Range);
	DrawFormatString(0, 48, GetColor(255, 255, 255), "Key:F.V  Atten0         %f", Atten0);
	DrawFormatString(0, 64, GetColor(255, 255, 255), "Key:G.B  Atten1         %f", Atten1);
	DrawFormatString(0, 80, GetColor(255, 255, 255), "Key:H.N  Atten2         %f / 100.0f", Atten2 * 100.0f);
}
