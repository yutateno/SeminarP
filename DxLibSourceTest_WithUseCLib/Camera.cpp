#include "Camera.hpp"

using namespace MYINPUTPAD;

Camera::Camera(VECTOR charaarea)
{
	area = VGet(0, 215, -300);
	view = VGet(0, 90, 0);

	chara = charaarea;
	moveflag = false;

	speed = DX_PI_F / 90;
	angle = 0.0f;

	//area + chara�̎��_����view + chara�̃^�[�Q�b�g������p�x�ɃJ������ݒu
	SetCameraPositionAndTarget_UpVecY(VAdd(area, chara), VAdd(view, chara));
}

Camera::~Camera()
{

}

void Camera::Process(VECTOR charaarea)
{
	SetCameraNearFar(1.0f, 2000.0f);	// �J�����̕`��͈͂��w��

	chara = charaarea;					// �L�����̈ʒu���X�V��������

	// ���ɉ�]��
	if (KeyData::Get(KEY_INPUT_LEFT) >= 1)
	{
		area = RLrotate(speed);	// ��]����
		angle += speed;	// �L�����N�^�[�̉�]����
	}
	// �E�ɉ�]��
	if (KeyData::Get(KEY_INPUT_RIGHT) >= 1)
	{
		area = RLrotate(-speed);	// ��]����
		angle -= speed;	// �L�����N�^�[�̉�]����
	}

	// �������̎��_����������̃^�[�Q�b�g������p�x�ɃJ������ݒu
	SetCameraPositionAndTarget_UpVecY(VAdd(area, chara), VAdd(view, chara));
}

// ang�p��]����
VECTOR Camera::RLrotate(float speed)
{
	VECTOR temp;
	temp = VGet(area.x, area.y, area.z);

	return VGet((float)(temp.x * cos(speed) + temp.z * sin(speed))
		, temp.y, (float)(-temp.x * sin(speed) + temp.z * cos(speed)));
}