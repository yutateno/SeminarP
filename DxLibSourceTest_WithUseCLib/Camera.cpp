#include "Camera.hpp"

using namespace MYINPUTPAD;

Camera::Camera(VECTOR charaarea)
{
	cameraArea = VGet(0, 620, 800);
	viewArea = VGet(0, 90, 0);

	charaArea = charaarea;
	moveflag = false;

	speed = DX_PI_F / 90;
	angle = 0.0f;

	SetCameraNearFar(100.0f, 3000.0f);	// �J�����̕`��͈͂��w��

	// �������̎��_����������̃^�[�Q�b�g������p�x�ɃJ������ݒu
	SetCameraPositionAndTarget_UpVecY(VAdd(cameraArea, charaArea), VAdd(viewArea, charaArea));
}

Camera::~Camera()
{

}

void Camera::Process(VECTOR charaarea, unsigned __int8 controllNumber)
{
	charaArea = charaarea;					// �L�����̈ʒu���X�V��������

	// ���ɉ�]��
	if (KeyData::Get(KEY_INPUT_LEFT) >= 1
		|| MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_RIGHT_AXIS_X) < 0)
	{
		RLrotate(speed, &cameraArea);	// ��]����
		angle += speed;
	}
	// �E�ɉ�]��
	if (KeyData::Get(KEY_INPUT_RIGHT) >= 1
		|| MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_RIGHT_AXIS_X) > 0)
	{
		RLrotate(-speed, &cameraArea);	// ��]����
		angle -= speed;
	}

	// �������̎��_����������̃^�[�Q�b�g������p�x�ɃJ������ݒu
	SetCameraPositionAndTarget_UpVecY(VAdd(cameraArea, charaArea), VAdd(viewArea, charaArea));
}

// ang�p��]����
void Camera::RLrotate(float speed, VECTOR* p_cameraArea)
{
	float tempX = p_cameraArea->x;
	p_cameraArea->x = tempX *cosf(speed) + p_cameraArea->z *sinf(speed);
	p_cameraArea->z = -tempX * sinf(speed) + p_cameraArea->z * cosf(speed);
}

float Camera::GetAngle()
{
	return angle;
}