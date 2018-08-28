#include "Camera.hpp"

using namespace MYINPUTPAD;


// �R���X�g���N�^
Camera::Camera(VECTOR charaarea, int collStageHandle)
{
	stageHandle = collStageHandle;

	cameraArea = VGet(0, 350, 500);
	viewArea = VGet(0, 150, 0);

	charaArea = charaarea;
	moveflag = false;

	speed = DX_PI_F / 90;
	angle = 0.0f;

	SetCameraNearFar(100.0f, 36000.0f);	// �J�����̕`��͈͂��w��

	// �������̎��_����������̃^�[�Q�b�g������p�x�ɃJ������ݒu
	SetCameraPositionAndTarget_UpVecY(VAdd(cameraArea, charaArea), VAdd(viewArea, charaArea));
}

// �f�X�g���N�^
Camera::~Camera()
{

}


// ���C���v���Z�X
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

	// ��L�[��������Ă����牺���猩�グ��
	if (KeyData::Get(KEY_INPUT_UP) >= 1
		|| MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_RIGHT_AXIS_Y) > 0)
	{
		// ����
		if (cameraArea.y > 240)
		{
			cameraArea = VAdd(cameraArea, VScale(VNorm(cameraArea), -10));	// �P�ʃx�N�g�������ă}�C�i�X�����ē�������Ɍ��炷
		}
	}

	// ���L�[��������Ă�����ォ�猩���낷
	if (KeyData::Get(KEY_INPUT_DOWN) >= 1
		|| MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_RIGHT_AXIS_Y) < 0)
	{
		// ����
		if (cameraArea.y < 400)
		{
			cameraArea = VAdd(cameraArea, VScale(VNorm(cameraArea), 10));	// VScale����Ȃ�
		}
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

// �J�����̃A���O����n���Q�b�^�[
float Camera::GetAngle()
{
	return angle;
}