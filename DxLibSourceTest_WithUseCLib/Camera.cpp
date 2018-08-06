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

	SetCameraNearFar(100.0f, 3000.0f);	// ƒJƒƒ‰‚Ì•`‰æ”ÍˆÍ‚ðŽw’è

	// ‘æˆêˆø”‚ÌŽ‹“_‚©‚ç‘æ“ñˆø”‚Ìƒ^[ƒQƒbƒg‚ðŒ©‚éŠp“x‚ÉƒJƒƒ‰‚ðÝ’u
	SetCameraPositionAndTarget_UpVecY(VAdd(cameraArea, charaArea), VAdd(viewArea, charaArea));
}

Camera::~Camera()
{

}

void Camera::Process(VECTOR charaarea, unsigned __int8 controllNumber)
{
	charaArea = charaarea;					// ƒLƒƒƒ‰‚ÌˆÊ’u‚ðXV‚µ‘±‚¯‚é

	// ¶‚É‰ñ“]’†
	if (KeyData::Get(KEY_INPUT_LEFT) >= 1
		|| MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_RIGHT_AXIS_X) < 0)
	{
		RLrotate(speed, &cameraArea);	// ‰ñ“]ˆ—
		angle += speed;
	}
	// ‰E‚É‰ñ“]’†
	if (KeyData::Get(KEY_INPUT_RIGHT) >= 1
		|| MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_RIGHT_AXIS_X) > 0)
	{
		RLrotate(-speed, &cameraArea);	// ‰ñ“]ˆ—
		angle -= speed;
	}

	// ‘æˆêˆø”‚ÌŽ‹“_‚©‚ç‘æ“ñˆø”‚Ìƒ^[ƒQƒbƒg‚ðŒ©‚éŠp“x‚ÉƒJƒƒ‰‚ðÝ’u
	SetCameraPositionAndTarget_UpVecY(VAdd(cameraArea, charaArea), VAdd(viewArea, charaArea));
}

// angŠp‰ñ“]‚·‚é
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