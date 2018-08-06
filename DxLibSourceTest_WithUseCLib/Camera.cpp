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

	SetCameraNearFar(100.0f, 3000.0f);	// カメラの描画範囲を指定

	// 第一引数の視点から第二引数のターゲットを見る角度にカメラを設置
	SetCameraPositionAndTarget_UpVecY(VAdd(cameraArea, charaArea), VAdd(viewArea, charaArea));
}

Camera::~Camera()
{

}

void Camera::Process(VECTOR charaarea, unsigned __int8 controllNumber)
{
	charaArea = charaarea;					// キャラの位置を更新し続ける

	// 左に回転中
	if (KeyData::Get(KEY_INPUT_LEFT) >= 1
		|| MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_RIGHT_AXIS_X) < 0)
	{
		RLrotate(speed, &cameraArea);	// 回転処理
		angle += speed;
	}
	// 右に回転中
	if (KeyData::Get(KEY_INPUT_RIGHT) >= 1
		|| MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_RIGHT_AXIS_X) > 0)
	{
		RLrotate(-speed, &cameraArea);	// 回転処理
		angle -= speed;
	}

	// 第一引数の視点から第二引数のターゲットを見る角度にカメラを設置
	SetCameraPositionAndTarget_UpVecY(VAdd(cameraArea, charaArea), VAdd(viewArea, charaArea));
}

// ang角回転する
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