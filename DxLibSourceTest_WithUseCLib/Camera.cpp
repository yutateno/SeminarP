#include "Camera.hpp"

using namespace MYINPUTPAD;


// コンストラクタ
Camera::Camera(VECTOR charaarea, int collStageHandle)
{
	stageHandle = collStageHandle;

	cameraArea = VGet(0, 350, 500);
	viewArea = VGet(0, 150, 0);

	charaArea = charaarea;
	moveflag = false;

	speed = DX_PI_F / 90;
	angle = 0.0f;

	SetCameraNearFar(100.0f, 36000.0f);	// カメラの描画範囲を指定

	// 第一引数の視点から第二引数のターゲットを見る角度にカメラを設置
	SetCameraPositionAndTarget_UpVecY(VAdd(cameraArea, charaArea), VAdd(viewArea, charaArea));
}

// デストラクタ
Camera::~Camera()
{

}


// メインプロセス
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

	// 上キーが押されていたら下から見上げる
	if (KeyData::Get(KEY_INPUT_UP) >= 1
		|| MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_RIGHT_AXIS_Y) > 0)
	{
		// 制限
		if (cameraArea.y > 240)
		{
			cameraArea = VAdd(cameraArea, VScale(VNorm(cameraArea), -10));	// 単位ベクトル化してマイナスかけて同一方向に減らす
		}
	}

	// 下キーが押されていたら上から見下ろす
	if (KeyData::Get(KEY_INPUT_DOWN) >= 1
		|| MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_RIGHT_AXIS_Y) < 0)
	{
		// 制限
		if (cameraArea.y < 400)
		{
			cameraArea = VAdd(cameraArea, VScale(VNorm(cameraArea), 10));	// VScaleいらない
		}
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

// カメラのアングルを渡すゲッター
float Camera::GetAngle()
{
	return angle;
}