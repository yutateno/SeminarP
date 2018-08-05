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

	//area + charaの視点からview + charaのターゲットを見る角度にカメラを設置
	SetCameraPositionAndTarget_UpVecY(VAdd(area, chara), VAdd(view, chara));
}

Camera::~Camera()
{

}

void Camera::Process(VECTOR charaarea)
{
	SetCameraNearFar(1.0f, 2000.0f);	// カメラの描画範囲を指定

	chara = charaarea;					// キャラの位置を更新し続ける

	// 左に回転中
	if (KeyData::Get(KEY_INPUT_LEFT) >= 1)
	{
		area = RLrotate(speed);	// 回転処理
		angle += speed;	// キャラクターの回転処理
	}
	// 右に回転中
	if (KeyData::Get(KEY_INPUT_RIGHT) >= 1)
	{
		area = RLrotate(-speed);	// 回転処理
		angle -= speed;	// キャラクターの回転処理
	}

	// 第一引数の視点から第二引数のターゲットを見る角度にカメラを設置
	SetCameraPositionAndTarget_UpVecY(VAdd(area, chara), VAdd(view, chara));
}

// ang角回転する
VECTOR Camera::RLrotate(float speed)
{
	VECTOR temp;
	temp = VGet(area.x, area.y, area.z);

	return VGet((float)(temp.x * cos(speed) + temp.z * sin(speed))
		, temp.y, (float)(-temp.x * sin(speed) + temp.z * cos(speed)));
}