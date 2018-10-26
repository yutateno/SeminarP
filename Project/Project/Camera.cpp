#include "Camera.hpp"

using namespace MY_XINPUT;

// コンストラクタ
Camera::Camera(const VECTOR charaarea, const int collStageHandle)
{
	stageHandle = MV1DuplicateModel(collStageHandle);
	MV1SetScale(stageHandle, VGet(1.75f, 1.0f, 1.75f));
	MV1SetupCollInfo(stageHandle, -1);									// モデルのコリジョン情報をセットアップ(-1による全体フレーム)
	MV1SetPosition(stageHandle, VGet(0.0f, 0.0f, 0.0f));				// ステージの座標を更新
	MV1SetFrameVisible(stageHandle, -1, false);							// ステージを描画させない（でもどうせDraw呼ばないからこれ意味ない気もする）
	MV1RefreshCollInfo(stageHandle, -1);								// ステージを描画させない（でもどうせDraw呼ばないからこれ意味ない気もする）

	cameraArea = VGet(0, 350, 500);
	viewArea = VGet(0, 150, 0);

	charaArea = VAdd(charaarea, VGet(0.0f, 80.0f, 0.0f));

	speed = DX_PI_F / 90;
	angle = 0.0f;

	SetCameraNearFar(100.0f, 10000.0f);	// カメラの描画範囲を指定

	// 第一引数の視点から第二引数のターゲットを見る角度にカメラを設置
	SetCameraPositionAndTarget_UpVecY(VAdd(cameraArea, charaArea), VAdd(viewArea, charaArea));
}

// デストラクタ
Camera::~Camera()
{
	MODEL_RELEASE(stageHandle);
}


// メインプロセス
void Camera::Process(const VECTOR charaarea, const unsigned __int8 controllNumber)
{
	VECTOR TestPosition = cameraArea;
	//static int zoom = 0;

	charaArea = VAdd(charaarea, VGet(0.0f, 80.0f, 0.0f));					// キャラの位置を更新し続ける

	// 左に回転中
	if (KeyData::Get(KEY_INPUT_LEFT) >= 1
		|| InputPad::GetPadThumbData(controllNumber, STICK_RIGHT_X) < 0)
	{
		RLrotate(speed, TestPosition);	// 回転処理
		angle += speed;
	}
	// 右に回転中
	if (KeyData::Get(KEY_INPUT_RIGHT) >= 1
		|| InputPad::GetPadThumbData(controllNumber, STICK_RIGHT_X) > 0)
	{
		RLrotate(-speed, TestPosition);	// 回転処理
		angle -= speed;
	}

	// 上キーが押されていたら下から見上げる
	if (KeyData::Get(KEY_INPUT_UP) >= 1
		|| InputPad::GetPadThumbData(controllNumber, STICK_RIGHT_Y) > 0)
	{
		// 制限
		if (TestPosition.y > 240)
		{
			TestPosition = VAdd(TestPosition, VScale(VNorm(TestPosition), -10));	// 単位ベクトル化してマイナスかけて同一方向に減らす
		}
	}

	// 下キーが押されていたら上から見下ろす
	if (KeyData::Get(KEY_INPUT_DOWN) >= 1
		|| ::InputPad::GetPadThumbData(controllNumber, STICK_RIGHT_Y) < 0)
	{
		// 制限
		if (TestPosition.y < 400)
		{
			TestPosition = VAdd(TestPosition, VScale(VNorm(TestPosition), 10));	// VScaleいらない
		}
	}

	//MV1_COLL_RESULT_POLY_DIM HRes;
	//int HitNum;

	//// 注視点からカメラの座標までの間にステージのポリゴンがあるか調べる
	//HRes = MV1CollCheck_Capsule(this->stageHandle, -1, VAdd(TestPosition, charaArea), charaArea, 10.0f);
	//HitNum = HRes.HitNum;
	//MV1CollResultPolyDimTerminate(HRes);
	//if (HitNum != 0)
	//{
	//	//TestPosition = VAdd(TestPosition, VAdd(VGet(0, 10.0f, 0), VScale(VNorm(VGet(TestPosition.x, 0, TestPosition.z)), -1)));	// ズームイン処理をさせる
	//	//zoom++;
	//	cameraArea = TestPosition;

	//	//float NotHitLength;
	//	//float HitLength;
	//	//float TestLength;

	//	//// あったら無い位置までプレイヤーに近づく

	//	//// ポリゴンに当たらない距離をセット
	//	//NotHitLength = 0.0f;

	//	//// ポリゴンに当たる距離をセット
	//	//HitLength = sqrt((charaArea.x - cameraArea.x) * (charaArea.x - cameraArea.x) + (charaArea.z - cameraArea.z) * (charaArea.z - cameraArea.z));;
	//	//do
	//	//{
	//	//	// 当たるかどうかテストする距離をセット( 当たらない距離と当たる距離の中間 )
	//	//	TestLength = NotHitLength + (HitLength - NotHitLength) / 2.0f;
	//	//	
	//	//	// 新しい座標で壁に当たるかテスト
	//	//	HRes = MV1CollCheck_Capsule(this->stageHandle, -1, this->charaArea, TestPosition, 10.0f);
	//	//	HitNum = HRes.HitNum;
	//	//	MV1CollResultPolyDimTerminate(HRes);
	//	//	if (HitNum != 0)
	//	//	{
	//	//		// 当たったら当たる距離を TestLength に変更する
	//	//		HitLength = TestLength;
	//	//	}
	//	//	else
	//	//	{
	//	//		// 当たらなかったら当たらない距離を TestLength に変更する
	//	//		NotHitLength = TestLength;
	//	//	}

	//	//	// HitLength と NoHitLength が十分に近づいていなかったらループ
	//	//} while (HitLength - NotHitLength > 0.1f);
	//}
	//else
	//{
	//	if (zoom > 0)
	//	{
	//		TestPosition = VAdd(TestPosition, VAdd(VGet(0, -10.0f, 0), VScale(VNorm(VGet(TestPosition.x, 0, TestPosition.z)), 1)));	// ズームアウト処理をさせる
	//		zoom--;
	//	}
	//	cameraArea = TestPosition;
	//}

	cameraArea = TestPosition;

#ifdef _CAMERA_DEBG
	printfDx("%d\n", HitNum);
	//DrawCapsule3D(VAdd(cameraArea, charaArea), VAdd(viewArea, charaArea), 5.0f, 8, GetColor(0, 255, 0), GetColor(255, 255, 255), false);		// 面白い
	DrawCapsule3D(VAdd(cameraArea, charaArea), charaArea, 5.0f, 8, GetColor(0, 255, 0), GetColor(255, 255, 255), false);		// 当たり判定を確認用の表示テスト
#endif // !_CAMERA_DEBG

}

void Camera::SetUp()
{
	SetupCamera_Perspective(60.0f * DX_PI_F / 180.0f);

	SetCameraNearFar(100.0f, 10000.0f);	// カメラの描画範囲を指定

	// 第一引数の視点から第二引数のターゲットを見る角度にカメラを設置
	SetCameraPositionAndTarget_UpVecY(VAdd(cameraArea, charaArea), VAdd(viewArea, charaArea));
}

// ang角回転する
void Camera::RLrotate(const float speed, VECTOR& p_cameraArea)
{
	float tempX = p_cameraArea.x;
	p_cameraArea.x = tempX * cosf(speed) + p_cameraArea.z *sinf(speed);
	p_cameraArea.z = -tempX * sinf(speed) + p_cameraArea.z * cosf(speed);
}

// カメラのアングルを渡すゲッター
const float Camera::GetAngle() const
{
	return angle;
}