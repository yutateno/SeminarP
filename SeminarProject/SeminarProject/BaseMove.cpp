#include "BaseMove.hpp"

bool BaseMove::endFlag;		// 終了フラッグ
ESceneNumber BaseMove::scene;	// 現在のシーン


// シャドウマップへの描画の準備
void BaseMove::ShadowCharaSetUpBefore()
{
	ShadowMap_DrawSetup(shadowMapCharaHandle);
}

// シャドウマップへの描画を終了
void BaseMove::ShadowCharaSetUpAfter()
{
	ShadowMap_DrawEnd();
}

// シャドウマップへの描画の準備
void BaseMove::ShadowAnotherCharaSetUpBefore()
{
	ShadowMap_DrawSetup(shadowMapAnotherCharaHandle);
}

// シャドウマップへの描画を終了
void BaseMove::ShadowAnotherCharaSetUpAfter()
{
	ShadowMap_DrawEnd();
}

// シャドウマップへの描画の準備
void BaseMove::ShadowNoMoveSetUpBefore()
{
	ShadowMap_DrawSetup(shadowMapNoMoveHandle);
}

// シャドウマップへの描画を終了
void BaseMove::ShadowNoMoveSetUpAfter()
{
	ShadowMap_DrawEnd();
}


// 描画に使用するシャドウマップを設定
void BaseMove::ShadowCharaDrawBefore()
{
	SetUseShadowMap(0, shadowMapCharaHandle);
}

// 描画に使用するシャドウマップの設定を解除
void BaseMove::ShadowCharaDrawAfter()
{
	SetUseShadowMap(0, -1);
}

// 描画に使用するシャドウマップを設定
void BaseMove::ShadowAnotherCharaDrawBefore()
{
	SetUseShadowMap(1, shadowMapAnotherCharaHandle);
}

// 描画に使用するシャドウマップの設定を解除
void BaseMove::ShadowAnotherCharaDrawAfter()
{
	SetUseShadowMap(1, -1);
}

// 描画に使用するシャドウマップを設定
void BaseMove::ShadowNoMoveDrawBefore()
{
	SetUseShadowMap(2, shadowMapNoMoveHandle);
}

// 描画に使用するシャドウマップの設定を解除
void BaseMove::ShadowNoMoveDrawAfter()
{
	SetUseShadowMap(2, -1);
}

void BaseMove::ShadowArea(const VECTOR charaArea)
{
	SetShadowMapDrawArea(shadowMapCharaHandle, VAdd(charaArea, shadowCharaLowArea), VAdd(charaArea, shadowCharaHighArea));
	SetShadowMapDrawArea(shadowMapAnotherCharaHandle, VAdd(charaArea, shadowAnotherCharaLowArea), VAdd(charaArea, shadowAnotherCharaHighArea));
}


// 二つのキャラの直線的距離
int BaseMove::GetDistance(const VECTOR alpha, const VECTOR beta)
{
	double distance = sqrt((alpha.x - beta.x) * (alpha.x - beta.x) + (alpha.z - beta.z) * (alpha.z - beta.z));
	return (int)distance;
}

BaseMove::BaseMove()
{
	SetLightEnable(TRUE);


	// フォグに関する
	SetFogEnable(TRUE);					// フォグを有効にする
	SetFogColor(128, 128, 128);			// フォグの色にする
	SetFogStartEnd(3500.0f, 6000.0f);	// フォグの開始距離


	backGround = 0;
	backGround = MakeScreen(1920, 1080);


	// シャドウマップハンドルの作成
	shadowMapCharaHandle = MakeShadowMap(2048, 2048);
	shadowMapAnotherCharaHandle = MakeShadowMap(512, 512);
	shadowMapNoMoveHandle = MakeShadowMap(256, 256);


	// シャドウマップに描画する範囲を設定
	shadowCharaLowArea = VGet(-500.0f, -1.0f, -500.0f);
	shadowCharaHighArea = VGet(500.0f, 10.0f, 500.0f);

	shadowAnotherCharaLowArea = VGet(-2000.0f, -1.0f, -2000.0f);
	shadowAnotherCharaHighArea = VGet(2000.0f, 100.0f, 2000.0f);

	shadowNoMoveLowArea = VGet(-2000.0f, -1.0f, -2000.0f);
	shadowNoMoveHighArea = VGet(2000.0f, 1000.0f, 2000.0f);

	SetShadowMapDrawArea(shadowMapCharaHandle, shadowCharaLowArea, shadowCharaHighArea);
	SetShadowMapDrawArea(shadowMapAnotherCharaHandle, shadowAnotherCharaLowArea, shadowAnotherCharaHighArea);
	SetShadowMapDrawArea(shadowMapNoMoveHandle, shadowNoMoveLowArea, shadowNoMoveHighArea);


	lightDire = VGet(-0.5f, -0.7f, 0.5f);
	// ライトの方向を設定
	SetLightDirection(lightDire);
	// シャドウマップが想定するライトの方向もセット
	SetShadowMapLightDirection(shadowMapCharaHandle, lightDire);
	SetShadowMapLightDirection(shadowMapAnotherCharaHandle, lightDire);
	SetShadowMapLightDirection(shadowMapNoMoveHandle, lightDire);
}

BaseMove::~BaseMove()
{
	// シャドウマップの削除
	SHADOW_MAP_RELEASE(shadowMapNoMoveHandle);
	SHADOW_MAP_RELEASE(shadowMapAnotherCharaHandle);
	SHADOW_MAP_RELEASE(shadowMapCharaHandle);
}

const bool BaseMove::GetEndFlag()
{
	return endFlag;
}

const ESceneNumber BaseMove::GetScene()
{
	return scene;
}

void BaseMove::SetScene(const ESceneNumber scene)
{
	this->scene = scene;
}
