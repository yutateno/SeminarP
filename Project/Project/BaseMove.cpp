#include "BaseMove.hpp"


bool BaseMove::endFlag;		// 終了フラッグ
ESceneNumber BaseMove::scene;	// 現在のシーン


// ---------------------------------------------------------------------
void BaseMove::ShadowCharaSetUpBefore()
{
	// シャドウマップへの描画の準備
	ShadowMap_DrawSetup(shadowMapCharaHandle);
}

// ---------------------------------------------------------------------
void BaseMove::ShadowCharaSetUpAfter()
{
	// シャドウマップへの描画を終了
	ShadowMap_DrawEnd();
}

// ---------------------------------------------------------------------
void BaseMove::ShadowAnotherCharaSetUpBefore()
{
	// シャドウマップへの描画の準備
	ShadowMap_DrawSetup(shadowMapAnotherCharaHandle);
}

// ---------------------------------------------------------------------
void BaseMove::ShadowAnotherCharaSetUpAfter()
{
	// シャドウマップへの描画を終了
	ShadowMap_DrawEnd();
}

// ---------------------------------------------------------------------
void BaseMove::ShadowNoMoveSetUpBefore()
{
	// シャドウマップへの描画の準備
	ShadowMap_DrawSetup(shadowMapNoMoveHandle);
}

// ---------------------------------------------------------------------
void BaseMove::ShadowNoMoveSetUpAfter()
{
	// シャドウマップへの描画を終了
	ShadowMap_DrawEnd();
}


// ---------------------------------------------------------------------
void BaseMove::ShadowCharaDrawBefore()
{
	// 描画に使用するシャドウマップを設定
	SetUseShadowMap(0, shadowMapCharaHandle);
}

// ---------------------------------------------------------------------
void BaseMove::ShadowCharaDrawAfter()
{
	// 描画に使用するシャドウマップの設定を解除
	SetUseShadowMap(0, -1);
}

// ---------------------------------------------------------------------
void BaseMove::ShadowAnotherCharaDrawBefore()
{
	// 描画に使用するシャドウマップを設定
	SetUseShadowMap(1, shadowMapAnotherCharaHandle);
}

// ---------------------------------------------------------------------
void BaseMove::ShadowAnotherCharaDrawAfter()
{
	// 描画に使用するシャドウマップの設定を解除
	SetUseShadowMap(1, -1);
}

// ---------------------------------------------------------------------
void BaseMove::ShadowNoMoveDrawBefore()
{
	// 描画に使用するシャドウマップを設定
	SetUseShadowMap(2, shadowMapNoMoveHandle);
}

// ---------------------------------------------------------------------
void BaseMove::ShadowNoMoveDrawAfter()
{
	// 描画に使用するシャドウマップの設定を解除
	SetUseShadowMap(2, -1);
}

// ---------------------------------------------------------------------
void BaseMove::ShadowArea(const VECTOR charaArea)
{
	// シャドウマップの範囲を更新
	SetShadowMapDrawArea(shadowMapCharaHandle, VAdd(charaArea, shadowCharaLowArea), VAdd(charaArea, shadowCharaHighArea));
	SetShadowMapDrawArea(shadowMapAnotherCharaHandle, VAdd(charaArea, shadowAnotherCharaLowArea), VAdd(charaArea, shadowAnotherCharaHighArea));
}


// ---------------------------------------------------------------------
int BaseMove::GetDistance(const VECTOR alpha, const VECTOR beta)
{
	return static_cast<int>(sqrt((alpha.x - beta.x) * (alpha.x - beta.x) + (alpha.z - beta.z) * (alpha.z - beta.z)));
}


// ---------------------------------------------------------------------
void BaseMove::SkyBoxDraw()
{
	// ライティングを無効にする
	SetUseLighting(FALSE);
	// Ｚバッファを有効にする
	SetUseZBuffer3D(TRUE);
	MV1DrawModel(skyBoxUp);
	MV1DrawModel(skyBoxUnder);
	// ライティングを有効にする
	SetUseLighting(TRUE);
	// Ｚバッファを無効にする
	SetUseZBuffer3D(FALSE);
}


// ---------------------------------------------------------------------
void BaseMove::SkyBoxProcess(const VECTOR characterArea)
{
	MV1SetPosition(skyBoxUp, characterArea);
	MV1SetPosition(skyBoxUnder, characterArea);
}

// ---------------------------------------------------------------------
void BaseMove::SetInitSkyBox(const int skyBoxUp)
{
	this->skyBoxUp = MV1DuplicateModel(skyBoxUp);
	MV1SetScale(this->skyBoxUp, VGet(170.0f, 170.0f, 170.0f));
	this->skyBoxUnder = MV1DuplicateModel(this->skyBoxUp);
	MV1SetScale(this->skyBoxUnder, VGet(170.0f, 170.0f, 170.0f));
	MV1SetRotationXYZ(this->skyBoxUnder, VGet(DX_PI_F, 0.0f, 0.0f));
}

// ---------------------------------------------------------------------
BaseMove::BaseMove()
{
	SetLightEnable(TRUE);


	// フォグに関する-------------------------
	/// フォグを有効にする
	SetFogEnable(TRUE);				
	/// フォグの色にする
	SetFogColor(128, 128, 128);		
	/// フォグの開始距離
	SetFogStartEnd(3500.0f, 6000.0f);	
	// ---------------------------------------


	// シャドウマップハンドルの作成----------------------------
	shadowMapCharaHandle = MakeShadowMap(2048, 2048);
	shadowMapAnotherCharaHandle = MakeShadowMap(512, 512);
	shadowMapNoMoveHandle = MakeShadowMap(256, 256);
	// --------------------------------------------------------


	// シャドウマップに描画する範囲を設定---------------------------
	shadowCharaLowArea = VGet(-500.0f, -1.0f, -500.0f);
	shadowCharaHighArea = VGet(500.0f, 10.0f, 500.0f);

	shadowAnotherCharaLowArea = VGet(-2000.0f, -1.0f, -2000.0f);
	shadowAnotherCharaHighArea = VGet(2000.0f, 100.0f, 2000.0f);

	shadowNoMoveLowArea = VGet(-2000.0f, -1.0f, -2000.0f);
	shadowNoMoveHighArea = VGet(2000.0f, 1000.0f, 2000.0f);

	SetShadowMapDrawArea(shadowMapCharaHandle, shadowCharaLowArea, shadowCharaHighArea);
	SetShadowMapDrawArea(shadowMapAnotherCharaHandle, shadowAnotherCharaLowArea, shadowAnotherCharaHighArea);
	SetShadowMapDrawArea(shadowMapNoMoveHandle, shadowNoMoveLowArea, shadowNoMoveHighArea);
	// -------------------------------------------------------------


	// ライトの方向を設定---------------------------
	lightDire = VGet(-0.5f, -0.7f, 0.5f);
	SetLightDirection(lightDire);
	// ---------------------------------------------


	// シャドウマップが想定するライトの方向もセット---------------------
	SetShadowMapLightDirection(shadowMapCharaHandle, lightDire);
	SetShadowMapLightDirection(shadowMapAnotherCharaHandle, lightDire);
	SetShadowMapLightDirection(shadowMapNoMoveHandle, lightDire);
	// -----------------------------------------------------------------


	// スカイボックスに関して-----------------
	skyBoxUnder = -1;
	skyBoxUp = -1;
	// ---------------------------------------
}


// ---------------------------------------------------------------------
BaseMove::~BaseMove()
{
	// シャドウマップの削除----------------------------
	SHADOW_MAP_RELEASE(shadowMapNoMoveHandle);
	SHADOW_MAP_RELEASE(shadowMapAnotherCharaHandle);
	SHADOW_MAP_RELEASE(shadowMapCharaHandle);

	// スカイボックスの削除----------
	MODEL_RELEASE(skyBoxUnder);
	MODEL_RELEASE(skyBoxUp);
}

// ---------------------------------------------------------------------
const bool BaseMove::GetEndFlag()
{
	return endFlag;
}

// ---------------------------------------------------------------------
const ESceneNumber BaseMove::GetScene()
{
	return scene;
}

// ---------------------------------------------------------------------
void BaseMove::SetScene(const ESceneNumber scene)
{
	this->scene = scene;
}
