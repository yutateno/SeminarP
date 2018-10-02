#pragma once

// インクルードファイル
#include "InputKey.hpp"
#include <math.h>



// DEBUGマクロ

// モデルのサークルデバッグ表示
//#define _MODEL_DEBUG

// 直近モデルをサーチした結果を出すサークルデバッグ表示
//#define _SEARCH_MODEL_DEBUG

// エリアのデバッグ用線
//#define _AREA_DEBUG

// キャラクタークラスのデバッグ
//#define _CHARACTER_DEBUG

// ムーブ１クラスのデバッグ
//#define _MOVE1_DEBUG

// ムーブ１で落ちてる剣のクラスのデバッグ
//#define _DROPITEM1_DEBUG

// カメラのデバッグ
#define _CAMERA_DEBG



// マクロ

// directx関係でよくある解放マクロ
#define SAFE_RELEASE(p) { if(p!=NULL) { (p)->Release(); (p)=NULL; } }

// ポインタ用解放マクロ
#define POINTER_RELEASE(p) { if(p!=NULL) {delete (p); (p)=NULL; } }

// 2D関連画像解放マクロ
#define GRAPHIC_RELEASE(p) { if(p!=-1) { DeleteGraph(p); (p)=-1; } }

// 3D関連画像解放マクロ
#define MODEL_RELEASE(p) { if(p!=-1) { MV1DeleteModel(p); (p)=-1; } }

// 音解放マクロ
#define SOUND_RELEASE(p) { if(p!=-1) { StopSoundMem(p); DeleteSoundMem(p); (p)=-1; } }

// 光源ハンドル解放マクロ
#define LIGHT_RELEASE(p) { if(p!=-1) { DeleteLightHandle(p); (p)=-1; } }

// シャドウマップ解放マクロ
#define SHADOW_MAP_RELEASE(p) { if(p!=-1) { DeleteShadowMap(p); (p)=-1; } }
