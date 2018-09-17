
// モデルのサークルデバッグ表示
//#define _MODEL_DEBUG

// 直近モデルをサーチした結果を出すサークルデバッグ表示
//#define _SEARCH_MODEL_DEBUG

// エリアのデバッグ用線
//#define _AREA_DEBUG

// キャラクタークラスのデバッグ
//#define _CHARACTER_DEBUG

// ムーブ１クラスのデバッグ
#define _MOVE1_DEBUG

// ムーブ１で落ちてる剣のクラスのデバッグ
//#define _DROPITEM1_DEBUG

// directx関係でよくある解放マクロ
#define SAFE_RELEASE(p) { if(p!=NULL) { (p)->Release(); (p)=NULL; } }

// ポインタ用解放マクロ
#define POINTER_RELEASE(p) { if(p!=NULL) {delete (p); (p)=NULL; } }
