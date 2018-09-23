#pragma once
#include "BaseMove.hpp"
#include "MainMove1.hpp"
#include "LoadThread.hpp"
#include "MainMove2.hpp"


class Manager
{
private:
	ESceneNumber e_preScene;		// 直前のシーン
	ESceneNumber e_nowScene;		// 今のシーン


	BaseMove* p_baseMove;			// シーンの基底クラス
	LoadThread* p_loadThread;		// ロードのクラス


	void SceneChange();				// シーンを切り替える


	// ムーブ１に関する
	const int max1 = 5;
	std::string move1str[5];
	ELOADFILE load1[5];



	// ムーブ２に関する
	const int max2 = 6;
	std::string move2str[6];
	ELOADFILE load2[6];



	int antiAliasScreen;



public:
	Manager();			// コンストラクタ
	~Manager();			// デストラクタ

	void Update(const unsigned __int8 controllNumber);		// メインプロセス
};