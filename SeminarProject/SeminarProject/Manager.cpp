#include "Manager.hpp"


void Manager::SceneChange()
{
	switch (e_nowScene)
	{
	case ESceneNumber::STARTLOAD:
		p_loadThread = new LoadThread();
		break;


	case ESceneNumber::FIRSTMOVE:
		p_baseMove = new MainMove1(p_loadThread->GetFile());
		p_baseMove->SetScene(e_nowScene);
		delete p_loadThread;
		p_loadThread = NULL;
		break;


		
	case ESceneNumber::SECONDLOAD:
		delete p_baseMove;
		p_baseMove = NULL;
		p_loadThread = new LoadThread();
		break;


	case ESceneNumber::SECONDMOVE:
		p_baseMove = new MainMove2(p_loadThread->GetFile());
		p_baseMove->SetScene(e_nowScene);
		delete p_loadThread;
		p_loadThread = NULL;
		break;


	default:
		break;
	}
}

Manager::Manager()
{
	e_preScene = ESceneNumber::STARTLOAD;
	e_nowScene = ESceneNumber::STARTLOAD;

	move1str[0] = "media\\ステージモデル\\move1_graphic.myn";
	move1str[1] = "media\\ステージモデル\\move1_hantei.myn";
	move1str[2] = "media\\CLPH\\motion\\CLPH_motionALL.myn";
	move1str[3] = "media\\剣\\sword.myn";
	load1[0] = ELOADFILE::mv1model;
	load1[1] = ELOADFILE::mv1model;
	load1[2] = ELOADFILE::mv1model;
	load1[3] = ELOADFILE::mv1model;


	move2str[0] = "media\\ステージモデル\\move1_hantei.myn";
	move2str[1] = "media\\CLPH\\motion\\CLPH_Nobirubinta.myn";
	move2str[2] = "media\\CLPH\\motion\\CLPH_motionALL.myn";
	load2[0] = ELOADFILE::mv1model;
	load2[1] = ELOADFILE::mv1model;
	load2[2] = ELOADFILE::mv1model;


	p_baseMove = NULL;
	p_loadThread = NULL;

	p_loadThread = new LoadThread();
}


Manager::~Manager()
{
	if (p_baseMove != NULL)
	{
		delete p_baseMove;
	}
	if (p_loadThread != NULL)
	{
		delete p_loadThread;
	}
}

void Manager::Update(unsigned __int8 controllNumber)
{
	if (e_nowScene == e_preScene)		// 今のシーンと直前のシーンが同じ
	{
		if (e_preScene == ESceneNumber::STARTLOAD)		// ロード中に変わった瞬間なら
		{
			p_loadThread->Process(max1, move1str, load1);		// ロードをする
			if (p_loadThread->GetNum() >= max1)		// ロードが終了したら
			{
				if (KeyData::Get(KEY_INPUT_Z) == 1)			// 終わったら一操作
				{
					e_nowScene = ESceneNumber::FIRSTMOVE;
				}
			}
		}
		else if (e_preScene == ESceneNumber::SECONDLOAD)
		{
			p_loadThread->Process(max2, move2str, load2);		// ロードをする
			if (p_loadThread->GetNum() >= max2)		// ロードが終了したら
			{
				if (KeyData::Get(KEY_INPUT_Z) == 1)			// 終わったら一操作
				{
					e_nowScene = ESceneNumber::SECONDMOVE;
				}
			}
		}
		else
		{
			p_baseMove->Draw();
			p_baseMove->Process(controllNumber);
			e_nowScene = p_baseMove->GetScene();
		}
	}
	else
	{
		SceneChange();
		e_preScene = e_nowScene;
	}

	//printfDx("now:%d\tpre:%d\n", e_nowScene, e_preScene);
}
