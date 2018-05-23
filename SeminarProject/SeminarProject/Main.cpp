#include "Main.h"

Main* g_pMain = NULL;

// 関数プロトタイプの宣言
LRESULT CALLBACK WndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

// アプリケーション
INT WINAPI WinMain(HINSTANCE hInstance, HINSTANCE, LPSTR, INT)
{
	g_pMain = new Main;
	if (g_pMain != NULL)
	{
		if (SUCCEEDED(g_pMain->InitWindow(hInstance, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, APP_NAME)))
		{
			if (SUCCEEDED(g_pMain->InitD3D()))
			{
				// 内容
				g_pMain->Loop();
			}
		}
		// アプリ終了
		g_pMain->DestroyD3D();
		delete g_pMain;
	}
	return 0;
}

// ウィンドウプロシージャ―
LRESULT CALLBACK WndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	return g_pMain->MsgProc(hWnd, uMsg, wParam, lParam);
}

// ウィンドウ作成
HRESULT Main::InitWindow(HINSTANCE hInstance, INT iX, INT iY, INT iWidth, INT iHeight, LPCWSTR WindowName)
{
	// ウィンドウ定義
	WNDCLASSEX wc;
	ZeroMemory(&wc, sizeof(wc));
	wc.cbSize = sizeof(wc);
	wc.style = CS_HREDRAW | CS_VREDRAW;
	wc.lpfnWndProc = WndProc;
	wc.hInstance = hInstance;
	wc.hIcon = LoadIcon(NULL, IDI_APPLICATION);
	wc.hCursor = LoadCursor(NULL, IDC_ARROW);
	wc.hbrBackground = (HBRUSH)GetStockObject(LTGRAY_BRUSH);
	wc.lpszClassName = WindowName;
	wc.hIconSm = LoadIcon(NULL, IDI_APPLICATION);
	RegisterClassEx(&wc);

	// ウィンドウ作成
	m_hWnd = CreateWindow(WindowName, WindowName, WS_OVERLAPPEDWINDOW, 0, 0, iWidth, iHeight, 0, 0, hInstance, 0);
	if (!m_hWnd)
	{
		return E_FAIL;
	}
	// ウィンドウの表示
	ShowWindow(m_hWnd, SW_SHOW);
	UpdateWindow(m_hWnd);

	return S_OK;
}

// ウィンドウプロシージャ―のメイン
LRESULT Main::MsgProc(HWND hWnd, UINT iMsg, WPARAM wParam, LPARAM lParam)
{
	switch (iMsg)
	{
	case WM_KEYDOWN:
		switch ((char)wParam)
		{
		case VK_ESCAPE:
			PostQuitMessage(0);
			break;
		}
		break;
	case WM_DESTROY:
		PostQuitMessage(0);
		break;
	}
	return DefWindowProc(hWnd, iMsg, wParam, lParam);
}

void Main::Loop()
{
	// メッシュ作成
	m_pMesh = new Mesh;
	if (FAILED(m_pMesh->Init(m_pDevice, m_pDeviceContext, "a_few_box.obj")))
	{
		return;
	}

	// メッセージループ
	MSG msg = { 0 };
	ZeroMemory(&msg, sizeof(msg));
	while (msg.message != WM_QUIT)
	{
		if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		else
		{
			// アプリケーションの処理
			App();
		}
	}
}

// アプリケーションの処理
void Main::App()
{
	Render();
}

HRESULT Main::InitD3D()
{
	// デバイスとスワップチェーンの作成
	DXGI_SWAP_CHAIN_DESC sd;
	ZeroMemory(&sd, sizeof(sd));
	sd.BufferCount = 1;
	sd.BufferDesc.Width = WINDOW_WIDTH;
	sd.BufferDesc.Height = WINDOW_HEIGHT;
	sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	sd.BufferDesc.RefreshRate.Numerator = 60;
	sd.BufferDesc.RefreshRate.Denominator = 1;
	sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
	sd.OutputWindow = m_hWnd;
	sd.SampleDesc.Count = 1;
	sd.SampleDesc.Quality = 0;
	sd.Windowed = TRUE;

	D3D_FEATURE_LEVEL pFeatureLevels = D3D_FEATURE_LEVEL_11_0;
	D3D_FEATURE_LEVEL* pFeatureLevel = NULL;

	if (FAILED(D3D11CreateDeviceAndSwapChain(NULL, D3D_DRIVER_TYPE_HARDWARE, NULL, 0, &pFeatureLevels, 1, D3D11_SDK_VERSION, &sd, &m_pSwapChain, &m_pDevice, pFeatureLevel, &m_pDeviceContext)))
	{
		return FALSE;
	}

	// レンダ―ターゲットビューの作成
	ID3D11Texture2D* pBackBuffer;
	m_pSwapChain->GetBuffer(0, __uuidof(ID3D11Texture2D), (LPVOID*)&pBackBuffer);
	m_pDevice->CreateRenderTargetView(pBackBuffer, NULL, &m_pRTV);
	SAFE_RELEASE(pBackBuffer);

	// 深度ステンシルビューの作成
	D3D11_TEXTURE2D_DESC descDepth;
	descDepth.Width = WINDOW_WIDTH;
	descDepth.Height = WINDOW_HEIGHT;
	descDepth.MipLevels = 1;
	descDepth.ArraySize = 1;
	descDepth.Format = DXGI_FORMAT_D32_FLOAT;
	descDepth.SampleDesc.Count = 1;
	descDepth.SampleDesc.Quality = 0;
	descDepth.Usage = D3D11_USAGE_DEFAULT;
	descDepth.BindFlags = D3D11_BIND_DEPTH_STENCIL;
	descDepth.CPUAccessFlags = 0;
	descDepth.MiscFlags = 0;
	m_pDevice->CreateTexture2D(&descDepth, NULL, &m_pDS);
	m_pDevice->CreateDepthStencilView(m_pDS, NULL, &m_pDSV);

	// レンダ―ターゲットビューと深度ステンシルビューをパイプラインにバインド
	m_pDeviceContext->OMSetRenderTargets(1, &m_pRTV, m_pDSV);

	// ビューポートの設定
	D3D11_VIEWPORT vp;
	vp.Width = WINDOW_WIDTH;
	vp.Height = WINDOW_HEIGHT;
	vp.MinDepth = 0.0f;
	vp.MaxDepth = 1.0f;
	vp.TopLeftX = 0;
	vp.TopLeftY = 0;
	m_pDeviceContext->RSSetViewports(1, &vp);

	// ラスタライザの設定
	D3D11_RASTERIZER_DESC rdc;
	ZeroMemory(&rdc, sizeof(rdc));
	rdc.CullMode = D3D11_CULL_NONE;
	rdc.FillMode = D3D11_FILL_SOLID;
	ID3D11RasterizerState* pIr = NULL;
	m_pDevice->CreateRasterizerState(&rdc, &pIr);
	m_pDeviceContext->RSSetState(pIr);

	return S_OK;
}

void Main::DestroyD3D()
{
	SAFE_DELETE(m_pMesh);
	SAFE_RELEASE(m_pSwapChain);
	SAFE_RELEASE(m_pRTV);
	SAFE_RELEASE(m_pDSV);
	SAFE_RELEASE(m_pDS);
	SAFE_RELEASE(m_pDevice);
}

// シーンを画面にレンダリングする
void Main::Render()
{
	D3DXMATRIX mView;
	D3DXMATRIX mProj;
	// 画面クリア
	float ClearColor[4] = { 0,0,1,1 };
	m_pDeviceContext->ClearRenderTargetView(m_pRTV, ClearColor);	// 画面クリア
	m_pDeviceContext->ClearDepthStencilView(m_pDSV, D3D11_CLEAR_DEPTH, 1.0f, 0);	// 深度バッファクリア
	// 視点座標変換
	D3DXVECTOR3 vEyePt(0.0f, 1.0f, -3.5f);	// カメラ位置
	D3DXVECTOR3 vLookAtPt(0.0f, 1.0f, 0.0f);	// 注視位置
	D3DXVECTOR3 vUpVec(0.0f, 1.0f, 0.0f);	// 上方位置
	D3DXMatrixLookAtLH(&mView, &vEyePt, &vLookAtPt, &vUpVec);
	// 射影変換
	D3DXMatrixPerspectiveFovLH(&mProj, D3DX_PI / 4, (FLOAT)WINDOW_WIDTH / (FLOAT)WINDOW_HEIGHT, 0.1f, 100.0f);
	// レンダリング
	D3DXVECTOR3 vmyLight(1.0f, 1.0f, 1.0f);
	D3DXVECTOR3 vmyEye(0.0f, 0.0f, -1.0f);
	m_pMesh->Render(mView, mProj, vmyLight, vmyEye);
	m_pMesh->m_fYaw += 0.001;
	// 画面更新
	m_pSwapChain->Present(0, 0);
	// FPS計算表示
	static DWORD time = 0;
	static int frame = 0;
	frame++;
	char str[50];
	sprintf(str, "fps=%d", frame);
	if (timeGetTime() - time > 1000)
	{
		time = timeGetTime();
		frame = 0;
		SetWindowTextA(m_hWnd, str);
	}
}