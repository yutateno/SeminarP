#include "Main.h"

Main* g_pMain = NULL;

// �֐��v���g�^�C�v�̐錾
LRESULT CALLBACK WndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

// �A�v���P�[�V����
INT WINAPI WinMain(HINSTANCE hInstance, HINSTANCE, LPSTR, INT)
{
	g_pMain = new Main;
	if (g_pMain != NULL)
	{
		if (SUCCEEDED(g_pMain->InitWindow(hInstance, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, APP_NAME)))
		{
			if (SUCCEEDED(g_pMain->InitD3D()))
			{
				// ���e
				g_pMain->Loop();
			}
		}
		// �A�v���I��
		g_pMain->DestroyD3D();
		delete g_pMain;
	}
	return 0;
}

// �E�B���h�E�v���V�[�W���\
LRESULT CALLBACK WndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	return g_pMain->MsgProc(hWnd, uMsg, wParam, lParam);
}

// �E�B���h�E�쐬
HRESULT Main::InitWindow(HINSTANCE hInstance, INT iX, INT iY, INT iWidth, INT iHeight, LPCWSTR WindowName)
{
	// �E�B���h�E��`
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

	// �E�B���h�E�쐬
	m_hWnd = CreateWindow(WindowName, WindowName, WS_OVERLAPPEDWINDOW, 0, 0, iWidth, iHeight, 0, 0, hInstance, 0);
	if (!m_hWnd)
	{
		return E_FAIL;
	}
	// �E�B���h�E�̕\��
	ShowWindow(m_hWnd, SW_SHOW);
	UpdateWindow(m_hWnd);

	return S_OK;
}

// �E�B���h�E�v���V�[�W���\�̃��C��
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
	// ���b�V���쐬
	m_pMesh = new Mesh;
	if (FAILED(m_pMesh->Init(m_pDevice, m_pDeviceContext, "a_few_box.obj")))
	{
		return;
	}

	// ���b�Z�[�W���[�v
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
			// �A�v���P�[�V�����̏���
			App();
		}
	}
}

// �A�v���P�[�V�����̏���
void Main::App()
{
	Render();
}

HRESULT Main::InitD3D()
{
	// �f�o�C�X�ƃX���b�v�`�F�[���̍쐬
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

	// �����_�\�^�[�Q�b�g�r���[�̍쐬
	ID3D11Texture2D* pBackBuffer;
	m_pSwapChain->GetBuffer(0, __uuidof(ID3D11Texture2D), (LPVOID*)&pBackBuffer);
	m_pDevice->CreateRenderTargetView(pBackBuffer, NULL, &m_pRTV);
	SAFE_RELEASE(pBackBuffer);

	// �[�x�X�e���V���r���[�̍쐬
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

	// �����_�\�^�[�Q�b�g�r���[�Ɛ[�x�X�e���V���r���[���p�C�v���C���Ƀo�C���h
	m_pDeviceContext->OMSetRenderTargets(1, &m_pRTV, m_pDSV);

	// �r���[�|�[�g�̐ݒ�
	D3D11_VIEWPORT vp;
	vp.Width = WINDOW_WIDTH;
	vp.Height = WINDOW_HEIGHT;
	vp.MinDepth = 0.0f;
	vp.MaxDepth = 1.0f;
	vp.TopLeftX = 0;
	vp.TopLeftY = 0;
	m_pDeviceContext->RSSetViewports(1, &vp);

	// ���X�^���C�U�̐ݒ�
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

// �V�[������ʂɃ����_�����O����
void Main::Render()
{
	D3DXMATRIX mView;
	D3DXMATRIX mProj;
	// ��ʃN���A
	float ClearColor[4] = { 0,0,1,1 };
	m_pDeviceContext->ClearRenderTargetView(m_pRTV, ClearColor);	// ��ʃN���A
	m_pDeviceContext->ClearDepthStencilView(m_pDSV, D3D11_CLEAR_DEPTH, 1.0f, 0);	// �[�x�o�b�t�@�N���A
	// ���_���W�ϊ�
	D3DXVECTOR3 vEyePt(0.0f, 1.0f, -3.5f);	// �J�����ʒu
	D3DXVECTOR3 vLookAtPt(0.0f, 1.0f, 0.0f);	// �����ʒu
	D3DXVECTOR3 vUpVec(0.0f, 1.0f, 0.0f);	// ����ʒu
	D3DXMatrixLookAtLH(&mView, &vEyePt, &vLookAtPt, &vUpVec);
	// �ˉe�ϊ�
	D3DXMatrixPerspectiveFovLH(&mProj, D3DX_PI / 4, (FLOAT)WINDOW_WIDTH / (FLOAT)WINDOW_HEIGHT, 0.1f, 100.0f);
	// �����_�����O
	D3DXVECTOR3 vmyLight(1.0f, 1.0f, 1.0f);
	D3DXVECTOR3 vmyEye(0.0f, 0.0f, -1.0f);
	m_pMesh->Render(mView, mProj, vmyLight, vmyEye);
	m_pMesh->m_fYaw += 0.001;
	// ��ʍX�V
	m_pSwapChain->Present(0, 0);
	// FPS�v�Z�\��
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