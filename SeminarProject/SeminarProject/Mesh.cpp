#include "Mesh.h"

Mesh::Mesh()
{
	ZeroMemory(this, sizeof(Mesh));
	m_fScale = 1.0f;
}

Mesh::~Mesh()
{
	SAFE_DELETE_ARRAY(m_pMaterial);
	SAFE_DELETE_ARRAY(m_ppIndexBuffer);
	SAFE_RELEASE(m_pConstantBuffer0);
	SAFE_RELEASE(m_pConstantBuffer1);
	SAFE_RELEASE(m_pVertexShader);
	SAFE_RELEASE(m_pPixelShader);
	SAFE_RELEASE(m_pVertexBuffer);
	SAFE_RELEASE(m_pVertexLayout);
	SAFE_RELEASE(m_pSampleLinear);
	SAFE_RELEASE(m_pTexture);
}

HRESULT Mesh::Init(ID3D11Device* pDevice, ID3D11DeviceContext* pContext, LPSTR FileName)
{
	m_pDevice = pDevice;
	m_pDeviceContext = pContext;

	if (FAILED(InitShader()))
	{
		MessageBox(0, L"メッシュ用シェーダ作成失敗", NULL, MB_OK);
		return E_FAIL;
	}
	/*if (FAILED(LoadStaticMesh(FileName)))
	{
		MessageBox(0, L"メッシュ作成失敗", NULL, MB_OK);
		return E_FAIL;
	}*/

	return S_OK;
}

HRESULT Mesh::InitShader()
{
	// ブロブ作成(シェーダの塊)
	ID3D10Blob* pCompileShader = NULL;
	ID3D10Blob* pErrors = NULL;

	// ブロブから頂点シェーダ作成
	// ファイル読み込み
	if (FAILED(D3DX11CompileFromFile(L"Simple.hlsl", NULL, NULL, "VS", "vs_5_0", 0, 0, NULL, &pCompileShader, &pErrors, NULL)))
	{
		MessageBox(0, L"hlsl読み込み失敗(VS)", NULL, MB_OK);
		return E_FAIL;
	}
	SAFE_RELEASE(pErrors);
	// 頂点シェーダ作成
	if (FAILED(m_pDevice->CreateVertexShader(pCompileShader->GetBufferPointer(), pCompileShader->GetBufferSize(), NULL, &m_pVertexShader)))
	{
		SAFE_RELEASE(pCompileShader);
		MessageBox(0, L"頂点シェーダ作成失敗", NULL, MB_OK);
		return E_FAIL;
	}
	// 頂点インプットレイアウトを定義
	D3D11_INPUT_ELEMENT_DESC layout[] =
	{
		{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
		{"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0},
		{"TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 24, D3D11_INPUT_PER_VERTEX_DATA, 0},
	};
	UINT numElements = sizeof(layout) / sizeof(layout[0]);
	//頂点インプットを作成
	if (FAILED(m_pDevice->CreateInputLayout(layout, numElements, pCompileShader->GetBufferPointer(), pCompileShader->GetBufferSize(), &m_pVertexLayout)))
	{
		return FALSE;
	}

	// ブロブからピクセルシェーダ作成
	// ファイル読み込み
	if (FAILED(D3DX11CompileFromFile(L"Simple.hlsl", NULL, NULL, "PS", "ps_5_0", 0, 0, NULL, &pCompileShader, &pErrors, NULL)))
	{
		SAFE_RELEASE(pCompileShader);
		MessageBox(0, L"ピクセルシェーダ作成失敗", NULL, MB_OK);
		return E_FAIL;
	}
	SAFE_RELEASE(pErrors);
	// ピクセルシェーダ作成
	if (FAILED(m_pDevice->CreatePixelShader(pCompileShader->GetBufferPointer(), pCompileShader->GetBufferSize(), NULL, &m_pPixelShader)))
	{
		SAFE_RELEASE(pCompileShader);
		MessageBox(0, L"ピクセルシェーダ作成", NULL, MB_OK);
		return E_FAIL;
	}
	SAFE_RELEASE(pCompileShader);
	// コンスタントバッファー作成(変換行列用)
	D3D11_BUFFER_DESC cb;
	cb.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
	cb.ByteWidth = sizeof(SIMPLECONSTANT_BUFFER0);
	cb.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
	cb.MiscFlags = 0;
	cb.Usage = D3D11_USAGE_DYNAMIC;
	if (FAILED(m_pDevice->CreateBuffer(&cb, NULL, &m_pConstantBuffer0)))
	{
		return E_FAIL;
	}
	// コンスタントバッファー作成(マテリアル用)
	cb.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
	cb.ByteWidth = sizeof(SIMPLECONSTANT_BUFFER1);
	cb.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
	cb.MiscFlags = 0;
	cb.Usage = D3D11_USAGE_DYNAMIC;
	if (FAILED(m_pDevice->CreateBuffer(&cb, NULL, &m_pConstantBuffer1)))
	{
		return E_FAIL;
	}

	return S_OK;
}