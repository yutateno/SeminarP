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

HRESULT Init(ID3D11Device* pDevice, ID3D11DeviceContext* pContext, LPSTR FileName)
{

}