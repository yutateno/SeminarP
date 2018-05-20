//�w�b�_�[�t�@�C���̃C���N���[�h
#include <windows.h>
#include <d3d11.h>
#include <d3dx10.h>
#include <d3dx11.h>
#include <d3dCompiler.h>
//�K�v�ȃ��C�u�����t�@�C���̃��[�h
#pragma comment(lib,"winmm.lib")
#pragma comment(lib,"d3dx10.lib")
#pragma comment(lib,"d3d11.lib")
#pragma comment(lib,"d3dx11.lib")
#pragma comment(lib,"d3dCompiler.lib")
//�x����\��
#pragma warning(disable : 4305)
//�萔��`
#define WINDOW_WIDTH 640 //�E�B���h�E��
#define WINDOW_HEIGHT 480 //�E�B���h�E����
#define APP_NAME L"DirectX11"
//�}�N��
#define SAFE_RELEASE(x) if(x){x->Release(); x=NULL;}

//���_�̍\����
struct SimpleVertex
{
	D3DXVECTOR3 Pos; //�ʒu
};
//Simple�V�F�[�_�[�p�̃R���X�^���g�o�b�t�@�[�̃A�v�����\���� �������V�F�[�_�[���̃R���X�^���g�o�b�t�@�[�ƈ�v���Ă���K�v����
struct SIMPLESHADER_CONSTANT_BUFFER
{
	D3DXMATRIX mWVP;
};
//
//class MAIN
//MAIN�N���X
class MAIN
{
public:
	MAIN();
	~MAIN();
	HRESULT InitWindow(HINSTANCE, INT, INT, INT, INT, LPCWSTR);
	LRESULT MsgProc(HWND, UINT, WPARAM, LPARAM);
	HRESULT InitD3D();
	HRESULT InitModel();
	HRESULT MakeShader(LPSTR, LPSTR, LPSTR, void**, ID3DBlob**);
	void Loop();
	void App();
	void Render();
	void DestroyD3D();

	HWND m_hWnd;
	ID3D11Device* m_pDevice;
	ID3D11DeviceContext* m_pDeviceContext;
	IDXGISwapChain* m_pSwapChain;
	ID3D11RenderTargetView* m_pBackBuffer_TexRTV;
	ID3D11DepthStencilView* m_pBuckBuffer_DSTexDSV;
	ID3D11Texture2D* m_pBuckBuffer_DSTex;
	ID3D11InputLayout* m_pVertexLayout;
	ID3D11VertexShader* m_pVertexShader;
	ID3D11PixelShader* m_pPixelShader;
	ID3D11Buffer* m_pConstantBuffer;
	ID3D11Buffer* m_pVertexBuffer;
};
