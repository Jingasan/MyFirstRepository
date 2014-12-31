/***** Rodrigues �̉�]�x�N�g������̉�]�s��̕��� *****/
// LevMarOptimization �֐����Ŏg�p


/*** �C���N���[�h�t�@�C�� ***/

/* �ݒ�p�w�b�_�t�@�C�� */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** ���O��Ԃ̐錾 ***/

/* C++ */
using namespace std;


/*** Rodrigues �̉�]�x�N�g������̉�]�s��̕��� ***/
Eigen::Matrix< double, 3, 3 > ReconstructRFromRodrigues(
	Eigen::Vector3d* RodriguesVector	// Rodrigues ��p������]�\�� r = [ r1, r2, r3 ]
){

	/*** �ϐ��̐錾�Ə����� ***/

	/* Rodrigues �ϊ����3�~1��]�x�N�g���ɂ��\�������3�~3�s�� [r]x */
	Eigen::Matrix< double, 3, 3 > RodriguesMatrix;
	RodriguesMatrix( 0, 0 ) = 0; RodriguesMatrix( 0, 1 ) = - ( *RodriguesVector )( 2 ); RodriguesMatrix( 0, 2 ) = ( *RodriguesVector )( 1 );
	RodriguesMatrix( 1, 0 ) = ( *RodriguesVector )( 2 ); RodriguesMatrix( 1, 1 ) = 0; RodriguesMatrix( 1, 2 ) = - ( *RodriguesVector )( 0 );
	RodriguesMatrix( 2, 0 ) = - ( *RodriguesVector )( 1 ); RodriguesMatrix( 2, 1 ) = ( *RodriguesVector )( 0 ); RodriguesMatrix( 2, 2 ) = 0;
	
	/* ��]�ʃ� */
	double theta = ( *RodriguesVector ).norm();	// �� = || r ||

	/* 3�~3�̒P�ʍs��̍쐬 */
	Eigen::Matrix< double, 3, 3 > I;
	I = Eigen::MatrixXd::Identity( 3, 3 );

	/* ��]�s�� R �̕��� */
	Eigen::Matrix< double, 3, 3 > R;
	R = I + ( sin( theta ) / theta ) * RodriguesMatrix + ( ( 1 - cos( theta ) ) / pow( theta, 2 ) ) * RodriguesMatrix * RodriguesMatrix;

	/* �Ԃ�l */
	return R;	// ��]�s��
}