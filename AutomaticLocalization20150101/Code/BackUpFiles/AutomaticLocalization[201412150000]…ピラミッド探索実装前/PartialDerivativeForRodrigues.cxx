/***** ��]�s��̕Δ��� *****/
// LevMarOptimization �֐����Ŏg�p


/*** �C���N���[�h�t�@�C�� ***/

/* �ݒ�p�w�b�_�t�@�C�� */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** ���O��Ԃ̐錾 ***/

/* C++ */
using namespace std;


/*** ��]�s��� r1, r2, r3 �����ɂ��Δ��� ***/
// rNum = 0 �̏ꍇ r1 �Ŕ���, rNum = 1 �̏ꍇ r2 �Ŕ���, rNum = 2 �̏ꍇ r3 �Ŕ������ꂽ��]�s�񂪓�����
Eigen::Matrix< double, 3, 3 > PartialDerivativeForRodrigues(
	int rNum,							// �Δ��������]�����ԍ� �� 0�Fr1, 1�Fr2, 2�Fr3
	Eigen::Vector3d* RodriguesVector	// Rodrigues ��p������]�\�� r = [ r1, r2, r3 ]
){

	/*** �ϐ��̐錾�Ə����� ***/

	/* Rodrigues �ϊ����3�~1��]�x�N�g���ɂ��\�������3�~3�s�� [r]x */
	Eigen::Matrix< double, 3, 3 > RodriguesMatrix;
	RodriguesMatrix( 0, 0 ) = 0; RodriguesMatrix( 0, 1 ) = - ( *RodriguesVector )( 2 ); RodriguesMatrix( 0, 2 ) = ( *RodriguesVector )( 1 );
	RodriguesMatrix( 1, 0 ) = ( *RodriguesVector )( 2 ); RodriguesMatrix( 1, 1 ) = 0; RodriguesMatrix( 1, 2 ) = - ( *RodriguesVector )( 0 );
	RodriguesMatrix( 2, 0 ) = - ( *RodriguesVector )( 1 ); RodriguesMatrix( 2, 1 ) = ( *RodriguesVector )( 0 ); RodriguesMatrix( 2, 2 ) = 0;

	/* Rodrigues �ϊ����3�~1��]�x�N�g���ɂ��\�������3�~3�s�� [r]x �� r1, r2, r3 �ŕΔ�������3�~3�s�� */
	Eigen::Matrix< double, 3, 3 > M[ 3 ];

	/* r1 �Ŕ��������ꍇ�� [r]x */
	M[ 0 ]( 0, 0 ) = 0; M[ 0 ]( 0, 1 ) = 0; M[ 0 ]( 0, 2 ) = 0;
	M[ 0 ]( 1, 0 ) = 0; M[ 0 ]( 1, 1 ) = 0; M[ 0 ]( 1, 2 ) = -1;
	M[ 0 ]( 2, 0 ) = 0; M[ 0 ]( 2, 1 ) = 1; M[ 0 ]( 2, 2 ) = 0;
	
	/* r2 �Ŕ��������ꍇ�� [r]x */
	M[ 1 ]( 0, 0 ) = 0; M[ 1 ]( 0, 1 ) = 0; M[ 1 ]( 0, 2 ) = 1;
	M[ 1 ]( 1, 0 ) = 0; M[ 1 ]( 1, 1 ) = 0; M[ 1 ]( 1, 2 ) = 0;
	M[ 1 ]( 2, 0 ) = -1; M[ 1 ]( 2, 1 ) = 0; M[ 1 ]( 2, 2 ) = 0;
	
	/* r3 �Ŕ��������ꍇ�� [r]x */
	M[ 2 ]( 0, 0 ) = 0; M[ 2 ]( 0, 1 ) = -1; M[ 2 ]( 0, 2 ) = 0;
	M[ 2 ]( 1, 0 ) = 1; M[ 2 ]( 1, 1 ) = 0; M[ 2 ]( 1, 2 ) = 0;
	M[ 2 ]( 2, 0 ) = 0; M[ 2 ]( 2, 1 ) = 0; M[ 2 ]( 2, 2 ) = 0;
	
	/* ��]�ʃ� */
	double theta = ( *RodriguesVector ).norm();	// �� = || r ||


	/*** �Δ������ꂽ��]�s�� ***/
	
	/* �������ꂽ��]�s��̊e�� */
	Eigen::Matrix< double, 3, 3 > term1 = ( ( *RodriguesVector )( rNum ) * cos( theta ) / pow( theta, 2 ) ) * RodriguesMatrix;
	Eigen::Matrix< double, 3, 3 > term2 = ( ( *RodriguesVector )( rNum ) * sin( theta ) / pow( theta, 3 ) ) * RodriguesMatrix;
	Eigen::Matrix< double, 3, 3 > term3 = ( sin( theta ) / theta ) * M[ rNum ];
	Eigen::Matrix< double, 3, 3 > term4 = ( ( *RodriguesVector )( rNum ) * sin( theta ) / pow( theta, 3 ) ) * RodriguesMatrix * RodriguesMatrix;
	Eigen::Matrix< double, 3, 3 > term5 = ( ( 2 * ( *RodriguesVector )( rNum ) ) * ( 1 - cos( theta ) ) / pow( theta, 4 ) ) * RodriguesMatrix * RodriguesMatrix;
	Eigen::Matrix< double, 3, 3 > term6 = ( ( 1 - cos( theta ) ) / pow( theta, 2 ) ) * ( ( M[ rNum ] * RodriguesMatrix ) + ( RodriguesMatrix * M[ rNum ] ) );

	/* �������ʂƂȂ��]�s�� : rNum = 0 �̏ꍇ r1 �Ŕ���, rNum = 1 �̏ꍇ r2 �Ŕ���, rNum = 2 �̏ꍇ r3 �Ŕ������ꂽ��]�s�񂪓����� */
	Eigen::Matrix< double, 3, 3 > dR = term1 - term2 + term3 + term4 - term5 + term6;


	/* �Ԃ�l */
	return dR;	// �������ʂƂȂ��]�s��
}