/***** Rodrigues ��p������]�\�� r = [ r1, r2, r3 ] �ւ̕ϊ� ( LevMarOptimization �֐����Ŏg�p ) *****/
// LevMarOptimization �֐����Ŏg�p


/*** �C���N���[�h�t�@�C�� ***/

/* �ݒ�p�w�b�_�t�@�C�� */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** ���O��Ԃ̐錾 ***/

/* C++ */
using namespace std;


/*** �ʏ�̉�]�s�� R ���� Rodrigues �̉�]�\�� r = [ r1, r2, r3 ] �ւ̕ϊ� ***/
int ConvertRodrigues(
	Eigen::Matrix< double, 3, 3 >* R,	// Rodrigues �ϊ��O�� 3 * 3 ��]�s��
	Eigen::Vector3d* RodriguesVector	// Rodrigues ��p������]�\�� r = [ r1, r2, r3 ]
){

	/* ��]�p */
	 double theta = acos( ( ( ( *R ).trace() - 1 ) / 2 ) );

	/* ���K�����ꂽ��]�� */
	Eigen::Vector3d NormalizedRaxis;
	Eigen::Vector3d a;
	a( 0 ) = ( *R )( 2, 1 ) - ( *R )( 1, 2 );
	a( 1 ) = ( *R )( 0, 2 ) - ( *R )( 2, 0 );
	a( 2 ) = ( *R )( 1, 0 ) - ( *R )( 0, 1 );
	NormalizedRaxis = a / a.norm();

	/* ��]�� */
	Eigen::Vector3d Raxis;
	Raxis = NormalizedRaxis * theta;

	/* Rodrigues ��p������]�\�� r = [ r1, r2, r3 ] */
	( *RodriguesVector ) = Raxis;

	return 0;
}