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
Eigen::Vector3d ConvertRodrigues(
	Eigen::Matrix< double, 3, 3 >* preR	// Rodrigues �ϊ��O�� 3 * 3 ��]�s��
){

	/* ��]�p */
	 double theta = acos( ( ( ( *preR ).trace() - 1.0 ) / 2.0 ) );

	/* ���K�����ꂽ��]�� */
	Eigen::Vector3d NormalizedRaxis;
	Eigen::Vector3d a;
	a( 0 ) = ( *preR )( 2, 1 ) - ( *preR )( 1, 2 );
	a( 1 ) = ( *preR )( 0, 2 ) - ( *preR )( 2, 0 );
	a( 2 ) = ( *preR )( 1, 0 ) - ( *preR )( 0, 1 );
	NormalizedRaxis = a / a.norm();

	/* ��]�� ( Rodrigues ��p������]�\�� r = [ r1, r2, r3 ] ) */
	Eigen::Vector3d Raxis;
	Raxis = NormalizedRaxis * theta;


	/* �Ԃ�l */
	return Raxis;	// Rodrigues ��p������]�\�� r = [ r1, r2, r3 ]
}