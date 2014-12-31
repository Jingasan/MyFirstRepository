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
void PartialDerivativeForRodrigues(
	Eigen::Vector3d* RodriguesVector,	// Rodrigues ��p������]�\�� r = [ r1, r2, r3 ]
	Eigen::Matrix3d* dR
){

	/* �ϐ��̒�` */
	double theta;					// ��]��3�����x�N�g���̃m����
	double theta2, theta3, theta4;	// theta ��X��
	Eigen::Matrix3d rx;				// �s�� rx
	Eigen::Matrix3d rx2;			// �s�� rx ��2��
	double cosTheta, sinTheta;		// �R�T�C���C�T�C���̒l
	Eigen::Matrix3d M[ 3 ];			// rx��r�̗v�f�ł��ꂼ����������Ƃ��̍s��(�Œ肳�ꂽ�l)
	Eigen::Matrix3d tmp;			// �ꎞ�I�Ɋm�ۂ���s��
	Eigen::Matrix3d r;				// Rodrigues �̉�]�x�N�g��
	
	/* Rodrigues �̉�]�x�N�g�� */
	r( 0 ) = (*RodriguesVector)( 0 );
	r( 1 ) = (*RodriguesVector)( 1 );
	r( 2 ) = (*RodriguesVector)( 2 );

	/* �Ƃ̎Z�o */
	theta = r.norm();

	/* �Ƃ� 0 �̏ꍇ �� �P�ʍs�� */
	if( theta == 0 ){
		for( int i = 0; i < 3; i++ ) dR[ i ] << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	}else{

		/* �Ƃ̏�Z */
		theta2 = theta * theta;		// 2��
		theta3 = theta2 * theta;	// 3��
		theta4 = theta3 * theta;	// 4��

		/* �s�� rx �̎Z�o */
		rx( 0, 0 ) = 0;	rx( 0, 1 ) = -1.0 * r( 2 ); rx( 0, 2 ) = r( 1 );
		rx( 1, 0 ) = r( 2 ); rx( 1, 1 ) = 0; rx( 1, 2 ) = -1.0 * r( 0 );
		rx( 2, 0 ) = -1.0 * r( 1 ); rx( 2, 1 ) = r( 0 ); rx( 2, 2 ) = 0;

		/* �s�� rx �̓��̎Z�o */
		rx2 = rx * rx;

		/* �s�� M �̍쐬 */
		M[ 0 ] << 0, 0, 0, 0, 0, -1, 0, 1, 0; 
		M[ 1 ] << 0, 0, 1, 0, 0, 0, -1, 0, 0; 
		M[ 2 ] << 0, -1, 0, 1, 0, 0, 0, 0, 0; 

		/* sin�ƁCcos�� �̒l���Z�o */
		cosTheta = cos( theta );
		sinTheta = sin( theta );

		/* ��]�s��̕Δ����F��]�x�N�g���R�v�f�ł��ꂼ����� */
		for( int i = 0; i < 3; i++ ){
			tmp = M[ i ] * rx + rx * M[ i ];
			dR[ i ] = ( ( r( i ) * cosTheta / theta2 ) - ( r( i ) * sinTheta / theta3 ) ) * rx.array()
				+ ( sinTheta / theta ) * M[ i ].array()
				+ ( ( r( i ) * sinTheta / theta3 ) - ( ( 2 * r( i ) * ( 1.0 - cosTheta ) ) / theta4 ) ) * rx2.array()
				+ ( ( 1.0 - cosTheta ) / theta2 ) * tmp.array();
		}
	}

	
}