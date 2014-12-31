/***** Levenberg Marquardt �@��p�����œK�� *****/


/*** �C���N���[�h�t�@�C�� ***/

/* �ݒ�p�w�b�_�t�@�C�� */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** ���O��Ԃ̐錾 ***/

/* C++ */
using namespace std;


/*** Levenberg Marquardt �@��p�����œK�� ***/
int LevMarOptimization(
	int** DistanceField,							// DF
	int** ClosestPointID,							// �ŋߓ_�� ID
	struct ModelInformation* targetInformation,		// �T���Ώۓ_�Q�̊e����
	struct ModelInformation* modelInformation,		// ���f���_�Q�̊e����
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudM,	// ���f���_�Q
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudT,	// �T���Ώۓ_�Q
	vector< Eigen::Matrix< double, 3, 3 > > *ER,	// �S�T�����ʂ̉�]�s��
	vector< Eigen::Vector3d > *Et,					// �S�T�����ʂ̕��i�x�N�g��
	vector< double > *DFval							// �S�T�����ʂ� DF �l
){
	

	/*** Marquart�@�̑O�����F�����ʒu�p�� ***/

	/* �_�Q�̃������m�� */
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr targetPointCloud( new pcl::PointCloud< pcl::PointXYZRGB > );		// �d�S�ϊ���̒T���Ώۓ_�Q
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr modelPointCloud( new pcl::PointCloud< pcl::PointXYZRGB > );		// �d�S�ϊ���̃��f���_�Q
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr transformedPointCloud( new pcl::PointCloud< pcl::PointXYZRGB > );	// �S�T�����ʂɂ��A�t�B���ϊ���̃��f���_�Q
	pcl::PointXYZRGB temp;																						// �_���W�̈ꎞ�i�[�p�ϐ�

	/* �T���Ώۓ_�Q */
	for( int i = 0; i < cloudT->size(); i++ ){
			
		/* �T���Ώۓ_�Q�� DF ���W�n�֕ϊ��FDF ���W�n�͓_�Q�̏d�S�����_ */
		temp.x = cloudT->points[ i ].x - ( targetInformation->gravX / (double)TOMETER );
		temp.y = cloudT->points[ i ].y - ( targetInformation->gravY / (double)TOMETER );
		temp.z = cloudT->points[ i ].z - ( targetInformation->gravZ / (double)TOMETER );
		temp.r = 0;
		temp.g = 255;
		temp.b = 0;
		targetPointCloud->points.push_back( temp );
	}
	targetPointCloud->width = targetPointCloud->points.size();
	targetPointCloud->height = 1;

	/* ���f���_�Q */
	for( int i = 0; i < cloudM->size(); i++ ){
			
		/* ���f���_�Q�� DF ���W�n�֕ϊ��FDF ���W�n�͓_�Q�̏d�S�����_ */
		temp.x = cloudM->points[ i ].x - ( modelInformation->gravX / (double)TOMETER );
		temp.y = cloudM->points[ i ].y - ( modelInformation->gravY / (double)TOMETER );
		temp.z = cloudM->points[ i ].z - ( modelInformation->gravZ / (double)TOMETER );
		temp.r = 255;
		temp.g = 0;
		temp.b = 0;
		modelPointCloud->points.push_back( temp );
	}
	modelPointCloud->width = cloudM->points.size();
	modelPointCloud->height = 1;

	/* �S�T�����ʂɂ��A�t�B���ϊ� */
	Eigen::Vector3d p;	// ���f���_�Q�̈ꎞ�i�[�p�z��
	Eigen::Vector3d ap;	// �A�t�B���ϊ���_�Q�̈ꎞ�i�[�p�z��
	for( int i = 0; i < cloudM->size(); i++ ){
		
		/* �S�T�����ʂɂ��A�t�B���ϊ� */
		p( 0 ) = modelPointCloud->points[ i ].x;
		p( 1 ) = modelPointCloud->points[ i ].y;
		p( 2 ) = modelPointCloud->points[ i ].z;
		ap = (*ER)[ 0 ] * p + (*Et)[ 0 ];
		temp.x = ap( 0 );
		temp.y = ap( 1 );
		temp.z = ap( 2 );
		temp.r = modelPointCloud->points[ i ].r;
		temp.g = modelPointCloud->points[ i ].g;
		temp.b = modelPointCloud->points[ i ].b;
		transformedPointCloud->points.push_back( temp );
	}
	transformedPointCloud->width = modelPointCloud->points.size();
	transformedPointCloud->height = 1;


	/*** �ϐ��̐錾 ***/
	Eigen::Matrix< double, 3, 3 > Rtemp;	// Rodrigues �ϊ��O�� 3 * 3 ��]�s��
	Eigen::Vector3d RodriguesVector;		// Rodrigues ��p������]�\�� r = [ r1, r2, r3 ]
	Eigen::Matrix< double, 3, 3 > dR[ 3 ];

	/*** Rodrigues ��p������]�\���ւ̕ϊ� ***/
	
	/* Rodrigues �ϊ��O�̉�]�s�� */
	Rtemp = (*ER)[ 0 ];

	/* Rodrigues �̉�]�\�� r = [ r1, r2, r3 ] �ւ̕ϊ� */
	ConvertRodrigues( &Rtemp, &RodriguesVector );

	/*** ��]�s��� r1, r2, r3 �����ɂ��Δ��� ***/
	dR[ 0 ] = PartialDerivativeForRodrigues( 0, &RodriguesVector );	// r1 �ɂ��Δ������ʂ� 3 * 3 ��]�s��
	dR[ 1 ] = PartialDerivativeForRodrigues( 1, &RodriguesVector );	// r2 �ɂ��Δ������ʂ� 3 * 3 ��]�s��
	dR[ 2 ] = PartialDerivativeForRodrigues( 2, &RodriguesVector );	// r3 �ɂ��Δ������ʂ� 3 * 3 ��]�s��




#ifdef COMMENT

	/* Rodrigues �̉�]�\���ɂ��\�������s�� [r]x �̍쐬 */
	Eigen::Matrix< double, 3, 3 > RodriguesMatrix;	// Rodrigues �̉�]�\���ɂ��\������� 3 * 3 �s�� [r]x
	RodriguesMatrix( 0, 0 ) = 0; RodriguesMatrix( 0, 1 ) = - RodriguesVector( 2 ); RodriguesMatrix( 0, 2 ) = RodriguesVector( 1 );
	RodriguesMatrix( 1, 0 ) = RodriguesVector( 2 ); RodriguesMatrix( 1, 1 ) = 0; RodriguesMatrix( 1, 2 ) = - RodriguesVector( 0 );
	RodriguesMatrix( 2, 0 ) = - RodriguesVector( 1 ); RodriguesMatrix( 2, 1 ) = RodriguesVector( 0 ); RodriguesMatrix( 2, 2 ) = 0;
	
	cout << endl << ">> Rodrigues �ϊ��O�̉�]�s�� R" << endl;
	cout << Rtemp << endl;
	cout << endl << ">> Rodrigues ��p������]�\�� r = [ r1, r2, r3 ]" << endl;
	cout << "r1: " << RodriguesVector( 0 ) << ", r2: " << RodriguesVector( 1 ) << ", r3: " << RodriguesVector( 2 ) << endl;
	cout << ">> Rodrigues ��p������]�\���ɂ��\�������s�� [r]x" << endl;
	cout << RodriguesMatrix << endl;
	cout << endl << ">> r1 �ɂ��Δ������ʂ̉�]�s��" << endl;
	cout << dR[ 0 ] << endl;
	cout << endl << ">> r2 �ɂ��Δ������ʂ̉�]�s��" << endl;
	cout << dR[ 1 ] << endl;
	cout << endl << ">> r3 �ɂ��Δ������ʂ̉�]�s��" << endl;
	cout << dR[ 2 ] << endl;

#endif





#ifdef COMMENT
	cout << endl << "--- �S�T������ ---" << endl;
	cout << "R:" << endl << (*ER)[ 0 ]( 0, 0 ) << " " << (*ER)[ 0 ]( 0, 1 ) << " " << (*ER)[ 0 ]( 0, 2 ) << endl;
	cout << (*ER)[ 0 ]( 1, 0 ) << " " << (*ER)[ 0 ]( 1, 1 ) << " " << (*ER)[ 0 ]( 1, 2 ) << endl;
	cout << (*ER)[ 0 ]( 2, 0 ) << " " << (*ER)[ 0 ]( 2, 1 ) << " " << (*ER)[ 0 ]( 2, 2 ) << endl;
	cout << "t:" << endl << (*Et)[ 0 ]( 0 ) << " "<< (*Et)[ 0 ]( 1 ) << " "<< (*Et)[ 0 ]( 2 ) << endl;
	cout << "DF:" << endl << (*DFval)[ 0 ] << endl << endl;
#endif


	return 0;
}