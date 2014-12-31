/***** ICP �A���S���Y����p�����œK�� *****/


/*** �C���N���[�h�t�@�C�� ***/

/* PCL 1.6.0 */
#include <pcl/registration/transformation_estimation.h>	// �_�Q�̍��W�ϊ�
#include <pcl/registration/icp.h>						// ICP

/* �ݒ�p�w�b�_�t�@�C�� */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** ���O��Ԃ̐錾 ***/

/* C++ */
using namespace std;


/*** ICP ��p�����œK��( PointXYZRGB ) ***/
int ICPOptimizationRGB(
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloudM,		// ���f���_�Q
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloudT,		// �T���Ώۓ_�Q
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr newCloudM,		// �ʒu���킹��̃��f���_�Q
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr mergedCloud,	// ICP ��̓����_�Q
	struct ModelInformation* targetInformation,				// �T���Ώۓ_�Q�̊e����
	struct ModelInformation* modelInformation,				// ���f���_�Q�̊e����
	vector< Eigen::Matrix< double, 3, 3 > > *ER,			// �S�T�����ʂ̉�]�s��
	vector< Eigen::Vector3d > *Et							// �S�T�����ʂ̕��i�x�N�g��
){

	/*** ICP �̑O�����F�����ʒu�p�� ***/

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
		temp.r = cloudT->points[ i ].r;
		temp.g = cloudT->points[ i ].g;
		temp.b = cloudT->points[ i ].b;
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
		temp.r = cloudM->points[ i ].r;
		temp.g = cloudM->points[ i ].g;
		temp.b = cloudM->points[ i ].b;
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


	/*** ICP �A���S���Y�� ***/
	cout << endl << "----- ICP �ɂ��œK���J�n -----" << "\r";
	pcl::IterativeClosestPoint< pcl::PointXYZRGB, pcl::PointXYZRGB > icp;
	icp.setInputCloud( transformedPointCloud );	// �ʒu���킹�Ώۂ̓_�Q
	icp.setInputTarget( targetPointCloud );	// �ʒu���킹��̓_�Q
	icp.setRANSACOutlierRejectionThreshold( RANSAC_OUTLIER_REJECTION_THRESHOLD );	// RANSAC��p�����O��l������臒l�ݒ�
	//icp.setMaxCorrespondenceDistance( MAX_CORRESPONDENCE_DISTANCE );	// �Ή������̍ő�l
	icp.setMaximumIterations( ICP_LOOP_MAX );	// ICP�̍ő唽����
	icp.align( *newCloudM );					// �ʒu���킹��̓_�Q
	cout << "----- ICP �ɂ��œK���I�� -----" << endl;


#ifdef ICPRESULT
	
	/* ICP ��̓_�Q�̓��� */
	*mergedCloud = *targetPointCloud + *newCloudM;

	/* ICP �̌��� */
	Eigen::Matrix4f Rt = icp.getFinalTransformation();	// R, t �� 4 * 4 �s��
	double icpRMSE = sqrt( newCloudM->size() * icp.getFitnessScore() ) / newCloudM->size();		// RMSE �]���l

	/* PCL ��p���� ICP ���� R, t �� RMSE �̃R�}���h���C���o�� */
	cout << endl << "----- ICP �œK������ [ R | t ] ----" << endl;
	cout << icp.getFinalTransformation() << endl;
	cout << "1�_�Ԃ������ RMSE : " << icpRMSE << endl;

	/* ICP ���ʂ̏o�͕ۑ� */
	FILE *outputFp1, *outputFp2;
	if( ( outputFp1 = fopen( IcpRtFileName, "w" ) ) == NULL ){ cout << "Caution!: " << IcpRtFileName << " open error" << endl; return 1; }
	if( ( outputFp2 = fopen( IcpRMSEFileName, "w" ) ) == NULL ){ cout << "Caution!: " << IcpRMSEFileName << " open error" << endl; return 1; }
	fprintf( outputFp1, "%f,%f,%f,%f\n%f,%f,%f,%f\n%f,%f,%f,%f\n%f,%f,%f,%f\n",
		Rt( 0, 0 ), Rt( 0, 1 ), Rt( 0, 2 ), Rt( 0, 3 ), Rt( 1, 0 ), Rt( 1, 1 ), Rt( 1, 2 ), Rt( 1, 3 ),
		Rt( 2, 0 ), Rt( 2, 1 ), Rt( 2, 2 ), Rt( 2, 3 ), Rt( 3, 0 ), Rt( 3, 1 ), Rt( 3, 2 ), Rt( 3, 3 )
	);
	fprintf( outputFp2, "RMSE,%f", icpRMSE );
	fclose( outputFp1 ); fclose( outputFp2 );

	/* �_�Q�̏o�͕ۑ� */
	savePointCloudRGBtoPCD( mergedCloud, IcpPCDFileName );
	savePointCloudRGBtoPLY( mergedCloud, IcpPLYFileName );

#endif

#ifdef VISUALIZE
	
	/* �������ꂽ�_�Q�̉��� */
	showPointCloudRGB( mergedCloud );

#endif

	return 0;
}


/*** ICP ��p�����œK��( PointXYZ ) ***/
int ICPOptimization(
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudM,			// ���f���_�Q
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudT,			// �T���Ώۓ_�Q
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr newCloudM,		// �ʒu���킹��̃��f���_�Q
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr mergedCloud,	// ICP ��̓����_�Q
	struct ModelInformation* targetInformation,				// �T���Ώۓ_�Q�̊e����
	struct ModelInformation* modelInformation,				// ���f���_�Q�̊e����
	vector< Eigen::Matrix< double, 3, 3 > > *ER,			// �S�T�����ʂ̉�]�s��
	vector< Eigen::Vector3d > *Et							// �S�T�����ʂ̕��i�x�N�g��
){
	
	/*** ICP �̑O�����F�����ʒu�p�� ***/

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


	/*** ICP �A���S���Y�� ***/
	cout << "----- ICP �ɂ��œK���J�n -----" << "\r";
	pcl::IterativeClosestPoint< pcl::PointXYZRGB, pcl::PointXYZRGB > icp;
	icp.setInputCloud( transformedPointCloud );	// �ʒu���킹�Ώۂ̓_�Q
	icp.setInputTarget( targetPointCloud );		// �ʒu���킹��̓_�Q
	icp.setRANSACOutlierRejectionThreshold( RANSAC_OUTLIER_REJECTION_THRESHOLD );	// RANSAC��p�����O��l������臒l�ݒ�
	//icp.setMaxCorrespondenceDistance( MAX_CORRESPONDENCE_DISTANCE );	// �Ή������̍ő�l
	icp.setMaximumIterations( ICP_LOOP_MAX );	// ICP�̍ő唽����
	icp.align( *newCloudM );					// �ʒu���킹��̓_�Q
	cout << "----- ICP �ɂ��œK���I�� -----" << endl;


#ifdef ICPRESULT
	
	/* ICP ��̓_�Q�̓��� */
	*mergedCloud = *targetPointCloud + *newCloudM;

	/* ICP �̌��� */
	Eigen::Matrix4f Rt = icp.getFinalTransformation();	// R, t �� 4 * 4 �s��
	double icpRMSE = sqrt( newCloudM->size() * icp.getFitnessScore() ) / newCloudM->size();		// RMSE �]���l

	
	/* PCL ��p���� ICP ���� R, t �� RMSE �̃R�}���h���C���o�� */
	cout << endl << "----- ICP �œK������ [ R | t ] ----" << endl;
	cout << icp.getFinalTransformation() << endl;
	cout << "1�_�Ԃ������ RMSE : " << icpRMSE << "[mm]" << endl;

	/* ICP ���ʂ̏o�͕ۑ� */
	FILE *outputFp1, *outputFp2;
	if( ( outputFp1 = fopen( IcpRtFileName, "w" ) ) == NULL ){ cout << "Caution!: " << IcpRtFileName << " open error" << endl; return 1; }
	if( ( outputFp2 = fopen( IcpRMSEFileName, "w" ) ) == NULL ){ cout << "Caution!: " << IcpRMSEFileName << " open error" << endl; return 1; }
	fprintf( outputFp1, "%f,%f,%f,%f\n%f,%f,%f,%f\n%f,%f,%f,%f\n%f,%f,%f,%f\n",
		Rt( 0, 0 ), Rt( 0, 1 ), Rt( 0, 2 ), Rt( 0, 3 ), Rt( 1, 0 ), Rt( 1, 1 ), Rt( 1, 2 ), Rt( 1, 3 ),
		Rt( 2, 0 ), Rt( 2, 1 ), Rt( 2, 2 ), Rt( 2, 3 ), Rt( 3, 0 ), Rt( 3, 1 ), Rt( 3, 2 ), Rt( 3, 3 )
	);
	fprintf( outputFp2, "RMSE,%f", icpRMSE );
	fclose( outputFp1 ); fclose( outputFp2 );

	/* �_�Q�̏o�͕ۑ� */
	savePointCloudRGBtoPCD( mergedCloud, IcpPCDFileName );
	savePointCloudRGBtoPLY( mergedCloud, IcpPLYFileName );

#endif

#ifdef VISUALIZE
	
	/* �������ꂽ�_�Q�̉��� */
	showPointCloudRGB( mergedCloud );

#endif

	return 0;
}