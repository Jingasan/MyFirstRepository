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
int ICPOptomizationRGB(
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloudM,		// ���f���_�Q
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloudT,		// �T���Ώۓ_�Q
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr newCloudM,		// �ʒu���킹��̃��f���_�Q
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr mergedCloud	// ICP ��̓����_�Q
	//Eigen::Matrix< double, 3, 3 > *RIcp,					// ��]�s��
	//Eigen::Vector3d *tIcp									// ���i�x�N�g��
){

	/* ICP �A���S���Y�� */
	cout << "----- ICP �ɂ��œK���J�n -----" << "\r";
	pcl::IterativeClosestPoint< pcl::PointXYZRGB, pcl::PointXYZRGB > icp;
	icp.setInputCloud( cloudM );					// �ʒu���킹�Ώۂ̓_�Q
	icp.setInputTarget( cloudT );					// �ʒu���킹��̓_�Q
	icp.setRANSACOutlierRejectionThreshold( 1.0 );	// RANSAC��p�����O��l������臒l�ݒ�
	//icp.setMaxCorrespondenceDistance( 0.3 );		// �Ή������̍ő�l
	icp.setMaximumIterations( ICP_LOOP_MAX );		// ICP�̍ő唽����
	icp.align( *newCloudM );						// �ʒu���킹��̓_�Q
	cout << "----- ICP �ɂ��œK���I�� -----" << endl;


#ifdef ICPRESULT
	
	/* ICP ��̓_�Q�̓��� */
	*mergedCloud = *cloudT + *newCloudM;

	/* ICP �̌��� */
	Eigen::Matrix4f Rt = icp.getFinalTransformation();	// R, t �� 4 * 4 �s��
	double icpRMSE = sqrt( icp.getFitnessScore() );		// RMSE �]���l

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
int ICPOptomization(
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudM,		// ���f���_�Q
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudT,		// �T���Ώۓ_�Q
	pcl::PointCloud< pcl::PointXYZ >::Ptr newCloudM,	// �ʒu���킹��̃��f���_�Q
	pcl::PointCloud< pcl::PointXYZ >::Ptr mergedCloud	// ICP ��̓����_�Q
	//Eigen::Matrix< double, 3, 3 > *RIcp,				// ��]�s��
	//Eigen::Vector3d *tIcp								// ���i�x�N�g��
){
	
	/* ICP �A���S���Y�� */
	cout << "----- ICP �ɂ��œK���J�n -----" << "\r";
	pcl::IterativeClosestPoint< pcl::PointXYZ, pcl::PointXYZ > icp;
	icp.setInputCloud( cloudM );					// �ʒu���킹�Ώۂ̓_�Q
	icp.setInputTarget( cloudT );					// �ʒu���킹��̓_�Q
	icp.setRANSACOutlierRejectionThreshold( 1.0 );	// RANSAC��p�����O��l������臒l�ݒ�
	//icp.setMaxCorrespondenceDistance( 0.3 );		// �Ή������̍ő�l
	icp.setMaximumIterations( ICP_LOOP_MAX );		// ICP�̍ő唽����
	icp.align( *newCloudM );						// �ʒu���킹��̓_�Q
	cout << "----- ICP �ɂ��œK���I�� -----" << endl;


#ifdef ICPRESULT
	
	/* ICP ��̓_�Q�̓��� */
	*mergedCloud = *cloudT + *newCloudM;

	/* ICP �̌��� */
	Eigen::Matrix4f Rt = icp.getFinalTransformation();	// R, t �� 4 * 4 �s��
	double icpRMSE = sqrt( icp.getFitnessScore() );		// RMSE �]���l
	
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
	savePointCloudtoPCD( mergedCloud, IcpPCDFileName );
	savePointCloudtoPLY( mergedCloud, IcpPLYFileName );

#endif

#ifdef VISUALIZE
	
	/* �������ꂽ�_�Q�̉��� */
	showPointCloud( mergedCloud );

#endif

	return 0;
}