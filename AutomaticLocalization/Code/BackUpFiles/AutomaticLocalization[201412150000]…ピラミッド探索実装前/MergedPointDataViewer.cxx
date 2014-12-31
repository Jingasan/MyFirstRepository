/***** 2�_�Q�̓����Ɖ��� *****/


/*** �C���N���[�h�t�@�C�� ***/

/* �ݒ�p�w�b�_�t�@�C�� */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** ���O��Ԃ̐錾 ***/

/* C++ */
using namespace std;


/*** 2�_�Q�̓����Ɖ���( PointXYZRGB ) ***/
int mergedPointDataViewerRGB( pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud1, pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud2, const char* outputFileName1, const char* outputFileName2 ){

	/* ������̓_�Q */
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr mergedPointCloud( new pcl::PointCloud< pcl::PointXYZRGB > );
	*mergedPointCloud = *cloud1 + *cloud2;


#ifdef VISUALIZE
	/* �������ꂽ�_�Q�̉��� */
	showPointCloudRGB( mergedPointCloud );
#endif


	/* �������ꂽ�_�Q�̏o�͕ۑ� */
	savePointCloudRGBtoPCD( mergedPointCloud, outputFileName1 );
	savePointCloudRGBtoPLY( mergedPointCloud, outputFileName2 );

	return 0;
}


/*** 2�_�Q�̓����Ɖ���( PointXYZ ) ***/
int mergedPointDataViewer( pcl::PointCloud< pcl::PointXYZ >::Ptr cloud1, pcl::PointCloud< pcl::PointXYZ >::Ptr cloud2, const char* outputFileName1, const char* outputFileName2 ){

	/* ������̓_�Q */
	pcl::PointCloud< pcl::PointXYZ >::Ptr mergedPointCloud( new pcl::PointCloud< pcl::PointXYZ > );
	*mergedPointCloud = *cloud1 + *cloud2;


#ifdef VISUALIZE
	/* �������ꂽ�_�Q�̉��� */
	showPointCloud( mergedPointCloud );
#endif


	/* �������ꂽ�_�Q�̏o�͕ۑ� */
	savePointCloudtoPCD( mergedPointCloud, outputFileName1 );
	savePointCloudtoPLY( mergedPointCloud, outputFileName2 );

	return 0;
}
