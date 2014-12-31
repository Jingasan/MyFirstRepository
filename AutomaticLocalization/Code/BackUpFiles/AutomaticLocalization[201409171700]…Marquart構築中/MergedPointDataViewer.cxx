/***** 2点群の統合と可視化 *****/


/*** インクルードファイル ***/

/* 設定用ヘッダファイル */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** 名前空間の宣言 ***/

/* C++ */
using namespace std;


/*** 2点群の統合と可視化( PointXYZRGB ) ***/
int mergedPointDataViewerRGB( pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud1, pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud2, const char* outputFileName1, const char* outputFileName2 ){

	/* 統合後の点群 */
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr mergedPointCloud( new pcl::PointCloud< pcl::PointXYZRGB > );
	*mergedPointCloud = *cloud1 + *cloud2;


#ifdef VISUALIZE
	/* 統合された点群の可視化 */
	showPointCloudRGB( mergedPointCloud );
#endif


	/* 統合された点群の出力保存 */
	savePointCloudRGBtoPCD( mergedPointCloud, outputFileName1 );
	savePointCloudRGBtoPLY( mergedPointCloud, outputFileName2 );

	return 0;
}


/*** 2点群の統合と可視化( PointXYZ ) ***/
int mergedPointDataViewer( pcl::PointCloud< pcl::PointXYZ >::Ptr cloud1, pcl::PointCloud< pcl::PointXYZ >::Ptr cloud2, const char* outputFileName1, const char* outputFileName2 ){

	/* 統合後の点群 */
	pcl::PointCloud< pcl::PointXYZ >::Ptr mergedPointCloud( new pcl::PointCloud< pcl::PointXYZ > );
	*mergedPointCloud = *cloud1 + *cloud2;


#ifdef VISUALIZE
	/* 統合された点群の可視化 */
	showPointCloud( mergedPointCloud );
#endif


	/* 統合された点群の出力保存 */
	savePointCloudtoPCD( mergedPointCloud, outputFileName1 );
	savePointCloudtoPLY( mergedPointCloud, outputFileName2 );

	return 0;
}
