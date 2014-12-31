/***** 関数の定義用ヘッダファイル *****/


/*** インクルードファイル ***/

/* PCL 1.6.0 */
#include <pcl/point_types.h>	// 点群のデータ型


/*** 関数の定義 ***/

/* 3次元点群の表示 */
int showPointCloudRGB( pcl::PointCloud< pcl::PointXYZRGB >::Ptr, int );	// PointXYZRGB
int showPointCloud( pcl::PointCloud< pcl::PointXYZ >::Ptr, int );		// PointXYZ

/* 3次元点群の出力保存 */
int savePointCloudRGBtoPCD( pcl::PointCloud< pcl::PointXYZRGB >::Ptr, const char* );	// PCD( PointXYZRGB )
int savePointCloudtoPCD( pcl::PointCloud< pcl::PointXYZ >::Ptr, const char* );			// PCD( PointXYZ )
int savePointCloudRGBtoPLY( pcl::PointCloud< pcl::PointXYZRGB >::Ptr, const char* );	// PLY( PointXYZRGB )
int savePointCloudtoPLY( pcl::PointCloud< pcl::PointXYZ >::Ptr, const char* );			// PLY( PointXYZ )
