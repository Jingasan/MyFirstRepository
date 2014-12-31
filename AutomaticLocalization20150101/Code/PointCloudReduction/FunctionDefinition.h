/***** 関数の定義用ヘッダファイル *****/


/*** インクルードファイル ***/

/* PCL 1.6.0 */
#include <pcl/point_types.h>	// 点群のデータ型


/*** 関数の定義 ***/

/* 3次元点群の読み込み */
int loadPCDPointCloudRGB( pcl::PointCloud< pcl::PointXYZRGB >::Ptr, const char* );	// PCD( PointXYZRGB )
int loadPCDPointCloud( pcl::PointCloud< pcl::PointXYZ >::Ptr, const char* );		// PCD( PointXYZ )
int loadPLYPointCloudRGB( pcl::PointCloud< pcl::PointXYZRGB >::Ptr, const char* );	// PLY( PointXYZRGB )
int loadPLYPointCloud( pcl::PointCloud< pcl::PointXYZ >::Ptr, const char* );		// PLY( PointXYZ )

/* 3次元エッジ点群数の削減 */
int reductPointCloudRGB( pcl::PointCloud< pcl::PointXYZRGB >::Ptr, pcl::PointCloud< pcl::PointXYZRGB >::Ptr, float, float, float );	// PointXYZRGB
int reductPointCloud( pcl::PointCloud< pcl::PointXYZ >::Ptr, pcl::PointCloud< pcl::PointXYZ >::Ptr, float, float, float );			// PointXYZ

/* 3次元点群の出力保存 */
int savePointCloudRGBtoPCD( pcl::PointCloud< pcl::PointXYZRGB >::Ptr, const char* );	// PCD( PointXYZRGB )
int savePointCloudtoPCD( pcl::PointCloud< pcl::PointXYZ >::Ptr, const char* );			// PCD( PointXYZ )
int savePointCloudRGBtoPLY( pcl::PointCloud< pcl::PointXYZRGB >::Ptr, const char* );	// PLY( PointXYZRGB )
int savePointCloudtoPLY( pcl::PointCloud< pcl::PointXYZ >::Ptr, const char* );			// PLY( PointXYZ )
