/***** 関数の定義用ヘッダファイル *****/


/*** インクルードファイル ***/

/* PCL 1.6.0 */
#include <pcl/point_types.h>	// 点群のデータ型


/*** 構造体の定義 ***/

/* 3次元点群の各種情報格納用 */
struct PointCloudInfo{
     double Xmin;	// 点群の X 座標の最小値
     double Xmax;	// 点群の X 座標の最大値
     double Ymin;	// 点群の Y 座標の最小値
     double Ymax;	// 点群の Y 座標の最大値
     double Zmin;	// 点群の Z 座標の最小値
     double Zmax;	// 点群の Z 座標の最大値
	 double gravX;	// 点群の重心 X
	 double gravY;	// 点群の重心 Y
	 double gravZ;	// 点群の重心 Z
};


/*** 関数の定義 ***/

/* 3次元点群の表示 */
int showPointCloudRGB( pcl::PointCloud< pcl::PointXYZRGB >::Ptr, int );	// PointXYZRGB
int showPointCloud( pcl::PointCloud< pcl::PointXYZ >::Ptr, int );		// PointXYZ

/* 3次元点群の出力保存 */
int savePointCloudRGBtoPCD( pcl::PointCloud< pcl::PointXYZRGB >::Ptr, const char* );	// PCD( PointXYZRGB )
int savePointCloudtoPCD( pcl::PointCloud< pcl::PointXYZ >::Ptr, const char* );			// PCD( PointXYZ )
int savePointCloudRGBtoPLY( pcl::PointCloud< pcl::PointXYZRGB >::Ptr, const char* );	// PLY( PointXYZRGB )
int savePointCloudtoPLY( pcl::PointCloud< pcl::PointXYZ >::Ptr, const char* );			// PLY( PointXYZ )
