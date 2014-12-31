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
     double deltaX;	// 点群の X 軸の幅
     double deltaY;	// 点群の Y 軸の幅
     double deltaZ;	// 点群の Z 軸の幅
     double max;	// 点群の最大幅
	 double gravX;	// 点群の重心 X
	 double gravY;	// 点群の重心 Y
	 double gravZ;	// 点群の重心 Z
     int iSize;		// DF の i 軸の幅
     int jSize;		// DF の j 軸の幅
     int kSize;		// DF の k 軸の幅
};


/*** 関数の定義 ***/

/* 点群の座標変換：点群の重心を原点座標に設定 */
int CoordinateTransformation(
	struct PointCloudInfo*,						// モデル点群情報
	struct PointCloudInfo*,						// 探索対象点群情報
	pcl::PointCloud< pcl::PointXYZ >::Ptr,		// 座標変換前のモデル点群
	pcl::PointCloud< pcl::PointXYZ >::Ptr,		// 座標変換前の探索対象点群
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr,	// 座標変換後のモデル点群
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr	// 座標変換後の探索対象点群
);

/* 通常の回転行列 R から Rodrigues の回転表現 r = [ r1, r2, r3 ] への変換 */
Eigen::Vector3d ConvertRodrigues(
	Eigen::Matrix< double, 3, 3 >*	// Rodrigues 変換前の 3 * 3 回転行列
);

/* Rodrigues の回転行列の復元 */
Eigen::Matrix< double, 3, 3 > ReconstructRFromRodrigues(
	Eigen::Vector3d*	// Rodrigues を用いた回転表現 r = [ r1, r2, r3 ]
);

/* 回転行列の r1, r2, r3 成分による偏微分 */
void PartialDerivativeForRodrigues(
	Eigen::Vector3d*,	// Rodrigues を用いた回転表現 r = [ r1, r2, r3 ]
	Eigen::Matrix3d* dR	// 
);
