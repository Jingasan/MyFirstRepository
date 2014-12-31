/***** 関数、構造体の定義 *****/


/*** インクルードファイル ***/

/* C */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>

/* C++ */
#include <iostream>
#include <fstream>
#include <vector>

/* PCL 1.6.0 */
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>	// 点群の可視化

/*** 名前空間の宣言 ***/
using namespace std;


/*** 構造体の定義 ***/

/* 3次元点群格納用 */
struct ModelPointCloud{
	double Xmodel;	// 点群モデル座標系の X
	double Ymodel;	// 点群モデル座標系の Y
	double Zmodel;	// 点群モデル座標系の Z
};

/* 3次元点群の各種情報格納用 */
struct ModelInformation{
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

/* DF 作成関数 */
void CreateDistanceFieldforPCDModel(
	int,
	int **,
	int **,
	int *,
	struct ModelPointCloud**,
	struct ModelInformation*,
	pcl::PointCloud< pcl::PointXYZ >::Ptr
);

/* DF 初期化関数 */
void BinarizationDFforPCDModel(
	int,
	int *,
	int **,
	struct ModelPointCloud**,
	int *,
	struct ModelInformation *,
	pcl::PointCloud< pcl::PointXYZ >::Ptr,
	double,
	double,
	double
);

/* 全探索 */
int ExhaustiveSearch(
	int**,										// DF
	struct ModelInformation*,					// 探索対象点群の各種情報
	struct ModelInformation*,					// モデル点群の各種情報
	int*,										// 探索対象点群とモデル点群との距離評価合計値の最小値
	pcl::PointCloud< pcl::PointXYZ >::Ptr,		// モデル点群
	pcl::PointCloud< pcl::PointXYZ >::Ptr,		// 探索対象点群
	vector< Eigen::Matrix< double, 3, 3 > >*,	// 全探索結果の回転行列
	vector< Eigen::Vector3d >*,					// 全探索結果の並進ベクトル
	vector< double >*							// 全探索結果の DF 値
	//Eigen::Matrix< double, 3, 3 >**			// 回転行列 R
);



/* 2点群の統合と可視化 */
int mergedPointDataViewerRGB( pcl::PointCloud< pcl::PointXYZRGB >::Ptr, pcl::PointCloud< pcl::PointXYZRGB >::Ptr, const char*, const char* );	// PointXYZRGB
int mergedPointDataViewer( pcl::PointCloud< pcl::PointXYZ >::Ptr, pcl::PointCloud< pcl::PointXYZ >::Ptr, const char*, const char* );			// PointXYZ

/* 点群の可視化 */
int showPointCloudRGB( pcl::PointCloud< pcl::PointXYZRGB >::Ptr );	// PointXYZRGB
int showPointCloud( pcl::PointCloud< pcl::PointXYZ >::Ptr );		// PointXYZ

/* 点群の出力保存 */
int savePointCloudRGBtoPCD( pcl::PointCloud< pcl::PointXYZRGB >::Ptr, const char* );	// PCD( PointXYZRGB )
int savePointCloudtoPCD( pcl::PointCloud< pcl::PointXYZ >::Ptr, const char* );			// PCD( PointXYZ )
int savePointCloudRGBtoPLY( pcl::PointCloud< pcl::PointXYZRGB >::Ptr, const char* );	// PLY( PointXYZRGB )
int savePointCloudtoPLY( pcl::PointCloud< pcl::PointXYZ >::Ptr, const char* );			// PLY( PointXYZ )
