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

/* OpenMP */
#include <omp.h>


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
	int,									// DF の分割数( DF のボクセルの分解能 )
	int**,									// DF
	int**,									// 最近点の ID
	int*,									// 探索対象点群数
	struct ModelPointCloud**,				// 探索対象点群( DF の座標系に直したもの, 単位:mm )
	struct ModelInformation*,				// 探索対象点群の各種情報
	pcl::PointCloud< pcl::PointXYZ >::Ptr	// 探索対象点群( 単位:m )
);

/* DF 初期化関数 */
void BinarizationDFforPCDModel(
	int,									// DF の分割数( DF のボクセルの分解能 )
	int*,									// DF 作成のために一時的に保存する箱
	int**,									// 最近点 ID
	struct ModelPointCloud**,				// 点群( DF の座標系に直したもの, 単位:mm )
	int*,									// 点群数
	struct ModelInformation*,				// 点群の各種情報
	pcl::PointCloud< pcl::PointXYZ >::Ptr,	// 入力点群
	double,									// 点群の X 方向の重心
	double,									// 点群の Y 方向の重心
	double									// 点群の Z 方向の重心
);

/* 全探索 */
int ExhaustiveSearch(
	int,										// DF の分割数( DF のボクセルの分解能 )
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

/* ICP アルゴリズムを用いた位置合わせ */
int ICPOptimizationRGB(
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr,	// モデル点群
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr,	// 探索対象点群
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr,	// 最適化後のモデル点群
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr,	// 最適化後のモデル点群と探索対象点群の統合結果
	struct ModelInformation*,					// 探索対象点群の各種情報
	struct ModelInformation*,					// モデル点群の各種情報
	vector< Eigen::Matrix< double, 3, 3 > >*,	// 全探索結果の回転行列
	vector< Eigen::Vector3d >*					// 全探索結果の並進ベクトル
);
int ICPOptimization(
	pcl::PointCloud< pcl::PointXYZ >::Ptr,		// モデル点群
	pcl::PointCloud< pcl::PointXYZ >::Ptr,		// 探索対象点群
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr,		// 最適化後のモデル点群
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr,		// 最適化後のモデル点群と探索対象点群の統合結果
	struct ModelInformation*,					// 探索対象点群の各種情報
	struct ModelInformation*,					// モデル点群の各種情報
	vector< Eigen::Matrix< double, 3, 3 > >*,	// 全探索結果の回転行列
	vector< Eigen::Vector3d >*					// 全探索結果の並進ベクトル
);

/* マーカート法を用いた位置合わせ */
int LevMarOptimization(
	int**,										// DF
	int**,										// 最近点 ID
	struct ModelInformation*,					// 探索対象点群の各種情報
	struct ModelInformation*,					// モデル点群の各種情報
	pcl::PointCloud< pcl::PointXYZ >::Ptr,		// モデル点群
	pcl::PointCloud< pcl::PointXYZ >::Ptr,		// 探索対象点群
	vector< Eigen::Matrix< double, 3, 3 > >*,	// 全探索結果の回転行列
	vector< Eigen::Vector3d >*,					// 全探索結果の並進ベクトル
	vector< double >*							// 全探索結果の DF 値
);

/* 通常の回転行列 R から Rodrigues の回転表現 r = [ r1, r2, r3 ] への変換 */
Eigen::Vector3d ConvertRodrigues(
	Eigen::Matrix< double, 3, 3 >*	// Rodrigues 変換前の 3 * 3 回転行列
);

/* 回転行列の r1, r2, r3 成分による偏微分 */
Eigen::Matrix< double, 3, 3 > PartialDerivativeForRodrigues(
	int,				// 偏微分する回転成分番号 ⇒ 0：r1, 1：r2, 2：r3
	Eigen::Vector3d*	// Rodrigues を用いた回転表現 r = [ r1, r2, r3 ]
);